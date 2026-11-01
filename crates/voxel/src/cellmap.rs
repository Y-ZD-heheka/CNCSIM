//! 单元格 → 顶点索引：**块 + 位掩码 + popcount**，不是一张 1600 万条的哈希表。
//!
//! ## 抄的是 OpenVDB 哪一点
//!
//! OpenVDB 的树里**只有 root 是哈希表**，往下全靠位运算：
//!
//! ```text
//! root（哈希，1~8 条） → internal 节点（32³ 槽，u64 位掩码 + popcount 找子节点）
//!                     → internal 节点（16³ 槽，同上）→ leaf（8³ = 512 个值的连续数组）
//! ```
//!
//! 取一个体素 = 1 次 root 哈希 + 3 次位运算 + 1 次数组下标；子节点在
//! `children[popcount(mask & 低位)]`——**位掩码就是"哪些槽有东西"的压缩索引**。
//!
//! 本模块把这套用在同一层上：顶点是按单元格生成的，而**每 512 个相邻单元格里
//! 只有几十个有顶点**（表面是二维的，体素空间是三维的）。所以：
//!
//! - 每 8³ 个单元格为一个**块**（与叶的体素格对齐，`cell >> 3`）；
//! - 每块存 512 位掩码（`[u64; 8]`，64 字节）+ 一个起始索引；
//! - 单元格的索引 = `base + popcount(掩码里它前面的位)`。
//!
//! 于是查找不碰任何"1600 万条的大表"，只碰一张 25 万条的块表（8 MB，常驻缓存）
//! 加一个 64 字节的掩码。**200 mm 毛坯实测：索引从 310 MB 降到 25 MB。**
//!
//! ## 为什么不直接存 `[u32; 512]` 每块
//!
//! 那是 25 万 × 2 KB = **500 MB**，比哈希表还大。位掩码把"有没有"压到 1 bit，
//! "是多少"由前缀和算出来——这是全篇唯一的关键技巧。
//!
//! ## 顶点顺序因此改变
//!
//! 索引 = 块序 + 掩码序，所以顶点顺序是**空间序**（按块坐标、再按块内偏移），
//! 不再是"叶迭代顺序"。这对渲染无影响，但**细格与 LOD 必须走同一个构建器**，
//! 否则 `k=1` 与细格逐位一致那条测试会红（事实上一开始就是它逼着两边共用）。

use crate::fxhash::FxHashMap;
use crate::MeshData;
use std::sync::atomic::{AtomicU32, Ordering};

/// 一个块覆盖 8³ = 512 个单元格
pub(crate) const BLOCK_CELLS: usize = 512;
const BLOCK_WORDS: usize = BLOCK_CELLS / 64; // 8 个 u64

/// 一个候选顶点。`cell` 是它的身份（去重靠它），位置/法向是内容。
///
/// 位置与法向在这里就转成 f32：最终要进 `MeshData`，早转省一半内存
/// （16 M 条 × 60 B → 36 B）。
#[derive(Clone, Copy)]
pub(crate) struct VertexRecord {
    pub cell: [i32; 3],
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    /// 调色板索引（ADR-005 的叶级溯源，见 `SparseGrid::creator_at`）
    pub creator: u8,
}

/// 块坐标（`cell >> 3`，算术右移，负坐标正确）
#[inline]
fn block_of(cell: [i32; 3]) -> [i32; 3] {
    [cell[0] >> 3, cell[1] >> 3, cell[2] >> 3]
}

/// 块内槽号（0..512），索引 = `x | y<<3 | z<<6`
#[inline]
fn local_of(cell: [i32; 3]) -> usize {
    ((cell[0] & 7) | ((cell[1] & 7) << 3) | ((cell[2] & 7) << 6)) as usize
}

/// 块查找缓存（线程局部）：记住上一次命中的块坐标与序号。
#[derive(Default)]
pub(crate) struct BlockCache {
    coord: [i32; 3],
    slot: u32,
    valid: bool,
}

/// 单元格 → 顶点索引。
pub(crate) struct CellMap {
    /// 块坐标 → 块序号（0..n，**已按坐标排序**）
    blocks: FxHashMap<[i32; 3], u32>,
    /// 块坐标，按序号排列
    coords: Vec<[i32; 3]>,
    /// 每块 8 个 u64：哪些槽有顶点
    words: Vec<u64>,
    /// 每块第一个顶点的全局索引（前缀和）
    base: Vec<u32>,
}

impl CellMap {
    /// 从各块产出的记录构建网格与索引。
    ///
    /// 两遍：
    /// 1. **串行**建块表 + 置位（1600 万次**小表**查找，表 8 MB，命中缓存）；
    /// 2. **并行**按块序算 rank 并写入顶点数组（索引与块一一对应，互不重叠）。
    ///
    /// 重复记录（相邻叶各算一次同一格）在 1 里天然合并——同一个 bit 置两次。
    /// 2 里它们会写同一个下标、写同样的字节，所以用 relaxed 原子写：
    /// **不用 unsafe，也不依赖"谁先谁后"**。
    pub(crate) fn build(chunks: &[Vec<VertexRecord>]) -> (MeshData, Self) {
        let mut blocks: FxHashMap<[i32; 3], u32> = FxHashMap::default();
        // 先按"出现顺序"编号，之后整体重排成坐标序
        let mut coords: Vec<[i32; 3]> = Vec::new();
        let mut words: Vec<u64> = Vec::new();
        let mut count: Vec<u32> = Vec::new();

        for chunk in chunks {
            for r in chunk {
                let b = block_of(r.cell);
                let slot = match blocks.get(&b) {
                    Some(&s) => s,
                    None => {
                        let s = coords.len() as u32;
                        blocks.insert(b, s);
                        coords.push(b);
                        words.extend_from_slice(&[0u64; BLOCK_WORDS]);
                        count.push(0);
                        s
                    }
                };
                let li = local_of(r.cell);
                let w = &mut words[slot as usize * BLOCK_WORDS + li / 64];
                let bit = 1u64 << (li % 64);
                if *w & bit == 0 {
                    *w |= bit;
                    count[slot as usize] += 1;
                }
            }
        }

        // ── 重排成坐标序：索引必须与线程调度无关 ──
        let mut order: Vec<u32> = (0..coords.len() as u32).collect();
        order.sort_unstable_by_key(|&i| coords[i as usize]);
        let mut sorted_coords = Vec::with_capacity(coords.len());
        let mut sorted_words = Vec::with_capacity(words.len());
        let mut base = Vec::with_capacity(count.len());
        let mut acc = 0u32;
        for &old in &order {
            let o = old as usize;
            base.push(acc);
            acc += count[o];
            sorted_coords.push(coords[o]);
            sorted_words.extend_from_slice(&words[o * BLOCK_WORDS..][..BLOCK_WORDS]);
        }
        let total = acc as usize;

        let mut map: FxHashMap<[i32; 3], u32> = FxHashMap::default();
        map.reserve(sorted_coords.len());
        for (i, c) in sorted_coords.iter().enumerate() {
            map.insert(*c, i as u32);
        }

        let cellmap = Self {
            blocks: map,
            coords: sorted_coords,
            words: sorted_words,
            base,
        };

        // ── 并行写顶点 ──
        // 先按 u32 分配（calloc，快），再就地当作原子数组用。
        let mut pos: Vec<u32> = vec![0; total * 3];
        let mut nrm: Vec<u32> = vec![0; total * 3];
        // 创建者标记：同一格可能被相邻两片叶各发一次（halo 格），两片叶的
        // 标记不一定相同。**必须用 `fetch_min` 而不是直接 store**：并行写同一
        // 下标的顺序不定，直接 store 会让结果随线程调度变化，破坏"逐位可复现"。
        // 取最小 = 取最早的那次切割，语义上也说得通（谁先碰到这个面就算谁的）。
        let mut col: Vec<u32> = vec![u32::MAX; total];
        {
            let pos_at = AtomicU32::from_mut_slice(&mut pos);
            let nrm_at = AtomicU32::from_mut_slice(&mut nrm);
            let col_at = AtomicU32::from_mut_slice(&mut col);
            let cm: &Self = &cellmap;
            crate::par::map_indexed(chunks.len().max(1), |i| {
                let Some(chunk) = chunks.get(i) else { return };
                for r in chunk {
                    let Some(idx) = cm.get(r.cell) else { continue };
                    let at = idx as usize * 3;
                    for k in 0..3 {
                        // relaxed：只要求"写进去"，不要求顺序——同一下标写的是同样的字节
                        pos_at[at + k].store(r.pos[k].to_bits(), Ordering::Relaxed);
                        nrm_at[at + k].store(r.normal[k].to_bits(), Ordering::Relaxed);
                    }
                    col_at[idx as usize].fetch_min(r.creator as u32, Ordering::Relaxed);
                }
            });
        }

        let mut mesh = MeshData {
            positions: pos.into_iter().map(f32::from_bits).collect::<Vec<f32>>().chunks(3)
                .map(|c| [c[0], c[1], c[2]]).collect(),
            normals: nrm.into_iter().map(f32::from_bits).collect::<Vec<f32>>().chunks(3)
                .map(|c| [c[0], c[1], c[2]]).collect(),
            // R = 调色板索引（叶级溯源），G = 余量偏差编码（暂无，写 128 = 零）
            colors: col
                .into_iter()
                .map(|c| [(c.min(255)) as u8, 128, 0, 255])
                .collect(),
            indices: Vec::new(),
            skipped_quads: 0,
        };
        mesh.positions.shrink_to_fit();
        mesh.normals.shrink_to_fit();

        (mesh, cellmap)
    }

    /// **带块查找缓存**的 `get`（线程局部，`BlockCache` 由调用方持有）。
    ///
    /// 四边形发射时每格要查 3 次（`p−e_b`、`p−e_b−e_c`、`p−e_c`），而遍历按
    /// 块序 + 槽序推进——相邻格几乎总在**同一块**里。`get` 里的
    /// `self.blocks.get(&block_of(cell))` 是 `FxHashMap` 查找（哈希 + 探测），
    /// 缓存命中后整段跳过。VTune 上 `CellMap::get` 占 3.6 s CPU / 45 s。
    #[inline]
    pub(crate) fn get_cached(&self, cell: [i32; 3], cache: &mut BlockCache) -> Option<u32> {
        let bc = block_of(cell);
        let slot = if cache.valid && cache.coord == bc {
            cache.slot as usize
        } else {
            let s = *self.blocks.get(&bc)?;
            cache.coord = bc;
            cache.slot = s;
            cache.valid = true;
            s as usize
        };
        self.get_in_block(cell, slot)
    }

    /// 已知块号时的块内查表。
    #[inline]
    fn get_in_block(&self, cell: [i32; 3], slot: usize) -> Option<u32> {
        let li = local_of(cell);
        let words = &self.words[slot * BLOCK_WORDS..][..BLOCK_WORDS];
        let w = li / 64;
        let bit = 1u64 << (li % 64);
        if words[w] & bit == 0 {
            return None;
        }
        // rank = 本块里排在它前面的置位数
        let mut rank = 0u32;
        for &x in &words[..w] {
            rank += x.count_ones();
        }
        rank += (words[w] & (bit - 1)).count_ones();
        Some(self.base[slot] + rank)
    }

    /// 单元格 → 顶点索引。查不到 = 这个格子没有顶点。
    #[inline]
    pub(crate) fn get(&self, cell: [i32; 3]) -> Option<u32> {
        let slot = *self.blocks.get(&block_of(cell))? as usize;
        let li = local_of(cell);
        let words = &self.words[slot * BLOCK_WORDS..][..BLOCK_WORDS];
        let w = li / 64;
        let bit = 1u64 << (li % 64);
        if words[w] & bit == 0 {
            return None;
        }
        // rank = 本块里排在它前面的置位数
        let mut rank = 0u32;
        for &x in &words[..w] {
            rank += x.count_ones();
        }
        rank += (words[w] & (bit - 1)).count_ones();
        Some(self.base[slot] + rank)
    }

    /// 按索引序（= 块序 + 槽序）遍历块区间 `[bs, be)` 内的所有单元格。
    ///
    /// **不返回 `Vec`**：1600 万个 `(cell, idx)` 要 256 MB，而且四边形发射本来就要
    /// 分块并行——按块区间切，索引天然由 `base + 块内 rank` 现算，不用存。
    ///
    /// 四边形必须按这个顺序发射：既保证可复现，也让"同一格被相邻叶各算一次"的
    /// 重复记录天然只剩一个（掩码里只有一位）。
    pub(crate) fn for_each_cell(&self, bs: usize, be: usize, mut f: impl FnMut([i32; 3], u32)) {
        let be = be.min(self.coords.len());
        for slot in bs..be {
            let b = self.coords[slot];
            let words = &self.words[slot * BLOCK_WORDS..][..BLOCK_WORDS];
            let mut idx = self.base[slot];
            for (w, &word) in words.iter().enumerate() {
                let mut bits = word;
                while bits != 0 {
                    let t = bits.trailing_zeros();
                    bits &= bits - 1;
                    let li = w * 64 + t as usize;
                    let cell = [
                        (b[0] << 3) + (li & 7) as i32,
                        (b[1] << 3) + ((li >> 3) & 7) as i32,
                        (b[2] << 3) + ((li >> 6) & 7) as i32,
                    ];
                    f(cell, idx);
                    idx += 1;
                }
            }
        }
    }

    /// 块数（诊断用）
    pub(crate) fn block_count(&self) -> usize {
        self.coords.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rec(cell: [i32; 3]) -> VertexRecord {
        VertexRecord {
            cell,
            pos: [cell[0] as f32, cell[1] as f32, cell[2] as f32],
            normal: [0.0, 0.0, 1.0],
            creator: 0,
        }
    }

    #[test]
    fn index_is_base_plus_popcount() {
        // 同一块里放三个格：索引必须按槽号递增，且与插入顺序无关
        let chunks = vec![
            vec![rec([0, 0, 0]), rec([5, 0, 0])],
            vec![rec([2, 0, 0])],
        ];
        let (mesh, cm) = CellMap::build(&chunks);
        assert_eq!(mesh.positions.len(), 3);
        assert_eq!(cm.get([0, 0, 0]), Some(0));
        assert_eq!(cm.get([2, 0, 0]), Some(1));
        assert_eq!(cm.get([5, 0, 0]), Some(2));
        assert_eq!(cm.get([1, 0, 0]), None, "没置位的格子必须查不到");
        // 位置写到了对应下标
        assert_eq!(mesh.positions[1], [2.0, 0.0, 0.0]);
        assert_eq!(mesh.normals[2], [0.0, 0.0, 1.0]);
    }

    #[test]
    fn duplicate_records_collapse_to_one_vertex() {
        let chunks = vec![vec![rec([3, 3, 3])], vec![rec([3, 3, 3])]];
        let (_mesh, cm) = CellMap::build(&chunks);
        assert_eq!(_mesh.positions.len(), 1, "同一格写两次仍然只有一个顶点");
        assert_eq!(cm.get([3, 3, 3]), Some(0));
    }

    #[test]
    fn negative_cells_use_floor_semantics() {
        // -1 >> 3 == -1，-1 & 7 == 7：块 -1 的最后一个槽
        let chunks = vec![vec![rec([-1, -1, -1]), rec([0, 0, 0])]];
        let (_mesh, cm) = CellMap::build(&chunks);
        assert_eq!(_mesh.positions.len(), 2);
        assert_eq!(cm.block_count(), 2, "-1 与 0 落在相邻的两个块里");
        let a = cm.get([-1, -1, -1]).unwrap();
        let b = cm.get([0, 0, 0]).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn iteration_is_in_index_order() {
        let chunks = vec![vec![rec([9, 0, 0]), rec([1, 0, 0]), rec([4, 0, 0])]];
        let (_mesh, cm) = CellMap::build(&chunks);
        let mut got = Vec::new();
        cm.for_each_cell(0, cm.block_count(), |c, i| got.push((c, i)));
        assert_eq!(
            got,
            vec![([1, 0, 0], 0), ([4, 0, 0], 1), ([9, 0, 0], 2)],
            "发出顺序必须与索引一致"
        );
    }
}
