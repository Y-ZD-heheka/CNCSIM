//! 表面提取：**Surface Nets**（对偶等值面），不是经典 Marching Cubes。
//!
//! ## 为什么不用经典 MC
//!
//! 经典 MC 需要 256 项边表 + 256×16 项三角表。**手打必错，且错了没有中间步骤可查**——
//! 一个少写的三角形只会让网格出现一个小洞，在几十万个三角形里看不出来。
//!
//! 对偶法只需要"每格一个顶点 + 每条符号变化棱一个四边形"，**水密是结构保证**：
//! 每条符号变化的棱恰好被 4 个单元格共享，恰好生成一个四边形，闭合由构造保证。
//!
//! | | 经典 MC | Surface Nets |
//! |---|---|---|
//! | 表数据 | 256 + 4096 项 | **0** |
//! | 水密性 | 需要正确查表 | **结构保证** |
//! | 顶点位置 | 在棱上（更贴合） | 在格内（12 条棱零点的平均） |
//! | 锐边 | 会倒圆 h/2 | 会倒圆约 h/2 |
//!
//! 锐边倒圆两者都有——ADR-002 已经把这个短板绕开了（过切判定用解析扫掠点测试，不用网格）。
//! 所以选表数据少、可验证的那个。
//!
//! ## 顶点位置
//!
//! 取单元格 12 条棱上**零点的平均**。平面上这是精确的（零点共面），
//! 球面上误差 `O(h²/R)`，比"取格心"（误差 `h/2`）好一个数量级。
//!
//! ## 法向
//!
//! 用 8 个角点值求梯度（每条轴 4 条棱的差分平均），指向**值增大**的方向 = 材料外。
//! 比面法向便宜，且平面上精确。
//!
//! ## 绕序
//!
//! 棱沿 `+a` 从内（负）走向外（正）时，四边形法向应指向 `+a`。
//! 从 `+a` 看下去，`(a,b,c)` 右手系里逆时针的顺序是
//! `[p(0,0), p−e_b(−1,0), p−e_b−e_c(−1,−1), p−e_c(0,−1)]`。
//! 反序（外→内）时法向指向 `−a`，需要翻转。
//!
//! 这条绕序由测试 `signed_volume_is_positive_and_matches_sphere` 锁住——
//! 绕反了体积会变负，一眼看出。

use crate::tree::{Leaf, LeafView, Scalar, SparseGrid};
use crate::MeshData;
use cncsim_math::DVec3;
use crate::cellmap::{BlockCache, CellMap, VertexRecord};
use crate::fxhash::FxHashMap;
use crate::leafset::LeafSet;

/// 单元格 8 个角点的局部偏移（索引 = `x | y<<1 | z<<2`）
pub(crate) const CORNER: [[i32; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [0, 1, 1],
    [1, 1, 1],
];

/// 由 8 个角点距离求顶点位置与法向。无符号变化时返回 `None`。
pub(crate) fn vertex_of(corners: &[f64; 8], origin_center: DVec3, h: f64) -> Option<(DVec3, DVec3)> {
    let mut neg = false;
    let mut pos = false;
    for &v in corners {
        if v < 0.0 {
            neg = true;
        } else {
            pos = true;
        }
    }
    if !(neg && pos) {
        return None;
    }

    // 12 条棱上的零点取平均。
    //
    // **完全展开**，不走 `a/b/c` 三元组循环：原来每条棱都要现构造 `u`/`w` 两个
    // 坐标再调 `corner_index`（3 次移位 + 2 次或），24 次调用合计占全进程 CPU
    // 时间 1.4%（`corner_index` 0.63 s / 45 s）。展开后角点索引是编译期常量，
    // 累加顺序与原实现逐位相同（x 轴 4 条 → y 轴 4 条 → z 轴 4 条）。
    let mut sum = [0.0f64; 3];
    let mut count = 0u32;
    // a = x：u = (0, ib, ic) = ib<<1 | ic<<2，w = u|1；零点 q = (t, ib, ic)
    for ib in 0..2usize {
        for ic in 0..2usize {
            let iu = (ib << 1) | (ic << 2);
            let (vu, vw) = (corners[iu], corners[iu | 1]);
            if (vu < 0.0) != (vw < 0.0) {
                let t = vu / (vu - vw);
                sum[0] += t;
                sum[1] += ib as f64;
                sum[2] += ic as f64;
                count += 1;
            }
        }
    }
    // a = y：u = (ib, 0, ic) = ib | ic<<2，w = u|2；零点 q = (ib, t, ic)
    for ib in 0..2usize {
        for ic in 0..2usize {
            let iu = ib | (ic << 2);
            let (vu, vw) = (corners[iu], corners[iu | 2]);
            if (vu < 0.0) != (vw < 0.0) {
                let t = vu / (vu - vw);
                sum[0] += ib as f64;
                sum[1] += t;
                sum[2] += ic as f64;
                count += 1;
            }
        }
    }
    // a = z：u = (ib, ic, 0) = ib | ic<<1，w = u|4；零点 q = (ib, ic, t)
    for ib in 0..2usize {
        for ic in 0..2usize {
            let iu = ib | (ic << 1);
            let (vu, vw) = (corners[iu], corners[iu | 4]);
            if (vu < 0.0) != (vw < 0.0) {
                let t = vu / (vu - vw);
                sum[0] += ib as f64;
                sum[1] += ic as f64;
                sum[2] += t;
                count += 1;
            }
        }
    }
    if count == 0 {
        return None;
    }
    let q = DVec3::new(sum[0], sum[1], sum[2]) / count as f64;
    let pos = origin_center + q * h;

    // 梯度：每条轴 4 条棱的差分平均，指向值增大（材料外）。
    // 同样展开成常量索引，累加顺序与三元组循环一致。
    let g = [
        ((corners[1] - corners[0])
            + (corners[3] - corners[2])
            + (corners[5] - corners[4])
            + (corners[7] - corners[6]))
            / 4.0
            / h,
        ((corners[2] - corners[0])
            + (corners[3] - corners[1])
            + (corners[6] - corners[4])
            + (corners[7] - corners[5]))
            / 4.0
            / h,
        ((corners[4] - corners[0])
            + (corners[5] - corners[1])
            + (corners[6] - corners[2])
            + (corners[7] - corners[3]))
            / 4.0
            / h,
    ];
    let n = DVec3::new(g[0], g[1], g[2]);
    let len = n.length();
    let normal = if len > 1e-12 { n / len } else { DVec3::Z };
    Some((pos, normal))
}

/// 一批四边形的产物（索引 + 丢洞计数）。并行时每块产一份，按块序拼接。
pub(crate) struct Quads {
    pub indices: Vec<u32>,
    pub skipped: usize,
}

/// 发射四边形：**细格与粗格共用同一段绕序逻辑**。
///
/// 每条符号变化的棱由它"起点所在单元格"发射一次，连接共享该棱的 4 个单元格顶点。
/// 按 `cellmap` 的索引序（块序 + 槽序）遍历——不能遍历哈希表，否则四边形随机
/// 重复发射（ADR-006 §7.1 坑二）。
///
/// `inside(p)` 只回答"最小角在 `p` 的单元格的角点是否在材料内"。细格用
/// `get() < 0`（存储值），粗格用 `sdf() < 0`（重采样值）——两者都只用到**符号**，
/// 而符号在窄带外被钳制也仍然正确。
/// 带**单元格过滤**的发射：只有 `keep(p)` 为真的格才发射四边形。
///
/// 分块重建用：让每块只发射自己范围内的格。每个四边形本来就由"最小角所在的格"
/// 发射一次，所以各块的三角形天然不重不漏——不需要事后按质心过滤。
pub(crate) fn emit_quads_in(
    cellmap: &CellMap,
    blocks: std::ops::Range<usize>,
    inside: impl Fn([i32; 3]) -> bool,
    keep: impl Fn([i32; 3]) -> bool,
) -> Quads {
    let mut out = Quads { indices: Vec::new(), skipped: 0 };
    let mut cache = BlockCache::default();
    cellmap.for_each_cell(blocks.start, blocks.end, |p, i00| {
        if !keep(p) {
            return;
        }
        let v0 = inside(p);
        for a in 0..3usize {
            let (b, c) = ((a + 1) % 3, (a + 2) % 3);
            let mut pa = p;
            pa[a] += 1;
            if v0 == inside(pa) {
                continue;
            }
            let mut pb = p;
            pb[b] -= 1;
            let mut pc = p;
            pc[c] -= 1;
            let mut pbc = p;
            pbc[b] -= 1;
            pbc[c] -= 1;

            let (Some(i10), Some(i11), Some(i01)) = (
                cellmap.get_cached(pb, &mut cache),
                cellmap.get_cached(pbc, &mut cache),
                cellmap.get_cached(pc, &mut cache),
            ) else {
                out.skipped += 1;
                continue;
            };

            // 内 → 外（沿 +a）时用逆时针，否则翻转
            let q = if v0 { [i00, i10, i11, i01] } else { [i01, i11, i10, i00] };
            out.indices.extend_from_slice(&[q[0], q[1], q[2]]);
            out.indices.extend_from_slice(&[q[0], q[2], q[3]]);
        }
    });
    out
}

// 顶点记录见 [`crate::cellmap::VertexRecord`]：位置/法向当场转 f32，省一半内存。

/// 并行阈值：叶太少时不值得 fork-join（一次 24 线程 fork ~0.5 ms）。
pub(crate) const PAR_MIN_LEAVES: usize = 128;

impl<S: Scalar> SparseGrid<S> {
    /// 提取等值面（`d = 0`），返回**世界坐标**的三角形网格。
    ///
    /// ## 并行：按叶分块，块间零共享
    ///
    /// 顶点生成只读树（`get`）与叶坐标表，块与块之间没有共享写；四边形发射只读
    /// `cell_vertex`。所以两段都能并行，**结果与线程数无关**：
    ///
    /// - 顶点按"块号顺序"合并，块内先做本地去重。串行版是"叶顺序 → 叶内 dz/dy/dx"，
    ///   分块后块是连续叶区间，合并顺序与串行完全一致，所以"同一个单元格被相邻叶
    ///   各算一次"时，留下的仍是串行会留下的那个（首次出现）。
    /// - 四边形按 `cells` 的连续区间切，块内顺序不变，拼接后索引序列与串行逐位相同。
    pub fn extract_surface(&self) -> MeshData {
        self.extract_surface_chunks(0)
    }

    /// `chunks = 0` 表示按线程数自动分块；大于 0 时强制分这么多块。
    ///
    /// 强制分块**只为对拍**：结果必须与块数无关（`parallel_matches_serial` 锁这条）。
    pub fn extract_surface_chunks(&self, chunks: usize) -> MeshData {
        // 先收集所有叶坐标（树不存"叶在哪"，只有正向下降）
        let mut leaves: Vec<[i32; 3]> = Vec::new();
        self.for_each_leaf(|c, _| leaves.push(c));
        // **必须排序**：`for_each_leaf` 走的是 HashMap，迭代顺序每次不同。
        // 不排序的话顶点顺序随机 → 仿真结果不可复现（项目硬门槛）。
        leaves.sort_unstable();
        let leaf_set = LeafSet::from_leaves(&leaves);

        // ── ① 生成顶点 ──
        //
        // 每个单元格**只由包含它最小角的那片叶处理**。但最小角可能落在未分配的
        // 叶里（例如实心块外侧全是背景值），那种单元格没有"主人"，会漏掉——
        // 表现就是网格在实体边界上有洞。所以每片叶还要处理一圈 halo（−1 层），
        // 条件是 halo 单元格的最小角**不在任何已分配的叶里**。
        let threads = crate::par::worker_count();
        let chunks = if chunks > 0 {
            chunks.clamp(1, leaves.len().max(1))
        } else if leaves.len() < PAR_MIN_LEAVES || threads == 1 {
            1
        } else {
            (threads * 4).min(leaves.len().max(1))
        };
        let per = leaves.len().div_ceil(chunks).max(1);
        let grid: &Self = self;
        let parts = crate::par::map_indexed(chunks, |i| {
            let s = i * per;
            let e = (s + per).min(leaves.len());
            if s >= e {
                return Vec::new();
            }
            grid.vertex_batch(&leaves, &leaf_set, s..e)
        });

        // ── ② 索引：块 + 位掩码 + popcount（OpenVDB 套路）──
        //
        // 顶点下标由"块序 + 块内 rank"决定，与分块数、线程调度完全无关；
        // 重复记录（相邻叶各算一次同一格）在掩码里天然合并成一位。
        let (mut mesh, cellmap) = CellMap::build(&parts);

        // ── ③ 四边形 ──
        let zero = S::default();
        let inside = |p: [i32; 3]| self.get(p) < zero;
        let quads = split_quads(&cellmap, &inside, threads);
        mesh.indices = quads.indices;
        mesh.skipped_quads = quads.skipped;

        mesh
    }

    /// 处理 `leaves[s..e]` 这一段叶，产出顶点（板内先做本地去重）。
    ///
    /// 取 `10³` 距离缓冲是整个提取的性能关键：逐格调 `get()` 时每片叶要
    /// 386 个 halo 格 × 8 个角 = 3088 次下降；先取缓冲只要 1000 次。
    /// 200mm 毛坯有 25 万片叶 → 7.7 亿次 vs 2.5 亿次下降。
    fn vertex_batch(
        &self,
        leaves: &[[i32; 3]],
        leaf_set: &LeafSet,
        range: std::ops::Range<usize>,
    ) -> Vec<VertexRecord> {
        const HALO_DIM: usize = 10; // 8 + 两侧各 1
        const HALO_AREA: usize = HALO_DIM * HALO_DIM;
        let h = self.config.h;
        // 每片叶平均产出 ~64 个顶点（200mm 毛坯 25 万叶 → 1600 万顶点）。
        // 不预分配的话 `out` 从 0 翻倍长到几十万条，每次扩容都要搬 `VertexRecord`
        // （32 字节）——VTune 上 `__rdl_realloc` + `copy_nonoverlapping` 合计 1.1 s CPU。
        let mut out: Vec<VertexRecord> = Vec::with_capacity(range.len().saturating_mul(64));
        let mut seen: FxHashMap<[i32; 3], ()> = FxHashMap::default();

        for lc in &leaves[range] {
            let base = [lc[0] << 3, lc[1] << 3, lc[2] << 3];

            let mut buf = [0.0f64; HALO_DIM * HALO_DIM * HALO_DIM];

            // ── 主体 8³：从**当前叶一次**取 ──
            //
            // 原来这 512 个体素也走 `self.get()`，每格一次三级查找
            // （root 哈希 → l2 → l1 → leaf）。VTune 实测 `SparseGrid::get` 占
            // 全进程 CPU 时间 **27%**（12 s / 45 s），是最大的实际计算热点。
            //
            // 这里一次 `leaf_view_with_slot` 拿到叶，512 个值直接读缓冲。
            // 剩下 488 个 halo 体素仍走 `get`（它们落在 26 个邻居叶里，
            // 批量取 27 片叶要复制 13.8 KB/叶，比逐点查更慢）。
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            let (view, slot) = self.leaf_view_with_slot(base);
            // 溯源：这片叶的 `creator` 起点（ADR-005）。没有叶（tile / 背景）
            // 就是原始毛坯面，索引 0。
            //
            // **按子块取**而不是整叶一个值：`creator_sub > 1` 时同一片叶里
            // 切过的子块和没切过的子块颜色不同，这正是"颜色溢出"的修法。
            let creator_leaf = slot.map(|(li, _)| li);
            // `sub == 1`：整片叶一个值，**提到循环外**——与加粒度之前逐字相同，
            // 16M 顶点的提取不会因为溯源多付一次逐顶点取址。
            // `sub > 1` 才需要按子块取（`None` 表示"走逐顶点分支"）。
            let uniform_creator = if self.creator_sub == 1 {
                Some(creator_leaf.map(|li| self.creator_at(li as u32)).unwrap_or(0))
            } else {
                None
            };
            match view {
                Some(LeafView::Leaf(Leaf::Dense(a))) => {
                    for hz in 0..8usize {
                        for hy in 0..8usize {
                            for hx in 0..8usize {
                                let d = self.to_distance(a[hx | (hy << 3) | (hz << 6)]);
                                buf[(hx + 1) + (hy + 1) * HALO_DIM + (hz + 1) * HALO_AREA] = d;
                                lo = lo.min(d);
                                hi = hi.max(d);
                            }
                        }
                    }
                }
                other => {
                    // tile / Constant / 背景：整叶同值，一次换算就够
                    // `Leaf::Constant` 给的是 `&S`，`LeafView::Tile` 给的是 `S`，
                    // 类型不同不能并成一个 or-pattern
                    let d = match other {
                        Some(LeafView::Leaf(Leaf::Constant(v))) => self.to_distance(*v),
                        Some(LeafView::Tile(v)) => self.to_distance(v),
                        _ => self.to_distance(self.background),
                    };
                    for hz in 1..9usize {
                        for hy in 1..9usize {
                            for hx in 1..9usize {
                                buf[hx + hy * HALO_DIM + hz * HALO_AREA] = d;
                            }
                        }
                    }
                    lo = d;
                    hi = d;
                }
            }

            // ── halo 一圈（488 个体素）──
            //
            // 6 个面（384 个）按**邻居叶整面复制**：一次 `leaf_view_with_slot`
            // 拿 8×8 个值，省掉 64 次三级查找。剩下 12 条棱 + 8 个角（104 个）
            // 仍逐点 `get`——按棱/角批量取叶要写 20 段映射，收益只有 104 次。
            macro_rules! blit_face {
                ($nlc:expr, $dst:expr, $src:expr) => {{
                    let nbase = [
                        ($nlc)[0] << 3,
                        ($nlc)[1] << 3,
                        ($nlc)[2] << 3,
                    ];
                    match self.leaf_view_with_slot(nbase).0 {
                        Some(LeafView::Leaf(Leaf::Dense(a))) => {
                            for j in 0..8usize {
                                for i in 0..8usize {
                                    let d = self.to_distance(a[($src)(i, j)]);
                                    buf[($dst)(i, j)] = d;
                                }
                            }
                        }
                        other => {
                            let d = match other {
                                Some(LeafView::Leaf(Leaf::Constant(v))) => self.to_distance(*v),
                                Some(LeafView::Tile(v)) => self.to_distance(v),
                                _ => self.to_distance(self.background),
                            };
                            for j in 0..8usize {
                                for i in 0..8usize {
                                    buf[($dst)(i, j)] = d;
                                }
                            }
                        }
                    }
                }};
            }
            let (lx, ly, lz) = (lc[0], lc[1], lc[2]);
            // x− 面：邻居叶的 x=7 面 → buf 的 hx=0
            blit_face!(
                [lx - 1, ly, lz],
                |i: usize, j: usize| (j + 1) * HALO_DIM + (i + 1) * HALO_AREA,
                |i: usize, j: usize| 7 | (j << 3) | (i << 6)
            );
            // x+ 面：邻居叶的 x=0 面 → buf 的 hx=9
            blit_face!(
                [lx + 1, ly, lz],
                |i: usize, j: usize| 9 + (j + 1) * HALO_DIM + (i + 1) * HALO_AREA,
                |i: usize, j: usize| (j << 3) | (i << 6)
            );
            // y− 面：邻居叶的 y=7 面 → buf 的 hy=0
            blit_face!(
                [lx, ly - 1, lz],
                |i: usize, j: usize| (i + 1) + (j + 1) * HALO_AREA,
                |i: usize, j: usize| i | (7 << 3) | (j << 6)
            );
            // y+ 面：邻居叶的 y=0 面 → buf 的 hy=9
            blit_face!(
                [lx, ly + 1, lz],
                |i: usize, j: usize| (i + 1) + 9 * HALO_DIM + (j + 1) * HALO_AREA,
                |i: usize, j: usize| i | (j << 6)
            );
            // z− 面：邻居叶的 z=7 面 → buf 的 hz=0
            blit_face!(
                [lx, ly, lz - 1],
                |i: usize, j: usize| (i + 1) + (j + 1) * HALO_DIM,
                |i: usize, j: usize| i | (j << 3) | (7 << 6)
            );
            // z+ 面：邻居叶的 z=0 面 → buf 的 hz=9
            blit_face!(
                [lx, ly, lz + 1],
                |i: usize, j: usize| (i + 1) + (j + 1) * HALO_DIM + 9 * HALO_AREA,
                |i: usize, j: usize| i | (j << 3)
            );

            // 棱（96 个）与角（8 个）：逐点 `get`。它们落在 20 个邻居叶里，
            // 按棱/角批量取叶要写 20 段映射，而总共只有 104 次查找。
            for hz in 0..HALO_DIM {
                for hy in 0..HALO_DIM {
                    for hx in 0..HALO_DIM {
                        let inside = (1..9).contains(&hx)
                            && (1..9).contains(&hy)
                            && (1..9).contains(&hz);
                        // 只在"恰好一个坐标在边界"（棱）或"两三个都在边界"（角）时走这里
                        let on_face = (hx == 0 || hx == 9) as u8
                            + (hy == 0 || hy == 9) as u8
                            + (hz == 0 || hz == 9) as u8;
                        if inside || on_face == 1 {
                            continue; // 主体已填；单面已由 blit_face 填
                        }
                        let v = self.get([
                            base[0] + hx as i32 - 1,
                            base[1] + hy as i32 - 1,
                            base[2] + hz as i32 - 1,
                        ]);
                        buf[hx + hy * HALO_DIM + hz * HALO_AREA] = self.to_distance(v);
                    }
                }
            }
            // 叶内（8³ 子块）是否有符号变化——`lo/hi` 在填主体时顺手累加，
            // 不再单独扫一遍 512 个值。
            let internal_change = lo < 0.0 && hi >= 0.0;

            for dz in -1..8i32 {
                for dy in -1..8i32 {
                    for dx in -1..8i32 {
                        let p = [base[0] + dx, base[1] + dy, base[2] + dz];
                        // 归属规则：最小角落在别的**已分配**叶里 → 交给那片叶
                        let owner = crate::leaf_coord(p);
                        if owner != *lc && leaf_set.contains(owner) {
                            continue;
                        }
                        // 8 个角全在叶内？
                        let fully_inside =
                            (0..7).contains(&dx) && (0..7).contains(&dy) && (0..7).contains(&dz);
                        if fully_inside && !internal_change {
                            // 叶内没有符号变化 → 这个单元格也不会有
                            continue;
                        }

                        let mut corners = [0.0f64; 8];
                        for (i, o) in CORNER.iter().enumerate() {
                            let hx = (dx + o[0] + 1) as usize;
                            let hy = (dy + o[1] + 1) as usize;
                            let hz = (dz + o[2] + 1) as usize;
                            corners[i] = buf[hx + hy * HALO_DIM + hz * HALO_AREA];
                        }

                        if let Some((pos, normal)) = vertex_of(&corners, self.voxel_center(p), h) {
                            // 顶点取**自己那一格最小角**所在子块的颜色。
                            // 叶外（halo 格、主人叶不存在）一律 0。
                            let creator = match uniform_creator {
                                Some(c) => c,
                                None => match creator_leaf {
                                    Some(li) if crate::leaf_coord(p) == *lc => {
                                        self.creator_at_voxel(li, p)
                                    }
                                    _ => 0,
                                },
                            };
                            // 本地去重：同一块内相邻叶会各算一次同一格。
                            // 必须用哈希，不能线性扫（200mm 毛坯一块就有几十万顶点）。
                            //
                            // **只有 halo 格会重复**：`fully_inside` 的格 8 个角都在
                            // 当前叶内，而归属规则保证它只由当前叶处理。跳过哈希
                            // 省掉 512 次/叶的插入（VTune 上 hashbrown 占 0.2 s CPU）。
                            let fresh = fully_inside || seen.insert(p, ()).is_none();
                            if fresh {
                                out.push(VertexRecord {
                                    cell: p,
                                    pos: [pos.x as f32, pos.y as f32, pos.z as f32],
                                    normal: [
                                        normal.x as f32,
                                        normal.y as f32,
                                        normal.z as f32,
                                    ],
                                    creator,
                                });
                            }
                        }
                    }
                }
            }
        }
        out
    }
}

/// 按**块区间**并行发射四边形，按块序拼接。
///
/// 切块而不是切"单元格列表"：块是索引的天然边界（每块的 `base` 固定），
/// 所以各块算出的索引互不重叠，拼接顺序也只影响三角形顺序、不影响数值。
pub(crate) fn split_quads(
    cellmap: &CellMap,
    inside: &(impl Fn([i32; 3]) -> bool + Sync),
    threads: usize,
) -> Quads {
    split_quads_in(cellmap, inside, threads, |_| true)
}

/// 带单元格过滤的并行发射。
pub(crate) fn split_quads_in(
    cellmap: &CellMap,
    inside: &(impl Fn([i32; 3]) -> bool + Sync),
    threads: usize,
    keep: impl Fn([i32; 3]) -> bool + Sync,
) -> Quads {
    let n = cellmap.block_count();
    if n == 0 {
        return Quads { indices: Vec::new(), skipped: 0 };
    }
    if n < 4 || threads == 1 {
        return emit_quads_in(cellmap, 0..n, inside, keep);
    }
    let chunks = (threads * 4).min(n);
    let per = n.div_ceil(chunks).max(1);
    let parts = crate::par::map_indexed(chunks, |i| {
        let s = i * per;
        if s >= n {
            return Quads { indices: Vec::new(), skipped: 0 };
        }
        emit_quads_in(cellmap, s..(s + per).min(n), inside, &keep)
    });
    let mut out = Quads {
        indices: Vec::with_capacity(parts.iter().map(|p| p.indices.len()).sum()),
        skipped: 0,
    };
    for p in parts {
        out.indices.extend_from_slice(&p.indices);
        out.skipped += p.skipped;
    }
    out
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::GridConfig;

    fn grid() -> SparseGrid<i8> {
        SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO)
    }

    #[test]
    fn solid_block_yields_outer_surface_only() {
        let mut g = grid();
        g.fill_box([0, 0, 0], [32, 32, 32], -127);
        // 实心块在背景场里**有**表面——它的外表面（材料 vs 背景）
        let m = g.extract_surface();
        assert!(!m.positions.is_empty(), "实心块应有外表面");
        assert_eq!(m.skipped_quads, 0, "闭合块不该有洞");
        // 所有顶点都应落在外表面附近，内部不该有顶点
        for p in &m.positions {
            let edge = 3.2;
            // 对偶法的顶点在格内，靠外的面大约在 edge − 0.1 处
            let on_face = p.iter().any(|&c| c < 0.06 || c > edge - 0.1);
            assert!(on_face, "内部不该有顶点：{p:?}");
        }
    }

    #[test]
    fn empty_grid_has_no_surface() {
        let g = grid();
        let m = g.extract_surface();
        assert!(m.positions.is_empty());
    }

    #[test]
    fn plane_vertices_lie_on_the_plane() {
        // 16³ 方块，x<4 实心、x>=4 空。
        // 注意：这是个**方块不是半空间**，六个面都会产生表面，
        // 所以必须按法向筛出 x=3|4 之间那一个面。
        let mut g = grid();
        for x in 0..16 {
            let v = if x < 4 { -127i8 } else { 127 };
            g.fill_box([x, 0, 0], [x + 1, 16, 16], v);
        }
        let m = g.extract_surface();
        assert!(!m.positions.is_empty(), "应提取出平面");

        let mut checked = 0;
        for (p, n) in m.positions.iter().zip(&m.normals) {
            if n[0] > 0.9 {
                // 体素中心 x=(i+0.5)*0.1；表面在 x=3 与 x=4 之间 → 0.40
                assert!(
                    (p[0] - 0.40).abs() < 1e-4,
                    "+x 面的顶点应在 x=0.40，实际 {}",
                    p[0]
                );
                checked += 1;
            }
        }
        assert!(checked > 0, "应存在 +x 朝向的顶点");
        assert_eq!(m.skipped_quads, 0, "平面不该有洞");
    }

    #[test]
    fn plane_normals_point_away_from_material() {
        let mut g = grid();
        for x in 0..16 {
            let v = if x < 4 { -127i8 } else { 127 };
            g.fill_box([x, 0, 0], [x + 1, 16, 16], v);
        }
        let m = g.extract_surface();
        // x=3|4 之间：材料在 −x 侧，法向必须是 +x
        let mut checked = 0;
        for (p, n) in m.positions.iter().zip(&m.normals) {
            if (p[0] - 0.40).abs() < 1e-3 {
                assert!(
                    n[0] > 0.9,
                    "x=0.40 处的法向应指向 +x，实际 {n:?}（顶点 {p:?}）"
                );
                checked += 1;
            }
        }
        assert!(checked > 0);
    }
}

#[cfg(test)]
mod dist_scale_tests {
    //! **反向测试**：`to_distance` 的除法**不能**换成预乘倒数。
    //!
    //! `v / MAX_MAG * band`（除法 + 乘法）看起来可以换成 `v * (band / MAX_MAG)`
    //! （一次乘法，快 2–3 倍）。实测**不等价**——127 不是 2 的幂，除法有舍入：
    //!
    //! ```text
    //! band=0.4 v=-122：除法 -3.84251968503937002541e-1
    //!                  乘法 -3.84251968503937058053e-1   （差 0.65 ulp）
    //! ```
    //!
    //! 这会改变顶点坐标 → 破坏逐位可复现。和 `cut.rs::int_threshold_tests`
    //! 是同一类陷阱：**数学上等价 ≠ 浮点下等价**。
    //!
    //! 真要提速只能查表（用原公式建 256 项表），但那省的是 3 ns/次，
    //! 而 `to_distance` 的调用次数（1000 次/叶）才是大头——见 `vertex_batch`。

    #[test]
    fn division_is_not_replaceable_by_premultiplied_reciprocal() {
        let mut diverged = 0;
        for band in [0.4f64, 0.8, 0.2, 1.0, 0.6] {
            for v in -127i32..=127 {
                let by_div = v as f64 / 127.0 * band;
                let by_mul = v as f64 * (band / 127.0);
                if by_div != by_mul {
                    diverged += 1;
                }
            }
        }
        assert!(
            diverged > 0,
            "两者竟然完全相等了——要么浮点路径变了，要么这个优化其实安全，需重新评估"
        );
    }
}
