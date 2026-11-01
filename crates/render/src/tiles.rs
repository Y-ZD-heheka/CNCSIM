//! 显示块的增量缓存：**只重建被切到的那几块**。
//!
//! 交互式仿真的关键不是"一帧画多少三角形"，而是"一刀下去要重算多少"。整体重新
//! 提取一次 200 mm 毛坯要 0.32 s（LOD k=4），60 fps 根本谈不上；而面铣一刀只扫过
//! 一条几毫米宽的带子，受影响的块是个位数。
//!
//! 两件事必须成立，缺一不可：
//!
//! 1. **分块提取不能有缝**（[`cncsim_voxel::SparseGrid::extract_surface_lod_in`]）：
//!    每个四边形由最小角所在的格发射一次，所以按格划分天然不重不漏，交接处的
//!    顶点靠多取一圈保证。
//! 2. **没变的块不能重传**：提取出的网格逐字节和旧的比，一样就保留旧版本号。

use crate::{TileUpload, VERTEX_BYTES, split_tiles};
use cncsim_math::DVec3;
use cncsim_voxel::{MeshData, SparseGrid};
use std::collections::BTreeMap;

pub struct TileCache {
    /// 每个显示块的边长（mm）
    tile_mm: f64,
    /// LOD 因子（粗格边长 = `k × h`）
    k: u32,
    tiles: BTreeMap<u64, TileEntry>,
    version: u64,
    /// 上一帧因为预算没传完的块号（FIFO）。
    ///
    /// **必须记着**：不记的话"下一帧再传"是假的——下一帧重新提取发现内容没变，
    /// 那些块就永远不会上传，屏幕上是缺的。
    pending: std::collections::VecDeque<u64>,
}

struct TileEntry {
    mesh: MeshData,
    version: u64,
}

/// 一次 [`TileCache::update`] 的结果。
pub struct UpdateResult {
    /// 这一帧该上传的块（按块号升序，受字节预算限制）
    pub uploads: Vec<TileUpload>,
    /// 重建出**内容有变**的块数（没变的不算）
    pub rebuilt: usize,
    /// 因为预算被推到下一帧的块数
    pub pending: usize,
    /// 被整块切光、从缓存里删掉的块数
    pub removed: usize,
}

impl TileCache {
    pub fn new(tile_mm: f64, k: u32) -> Self {
        Self {
            tile_mm: tile_mm.max(1e-3),
            k: k.max(1),
            tiles: BTreeMap::new(),
            version: 0,
            pending: std::collections::VecDeque::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.tiles.len()
    }
    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty()
    }
    pub fn version_of(&self, tile_id: u64) -> Option<u64> {
        self.tiles.get(&tile_id).map(|t| t.version)
    }
    pub fn tile_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.tiles.keys().copied()
    }

    /// 把所有块拼回一个网格（诊断 / 导出用）。块内索引会被整体偏移。
    pub fn merged(&self) -> MeshData {
        let mut out = MeshData::default();
        for t in self.tiles.values() {
            let base = out.positions.len() as u32;
            out.positions.extend_from_slice(&t.mesh.positions);
            out.normals.extend_from_slice(&t.mesh.normals);
            out.colors.extend_from_slice(&t.mesh.colors);
            out.indices.extend(t.mesh.indices.iter().map(|i| i + base));
        }
        out
    }

    /// 重建与 `dirty`（世界坐标包围盒）相交的块。`None` = 全量。
    ///
    /// ## 为什么要"多扩一圈再扔一圈"
    ///
    /// 一次提取只覆盖一个矩形区域，而**三角形按质心归属块**：区域边上的格会发射
    /// 质心落在区域外的三角形，区域外的块也会发射质心落在区域内的。于是：
    ///
    /// - 只替换"完全在区域内部"的块才安全（否则会拿到残缺的网格）；
    /// - 而被切削影响的三角形可能正好落在区域边界的块里。
    ///
    /// 所以提取范围比"要替换的块"再**向外多扩一块**：替换范围的每个块都完整地
    /// 落在提取范围里，边界上那些半残的块直接扔掉（它们离切削足够远，本来就没变）。
    pub fn update(
        &mut self,
        grid: &SparseGrid<i8>,
        dirty: Option<(DVec3, DVec3)>,
        budget_bytes: usize,
    ) -> UpdateResult {
        let h = grid.config.h;
        let origin = grid.origin;
        let cell = self.k as f64 * h;
        let t = self.tile_mm;

        // 脏区域 → 块索引范围（闭区间）
        let (wlo, whi) = match dirty {
            Some((a, b)) => (a, b),
            None => match grid.bounds() {
                Some((vlo, vhi)) => (
                    DVec3::new(
                        origin.x + vlo[0] as f64 * h,
                        origin.y + vlo[1] as f64 * h,
                        origin.z + vlo[2] as f64 * h,
                    ),
                    DVec3::new(
                        origin.x + (vhi[0] + 1) as f64 * h,
                        origin.y + (vhi[1] + 1) as f64 * h,
                        origin.z + (vhi[2] + 1) as f64 * h,
                    ),
                ),
                None => {
                    return UpdateResult {
                        uploads: Vec::new(),
                        rebuilt: 0,
                        pending: 0,
                        removed: 0,
                    };
                }
            },
        };
        let idx = |v: DVec3, lo: bool| -> [i32; 3] {
            let f = |x: f64| {
                let q = x / t;
                if lo { q.floor() as i32 } else { q.ceil() as i32 }
            };
            [f(v.x), f(v.y), f(v.z)]
        };
        let core_lo = idx(wlo, true);
        let core_hi = idx(whi, false);

        // 提取范围：替换范围再外扩一块
        let ex_lo = [core_lo[0] - 2, core_lo[1] - 2, core_lo[2] - 2];
        let ex_hi = [core_hi[0] + 2, core_hi[1] + 2, core_hi[2] + 2];
        let to_cell = |v: f64, ceil: bool| {
            let q = v / cell;
            if ceil { q.ceil() as i32 } else { q.floor() as i32 }
        };
        let lo_c = [
            to_cell(ex_lo[0] as f64 * t, false),
            to_cell(ex_lo[1] as f64 * t, false),
            to_cell(ex_lo[2] as f64 * t, false),
        ];
        let hi_c = [
            to_cell(ex_hi[0] as f64 * t, true),
            to_cell(ex_hi[1] as f64 * t, true),
            to_cell(ex_hi[2] as f64 * t, true),
        ];

        let mesh = grid.extract_surface_lod_in(self.k, lo_c, hi_c);
        let mut fresh: std::collections::HashMap<u64, MeshData> = split_tiles(&mesh, t as f32)
            .into_iter()
            .map(|t| (t.tile_id, t.mesh))
            .collect();

        // 只替换"离边界还有一块"的那些块：core ± 1
        let mut changed: Vec<u64> = Vec::new();
        let mut seen: Vec<u64> = Vec::new();
        let mut removed = 0usize;
        for i in (core_lo[0] - 1)..=(core_hi[0] + 1) {
            for j in (core_lo[1] - 1)..=(core_hi[1] + 1) {
                for k in (core_lo[2] - 1)..=(core_hi[2] + 1) {
                    let id = crate::tile_id_of(i, j, k);
                    seen.push(id);
                    match fresh.remove(&id) {
                        Some(m) => {
                            let same = self
                                .tiles
                                .get(&id)
                                .is_some_and(|old| same_mesh(&old.mesh, &m));
                            if same {
                                continue;
                            }
                            self.version += 1;
                            let v = self.version;
                            self.tiles
                                .insert(id, TileEntry { mesh: m, version: v });
                            changed.push(id);
                        }
                        None => {
                            // 这块切没了
                            if self.tiles.remove(&id).is_some() {
                                removed += 1;
                            }
                        }
                    }
                }
            }
        }

        let rebuilt = changed.len();
        for id in &changed {
            if !self.pending.contains(id) {
                self.pending.push_back(*id);
            }
        }
        self.pending.retain(|id| self.tiles.contains_key(id));

        let mut uploads = Vec::new();
        let mut bytes = 0usize;
        while let Some(&id) = self.pending.front() {
            let Some(e) = self.tiles.get(&id) else {
                self.pending.pop_front();
                continue;
            };
            let b = e.mesh.positions.len() * (VERTEX_BYTES + 4) + e.mesh.indices.len() * 4;
            if !uploads.is_empty() && bytes + b > budget_bytes {
                break;
            }
            bytes += b;
            uploads.push(TileUpload {
                tile_id: id,
                mesh: e.mesh.clone(),
                version: e.version,
            });
            self.pending.pop_front();
        }
        UpdateResult {
            uploads,
            rebuilt,
            pending: self.pending.len(),
            removed,
        }
    }
}

/// 逐字节比较两个网格（先比长度，再比内容）。用来决定"这块要不要重传"。
fn same_mesh(a: &MeshData, b: &MeshData) -> bool {
    a.positions.len() == b.positions.len()
        && a.indices.len() == b.indices.len()
        && a.positions.iter().zip(&b.positions).all(|(x, y)| x == y)
        && a.indices == b.indices
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_math::DVec3;
    use cncsim_motion::Pose;
    use cncsim_tool::flat_end_mill;
    use cncsim_voxel::GridConfig;

    fn mesh_bounds(m: &MeshData) -> (DVec3, DVec3) {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for p in &m.positions {
        lo = DVec3::new(
            lo.x.min(p[0] as f64),
            lo.y.min(p[1] as f64),
            lo.z.min(p[2] as f64),
        );
        hi = DVec3::new(
            hi.x.max(p[0] as f64),
            hi.y.max(p[1] as f64),
            hi.z.max(p[2] as f64),
        );
    }
    (lo, hi)
}

    /// 12×12×8 mm 毛坯（h=0.1）
    fn block() -> SparseGrid<i8> {
        let mut g = SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO);
        g.fill_box([0, 0, 0], [120, 120, 80], -127);
        g.prune();
        g
    }

    fn cut(g: &mut SparseGrid<i8>, x0: f64, x1: f64, y: f64, z: f64) {
        g.cut_segment(
            &Pose {
                tip: DVec3::new(x0, y, z),
                axis: DVec3::Z,
            },
            &Pose {
                tip: DVec3::new(x1, y, z),
                axis: DVec3::Z,
            },
            &flat_end_mill(6.0, 20.0),
        );
    }

    #[test]
    fn full_update_matches_a_single_extraction() {
        let g = block();
        let mut cache = TileCache::new(4.0, 2);
        cache.update(&g, None, usize::MAX);
        let merged = cache.merged();
        let whole = g.extract_surface_lod(2);
        assert_eq!(
            merged.indices.len(),
            whole.indices.len(),
            "分块拼起来应与整体提取的三角形数一致"
        );
        assert!(!cache.is_empty());
    }

    #[test]
    fn incremental_update_matches_a_full_rebuild() {
        let mut g = block();
        let mut cache = TileCache::new(4.0, 2);
        cache.update(&g, None, usize::MAX);

        // 再切一刀，只报这一刀的包围盒
        // 切 2 mm 深（0.1 mm 的刀在 h_c=0.2 的粗格上改不动表面——这是真实约束，不是测试误差）
        cut(&mut g, 2.0, 10.0, 6.0, 6.0);
        let dirty = (
            DVec3::new(0.0, 3.0, 5.0),
            DVec3::new(11.0, 9.0, 8.1),
        );
        let r = cache.update(&g, Some(dirty), usize::MAX);
        assert!(r.rebuilt > 0, "切了一刀，至少有一块要重建");
        assert_eq!(r.pending, 0, "预算无限，不该有排队");

        // 增量更新的结果必须和"从零全量重建"一致
        let mut fresh = TileCache::new(4.0, 2);
        fresh.update(&g, None, usize::MAX);
        let a = cache.merged();
        let b = fresh.merged();
        assert_eq!(
            a.indices.len(),
            b.indices.len(),
            "增量 {} 三角形 ≠ 全量 {}：脏区域算小了",
            a.indices.len(),
            b.indices.len()
        );
        let mut ka: Vec<[u32; 3]> = a
            .positions
            .iter()
            .map(|p| [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()])
            .collect();
        let mut kb: Vec<[u32; 3]> = b
            .positions
            .iter()
            .map(|p| [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()])
            .collect();
        ka.sort_unstable();
        kb.sort_unstable();
        assert_eq!(ka, kb, "顶点集合也应一致");
    }

    #[test]
    fn tiles_far_from_the_cut_keep_their_version() {
        let mut g = block();
        let mut cache = TileCache::new(4.0, 2);
        cache.update(&g, None, usize::MAX);
        let before: Vec<(u64, u64)> = cache
            .tile_ids()
            .map(|id| (id, cache.version_of(id).unwrap()))
            .collect();

        cut(&mut g, 2.0, 10.0, 6.0, 6.0);
        cache.update(
            &g,
            Some((DVec3::new(0.0, 3.0, 5.0), DVec3::new(11.0, 9.0, 8.1))),
            usize::MAX,
        );

        let mut unchanged = 0;
        for (id, v) in before {
            if cache.version_of(id) == Some(v) {
                unchanged += 1;
            }
        }
        assert!(
            unchanged > 0,
            "远离切削的块不该被重传（全 {} 块都变了）",
            cache.len()
        );
    }

    #[test]
    fn budget_defers_uploads_but_keeps_them_queued() {
        let g = block();
        let mut cache = TileCache::new(2.0, 1);
        // 预算给 1 字节：每次只放一块过去，其余排队
        let r = cache.update(&g, None, 1);
        assert_eq!(r.uploads.len(), 1, "预算再小也得传一块，否则永远卡住");
        assert!(r.pending > 0, "剩下的应该排队");
        let first_pending = r.pending;
        // 下一帧把剩下的都传完
        let r2 = cache.update(&g, None, usize::MAX);
        assert!(
            r2.uploads.len() >= first_pending,
            "重算后应该能一次传完"
        );
    }

    /// 性能基准（默认跳过）：200 mm 毛坯上，全量重建 vs 一次面铣的增量重建。
    ///
    /// ```bash
    /// cargo test --release -p cncsim-render --lib -- --ignored --nocapture perf
    /// ```
    #[test]
    #[ignore = "性能基准，几秒，默认不跑"]
    fn perf_incremental_vs_full() {
        let mut g = SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO);
        g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
        g.prune();

        let mut cache = TileCache::new(6.4, 4);
        let t0 = std::time::Instant::now();
        let r = cache.update(&g, None, usize::MAX);
        let full_ms = t0.elapsed().as_secs_f64() * 1e3;
        println!(
            "全量：{full_ms:.0} ms，{} 块，{:.0} 万三角形",
            cache.len(),
            cache.merged().indices.len() as f64 / 3.0 / 1e4
        );
        let _ = r;

        // 面铣一刀：200 mm 长、6 mm 宽、1 mm 深
        g.cut_segment(
            &Pose { tip: DVec3::new(0.0, 100.0, 99.0), axis: DVec3::Z },
            &Pose { tip: DVec3::new(200.0, 100.0, 99.0), axis: DVec3::Z },
            &flat_end_mill(6.0, 20.0),
        );
        let t0 = std::time::Instant::now();
        let r = cache.update(
            &g,
            Some((DVec3::new(-1.0, 96.0, 98.5), DVec3::new(201.0, 104.0, 100.5))),
            usize::MAX,
        );
        let inc_ms = t0.elapsed().as_secs_f64() * 1e3;
        println!(
            "增量：{inc_ms:.1} ms（重建 {} 块 / 上传 {} 块 / 删 {} 块），{:.0}× 快于全量",
            r.rebuilt,
            r.uploads.len(),
            r.removed,
            full_ms / inc_ms.max(0.001)
        );
        assert!(r.rebuilt > 0);
    }

    #[test]
    fn fully_cut_away_tiles_are_dropped() {
        let mut g = block();
        let mut cache = TileCache::new(4.0, 2);
        cache.update(&g, None, usize::MAX);
        let n0 = cache.len();
        // 把毛坯从 z=4 以上整个铣掉（两道，覆盖 12 mm 宽）：
        // z∈[4,8] 那批块会变成空的，必须从缓存里删掉
        cut(&mut g, -1.0, 13.0, 3.0, 4.0);
        cut(&mut g, -1.0, 13.0, 9.0, 4.0);
        let r = cache.update(
            &g,
            Some((DVec3::new(-1.0, -1.0, -1.0), DVec3::new(13.0, 13.0, 9.0))),
            usize::MAX,
        );
        assert!(r.removed > 0, "顶面被削平后，上面的块应该是空的");
        // 块数可能变多：切出来的新表面落在原来完全内部的块里，那些块本来是空的
        assert!(cache.len() < n0, "整体应该变少：{} → {}", n0, cache.len());
        // 删掉之后不能还有几何悬在半空
        for id in cache.tile_ids() {
            let (_, hi) = mesh_bounds(&cache.tiles[&id].mesh);
            assert!(hi.z <= 4.05, "z={} 之上不该还有块", hi.z);
        }
    }
}

