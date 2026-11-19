//! 网格 → 采样 SDF 场。
//!
//! 两个正交的 pass，合起来给出与毛坯场同语义的设计场（负=材料，钳制 ±band）：
//!
//! 1. **符号**：逐行视差。每行 (y,z) 固定，对每条三角形做 2D 投影包含测试，
//!    交点 x 排序后行走整行——体素中心落在奇数个交点之后即在体内。
//!    对**水密**网格这是精确的，凹形、多连通都行（CAM/切片器导出的 STL 天然水密）。
//!    每行 (y,z) 加微小确定性抖动（±0.0005h），避免射线恰好打在边/顶点上。
//! 2. **距离**：逐三角形光栅化。到三角形距离 < band 的点必落在其 AABB 外扩 band
//!    之内——所以对每条三角形只遍历"AABB+band ∩ 域"的体素，算精确的
//!    点-三角形最近距离，逐格取 min。没被覆盖到的体素距离 ≥ band，
//!    保持 ±band（与毛坯场的窄带钳制同语义，见 `cut` 模块的教训）。
//!
//! 两 pass 都与三角形遍历顺序无关（min 可交换、行序固定），逐位可复现。

use crate::fxhash::FxHashMap;
use crate::stl::Tri;
use crate::tree::SparseGrid;
use crate::GridConfig;
use cncsim_math::DVec3;

/// 点到三角形的最近点（clamped barycentric，Ericson《Real-Time Collision Detection》§5.1.5）。
///
/// 按顶点/边/内部的区域划分，无迭代、无退化分支陷阱。
pub fn closest_point_on_triangle(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> DVec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a; // 顶点 A 区
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b; // 顶点 B 区
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab * v; // AB 边区
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c; // 顶点 C 区
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac * w; // AC 边区
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w; // BC 边区
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab * v + ac * w // 内部区
}

/// 把网格体素化为设计场。`domain` 是体素数；网格域外/网格外都是空气（+band）。
///
/// **要求网格水密**——非水密网格的视差符号不可靠（距离仍精确，但内外可能互换）。
pub fn mesh_to_field(tris: &[Tri], config: GridConfig, origin: DVec3, domain: [i32; 3]) -> SparseGrid<i8> {
    let mut grid = SparseGrid::<i8>::new(config, origin);
    parity_pass(tris, &mut grid, domain);
    distance_pass(tris, &mut grid, domain);
    grid.prune();
    grid
}

/// 每行 (j,k) 的射线抖动量（单位：体素，±0.0005）。
///
/// 轴对齐盒的三角形边经常正好落在体素中心的坐标上——不抖的话射线
/// 恰好擦边的概率不是"几乎为零"而是"必然"。
fn jitter(j: i32, k: i32) -> (f64, f64) {
    let h1 = (j as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (k as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    let h2 = h1 ^ (h1 >> 33);
    let a = ((h1 >> 11) & 0xFFFFF) as f64 / (0xFFFFF as f64) - 0.5;
    let b = ((h2 >> 11) & 0xFFFFF) as f64 / (0xFFFFF as f64) - 0.5;
    (a * 1e-3, b * 1e-3)
}

/// +X 射线在 (y,z) 处与三角形的交点 x。2D 投影包含测试 + 平面插值。
fn ray_x_hit(t: &Tri, y: f64, z: f64) -> Option<f64> {
    let [a, b, c] = *t;
    let d = (b.y - a.y) * (c.z - a.z) - (c.y - a.y) * (b.z - a.z);
    if d.abs() < 1e-15 {
        return None; // 投影退化：射线与平面平行（掠射交给抖动规避）
    }
    let w0 = ((b.y - y) * (c.z - z) - (c.y - y) * (b.z - z)) / d;
    let w1 = ((c.y - y) * (a.z - z) - (a.y - y) * (c.z - z)) / d;
    let w2 = 1.0 - w0 - w1;
    // −1e-12 的余量：抖动后仍落在边上时算命中而不是漏
    if w0 < -1e-12 || w1 < -1e-12 || w2 < -1e-12 {
        return None;
    }
    Some(w0 * a.x + w1 * b.x + w2 * c.x)
}

/// 符号 pass：视差行走每一行，体内体素写 −band。
fn parity_pass(tris: &[Tri], grid: &mut SparseGrid<i8>, domain: [i32; 3]) {
    if domain[0] <= 0 || domain[1] <= 0 || domain[2] <= 0 {
        return;
    }
    let h = grid.config.h;
    let o = grid.origin;

    // (y,z) 平面的 2D 桶：8×8 体素一桶，键 = (j>>3, k>>3)。
    // 三角形按其 (y,z) 投影 AABB 插桶（±1 体素余量盖住抖动）。
    let mut buckets: FxHashMap<[i32; 2], Vec<u32>> = FxHashMap::default();
    for (ti, t) in tris.iter().enumerate() {
        let (ylo, yhi) = (t[0].y.min(t[1].y).min(t[2].y), t[0].y.max(t[1].y).max(t[2].y));
        let (zlo, zhi) = (t[0].z.min(t[1].z).min(t[2].z), t[0].z.max(t[1].z).max(t[2].z));
        let j0 = (((ylo - o.y) / h - 0.5).floor() as i32 - 1).max(0);
        let j1 = (((yhi - o.y) / h - 0.5).ceil() as i32 + 1).min(domain[1] - 1);
        let k0 = (((zlo - o.z) / h - 0.5).floor() as i32 - 1).max(0);
        let k1 = (((zhi - o.z) / h - 0.5).ceil() as i32 + 1).min(domain[2] - 1);
        if j0 > j1 || k0 > k1 {
            continue; // 投影在域外
        }
        // **每桶一次**，不是每格一次——重复插入会让同一平面产生几十个
        // 相同交点，奇偶视差被两两抵消，整场判定为外（实测踩过）。
        for bk in (k0 >> 3)..=(k1 >> 3) {
            for bj in (j0 >> 3)..=(j1 >> 3) {
                buckets.entry([bj, bk]).or_default().push(ti as u32);
            }
        }
    }

    let mut xs: Vec<f64> = Vec::new();
    for k in 0..domain[2] {
        for j in 0..domain[1] {
            // 行的 (y,z)：分量顺序是 (i=x, j=y, k=z)——取 [0, j, k]
            let c = grid.voxel_center([0, j, k]);
            let (dj, dk) = jitter(j, k);
            let (y, z) = (c.y + dj * h, c.z + dk * h);
            let Some(cand) = buckets.get(&[j >> 3, k >> 3]) else {
                continue; // 这一带没有任何三角形投影
            };
            xs.clear();
            for &ti in cand {
                if let Some(x) = ray_x_hit(&tris[ti as usize], y, z) {
                    xs.push(x);
                }
            }
            if xs.len() < 2 {
                continue; // 奇数个交点本身说明网格不水密；0/1 交点整行在体外
            }
            xs.sort_by(f64::total_cmp);

            // 行走：交点严格在体素中心之前 → 奇偶翻转
            let mut idx = 0usize;
            let mut crossings = 0usize;
            for i in 0..domain[0] {
                let x = grid.voxel_center([i, j, k]).x;
                while idx < xs.len() && xs[idx] < x {
                    crossings += 1;
                    idx += 1;
                }
                if crossings % 2 == 1 {
                    grid.set([i, j, k], i8::MIN);
                }
            }
        }
    }
}

/// 三角 AABB 以及可触及的体素中心区间。下标按输入三角顺序进入各桶。
struct RasterTriangle<'a> {
    points: &'a Tri,
    lo: DVec3,
    hi: DVec3,
    first: [i32; 3],
    last: [i32; 3],
}

type DistanceLeaf = ([i32; 3], Box<[i8; crate::tree::LEAF_VOXELS]>);

/// 距离 pass：按 (y,z) 叶条带分工。条带相互独立，条带内按输入顺序精确求最近距离。
/// 完成一批即写回稀疏树，避免分配整个域大小的稠密网格。
fn distance_pass(tris: &[Tri], grid: &mut SparseGrid<i8>, domain: [i32; 3]) {
    if domain.iter().any(|&n| n <= 0) {
        return;
    }
    // 分批建桶并提交。每批最多 64K 条三角记录（64 位平台约 5 MiB），
    // 局部候选编号用 u16；百万面输入的临时索引无需随总面数增长。
    for group in tris.chunks(1 << 16) {
        raster_triangle_batch(group, grid, domain);
    }
}

fn raster_triangle_batch(tris: &[Tri], grid: &mut SparseGrid<i8>, domain: [i32; 3]) {
    let h = grid.config.h;
    let band = grid.config.band_mm();
    let mut records = Vec::with_capacity(tris.len());
    let mut buckets = FxHashMap::<[i32; 2], Vec<u16>>::default();
    for t in tris {
        let [a, b, c] = *t;
        let lo = DVec3::new(
            a.x.min(b.x).min(c.x),
            a.y.min(b.y).min(c.y),
            a.z.min(b.z).min(c.z),
        );
        let hi = DVec3::new(
            a.x.max(b.x).max(c.x),
            a.y.max(b.y).max(c.y),
            a.z.max(b.z).max(c.z),
        );
        let origins = [grid.origin.x, grid.origin.y, grid.origin.z];
        let first = std::array::from_fn(|i| {
            let n = (([lo.x, lo.y, lo.z][i] - origins[i]) / h - 0.5 - band / h).floor() as i32;
            n.saturating_sub(1).max(0)
        });
        let last = std::array::from_fn(|i| {
            let n = (([hi.x, hi.y, hi.z][i] - origins[i]) / h - 0.5 + band / h).ceil() as i32;
            n.saturating_add(1).min(domain[i] - 1)
        });
        if (0..3).any(|axis| first[axis] > last[axis]) {
            continue;
        }
        let id = records.len() as u16;
        records.push(RasterTriangle {
            points: t,
            lo,
            hi,
            first,
            last,
        });
        for z in (first[2] >> 3)..=(last[2] >> 3) {
            for y in (first[1] >> 3)..=(last[1] >> 3) {
                buckets.entry([y, z]).or_default().push(id);
            }
        }
    }
    let mut buckets: Vec<_> = buckets.into_iter().collect();
    buckets.sort_unstable_by_key(|b| b.0);
    let mut write_cache = crate::tree::WriteCache::default();
    for batch in buckets.chunks(crate::par::worker_count().max(1) * 4) {
        let results = crate::par::map_indexed(batch.len(), |i| {
            let (tile, candidates) = &batch[i];
            raster_strip(grid, &records, *tile, candidates)
        });
        for (leaf, values) in results.into_iter().flatten() {
            grid.blit_leaf(leaf, values.as_slice(), &mut write_cache);
        }
    }
}

fn raster_strip(
    grid: &SparseGrid<i8>,
    records: &[RasterTriangle],
    [by, bz]: [i32; 2],
    candidates: &[u16],
) -> Vec<DistanceLeaf> {
    use crate::tree::{LEAF_VOXELS, Leaf, LeafView, leaf_index};
    let band = grid.config.band_mm();
    let mut leaves = FxHashMap::<i32, (Box<[i8; LEAF_VOXELS]>, bool)>::default();
    for &id in candidates {
        let RasterTriangle {
            points,
            lo,
            hi,
            first,
            last,
        } = &records[id as usize];
        for bx in (first[0] >> 3)..=(last[0] >> 3) {
            let origin = [bx << 3, by << 3, bz << 3];
            let (values, dirty) = leaves.entry(bx).or_insert_with(|| {
                let values = match grid.leaf_or_tile(origin) {
                    Some(LeafView::Leaf(Leaf::Dense(values))) => values.clone(),
                    Some(LeafView::Leaf(Leaf::Constant(v))) => Box::new([*v; LEAF_VOXELS]),
                    Some(LeafView::Tile(v)) => Box::new([v; LEAF_VOXELS]),
                    None => Box::new([grid.background; LEAF_VOXELS]),
                };
                (values, false)
            });
            let xmin = first[0].max(origin[0]);
            let xmax = last[0].min(origin[0] + 7);
            for z in first[2].max(origin[2])..=last[2].min(origin[2] + 7) {
                for y in first[1].max(origin[1])..=last[1].min(origin[1] + 7) {
                    let row = grid.voxel_center([xmin, y, z]);
                    let gy = (lo.y - row.y).max(row.y - hi.y).max(0.0);
                    let gz = (lo.z - row.z).max(row.z - hi.z).max(0.0);
                    let yz2 = gy * gy + gz * gz;
                    if yz2 >= (band + 1e-9) * (band + 1e-9) {
                        continue;
                    }
                    for x in xmin..=xmax {
                        let p = grid.voxel_center([x, y, z]);
                        let gx = (lo.x - p.x).max(p.x - hi.x).max(0.0);
                        let idx = leaf_index([x, y, z]);
                        let cur = grid.to_distance(values[idx]);
                        // AABB 距离是真实距离的下界。余量保留定点量化边界附近的候选。
                        let limit = cur.abs().min(band) + 1e-9;
                        if gx * gx + yz2 >= limit * limit {
                            continue;
                        }
                        let q = closest_point_on_triangle(p, points[0], points[1], points[2]);
                        let d = (p - q).length();
                        if d < band && d < cur.abs() {
                            let v = grid.to_value(if cur < 0.0 { -d } else { d });
                            *dirty |= values[idx] != v;
                            values[idx] = v;
                        }
                    }
                }
            }
        }
    }
    let mut changed: Vec<_> = leaves
        .into_iter()
        .filter(|(_, (_, dirty))| *dirty)
        .map(|(bx, (data, _))| ([bx, by, bz], data))
        .collect();
    changed.sort_unstable_by_key(|(leaf, _)| leaf[0]);
    changed
}

/// 设计面采样点：每条三角形按 `step` 铺 barycentric 网格（面片中心）。
///
/// 解析过切测试用——对 `DesignModel::Field`，设计面就是这些点。
pub fn surface_points(tris: &[Tri], step: f64) -> Vec<DVec3> {
    let step = step.max(1e-6);
    let mut pts = Vec::new();
    for &[a, b, c] in tris {
        let e1 = b - a;
        let e2 = c - a;
        let nu = (e1.length() / step).ceil().max(1.0) as usize;
        let nv = (e2.length() / step).ceil().max(1.0) as usize;
        for iu in 0..nu {
            for iv in 0..nv {
                let u = (iu as f64 + 0.5) / nu as f64;
                let v = (iv as f64 + 0.5) / nv as f64;
                if u + v > 1.0 {
                    continue; // 三角形外
                }
                pts.push(a + e1 * u + e2 * v);
            }
        }
    }
    pts
}

/// 设计面采样点 + 该点所在三角形的**单位法向**（STL 的面法向，朝外）。
///
/// 验收要按"面朝哪"筛：法向朝下（`n.z < 0`）的是倒扣面，一次装夹从上面下来
/// 根本够不到，不能算进"切坏了"。`surface_points` 只给位置，配不出这个判断。
pub fn surface_points_with_normals(tris: &[Tri], step: f64) -> Vec<(DVec3, DVec3)> {
    let step = step.max(1e-6);
    let mut out = Vec::new();
    for &[a, b, c] in tris {
        let mut n = (b - a).cross(c - a);
        let l = n.length();
        if l > 1e-12 {
            n = n * (1.0 / l);
        } else {
            continue; // 退化三角形：没有法向可言
        }
        let e1 = b - a;
        let e2 = c - a;
        let nu = (e1.length() / step).ceil().max(1.0) as usize;
        let nv = (e2.length() / step).ceil().max(1.0) as usize;
        for iu in 0..nu {
            for iv in 0..nv {
                let u = (iu as f64 + 0.5) / nu as f64;
                let v = (iv as f64 + 0.5) / nv as f64;
                if u + v > 1.0 {
                    continue;
                }
                out.push((a + e1 * u + e2 * v, n));
            }
        }
    }
    out
}

/// 设计面采样点 + 单位外法向 + **该样本代表的面积**（mm²）。
///
/// 面积是给"占比"用的：可达性分析要报"多少面积够不到"，按点数平均会偏向
/// 三角形密的区域（STL 的细分密度和真实面积无关）。
pub fn surface_samples(tris: &[Tri], step: f64) -> Vec<(DVec3, DVec3, f64)> {
    let step = step.max(1e-6);
    let mut out = Vec::new();
    for &[a, b, c] in tris {
        let cr = (b - a).cross(c - a);
        let l = cr.length();
        if l <= 1e-12 {
            continue; // 退化三角形：没法向、没面积
        }
        let n = cr * (1.0 / l);
        let area = 0.5 * l;
        let e1 = b - a;
        let e2 = c - a;
        let nu = (e1.length() / step).ceil().max(1.0) as usize;
        let nv = (e2.length() / step).ceil().max(1.0) as usize;
        let mut local: Vec<DVec3> = Vec::with_capacity(nu * nv);
        for iu in 0..nu {
            for iv in 0..nv {
                let u = (iu as f64 + 0.5) / nu as f64;
                let v = (iv as f64 + 0.5) / nv as f64;
                if u + v > 1.0 {
                    continue;
                }
                local.push(a + e1 * u + e2 * v);
            }
        }
        let share = area / local.len() as f64;
        for p in local {
            out.push((p, n, share));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ValueType;

    fn cfg() -> GridConfig {
        GridConfig { h: 0.1, band_voxels: 4, value_type: ValueType::I8 }
    }

    /// 轴对齐盒的 12 条三角形（绕向不重要——视差只看水密性）。
    fn box_tris(lo: (f64, f64, f64), hi: (f64, f64, f64)) -> Vec<Tri> {
        let (x0, y0, z0) = lo;
        let (x1, y1, z1) = hi;
        let p = |x, y, z| DVec3::new(x, y, z);
        let q = |a: (f64, f64, f64),
                     b: (f64, f64, f64),
                     c: (f64, f64, f64),
                     d: (f64, f64, f64)|
         -> Vec<Tri> {
            vec![
                [p(a.0, a.1, a.2), p(b.0, b.1, b.2), p(c.0, c.1, c.2)],
                [p(a.0, a.1, a.2), p(c.0, c.1, c.2), p(d.0, d.1, d.2)],
            ]
        };
        let mut tris = Vec::new();
        tris.extend(q((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0))); // 底
        tris.extend(q((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))); // 顶
        tris.extend(q((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))); // −Y
        tris.extend(q((x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1))); // +Y
        tris.extend(q((x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1))); // −X
        tris.extend(q((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))); // +X
        tris
    }

    fn box_sdf(p: DVec3, lo: DVec3, hi: DVec3) -> f64 {
        let dx = (lo.x - p.x).max(p.x - hi.x);
        let dy = (lo.y - p.y).max(p.y - hi.y);
        let dz = (lo.z - p.z).max(p.z - hi.z);
        dx.max(dy).max(dz)
    }

    #[test]
    fn closest_point_hits_vertices_edges_and_face() {
        let a = DVec3::ZERO;
        let b = DVec3::new(4.0, 0.0, 0.0);
        let c = DVec3::new(0.0, 4.0, 0.0);
        // 三个顶点区
        assert_eq!(closest_point_on_triangle(DVec3::new(-1.0, -1.0, 0.5), a, b, c), a);
        assert_eq!(closest_point_on_triangle(DVec3::new(5.0, -1.0, 0.0), a, b, c), b);
        assert_eq!(closest_point_on_triangle(DVec3::new(-1.0, 5.0, 0.0), a, b, c), c);
        // AB 边区：垂足 (2,0,0)
        let q = closest_point_on_triangle(DVec3::new(2.0, -3.0, 1.0), a, b, c);
        assert!((q - DVec3::new(2.0, 0.0, 0.0)).length() < 1e-12);
        // 内部区：垂足 (1,1,0)
        let q = closest_point_on_triangle(DVec3::new(1.0, 1.0, 2.0), a, b, c);
        assert!((q - DVec3::new(1.0, 1.0, 0.0)).length() < 1e-12);
        // 斜三角形（钝角）也走对区域：最近点是最钝的顶点
        let a = DVec3::ZERO;
        let b = DVec3::new(1.0, 0.0, 0.0);
        let c = DVec3::new(10.0, 0.1, 0.0);
        let q = closest_point_on_triangle(DVec3::new(-1.0, 0.0, 0.0), a, b, c);
        assert!((q - a).length() < 1e-12);
    }

    #[test]
    fn box_field_matches_analytic_sdf() {
        let lo = DVec3::new(0.5, 0.5, 0.5);
        let hi = DVec3::new(2.5, 2.5, 2.5);
        let tris = box_tris((0.5, 0.5, 0.5), (2.5, 2.5, 2.5));
        let grid = mesh_to_field(&tris, cfg(), DVec3::ZERO, [30, 30, 30]);

        // 带内逐点对拍解析盒 SDF；带外只对符号（场被钳制在 ±band）。
        // 注意：max-component 盒 SDF 只在"面区"是精确欧氏距离——角/棱阴影区
        //（≥2 个分量在盒外）它低估距离，而场算的是真欧氏距离，那里只对符号。
        let mut checked_band = 0;
        for k in 0..30 {
            for j in 0..30 {
                for i in 0..30 {
                    let p = grid.voxel_center([i, j, k]);
                    let exact = box_sdf(p, lo, hi);
                    let got = grid.sdf(p);
                    let outside_axes = (p.x < lo.x || p.x > hi.x) as u32
                        + (p.y < lo.y || p.y > hi.y) as u32
                        + (p.z < lo.z || p.z > hi.z) as u32;
                    let in_face_region = exact < 0.0 || outside_axes <= 1;
                    if exact.abs() < 0.35 && in_face_region {
                        assert!(
                            (got - exact).abs() < 0.005,
                            "({i},{j},{k}) 场 {got} vs 解析 {exact}"
                        );
                        checked_band += 1;
                    } else {
                        assert_eq!(
                            got.signum(),
                            exact.signum(),
                            "({i},{j},{k}) 符号应一致：场 {got} vs 解析 {exact}"
                        );
                    }
                }
            }
        }
        assert!(checked_band > 500, "带内应覆盖大量体素：{checked_band}");
        // 深入体内：饱和材料。约定与毛坯场的 `fill_box(i8::MIN)` 一致，
        // 读回 −128/127 × band ≈ −0.403（略超 −band 的饱和值）
        let c = grid.sdf(DVec3::new(1.5, 1.5, 1.5));
        assert!((c - grid.to_distance(i8::MIN)).abs() < 1e-12, "体内深部应是饱和材料，实际 {c}");
    }

    #[test]
    fn distance_raster_matches_brute_force_at_clipped_tile_boundaries() {
        let mut tris = box_tris((-0.23, 0.04, 0.05), (1.41, 1.62, 1.47));
        tris.extend(box_tris((1.62, 0.55, 0.19), (2.16, 1.24, 1.28)));
        let origin = DVec3::new(-0.33, -0.29, -0.25);
        let domain = [23, 19, 17];
        let grid = mesh_to_field(&tris, cfg(), origin, domain);
        for z in 0..domain[2] {
            for y in 0..domain[1] {
                for x in 0..domain[0] {
                    let ijk = [x, y, z];
                    let p = grid.voxel_center(ijk);
                    let distance = tris
                        .iter()
                        .map(|t| (p - closest_point_on_triangle(p, t[0], t[1], t[2])).length())
                        .fold(f64::INFINITY, f64::min);
                    let inside = box_sdf(
                        p,
                        DVec3::new(-0.23, 0.04, 0.05),
                        DVec3::new(1.41, 1.62, 1.47),
                    ) < 0.0
                        || box_sdf(
                            p,
                            DVec3::new(1.62, 0.55, 0.19),
                            DVec3::new(2.16, 1.24, 1.28),
                        ) < 0.0;
                    let expected = if distance >= cfg().band_mm() && inside {
                        i8::MIN
                    } else {
                        grid.to_value(if inside { -distance } else { distance })
                    };
                    assert_eq!(grid.get(ijk), expected, "distance mismatch at {ijk:?}");
                }
            }
        }
        assert_eq!(grid.get([22, 19, 16]), grid.background);
    }

    #[test]
    fn last_u16_candidate_and_next_batch_both_contribute_distances() {
        let patch = |x, z| {
            [DVec3::new(x, 0.0, z), DVec3::new(x + 0.16, 0.0, z), DVec3::new(x, 0.16, z)]
        };
        let mut tris = vec![patch(0.0, 0.33); u16::MAX as usize];
        tris.push(patch(0.8, 0.16));
        tris.push(patch(0.0, 0.09));
        let mut grid = SparseGrid::<i8>::new(cfg(), DVec3::ZERO);
        distance_pass(&tris, &mut grid, [9, 1, 1]);
        assert_eq!(grid.get([8, 0, 0]), grid.to_value(0.11));
        assert_eq!(grid.get([0, 0, 0]), grid.to_value(0.04));
    }

    #[test]
    fn concave_l_shape_has_correct_parity() {
        // L 形棱柱（凹体）：截面 (0,0)-(2,0)-(2,1)-(1,1)-(1,2)-(0,2)，z 0..1
        let (z0, z1) = (0.0f64, 1.0f64);
        let p = |x, y, z| DVec3::new(x, y, z);
        let poly = [
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 2.0),
            (0.0, 2.0),
        ];
        let mut tris: Vec<Tri> = Vec::new();
        // 侧面：每条边一个四边形 → 2 三角形
        for w in poly.windows(2).chain(std::iter::once(&[poly[5], poly[0]][..])) {
            let (x0, y0) = (w[0].0, w[0].1);
            let (x1, y1) = (w[1].0, w[1].1);
            tris.push([p(x0, y0, z0), p(x1, y1, z0), p(x1, y1, z1)]);
            tris.push([p(x0, y0, z0), p(x1, y1, z1), p(x0, y0, z1)]);
        }
        // 顶/底盖：从 v0 出发的扇形三角化（相邻窗口不共顶点，盖不住六边形）
        for i in 1..poly.len() - 1 {
            let (a, b, c) = (poly[0], poly[i], poly[i + 1]);
            tris.push([p(a.0, a.1, z1), p(b.0, b.1, z1), p(c.0, c.1, z1)]);
            tris.push([p(a.0, a.1, z0), p(c.0, c.1, z0), p(b.0, b.1, z0)]);
        }

        let grid = mesh_to_field(&tris, cfg(), DVec3::ZERO, [30, 30, 30]);
        // L 的两臂内：材料
        assert!(grid.sdf(DVec3::new(1.5, 0.5, 0.5)) < 0.0, "横臂内应是材料");
        assert!(grid.sdf(DVec3::new(0.5, 1.5, 0.5)) < 0.0, "竖臂内应是材料");
        // 凹进去的缺口 (1..2, 1..2)：空气——视差必须处理凹形
        assert!(grid.sdf(DVec3::new(1.5, 1.5, 0.5)) > 0.0, "缺口应是空气");
        // L 外面
        assert!(grid.sdf(DVec3::new(2.5, 0.5, 0.5)) > 0.0, "L 外应是空气");
    }

    #[test]
    fn two_separate_boxes_are_both_solid() {
        let mut tris = box_tris((0.0, 0.0, 0.0), (1.0, 1.0, 1.0));
        tris.extend(box_tris((2.0, 0.0, 0.0), (3.0, 1.0, 1.0)));
        let grid = mesh_to_field(&tris, cfg(), DVec3::ZERO, [40, 20, 20]);
        assert!(grid.sdf(DVec3::new(0.5, 0.5, 0.5)) < 0.0, "盒 1 内");
        assert!(grid.sdf(DVec3::new(2.5, 0.5, 0.5)) < 0.0, "盒 2 内");
        assert!(grid.sdf(DVec3::new(1.5, 0.5, 0.5)) > 0.0, "两盒之间是空气");
    }



    #[test]
    fn surface_points_lie_on_the_faces() {
        let tris = box_tris((0.0, 0.0, 0.0), (2.0, 2.0, 2.0));
        let pts = surface_points(&tris, 0.5);
        assert!(pts.len() > 50, "2mm 盒 @0.5 应有上百个点：{}", pts.len());
        let lo = DVec3::ZERO;
        let hi = DVec3::new(2.0, 2.0, 2.0);
        for p in &pts {
            let d = box_sdf(*p, lo, hi);
            assert!(d.abs() < 1e-9, "采样点应在盒面上，实际 d={d}");
        }
    }
}
