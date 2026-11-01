//! EPA（Expanding Polytope Algorithm）：穿透深度与接触点。
//!
//! [`crate::convex::distance`] 只说"撞了"（距离 0）；报告要说"**撞进去多深、从哪里退**"。
//! EPA 补的就是这两项。
//!
//! ## 在 Minkowski 差里做
//!
//! 设 `W = A − B = { a − b }`。两体相交 ⟺ 原点在 `W` 内。
//! 原点到 `W` 边界的**最短距离**就是穿透深度，最近边界点的方向就是分离轴。
//!
//! 算法：拿 GJK 收敛时的单纯形当初始四面体（不够 4 点就自己扩），然后反复
//!
//! ```text
//! 找离原点最近的面 f              （它是当前"最浅的出口"）
//! 沿 f 的法向 n 取支撑点 w
//! 若 w·n − d(f) < ε  →  收敛，depth = d(f)
//! 否则把 w 加进多面体，删掉所有"看得见" w 的面，用新点与边界边补面
//! ```
//!
//! 每轮只会把多面体撑得更大，`d(最近面)` 单调不减且有上界，所以一定收敛。
//!
//! ## 方向约定（容易搞反）
//!
//! `W` 的最近边界点 = `point_a − point_b`。要把两体分开，**A 得沿 `−n` 移动**
//! （把 A 平移 `t` 等价于把 `W` 平移 `t`，要让原点落到 `W` 外，`t = −n·depth`）。
//!
//! 举例：A 球心 0、半径 3，B 球心 4、半径 3。`W` 是半径 6、中心 −4 的球，
//! 原点在内部，最近边界在 **+X** 方向、距离 2。所以 `axis = +X`，
//! 而 A 要往 **−X** 移 2 才分离——`point_a − point_b = +2X` 也印证这一点
//! （A 最深进入 B 的点在 +X 侧，B 最深进入 A 的点在 −X 侧）。
//!
//! **注意 `axis` 在穿透时不是"从 b 指向 a"。** 分离时 `point_a − point_b` 确实从 b
//! 指向 a；穿透时它反过来——因为 `point_a` 不再是"a 上离 b 最近的点"，而是
//! "a 上扎得最深的点"。两者都等于 `point_a − point_b` 的方向，只是几何含义变了。
//! 所以这里叫 `axis` 而不是 `normal`，且**分离位移是 `−axis · depth`**。
//!
//! ## 精度与代价
//!
//! 收敛容差 1e-7 mm，迭代上限 64。单次几微秒（面数一般 < 40），
//! 只在 GJK 报接触时才跑——绝大多数查询停在 GJK 那一步。

use crate::convex::{Support, Vtx, gjk_raw};
use cncsim_math::DVec3;

/// 一次穿透。
#[derive(Clone, Copy, Debug)]
pub struct Penetration {
    /// 穿透深度（mm）。`0` 表示只是接触。
    pub depth: f64,
    /// 分离轴：`(point_a − point_b)` 归一化，也就是 Minkowski 差边界上离原点最近点的方向。
    ///
    /// **把 `a` 沿 `−axis` 移 [`Penetration::depth`] 即可分离。**
    /// 穿透时它**不是**"从 b 指向 a"（见模块文档的球例）。
    pub axis: DVec3,
    /// `a` 上最深进入 `b` 的点
    pub point_a: DVec3,
    /// `b` 上最深进入 `a` 的点
    pub point_b: DVec3,
    /// EPA 迭代次数（诊断用）
    pub iterations: u32,
}

impl Penetration {
    /// `a` 应该施加的位移（把两体刚好推开）：`−axis · depth`。
    ///
    /// 名字起得具体，是因为这里最容易把符号搞反：
    /// `point_a − point_b` **不是**分离方向，而是它的反面。
    pub fn separation(&self) -> DVec3 {
        self.axis * -self.depth
    }
}

/// 收敛容差（mm）：支撑点沿法向比当前面多出的距离小于它就停。
const EPA_TOL: f64 = 1e-7;
const EPA_MAX_ITER: u32 = 64;

#[derive(Clone, Copy)]
struct Face {
    v: [usize; 3],
    /// 外法向（单位，背离多面体内部）
    n: DVec3,
    /// 原点到该面的有符号距离（沿 `n`，对包含原点的多面体恒 ≥ 0）
    d: f64,
}

/// 两体穿透。**不相交时返回 `None`**（先跑 GJK，接触了才做 EPA）。
pub fn penetration(a: &impl Support, b: &impl Support) -> Option<Penetration> {
    let (gjk, simplex, n) = gjk_raw(a, b);
    if !gjk.touching() {
        return None;
    }

    let mut verts: Vec<Vtx> = simplex[..n].to_vec();
    if !expand_to_tetrahedron(a, b, &mut verts) {
        // 退化：构造不出有体积的四面体。几乎总意味着"只是擦到"。
        return Some(Penetration {
            depth: 0.0,
            axis: gjk.normal,
            point_a: gjk.point_a,
            point_b: gjk.point_b,
            iterations: 0,
        });
    }

    // 如果原点不在四面体内，说明只是极浅的接触（GJK 的 |v| ≤ 1e-9）
    if !origin_in_tetra(verts[0].w, verts[1].w, verts[2].w, verts[3].w) {
        return Some(Penetration {
            depth: 0.0,
            axis: gjk.normal,
            point_a: gjk.point_a,
            point_b: gjk.point_b,
            iterations: 0,
        });
    }

    // 内部参考点：四面体**重心**。不能用原点——原点可能正好落在某个面上
    // （GJK 的单纯形本来就贴着原点），那时 `n·O` 判不出哪边是外。
    let inner = (verts[0].w + verts[1].w + verts[2].w + verts[3].w) * 0.25;
    let mut faces = vec![
        face_of(&verts, 0, 1, 2, inner),
        face_of(&verts, 0, 2, 3, inner),
        face_of(&verts, 0, 3, 1, inner),
        face_of(&verts, 1, 3, 2, inner),
    ];
    faces.retain(|f| f.is_some());
    let mut faces: Vec<Face> = faces.into_iter().flatten().collect();
    if faces.len() < 4 {
        return Some(Penetration {
            depth: 0.0,
            axis: gjk.normal,
            point_a: gjk.point_a,
            point_b: gjk.point_b,
            iterations: 0,
        });
    }

    let mut iterations = 0u32;
    loop {
        iterations += 1;

        // 最近面 = 最浅的出口
        let mut best = 0usize;
        let mut best_d = f64::INFINITY;
        for (i, f) in faces.iter().enumerate() {
            if f.d < best_d {
                best_d = f.d;
                best = i;
            }
        }
        let f = faces[best];

        let w = support_diff(a, b, f.n);
        let progress = w.w.dot(f.n) - f.d;
        if progress < EPA_TOL || iterations > EPA_MAX_ITER {
            let (pa, pb) = contact_points(&verts, &f);
            return Some(Penetration {
                depth: f.d,
                axis: f.n,
                point_a: pa,
                point_b: pb,
                iterations,
            });
        }

        // 扩张：加入 w，删掉所有"看得见"w 的面，用 horizon 边补新面
        let new_idx = verts.len();
        verts.push(w);
        let mut horizon: Vec<(usize, usize)> = Vec::new();
        let mut kept: Vec<Face> = Vec::with_capacity(faces.len() + 4);
        for face in faces.drain(..) {
            let visible = face.n.dot(w.w - verts[face.v[0]].w) > 1e-12;
            if visible {
                for k in 0..3 {
                    push_edge(&mut horizon, face.v[k], face.v[(k + 1) % 3]);
                }
            } else {
                kept.push(face);
            }
        }
        faces = kept;
        for (i, j) in horizon {
            if let Some(nf) = face_of(&verts, i, j, new_idx, inner) {
                faces.push(nf);
            }
        }
        if faces.is_empty() {
            // 数值崩了：退回 GJK 的信息，比 panic 好
            return Some(Penetration {
                depth: 0.0,
                axis: gjk.normal,
                point_a: gjk.point_a,
                point_b: gjk.point_b,
                iterations,
            });
        }
    }
}

/// 面的顶点集合里取重心坐标加权的接触点。
///
/// 用**投影点**的重心坐标而不是面重心：收敛后最近边界点是 `n·d`，
/// 它在面内的位置才对应真正的接触位置。面本身通常已经很小，但这是免费的精度。
fn contact_points(verts: &[Vtx], f: &Face) -> (DVec3, DVec3) {
    let [i, j, k] = f.v;
    let p = f.n * f.d;
    let (p0, p1, p2) = (verts[i].w, verts[j].w, verts[k].w);

    // 面积比重心坐标
    let area = (p1 - p0).cross(p2 - p0).length();
    let (u, v, w) = if area > 1e-18 {
        (
            (p1 - p).cross(p2 - p).length(),
            (p0 - p).cross(p2 - p).length(),
            (p0 - p).cross(p1 - p).length(),
        )
    } else {
        (1.0, 0.0, 0.0)
    };
    let sum = u + v + w;
    let (u, v, w) = if sum > 1e-18 {
        (u / sum, v / sum, w / sum)
    } else {
        (1.0, 0.0, 0.0)
    };

    let pa = verts[i].a * u + verts[j].a * v + verts[k].a * w;
    let pb = verts[i].b * u + verts[j].b * v + verts[k].b * w;
    (pa, pb)
}

fn support_diff(a: &impl Support, b: &impl Support, dir: DVec3) -> Vtx {
    let sa = a.support(dir);
    let sb = b.support(-dir);
    Vtx { w: sa - sb, a: sa, b: sb }
}

/// 构造一个三角面，法向统一翻成"背离多面体内部"。
///
/// `inner` 是**一定在多面体内部**的点（用四面体重心；扩张后重心仍在内部，
/// 因为 polytope 只会变大）。不能拿原点当参考点——GJK 留下的单纯形贴着原点，
/// 原点可能正好在某个面的平面上，`n·O = 0` 判不出朝向。
/// 判错朝向会让最深的那个面被当成"最浅出口"，深度直接报 0。
fn face_of(verts: &[Vtx], i: usize, j: usize, k: usize, inner: DVec3) -> Option<Face> {
    let (p0, p1, p2) = (verts[i].w, verts[j].w, verts[k].w);
    let nrm = (p1 - p0).cross(p2 - p0);
    let len = nrm.length();
    if len < 1e-14 {
        return None;
    }
    let mut n = nrm / len;
    let mut v = [i, j, k];
    if n.dot(inner - p0) > 0.0 {
        n = -n;
        v = [i, k, j];
    }
    // d 是原点到该平面的有符号距离；原点在内部时为正
    let d = n.dot(p0);
    Some(Face { v, n, d })
}

/// 无向边的出现次数：出现两次表示被两个可见面共享（内部边，丢弃），
/// 只出现一次的是 horizon（保留）。
fn push_edge(edges: &mut Vec<(usize, usize)>, i: usize, j: usize) {
    let key = if i < j { (i, j) } else { (j, i) };
    if let Some(pos) = edges.iter().position(|&e| e == key) {
        edges.swap_remove(pos);
    } else {
        edges.push(key);
    }
}

/// 把 GJK 留下的 1–3 点单纯形扩成有体积的四面体。失败（退化）返回 `false`。
fn expand_to_tetrahedron(a: &impl Support, b: &impl Support, verts: &mut Vec<Vtx>) -> bool {
    while verts.len() < 4 {
        let next = match verts.len() {
            1 => {
                // 找离这个点最远的支撑点
                let p0 = verts[0].w;
                let mut best: Option<Vtx> = None;
                let mut best_d = 0.0f64;
                for dir in [DVec3::X, -DVec3::X, DVec3::Y, -DVec3::Y, DVec3::Z, -DVec3::Z] {
                    let c = support_diff(a, b, dir);
                    let d = (c.w - p0).length_squared();
                    if d > best_d {
                        best_d = d;
                        best = Some(c);
                    }
                }
                // 1e-12 mm² 以下：两个体在数值上是同一个点，没有深度可谈
                if best_d < 1e-12 {
                    return false;
                }
                best
            }
            2 => {
                // 垂直于 p0→p1，取能撑出最大三角形的那一侧
                let axis = verts[1].w - verts[0].w;
                let u = any_perp(axis);
                let v = axis.cross(u);
                let mut best: Option<Vtx> = None;
                let mut best_h = 0.0f64;
                for dir in [u, -u, v, -v] {
                    let c = support_diff(a, b, dir);
                    // 新点到直线 p0p1 的距离
                    let h = (c.w - verts[0].w).cross(axis).length();
                    if h > best_h {
                        best_h = h;
                        best = Some(c);
                    }
                }
                if best_h < 1e-7 {
                    return false;
                }
                best
            }
            _ => {
                // 三角形法向两侧取更远的那个
                let nrm = (verts[1].w - verts[0].w).cross(verts[2].w - verts[0].w);
                if nrm.length() < 1e-14 {
                    return false;
                }
                let n = nrm.normalize();
                let p = support_diff(a, b, n);
                let q = support_diff(a, b, -n);
                let dp = (p.w - verts[0].w).dot(n).abs();
                let dq = (q.w - verts[0].w).dot(n).abs();
                let (c, d) = if dp >= dq { (p, dp) } else { (q, dq) };
                if d < 1e-7 {
                    return false;
                }
                Some(c)
            }
        };
        match next {
            Some(v) => verts.push(v),
            None => return false,
        }
    }
    true
}

fn any_perp(v: DVec3) -> DVec3 {
    let helper = if v.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    v.cross(helper).normalize()
}

/// 原点是否在四面体内（共面时返回 `false`）。
fn origin_in_tetra(a: DVec3, b: DVec3, c: DVec3, d: DVec3) -> bool {
    let o = DVec3::ZERO;
    let volume = (b - a).cross(c - a).dot(d - a);
    if volume.abs() < 1e-18 {
        return false;
    }
    let same = |p: DVec3, q: DVec3, r: DVec3, x: DVec3, y: DVec3| {
        let n = (q - p).cross(r - p);
        n.dot(x - p) * n.dot(y - p) >= 0.0
    };
    same(a, b, c, o, d) && same(a, b, d, o, c) && same(a, c, d, o, b) && same(b, c, d, o, a)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convex::{Box3, Cone, Sphere, aabb_gap, distance};
    use cncsim_math::DMat3;

    #[test]
    fn separated_bodies_have_no_penetration() {
        let a = Sphere { c: DVec3::ZERO, r: 3.0 };
        let b = Sphere { c: DVec3::new(10.0, 0.0, 0.0), r: 2.0 };
        assert!(penetration(&a, &b).is_none());
    }

    #[test]
    fn spheres_report_analytic_depth_and_direction() {
        // A 球心 0 半径 3，B 球心 4 半径 3 → 重叠 2
        let a = Sphere { c: DVec3::ZERO, r: 3.0 };
        let b = Sphere { c: DVec3::new(4.0, 0.0, 0.0), r: 3.0 };
        let p = penetration(&a, &b).expect("应报穿透");
        // 球面被多面体平面逼近，深度略小于真值；64 次迭代实测 ~2e-5 mm
        assert!((p.depth - 2.0).abs() < 1e-4, "深度应为 6−4=2，实际 {}", p.depth);
        // axis = (pa − pb) 方向 = +X（a 在左、b 在右，最深的点在 a 的 +X 侧）
        // 球的最近面是多面体的一块平面，朝向有几毫弧度的偏差
        assert!((p.axis - DVec3::X).length() < 1e-2, "分离轴应 +X，实际 {:?}", p.axis);

        // point_a 在 A 的最 +X 处，point_b 在 B 的最 −X 处。
        // 容差 1e-2 是"平面逼近球面"的代价：最近面是一块平板，接触点在板内偏一点。
        // 要更准得换解析特例（球对球有闭式解），但 EPA 的职责是通用形状。
        assert!((p.point_a - DVec3::new(3.0, 0.0, 0.0)).length() < 1e-2, "实际 {:?}", p.point_a);
        assert!((p.point_b - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-2, "实际 {:?}", p.point_b);

        // 分离位移 = −axis·depth：A 在左，要往 −X 推 2
        let s = p.separation();
        // 同上：球面的多面体逼近让方向有几毫弧度偏差，位移的横向分量 ~1e-2
        assert!((s - DVec3::new(-2.0, 0.0, 0.0)).length() < 2e-2, "分离位移应 −2X，实际 {s:?}");
    }

    #[test]
    fn pushing_apart_by_the_reported_depth_just_touches() {
        // **这一条是 EPA 的总验收**：照着它给的位移推开，两体应该恰好接触。
        // 深度错了、方向反了、点错了，这条都会红。
        let a = Sphere { c: DVec3::ZERO, r: 3.0 };
        let b = Sphere { c: DVec3::new(4.0, 0.0, 0.0), r: 3.0 };
        let p = penetration(&a, &b).unwrap();
        let moved = Sphere { c: a.c + p.separation(), r: a.r };
        let after = distance(&moved, &b);
        assert!(
            after.distance < 1e-6,
            "推开 {} 之后应恰好接触，实际距离 {}",
            p.depth,
            after.distance
        );
        // 多推 1% 就该分离开
        let more = Sphere { c: a.c + p.separation() * 1.01, r: a.r };
        assert!(distance(&more, &b).distance > 1e-4);
    }

    #[test]
    fn boxes_report_the_shallowest_axis() {
        // 两个 2×2×2 的盒，中心相差 (1.5, 0, 0) → 只在 X 方向重叠 0.5
        let a = Box3::aabb_shape(DVec3::ZERO, DVec3::splat(1.0));
        let b = Box3::aabb_shape(DVec3::new(1.5, 0.0, 0.0), DVec3::splat(1.0));
        let p = penetration(&a, &b).unwrap();
        assert!((p.depth - 0.5).abs() < 1e-5, "实际 {}", p.depth);
        assert!(p.axis.x.abs() > 0.999, "应沿 X，实际 {:?}", p.axis);

        let moved = Box3::aabb_shape(a.c + p.separation(), a.half);
        assert!(distance(&moved, &b).distance < 1e-6, "推开后应接触");
    }

    #[test]
    fn concentric_boxes_pick_the_minimum_overlap() {
        // 同心，A 是 2×2×6、B 是 2×2×2 → 最小重叠在 X/Y 的 2.0
        let a = Box3::aabb_shape(DVec3::ZERO, DVec3::new(1.0, 1.0, 3.0));
        let b = Box3::aabb_shape(DVec3::ZERO, DVec3::splat(1.0));
        let p = penetration(&a, &b).unwrap();
        // 每个方向的重叠都是 2.0（完全包含），深度取最小的那个 → 2.0
        assert!((p.depth - 2.0).abs() < 1e-4, "实际 {}", p.depth);
    }

    #[test]
    fn cylinder_into_a_plate_reports_axial_depth() {
        // φ6 圆柱沿轴插进 2 mm 厚的板 0.5 mm
        let cyl = Cone {
            c: DVec3::ZERO,
            axis: DVec3::Z,
            half_h: 10.0,
            r_lo: 3.0,
            r_hi: 3.0,
        };
        // 板：z ∈ [−1, 1]（厚 2），x/y 很大
        let plate = Box3::aabb_shape(DVec3::new(0.0, 0.0, 10.0), DVec3::new(50.0, 50.0, 1.0));
        // 圆柱顶面 z = 10，板底面 z = 9 → 插入 1.0
        let p = penetration(&cyl, &plate).expect("应穿透");
        assert!((p.depth - 1.0).abs() < 1e-4, "轴向深度应为 1，实际 {}", p.depth);
        assert!(p.axis.z.abs() > 0.99, "应沿 Z，实际 {:?}", p.axis);
        assert!(distance(&cyl, &plate).touching());
    }

    #[test]
    fn separating_by_reported_depth_works_for_random_pairs() {
        // 随机穿透对：照着 depth / direction 推开，必须刚好接触。
        struct Rng(u64);
        impl Rng {
            fn next_f64(&mut self) -> f64 {
                let mut x = self.0;
                x ^= x >> 12;
                x ^= x << 25;
                x ^= x >> 27;
                self.0 = x;
                (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
            }
            fn range(&mut self, lo: f64, hi: f64) -> f64 {
                lo + (hi - lo) * self.next_f64()
            }
            fn vec(&mut self, s: f64) -> DVec3 {
                DVec3::new(self.range(-s, s), self.range(-s, s), self.range(-s, s))
            }
        }

        let mut rng = Rng(0x5EED_2026_1029);
        let mut checked = 0;
        for _ in 0..120 {
            let cone = Cone {
                c: DVec3::ZERO,
                axis: DVec3::Z,
                half_h: rng.range(3.0, 10.0),
                r_lo: rng.range(0.5, 3.0),
                r_hi: rng.range(0.5, 3.0),
            };
            let bx = Box3 {
                c: rng.vec(2.0),
                half: DVec3::new(rng.range(1.0, 4.0), rng.range(1.0, 4.0), rng.range(1.0, 4.0)),
                rot: DMat3::IDENTITY,
            };
            // 先确认真的相交，再验推开
            if aabb_gap(&cone, &bx) > 0.0 {
                continue;
            }
            let Some(p) = penetration(&cone, &bx) else {
                // 包围盒相交但实际分离：允许，不算错
                continue;
            };
            if p.depth < 1e-5 {
                continue; // 恰好相切，推开没意义
            }
            let moved = Cone { c: cone.c + p.separation(), ..cone };
            let after = distance(&moved, &bx);
            assert!(
                after.distance < 1e-4,
                "推开 {:.6} mm 后仍相距 {}（cone.axis={:?}、depth={}、dir={:?}）",
                p.depth,
                after.distance,
                cone.axis,
                p.depth,
                p.axis
            );
            checked += 1;
        }
        println!("随机推开验收：{checked} 例");
        assert!(checked > 30, "有效样本太少（{checked}）");
    }
}
