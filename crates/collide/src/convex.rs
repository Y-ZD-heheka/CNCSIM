//! 凸体 × 凸体：GJK 距离查询。**解析 support function，不做多边形离散。**
//!
//! [ADR-004](../../../docs/ADR-004-碰撞检查方案.md) §2.6：上一版设计把旋转体离散成
//! 16 边棱柱，外切误差 `r(1−cos(π/16)) ≈ 0.019r`——对 R=30 的刀夹是 **0.57 mm 的系统性误报**。
//! 每把刀都"差 0.57 mm 就撞"，报告就没人看了。
//!
//! GJK 只需要支持映射 `support(dir) = 沿 dir 最远的点`，圆台/球/盒的支持映射都是解析的：
//!
//! ```text
//! 圆台：轴向取 r 大的那一端，径向沿 dir 的水平投影
//! 球  ：c + dir̂·r
//! 盒  ：逐轴取符号
//! ```
//!
//! 于是凸体对凸体的距离**精确到浮点**，零建模误差。这条让"刀夹 / 夹具 / 台面 vs 刀具"
//! 这一类碰撞真正达到解析级，而毛坯那一类由体素场的 1.6 µm（平面）/ 2.0 µm（曲面）兜底
//! （见 [`crate::continuous`]）。
//!
//! ## 为什么实现的是"距离"而不是"相交判定"
//!
//! 保守推进（[`crate::continuous`]）要的是**距离下界**来算安全步长；布尔判定只能给出
//! "这一段撞了/没撞"，无法放进 CA 的循环。所以这里直接算距离，`distance == 0` 即接触。
//!
//! 副产品：最近点对 `(point_a, point_b)` 就是碰撞报告要的"哪里撞了"，
//! 法向 `normal` 是"往哪边退"。
//!
//! ## 用什么保证算对
//!
//! GJK 的经典 bug 是**单纯形维护错**（该丢的点没丢、该留的丢了），只在少数几何下显形。
//! 这里不用增量维护：每轮把当前点集（≤ 4）的**所有非空子集**各求一次"到原点的最近点"
//! （点 / 线段 / 三角形 / 四面体都有闭式解），取最小者。最多 15 个子集 × 每次迭代，
//! 单次查询微秒级；换掉的是整类退化 bug。
//!
//! 最优性用 **KKT 条件**自证（`tests` 里的随机对拍就是这个）：
//! 设 `d = point_a − point_b`，则必须有
//!
//! ```text
//! ⟨d̂, point_a⟩ ≥ ⟨d̂, support_a(d̂)⟩ − ε      ← point_a 确实是 a 在 d̂ 方向的最远点
//! ⟨d̂, point_b⟩ ≤ ⟨d̂, support_b(−d̂)⟩ + ε
//! ```
//!
//! 这两条成立 ⟺ `d` 就是两体的最短距离——不需要 oracle 也能验。

use cncsim_math::{DMat3, DMat4, DVec3};
use cncsim_tool::Profile;

// ───────────────────────────── 支持映射 ─────────────────────────────

/// 凸体：只要一个支持映射。
pub trait Support {
    /// 沿 `dir` 最远的点（世界坐标）。`dir` 不必归一化。
    fn support(&self, dir: DVec3) -> DVec3;
    /// 世界坐标 AABB `(lo, hi)`，必须**包住**物体（可以偏大，不能偏小）。
    fn aabb(&self) -> (DVec3, DVec3);
}

#[inline]
fn unit_or_zero(d: DVec3) -> DVec3 {
    let l = d.length();
    if l > 1e-15 { d / l } else { DVec3::ZERO }
}

#[inline]
fn vmin(a: DVec3, b: DVec3) -> DVec3 {
    DVec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z))
}

#[inline]
fn vmax(a: DVec3, b: DVec3) -> DVec3 {
    DVec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))
}

/// 球。刀尖球头、简化后的主轴鼻端都用它。
#[derive(Clone, Copy, Debug)]
pub struct Sphere {
    pub c: DVec3,
    pub r: f64,
}

impl Support for Sphere {
    fn support(&self, dir: DVec3) -> DVec3 {
        self.c + unit_or_zero(dir) * self.r
    }
    fn aabb(&self) -> (DVec3, DVec3) {
        (self.c - DVec3::splat(self.r), self.c + DVec3::splat(self.r))
    }
}

/// 圆台（含圆柱 `r_lo == r_hi`、圆锥 `r_lo == 0`）。
///
/// `c` 是**轴向中点**，`axis` 单位向量。`r_lo` 在 `c − axis·half_h` 端，
/// `r_hi` 在 `c + axis·half_h` 端。旋转体的母线分解出来就是它。
#[derive(Clone, Copy, Debug)]
pub struct Cone {
    pub c: DVec3,
    pub axis: DVec3,
    pub half_h: f64,
    pub r_lo: f64,
    pub r_hi: f64,
}

impl Support for Cone {
    fn support(&self, dir: DVec3) -> DVec3 {
        let d = dir.dot(self.axis);
        // 退化圆台（`half_h == 0`，平底刀的端面）：两端在同一个 z 平面上。
        // 此时 `r_lo / r_hi` 是**同心**的两个圆，凸包是半径 max 的那个圆盘——
        // 按 d 的正负去挑 r_lo 会挑到圆心，距离虚高。轴向分量一律不参与。
        //
        // 注意乘的是 `radial̂` 不是 `dir̂`：`c + dir̂·r` 会在 dir 有轴向分量时
        // 跑到圆盘**外**（z 跑掉），那是非法 support，GJK 会在此处收敛到错误值。
        if self.half_h < 1e-12 {
            let r = self.r_lo.max(self.r_hi);
            let radial = dir - self.axis * d;
            return self.c + unit_or_zero(radial) * r;
        }
        // **两个候选都要算**。轴向正负只说明哪一端的 z 更有利，但半径差可能压过它：
        //   f(端) = z_端·d + r_端·|dir_radial|
        // 当 `−2h·d` 与 `(r_lo − r_hi)·|dir_radial|` 反号时就会翻盘（细长圆台尤其常见）。
        // 早版本只按 `d > 0` 挑端，对 `r_lo ≠ r_hi` 的圆台给出**非法 support**——
        // 支撑点不是最远点，GJK 于是收敛到偏大的距离（实测某随机用例虚高 0.06 mm，
        // 而最优性断言查不出来，因为两边用的是同一个错的 support）。
        let radial = unit_or_zero(dir - self.axis * d);
        let lo = self.c - self.axis * self.half_h + radial * self.r_lo;
        let hi = self.c + self.axis * self.half_h + radial * self.r_hi;
        if lo.dot(dir) >= hi.dot(dir) { lo } else { hi }
    }
    fn aabb(&self) -> (DVec3, DVec3) {
        let r = self.r_lo.max(self.r_hi);
        let h = self.axis * self.half_h;
        let cap = DVec3::new(
            h.x.abs() + r * (1.0 - self.axis.x * self.axis.x).max(0.0).sqrt(),
            h.y.abs() + r * (1.0 - self.axis.y * self.axis.y).max(0.0).sqrt(),
            h.z.abs() + r * (1.0 - self.axis.z * self.axis.z).max(0.0).sqrt(),
        );
        (self.c - cap, self.c + cap)
    }
}

/// 胶囊：球沿线段扫掠。给"球头刀 + 细长刀柄"的粗筛用。
#[derive(Clone, Copy, Debug)]
pub struct Capsule {
    pub a: DVec3,
    pub b: DVec3,
    pub r: f64,
}

impl Support for Capsule {
    fn support(&self, dir: DVec3) -> DVec3 {
        let end = if (self.b - self.a).dot(dir) > 0.0 { self.b } else { self.a };
        end + unit_or_zero(dir) * self.r
    }
    fn aabb(&self) -> (DVec3, DVec3) {
        (vmin(self.a, self.b) - DVec3::splat(self.r), vmax(self.a, self.b) + DVec3::splat(self.r))
    }
}

/// 有向盒。夹具、台面、机床铸件的主力形状。
#[derive(Clone, Copy, Debug)]
pub struct Box3 {
    pub c: DVec3,
    pub half: DVec3,
    /// 列 = 盒的局部轴
    pub rot: DMat3,
}

impl Box3 {
    pub fn aabb_shape(c: DVec3, half: DVec3) -> Self {
        Self { c, half, rot: DMat3::IDENTITY }
    }
}

impl Support for Box3 {
    fn support(&self, dir: DVec3) -> DVec3 {
        let ld = self.rot.transpose().mul_vec3(dir);
        let s = DVec3::new(
            if ld.x >= 0.0 { self.half.x } else { -self.half.x },
            if ld.y >= 0.0 { self.half.y } else { -self.half.y },
            if ld.z >= 0.0 { self.half.z } else { -self.half.z },
        );
        self.c + self.rot.mul_vec3(s)
    }
    fn aabb(&self) -> (DVec3, DVec3) {
        // 半宽 = Σ |列向量| · half，逐分量
        let e = DVec3::new(
            self.rot.cols[0].x.abs() * self.half.x
                + self.rot.cols[1].x.abs() * self.half.y
                + self.rot.cols[2].x.abs() * self.half.z,
            self.rot.cols[0].y.abs() * self.half.x
                + self.rot.cols[1].y.abs() * self.half.y
                + self.rot.cols[2].y.abs() * self.half.z,
            self.rot.cols[0].z.abs() * self.half.x
                + self.rot.cols[1].z.abs() * self.half.y
                + self.rot.cols[2].z.abs() * self.half.z,
        );
        (self.c - e, self.c + e)
    }
}

/// 顶点表的凸包（支持映射 = 逐点比 `dot`）。
///
/// 给"从 STEP 导入的机床件"用：不需要三角化，只要它的顶点坐标。
/// 支持点查询是 O(n)，几十个点的零件完全够用。
#[derive(Clone, Debug)]
pub struct Hull {
    pub pts: Vec<DVec3>,
}

impl Support for Hull {
    fn support(&self, dir: DVec3) -> DVec3 {
        let mut best = self.pts[0];
        let mut m = best.dot(dir);
        for &p in &self.pts[1..] {
            let v = p.dot(dir);
            if v > m {
                m = v;
                best = p;
            }
        }
        best
    }
    fn aabb(&self) -> (DVec3, DVec3) {
        let mut lo = self.pts[0];
        let mut hi = self.pts[0];
        for &p in &self.pts[1..] {
            lo = vmin(lo, p);
            hi = vmax(hi, p);
        }
        (lo, hi)
    }
}

/// 刚体放置：把**局部坐标**下的形状搬到世界。
///
/// 存在理由是别让每个形状都自己带一套旋转逻辑——`support` 只需在局部求一次，
/// 而 `Pose` 只有 (tip, axis) 两个自由度，装不下绕轴的自转，所以这里要完整的 `DMat4`。
pub struct Placed<S> {
    pub shape: S,
    pub xf: DMat4,
}

impl<S> Placed<S> {
    pub fn new(shape: S, xf: DMat4) -> Self {
        Self { shape, xf }
    }
}

impl<S: Support> Support for Placed<S> {
    fn support(&self, dir: DVec3) -> DVec3 {
        let inv = self.xf.rotation().transpose();
        self.xf.transform_point3(self.shape.support(inv.mul_vec3(dir)))
    }
    fn aabb(&self) -> (DVec3, DVec3) {
        let (lo, hi) = self.shape.aabb();
        let c = (lo + hi) * 0.5;
        let e = (hi - lo) * 0.5;
        let mut out_lo = DVec3::splat(f64::INFINITY);
        let mut out_hi = DVec3::splat(f64::NEG_INFINITY);
        for i in 0..8 {
            let s = DVec3::new(
                if i & 1 == 0 { -e.x } else { e.x },
                if i & 2 == 0 { -e.y } else { e.y },
                if i & 4 == 0 { -e.z } else { e.z },
            );
            let p = self.xf.transform_point3(c + s);
            out_lo = vmin(out_lo, p);
            out_hi = vmax(out_hi, p);
        }
        (out_lo, out_hi)
    }
}

/// 把刀柄母线分解成一串圆台（相邻两点之间一段）。
///
/// **整把刀不必是凸的**：凸体并集到另一点集的距离 = 各凸体距离的最小值，
/// 所以逐段求 GJK、取最小，结果仍然正确。唯一要求是每段本身凸——母线相邻两点
/// 之间的圆台天然满足。
///
/// 返回的是**局部坐标**（刀尖在原点、刀轴 +Z），调用方套 [`Placed`]。
pub fn cones_of(profile: &Profile) -> Vec<Cone> {
    let pts = profile.points();
    let mut out = Vec::with_capacity(pts.len().saturating_sub(1));
    for s in pts.windows(2) {
        let (p0, p1) = (s[0], s[1]);
        let (z0, z1) = (p0.y, p1.y);
        out.push(Cone {
            c: DVec3::new(0.0, 0.0, (z0 + z1) * 0.5),
            axis: DVec3::Z,
            half_h: (z1 - z0).abs() * 0.5,
            // 轴向上"低端 / 高端"的半径，与点位无关
            r_lo: if z1 >= z0 { p0.x } else { p1.x },
            r_hi: if z1 >= z0 { p1.x } else { p0.x },
        });
    }
    out
}

// ───────────────────────────── GJK ─────────────────────────────

/// 一次 GJK 查询的结果。
#[derive(Clone, Copy, Debug)]
pub struct Gjk {
    /// 两体距离（mm）。`0` 表示接触或穿透。
    pub distance: f64,
    /// `a` 上最靠近 `b` 的点（世界坐标）
    pub point_a: DVec3,
    /// `b` 上最靠近 `a` 的点
    pub point_b: DVec3,
    /// 分离方向（从 `b` 指向 `a`，单位向量）。`distance == 0` 时只是个提示方向。
    pub normal: DVec3,
    /// GJK 迭代次数。明显偏大（> 30）说明几何退化，值得看一眼。
    pub iterations: u32,
}

impl Gjk {
    /// 是否接触 / 穿透。
    pub fn touching(&self) -> bool {
        self.distance <= 0.0
    }
}

const MAX_ITER: u32 = 48;
/// 收敛判据的相对容差（作用在 `|v|²` 上）
const REL_EPS: f64 = 1e-10;
/// `|v|²` 的下限：1e-9 mm 以内视为接触
const ABS_EPS: f64 = 1e-18;

/// Minkowski 差里的一个点：`w = a − b`，同时记住它在两个原体上的出处。
///
/// EPA（[`crate::penetration`]）要拿它反推接触点，所以是 `pub(crate)`。
#[derive(Clone, Copy, Default)]
pub(crate) struct Vtx {
    pub(crate) w: DVec3,
    pub(crate) a: DVec3,
    pub(crate) b: DVec3,
}

/// 两体的 AABB 间距（不相交时 > 0）。**真实距离的严格下界。**
///
/// 想省时间就在调 [`distance`] 之前先问它：这把刀离夹具 40 mm 时，
/// 一次 8 字节的比较就够，不必跑 GJK。
pub fn aabb_gap(a: &impl Support, b: &impl Support) -> f64 {
    let (alo, ahi) = a.aabb();
    let (blo, bhi) = b.aabb();
    aabb_gap_of(alo, ahi, blo, bhi)
}

fn aabb_gap_of(alo: DVec3, ahi: DVec3, blo: DVec3, bhi: DVec3) -> f64 {
    let mut s = 0.0;
    for i in 0..3 {
        let d = (blo.component(i) - ahi.component(i)).max(alo.component(i) - bhi.component(i));
        if d > 0.0 {
            s += d * d;
        }
    }
    s.sqrt()
}

/// 两体距离。**总是精确**——最近点对是碰撞报告要的"哪里撞了"，不能拿盒面近似糊过去。
///
/// 想先用包围盒粗筛就用 [`aabb_gap`]；廉价、无副作用。
pub fn distance(a: &impl Support, b: &impl Support) -> Gjk {
    gjk_raw(a, b).0
}

/// GJK 的完整产物：距离 + 收敛时的单纯形（EPA 从这里起步）。
///
/// 单纯形的**前 `n` 个**有效。接触时它不一定是个四面体（可能只有 1–3 点），
/// EPA 要自己把它扩成四面体——见 [`crate::penetration`]。
pub(crate) fn gjk_raw(
    a: &impl Support,
    b: &impl Support,
) -> (Gjk, [Vtx; 4], usize) {
    let (alo, ahi) = a.aabb();
    let (blo, bhi) = b.aabb();
    // 中心差方向只当**初始搜索方向**用：分离的两体基本上一两步就收敛。
    // （早版本在这里直接返回盒间距离，快了但两点变成盒面上的位置——报告会说错地方。）
    let center_dir = {
        let d = unit_or_zero((alo + ahi) * 0.5 - (blo + bhi) * 0.5);
        if d == DVec3::ZERO { DVec3::X } else { d }
    };

    // 初始点必须落在 a−b 里
    let pa = a.support(center_dir);
    let pb = b.support(-center_dir);
    let mut v = pa - pb;
    let mut simplex = [Vtx::default(); 4];
    simplex[0] = Vtx { w: v, a: pa, b: pb };
    if v.length_squared() <= ABS_EPS {
        return (
            Gjk {
                distance: 0.0,
                point_a: pa,
                point_b: pb,
                normal: center_dir,
                iterations: 1,
            },
            simplex,
            1,
        );
    }
    let fallback = unit_or_zero(-v);

    let mut n = 1usize;
    let mut best_a = pa;
    let mut best_b = pb;
    let mut iterations = 0u32;

    loop {
        iterations += 1;
        let d = -v;
        let sa = a.support(d);
        let sb = b.support(-d);
        let w = Vtx { w: sa - sb, a: sa, b: sb };

        // v 已经是 a−b 里离原点最近的点 ⟺ 没有点比它更近
        if v.length_squared() - v.dot(w.w) <= REL_EPS * v.length_squared() {
            break;
        }
        debug_assert!(n < 4, "单纯形最多 4 点；收窄漏了零权重点");
        simplex[n] = w;
        n += 1;

        let (v2, wt) = closest_to_origin(&simplex, n);
        if v2.length_squared() <= ABS_EPS {
            // 原点在凸包里 → 接触 / 穿透。单纯形留给 EPA。
            let dir = unit_or_zero(d);
            return (
                Gjk {
                    distance: 0.0,
                    point_a: a.support(dir),
                    point_b: b.support(-dir),
                    normal: if dir == DVec3::ZERO { fallback } else { dir },
                    iterations,
                },
                simplex,
                n,
            );
        }

        // 用重心坐标把最近点对投回两个体上（== 加权平均的支撑点）
        let mut npa = DVec3::ZERO;
        let mut npb = DVec3::ZERO;
        for k in 0..wt.len {
            npa += simplex[wt.idx[k]].a * wt.bary[k];
            npb += simplex[wt.idx[k]].b * wt.bary[k];
        }
        best_a = npa;
        best_b = npb;

        // 收窄到**权重严格为正**的那几点。
        //
        // 权重为 0 的点必须丢，哪怕它出现在最优子集里：四面体不含原点时，
        // `closest_on` 给的是某个面，返回的 4 元权重里恰好有一个 0。留着它，
        // 下一轮 push 就变 5 点、超出单纯形容量——这正是 GJK 最经典的
        // "该丢的点没丢"。顺序不能变：`closest_to_origin` 要求最新点在最末。
        let mut kept = [Vtx::default(); 4];
        let mut m = 0usize;
        for k in 0..wt.len {
            if wt.bary[k] > 1e-12 {
                kept[m] = simplex[wt.idx[k]];
                m += 1;
            }
        }
        if m == 0 {
            // 只剩数值噪声时的兜底：留下最新加入的那个点
            kept[0] = simplex[wt.idx[wt.len - 1]];
            m = 1;
        }
        simplex = kept;
        n = m;
        v = v2;

        if iterations >= MAX_ITER {
            break;
        }
    }

    let dist = v.length();
    (
        Gjk {
            distance: dist,
            point_a: best_a,
            point_b: best_b,
            normal: if dist > 1e-9 { (best_a - best_b) / dist } else { fallback },
            iterations,
        },
        simplex,
        n,
    )
}

/// 只判"是否接触及以上"。
///
/// 目前就是 `distance(..).touching()` —— **没有**单独写布尔版 GJK。
/// 布尔版能早停（找到包含原点的单纯形就返回），省的是最后几次收敛迭代；
/// 但它要另一套单纯形维护逻辑，而单纯形维护正是 GJK 最容易写错的地方。
/// 等性能分析真的指出这项是瓶颈再拆。
pub fn intersects(a: &impl Support, b: &impl Support) -> bool {
    distance(a, b).touching()
}

/// 子集的索引与重心坐标。**定长数组**：热路径上不能有堆分配。
#[derive(Clone, Copy, Default)]
struct Weights {
    idx: [usize; 4],
    bary: [f64; 4],
    len: usize,
}

/// 枚举点集（≤ 4 点）的子集，取"到原点最近"的那个。
///
/// **只枚举含 `pts[n-1]` 的子集**：不含它的那些子集上一轮就评过了，当前最近点
/// 已经是它们的下确界——再加一个点不会让老的子集变近。这一步把枚举量从 `2ⁿ−1`
/// 压到 `2ⁿ⁻¹`（n=4 时 15 → 8）。
///
/// **为什么穷举而不是增量维护单纯形**：增量维护的经典 bug 是"该丢的点没丢"，
/// 只在少数几何下显形，而且错了以后距离**偏小**（更危险的方向）。
/// 闭式解 + 穷举是 O(1) 且不可能错，省下的时间用定长数组找回来。
fn closest_to_origin(pts: &[Vtx; 4], n: usize) -> (DVec3, Weights) {
    debug_assert!(n >= 1 && n <= 4);
    let last = n - 1;
    let mut best = f64::INFINITY;
    let mut out = Weights::default();
    let mut best_p = DVec3::ZERO;

    // mask 的第 last 位恒为 1
    for mask in 0u32..(1 << last) {
        let mut w = [DVec3::ZERO; 4];
        let mut idx = [0usize; 4];
        let mut k = 0usize;
        for i in 0..last {
            if mask >> i & 1 == 1 {
                w[k] = pts[i].w;
                idx[k] = i;
                k += 1;
            }
        }
        w[k] = pts[last].w;
        idx[k] = last;
        k += 1;

        match closest_on(&w[..k]) {
            // 原点在这个子集的凸包内 → 整体距离就是 0，不用再找
            None => {
                out = Weights { idx, bary: [0.0; 4], len: k };
                return (DVec3::ZERO, out);
            }
            Some((p, bary)) => {
                let d2 = p.length_squared();
                if d2 < best {
                    best = d2;
                    best_p = p;
                    out = Weights { idx, bary, len: k };
                }
            }
        }
    }
    (best_p, out)
}

/// 原点到一个点集凸包的最近点。`None` = 原点在凸包内（距离 0）。
///
/// 权重按输入点数对齐（`w.len()` 个），未参与的点权重为 0。
fn closest_on(w: &[DVec3]) -> Option<(DVec3, [f64; 4])> {
    let mut out = [0.0f64; 4];
    match w.len() {
        1 => {
            out[0] = 1.0;
            Some((w[0], out))
        }
        2 => {
            let (p, b) = closest_segment(w[0], w[1]);
            out[..2].copy_from_slice(&b);
            Some((p, out))
        }
        3 => {
            let (p, b) = closest_triangle(w[0], w[1], w[2]);
            out[..3].copy_from_slice(&b);
            Some((p, out))
        }
        4 => {
            if origin_in_tetra(w[0], w[1], w[2], w[3]) {
                return None;
            }
            // 不在内部 → 最近点必在某个面上
            let mut best_p = DVec3::ZERO;
            let mut best_d2 = f64::INFINITY;
            for (i, j, k) in [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)] {
                let (p, b3) = closest_triangle(w[i], w[j], w[k]);
                let d2 = p.length_squared();
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_p = p;
                    out = [0.0; 4];
                    out[i] = b3[0];
                    out[j] = b3[1];
                    out[k] = b3[2];
                }
            }
            Some((best_p, out))
        }
        _ => None,
    }
}

/// 原点到线段的最近点 + 重心坐标。
fn closest_segment(a: DVec3, b: DVec3) -> (DVec3, [f64; 2]) {
    let ab = b - a;
    let d2 = ab.length_squared();
    if d2 < 1e-30 {
        return (a, [1.0, 0.0]);
    }
    let t = (-a.dot(ab) / d2).clamp(0.0, 1.0);
    (a + ab * t, [1.0 - t, t])
}

/// 原点到三角形的最近点 + 重心坐标（Ericson《Real-Time Collision Detection》5.1.5）。
fn closest_triangle(a: DVec3, b: DVec3, c: DVec3) -> (DVec3, [f64; 3]) {
    let ab = b - a;
    let ac = c - a;
    let ap = -a;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, [1.0, 0.0, 0.0]);
    }

    let bp = -b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, [0.0, 1.0, 0.0]);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return (a + ab * t, [1.0 - t, t, 0.0]);
    }

    let cp = -c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, [0.0, 0.0, 1.0]);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return (a + ac * t, [1.0 - t, 0.0, t]);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * t, [0.0, 1.0 - t, t]);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (a + ab * v + ac * w, [1.0 - v - w, v, w])
}

/// 原点是否在四面体内（四点必须不共面；共面时返回 `false`，交给面去处理）。
fn origin_in_tetra(a: DVec3, b: DVec3, c: DVec3, d: DVec3) -> bool {
    let o = DVec3::ZERO;
    let volume = (b - a).cross(c - a).dot(d - a);
    if volume.abs() < 1e-18 {
        return false; // 退化：四点共面，没有"内部"
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

    /// 固定种子 xorshift（项目约定：不引 rand，见 ADR-003 §2）
    struct Rng(u64);
    impl Rng {
        fn new() -> Self {
            Self(0x5EED_2026_1029)
        }
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
            DVec3::new(
                self.range(-s, s),
                self.range(-s, s),
                self.range(-s, s),
            )
        }
        fn unit(&mut self) -> DVec3 {
            loop {
                let v = self.vec(1.0);
                let l = v.length();
                if l > 0.1 && l <= 1.0 {
                    return v / l;
                }
            }
        }
    }

    /// GJK 的充要最优性条件：`pa` 必须是 a 在 `d̂` 上的最远点，`pb` 是 b 在 `−d̂` 上的最远点。
    /// 两条都成立 ⟺ `|pa − pb|` 就是最短距离。不需要 oracle。
    fn assert_optimal(a: &impl Support, b: &impl Support, g: &Gjk, tag: &str) {
        if g.distance <= 1e-9 {
            return;
        }
        // d 从 b 指向 a。`point_a` 是最小化 |pa − pb| 的解，所以它是 a 沿 **−d** 的极值点
        // （沿 d 投影最小），`point_b` 是 b 沿 +d 的极值点。写反了就会误报。
        let d = (g.point_a - g.point_b) / g.distance;
        let sa = a.support(-d);
        let sb = b.support(d);
        let over_a = d.dot(g.point_a) - d.dot(sa);
        let over_b = d.dot(sb) - d.dot(g.point_b);
        let tol = 1e-9 * (1.0 + g.distance);
        assert!(
            over_a <= tol && over_b <= tol,
            "{tag}: 不满足最优性（pa 差 {over_a:e}，pb 差 {over_b:e}，d={}）",
            g.distance
        );
        // 距离也要与点对一致
        assert!(
            (g.point_a - g.point_b).length() - g.distance < 1e-9,
            "{tag}: 点对距离与 distance 不符"
        );
    }

    #[test]
    fn sphere_vs_sphere_matches_analytic() {
        let a = Sphere { c: DVec3::new(0.0, 0.0, 0.0), r: 3.0 };
        let b = Sphere { c: DVec3::new(10.0, 0.0, 0.0), r: 2.0 };
        let g = distance(&a, &b);
        assert!((g.distance - 5.0).abs() < 1e-9, "实际 {}", g.distance);
        assert!(!g.touching());
        assert!((g.point_a - DVec3::new(3.0, 0.0, 0.0)).length() < 1e-9);
        assert!((g.point_b - DVec3::new(8.0, 0.0, 0.0)).length() < 1e-9);
        assert!(g.normal.x.abs() > 0.99);
    }

    #[test]
    fn overlapping_spheres_are_touching() {
        let a = Sphere { c: DVec3::ZERO, r: 3.0 };
        let b = Sphere { c: DVec3::new(1.0, 0.0, 0.0), r: 2.0 };
        assert!(distance(&a, &b).touching(), "相交的两个球必须报接触");
        assert!(intersects(&a, &b));
    }

    #[test]
    fn concentic_spheres_report_contact() {
        // 一个完全在另一个内部：这是 GJK 最容易死循环的形状
        let a = Sphere { c: DVec3::ZERO, r: 10.0 };
        let b = Sphere { c: DVec3::new(0.1, 0.0, 0.0), r: 1.0 };
        let g = distance(&a, &b);
        assert!(g.touching());
        assert!(g.iterations < MAX_ITER, "不该迭代到上限，实际 {}", g.iterations);
    }

    #[test]
    fn axis_aligned_boxes_match_analytic() {
        let a = Box3::aabb_shape(DVec3::ZERO, DVec3::splat(1.0));
        let b = Box3::aabb_shape(DVec3::new(5.0, 0.0, 0.0), DVec3::splat(1.0));
        let g = distance(&a, &b);
        assert!((g.distance - 3.0).abs() < 1e-9, "实际 {}", g.distance);

        // 对角错开：(3,4,0) 的盒心差 → 面对齐时距离 = 3−1 = 2
        let c = Box3::aabb_shape(DVec3::new(4.0, 0.0, 0.0), DVec3::splat(1.0));
        assert!((distance(&a, &c).distance - 2.0).abs() < 1e-9);
    }

    #[test]
    fn box_corner_to_corner_distance() {
        let a = Box3::aabb_shape(DVec3::ZERO, DVec3::splat(1.0));
        let b = Box3::aabb_shape(DVec3::new(4.0, 4.0, 0.0), DVec3::splat(1.0));
        // 最近点对是 (1,1,±1) 与 (3,3,±1) → √8
        let g = distance(&a, &b);
        assert!((g.distance - 8.0f64.sqrt()).abs() < 1e-9, "实际 {}", g.distance);
    }

    #[test]
    fn cylinder_side_to_sphere_is_analytic() {
        // 圆柱轴从 (0,0,-10) 到 (0,0,10)，半径 3；球在 x 方向
        let cyl = Cone {
            c: DVec3::ZERO,
            axis: DVec3::Z,
            half_h: 10.0,
            r_lo: 3.0,
            r_hi: 3.0,
        };
        let s = Sphere { c: DVec3::new(10.0, 0.0, 0.0), r: 2.0 };
        let g = distance(&cyl, &s);
        assert!((g.distance - 5.0).abs() < 1e-9, "侧面距离应为 10−3−2，实际 {}", g.distance);
        assert_optimal(&cyl, &s, &g, "cyl-sphere");
    }

    #[test]
    fn cylinder_end_to_sphere_is_analytic() {
        // 轴向上：球在 z=+20，圆柱顶面在 z=+10，边缘半径 3
        let cyl = Cone { c: DVec3::ZERO, axis: DVec3::Z, half_h: 10.0, r_lo: 3.0, r_hi: 3.0 };
        let s = Sphere { c: DVec3::new(0.0, 0.0, 20.0), r: 2.0 };
        let g = distance(&cyl, &s);
        // 球心在轴上 → 最近点是顶面圆心，距离 20−10−2 = 8
        assert!((g.distance - 8.0).abs() < 1e-9, "实际 {}", g.distance);
    }

    #[test]
    fn degenerate_flat_cone_acts_as_a_disc() {
        // 平底刀的端面：half_h = 0，半径从 0 到 3。它是一个半径 3 的圆盘。
        let disc = Cone { c: DVec3::ZERO, axis: DVec3::Z, half_h: 0.0, r_lo: 0.0, r_hi: 3.0 };
        // 正下方 5 mm 处一个点球 → 圆盘底面距离 5−0（球半径 0.001）
        let s = Sphere { c: DVec3::new(0.0, 0.0, -5.0), r: 0.001 };
        let g = distance(&disc, &s);
        assert!(
            (g.distance - 4.999).abs() < 1e-6,
            "圆盘正下方 5 mm 的点距离应 ≈ 5，实际 {}",
            g.distance
        );
        assert!((g.point_a - DVec3::ZERO).length() < 1e-9, "最近点应是圆盘中心，实际 {:?}", g.point_a);

        // 侧面：球在 (4,0,-5)，最近点是圆盘边缘 (3,0,0) → √(1+25) − 0.001
        let s2 = Sphere { c: DVec3::new(4.0, 0.0, -5.0), r: 0.001 };
        let g2 = distance(&disc, &s2);
        let want = (1.0f64 + 25.0).sqrt() - 0.001;
        assert!((g2.distance - want).abs() < 1e-9, "实际 {}，期望 {want}", g2.distance);
    }

    #[test]
    fn flat_end_mill_decomposes_into_cones_that_cover_the_surface() {
        let prof = cncsim_tool::flat_end_mill(6.0, 20.0);
        let cones = cones_of(&prof);
        assert_eq!(cones.len(), prof.points().len() - 1);
        // 最底面必须是退化圆盘（half_h = 0），半径 3
        let base = cones[0];
        assert!(base.half_h < 1e-12, "平底刀第一段应是端面，实际 half_h={}", base.half_h);
        assert!((base.r_lo.max(base.r_hi) - 3.0).abs() < 1e-12);

        // 每个采样点都应落在某个圆台内（距离 ≤ 0）
        for q in profile_surface_samples(&prof, 1.0) {
            let mut best = f64::INFINITY;
            for c in &cones {
                best = best.min(distance(c, &Sphere { c: q, r: 0.0 }).distance);
            }
            assert!(best <= 1e-9, "母线点 {q:?} 不在任何圆台内（距离 {best}）");
        }
    }

    fn profile_surface_samples(profile: &Profile, step: f64) -> Vec<DVec3> {
        let mut out = Vec::new();
        for s in profile.points().windows(2) {
            let (a, b) = (s[0], s[1]);
            let len = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
            let n = (len / step).ceil().max(1.0) as usize;
            for i in 0..=n {
                let u = i as f64 / n as f64;
                out.push(DVec3::new(a.x + (b.x - a.x) * u, 0.0, a.y + (b.y - a.y) * u));
            }
        }
        out
    }

    #[test]
    fn random_convex_pairs_satisfy_optimality() {
        let mut rng = Rng::new();
        let prof = cncsim_tool::flat_end_mill(6.0, 20.0);
        let cones = cones_of(&prof);
        let mut checked = 0;
        let mut touching = 0;

        for i in 0..400 {
            // 随机取一段刀柄 + 一个随机的球/盒/圆柱
            let ci = (rng.next_f64() * cones.len() as f64) as usize % cones.len();
            let cone = cones[ci];
            let off = rng.vec(14.0);
            let placed = Placed::new(cone, DMat4::from_translation(off));

            match i % 3 {
                0 => {
                    let s = Sphere { c: rng.vec(14.0), r: rng.range(0.2, 3.0) };
                    let g = distance(&placed, &s);
                    assert_optimal(&placed, &s, &g, &format!("cone{ci}-sphere"));
                    if g.touching() {
                        touching += 1;
                    }
                }
                1 => {
                    let b = Box3 {
                        c: rng.vec(14.0),
                        half: DVec3::new(
                            rng.range(0.5, 4.0),
                            rng.range(0.5, 4.0),
                            rng.range(0.5, 4.0),
                        ),
                        rot: DMat3::IDENTITY,
                    };
                    let g = distance(&placed, &b);
                    assert_optimal(&placed, &b, &g, &format!("cone{ci}-box"));
                    if g.touching() {
                        touching += 1;
                    }
                }
                _ => {
                    let s = Sphere { c: rng.vec(30.0), r: rng.range(0.5, 8.0) };
                    let g = distance(&placed, &s);
                    assert_optimal(&placed, &s, &g, &format!("cone{ci}-far-sphere"));
                }
            }
            checked += 1;
        }
        assert_eq!(checked, 400);
        assert!(touching > 5, "样本里应有一部分是穿透的（实际 {touching}），否则没测到 GJK 的零距离分支");
    }

    #[test]
    fn distance_is_never_above_any_sampled_pair() {
        // 下界性质：任取 a、b 上各一点，其间距必须 ≥ GJK 距离。
        // 这条比"和 oracle 一致"弱，但它不需要 oracle，且能抓住"距离虚高"的方向。
        let mut rng = Rng::new();
        let prof = cncsim_tool::flat_end_mill(8.0, 25.0);
        let cones = cones_of(&prof);
        for i in 0..120 {
            let cone = cones[i % cones.len()];
            let placed = Placed::new(cone, DMat4::from_translation(rng.vec(12.0)));
            let b = Box3 { c: rng.vec(12.0), half: DVec3::splat(rng.range(1.0, 5.0)), rot: DMat3::IDENTITY };
            let g = distance(&placed, &b);
            for _ in 0..50 {
                let pa = placed.support(rng.unit());
                let pb = b.support(rng.unit());
                assert!(
                    (pa - pb).length() >= g.distance - 1e-9,
                    "采样点对比 GJK 距离还近：{} < {}",
                    (pa - pb).length(),
                    g.distance
                );
            }
        }
    }

    #[test]
    fn aabb_gap_is_a_lower_bound_but_distance_stays_exact() {
        let a = Sphere { c: DVec3::ZERO, r: 3.0 };
        let b = Sphere { c: DVec3::new(100.0, 0.0, 0.0), r: 2.0 };
        // 粗筛：盒间 95，和真值一致（球对球时包围盒不给额外信息）
        assert!((aabb_gap(&a, &b) - 95.0).abs() < 1e-9);
        // 精确查询：即使这么远也要给出真正的最近点
        let g = distance(&a, &b);
        assert!((g.distance - 95.0).abs() < 1e-9, "实际 {}", g.distance);
        assert!((g.point_a - DVec3::new(3.0, 0.0, 0.0)).length() < 1e-9, "最近点必须是球面上的点，不是盒面近似");
        assert!((g.point_b - DVec3::new(98.0, 0.0, 0.0)).length() < 1e-9);
    }

    /// **支撑点合法性**：`support(d)` 必须是沿 `d` 真正最远的点。
    ///
    /// 这是整套 GJK 唯一的隐含前提——最优性断言查不出这一类 bug，因为断言里的
    /// `support` 和 GJK 里的是同一个（错的）实现，两边一起错就自洽了。
    /// 必须拿一条**独立的**表面采样来验。
    ///
    /// 实测抓到的 bug：`Cone::support` 只按轴向正负挑端圆，没比较半径，
    /// 对 `r_lo ≠ r_hi` 的细长圆台返回非最远点 → 某随机用例距离虚高 0.06 mm。
    fn assert_support_is_valid(
        shape: &impl Support,
        surface: &[DVec3],
        rng: &mut Rng,
        tag: &str,
    ) {
        for _ in 0..300 {
            let d = rng.unit();
            let got = shape.support(d).dot(d);
            let mut best = f64::NEG_INFINITY;
            for &p in surface {
                best = best.max(p.dot(d));
            }
            assert!(
                got >= best - 1e-9,
                "{tag}: support({d:?}) 投影 {got} < 表面点 {best}（少了 {}）",
                best - got
            );
        }
    }

    #[test]
    fn every_shape_has_a_legal_support_map() {
        let mut rng = Rng::new();

        // 圆台：跨 r_lo / r_hi 大小关系两侧都要覆盖
        for (r_lo, r_hi, half_h) in [
            (0.4, 3.4, 8.6),
            (3.4, 0.4, 8.6),
            (2.0, 2.0, 5.0),
            (0.0, 3.0, 4.0),
            (0.0, 3.0, 0.0), // 退化：平底刀端面
        ] {
            let c = rng.vec(3.0);
            let axis = rng.unit();
            let cone = Cone { c, axis, half_h, r_lo, r_hi };
            let (pts, _) = sample_cone_surface(&cone, 0.2);
            assert_support_is_valid(&cone, &pts, &mut rng, &format!("cone({r_lo},{r_hi},{half_h})"));
        }

        // 球
        let sp = Sphere { c: rng.vec(2.0), r: 3.0 };
        let sphere_pts: Vec<DVec3> = (0..600)
            .map(|_| sp.c + rng.unit() * sp.r)
            .collect();
        assert_support_is_valid(&sp, &sphere_pts, &mut rng, "sphere");

        // 盒
        let bx = Box3 {
            c: rng.vec(2.0),
            half: DVec3::new(1.0, 2.0, 3.0),
            rot: DMat3::IDENTITY,
        };
        let box_pts: Vec<DVec3> = (0..8)
            .map(|i| {
                bx.c + DVec3::new(
                    if i & 1 == 0 { -1.0 } else { 1.0 },
                    if i & 2 == 0 { -2.0 } else { 2.0 },
                    if i & 4 == 0 { -3.0 } else { 3.0 },
                )
            })
            .collect();
        assert_support_is_valid(&bx, &box_pts, &mut rng, "box");

        // 胶囊
        let cap = Capsule { a: rng.vec(2.0), b: rng.vec(2.0), r: 1.5 };
        let mut cap_pts = Vec::new();
        for _ in 0..600 {
            let u = rng.next_f64();
            cap_pts.push(cap.a.lerp(cap.b, u) + rng.unit() * cap.r);
        }
        assert_support_is_valid(&cap, &cap_pts, &mut rng, "capsule");

        // 顶点表
        let hull = Hull {
            pts: vec![
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(5.0, 0.0, 0.0),
                DVec3::new(0.0, 3.0, 0.0),
                DVec3::new(1.0, 1.0, 4.0),
            ],
        };
        assert_support_is_valid(&hull, &hull.pts, &mut rng, "hull");
    }

    /// 盒的有符号距离（盒**外**就是精确欧氏距离，所以可直接当 oracle）
    fn sdf_box(p: DVec3, c: DVec3, half: DVec3) -> f64 {
        let q = DVec3::new(
            (p.x - c.x).abs() - half.x,
            (p.y - c.y).abs() - half.y,
            (p.z - c.z).abs() - half.z,
        );
        let out = DVec3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).length();
        out + q.x.max(q.y.max(q.z)).min(0.0)
    }

    /// 圆台表面采样（侧面 + 两端圆盘）。
    ///
    /// 这是**与 GJK 完全无关**的一条路：如果 GJK 的单纯形逻辑写错了，这里会红。
    fn sample_cone_surface(c: &Cone, step: f64) -> (Vec<DVec3>, f64) {
        let mut pts = Vec::new();
        let mut gap = 0.0f64;
        let e1 = if c.axis.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
        let e1 = (e1 - c.axis * e1.dot(c.axis)).normalize();
        let e2 = c.axis.cross(e1);

        // 侧面
        let slant = ((c.r_hi - c.r_lo).powi(2) + (2.0 * c.half_h).powi(2)).sqrt();
        let nu = (slant / step).ceil().max(1.0) as usize;
        gap = gap.max(slant / nu as f64);
        let r_max = c.r_lo.max(c.r_hi);
        let nv = (core::f64::consts::TAU * r_max / step).ceil().max(8.0) as usize;
        gap = gap.max(core::f64::consts::TAU * r_max / nv as f64);
        for i in 0..=nu {
            let u = i as f64 / nu as f64;
            let r = c.r_lo + (c.r_hi - c.r_lo) * u;
            let z = -c.half_h + 2.0 * c.half_h * u;
            for k in 0..nv {
                let a = core::f64::consts::TAU * k as f64 / nv as f64;
                pts.push(c.c + c.axis * z + (e1 * a.cos() + e2 * a.sin()) * r);
            }
        }
        // 两端圆盘（退化圆台两端重合，加一次就够）
        for (z, r, dup) in [(-c.half_h, c.r_lo, false), (c.half_h, c.r_hi, c.half_h > 1e-12)] {
            if dup || r == 0.0 {
                let _ = dup;
            }
            let nr = (r / step).ceil().max(1.0) as usize;
            for ir in 0..=nr {
                let rr = r * ir as f64 / nr as f64;
                let nv2 = (core::f64::consts::TAU * rr / step).ceil().max(8.0) as usize;
                for k in 0..nv2 {
                    let a = core::f64::consts::TAU * k as f64 / nv2 as f64;
                    pts.push(c.c + c.axis * z + (e1 * a.cos() + e2 * a.sin()) * rr);
                }
            }
            gap = gap.max(step);
        }
        (pts, gap)
    }

    #[test]
    fn gjk_matches_dense_surface_sampling() {
        // 圆台表面密集采样 → 每个点到盒的解析距离，取最小 = 真实距离的**上界**。
        // GJK 必须落在这个上界与「上界 − 采样间隙」之间。
        let mut rng = Rng::new();
        let mut worst_lo = 0.0f64;
        let mut worst_hi = 0.0f64;
        let mut checked = 0;

        for _ in 0..40 {
            let cone = Cone {
                c: rng.vec(6.0),
                axis: rng.unit(),
                half_h: rng.range(1.0, 12.0),
                r_lo: rng.range(0.0, 4.0),
                r_hi: rng.range(0.0, 4.0),
            };
            let bx = Box3 {
                c: rng.vec(14.0),
                half: DVec3::new(rng.range(1.0, 5.0), rng.range(1.0, 5.0), rng.range(1.0, 5.0)),
                rot: DMat3::IDENTITY,
            };
            let g = distance(&cone, &bx);
            if g.touching() {
                continue; // 穿透时采样法给不出深度，跳过
            }

            let (pts, gap) = sample_cone_surface(&cone, 0.25);
            let sampled = pts
                .iter()
                .map(|&p| sdf_box(p, bx.c, bx.half).max(0.0))
                .fold(f64::INFINITY, f64::min);

            // GJK ≤ 采样上界
            worst_hi = worst_hi.max(g.distance - sampled);
            assert!(
                g.distance <= sampled + 1e-9,
                "GJK {} 超过采样上界 {sampled}（说明距离虚高）
cone={cone:?}
box={bx:?}
iters={} pa={:?} pb={:?}",
                g.distance, g.iterations, g.point_a, g.point_b
            );
            // GJK ≥ 采样上界 − 间隙（1-Lipschitz）
            worst_lo = worst_lo.max(sampled - gap - g.distance);
            assert!(
                g.distance >= sampled - gap - 1e-9,
                "GJK {} 低于采样下界 {}（说明距离虚低）",
                g.distance,
                sampled - gap
            );
            checked += 1;
        }
        println!("采样对拍 {checked} 例：GJK−上界 ≤ {worst_hi:.2e}，下界−GJK ≤ {worst_lo:.2e}");
        assert!(checked > 20, "有效样本太少（{checked}）");
    }

    #[test]
    fn capsule_vs_box_matches_analytic() {
        // 竖直胶囊半径 2，盒在 x 方向 10 mm 外：距离 = 10 − 1 − 2 = 7
        let cap = Capsule { a: DVec3::new(0.0, 0.0, -5.0), b: DVec3::new(0.0, 0.0, 5.0), r: 2.0 };
        let b = Box3::aabb_shape(DVec3::new(10.0, 0.0, 0.0), DVec3::splat(1.0));
        assert!((distance(&cap, &b).distance - 7.0).abs() < 1e-9);
    }

    #[test]
    fn hull_support_picks_extreme_vertex() {
        let h = Hull {
            pts: vec![
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(3.0, 0.0, 0.0),
                DVec3::new(0.0, 4.0, 0.0),
                DVec3::new(0.0, 0.0, 5.0),
            ],
        };
        assert_eq!(h.support(DVec3::X), DVec3::new(3.0, 0.0, 0.0));
        assert_eq!(h.support(DVec3::Y), DVec3::new(0.0, 4.0, 0.0));
        assert_eq!(h.support(DVec3::Z), DVec3::new(0.0, 0.0, 5.0));
        let s = Sphere { c: DVec3::new(0.0, 0.0, 20.0), r: 1.0 };
        assert!((distance(&h, &s).distance - 14.0).abs() < 1e-9);
    }

    #[test]
    fn rotated_box_uses_its_own_axes() {
        // 45° 绕 Z 的盒：半宽 (1,1,1) 旋转后 AABB 半宽是 √2
        let ang = std::f64::consts::FRAC_PI_4;
        let rot = DMat3::from_cols(
            DVec3::new(ang.cos(), ang.sin(), 0.0),
            DVec3::new(-ang.sin(), ang.cos(), 0.0),
            DVec3::Z,
        );
        let b = Box3 { c: DVec3::ZERO, half: DVec3::splat(1.0), rot };
        let (lo, hi) = b.aabb();
        assert!((hi.x - 2.0f64.sqrt()).abs() < 1e-12, "AABB 半宽应 √2，实际 {}", hi.x);
        let _ = lo;

        // 沿 +X 的球：到旋转盒的距离 = 8 − 1（盒沿 X 的支撑是顶点，但顶点 x 投影 = √2）
        let s = Sphere { c: DVec3::new(8.0, 0.0, 0.0), r: 1.0 };
        let g = distance(&b, &s);
        assert!((g.distance - (7.0 - 2.0f64.sqrt())).abs() < 1e-9, "实际 {}", g.distance);
    }
}
