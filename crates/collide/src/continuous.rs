//! 连续碰撞：**保守推进**（conservative advancement）+ 刀柄表面采样。
//!
//! 这是 [ADR-004](../../../docs/ADR-004-碰撞检查方案.md) §2.2 / §3 的第一片实现：
//! 窄相 A（刀柄表面 × 毛坯 SDF）+ 连续化。凸体 × 凸体的 GJK 是下一片。
//!
//! ## 为什么必须连续化
//!
//! 100 m/min 快移 = 1.67 mm/ms。薄壁、压板边缘、转台挡边都能薄到 1–2 mm。
//! **按固定时间步采样，2 mm 的薄壁会被整步跳过**——报"无碰撞"而实际撞了。
//!
//! 一个漏报的碰撞检测比没有碰撞检测更危险，因为它给了虚假的安全感。
//!
//! ## 保守推进为什么不会漏
//!
//! 设 `d(t)` 是位姿 `t` 下刀具到材料的距离**下界**，`v` 是这一段里**任意点**的最大速度。
//! 那么 `t + d(t)/v` 之前**不可能**发生碰撞（走完这段距离都要这么久）。
//!
//! ```text
//! t ← t + max(d(t)/v, 下限)
//! ```
//!
//! 只要 `d` 是下界、`v` 是上界，步长就永远偏小 → 只会迟到、不会跳过。
//! 迭代耗尽时**保守地报碰撞**：误报能人工排除，漏报不能。
//!
//! ## `d` 怎么变成"下界"
//!
//! 两个来源，都要减掉：
//!
//! | 来源 | 量级 | 为什么要减 |
//! |---|---|---|
//! | 表面采样间隙 `δ` | 由 `dz / dφ` 定 | 采样点是**子集**，最靠近材料的那个点可能没被采到；`sdf` 是 1-Lipschitz，所以真实距离 ≥ 采样最小值 − δ |
//! | 体素量化 + 三线性重建 | 平面 1.6 µm / 曲面 2.0 µm | 见《计算精度分析》§2 |
//!
//! 减掉之后 `d` 才是**严格下界**，CA 的不漏报才成立。**代价**是保守推进会在
//! 距离 `ε` 处就停下报碰撞——这也是 ADR-004 要求"报出间隙"的原因。
//!
//! ## 检出下限
//!
//! 方向 B 的覆盖保证由**采样间隙**决定：穿透区是刀具表面上的二维补丁，深度 `ε`，
//! 间隙 `δ` 的采样能发现它的条件是 `ε > δ²/(2R)`（和 ADR-004 §2.5 同一个式子）。
//!
//! | δ | 采样点数（φ6×20 刀柄） | 检出下限（R=3） |
//! |---|---|---|
//! | 0.2 mm | ≈ 22 000 | 6.7 µm |
//! | 0.5 mm | ≈ 3 500 | 42 µm |
//! | 1.0 mm | ≈ 880 | 167 µm |
//!
//! 默认 0.5 mm：42 µm 对"撞没撞"够了，而 3 500 次 `sdf` 约 0.2 ms——
//! 只在 AABB 相交时才跑（ADR-004 §2.2 实测占比 < 5%）。要更细就传更小的 δ。

use crate::DistanceField;
use cncsim_math::DVec3;
use cncsim_motion::{Pose, slerp};
use cncsim_tool::Profile;

/// 体素 SDF 的符号可信度下限（平面 1.6 µm、曲面 2.0 µm 取大者）。
/// 见 `docs/计算精度分析.md` §2。
const FIELD_ERROR_MM: f64 = 0.002;

/// 刀具表面采样点（刀具局部坐标，`z` 沿刀轴）+ 采样间隙。
pub struct SurfaceBody {
    /// 局部坐标点；世界坐标 = `tip + e1·x + e2·y + axis·z`
    local: Vec<DVec3>,
    /// 相邻采样点的最大间隙（mm）——下界要减掉它
    gap: f64,
    /// 离刀尖最远的采样点距离（用于推速度上界）
    reach: f64,
}

impl SurfaceBody {
    /// 按母线生成旋转面采样。
    ///
    /// `dz` 是沿母线的步长，`dphi` 是周向弧长步长。实际间隙取 `max(dz, dphi)`。
    pub fn revolve(profile: &Profile, dz: f64, dphi: f64) -> Self {
        let dz = dz.max(1e-3);
        let dphi = dphi.max(1e-3);
        let mut local = Vec::new();
        let mut gap = 0.0f64;
        let mut reach = 0.0f64;

        let pts = profile.points();
        for seg in pts.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            let len = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
            let steps = (len / dz).ceil().max(1.0) as usize;
            for i in 0..=steps {
                let u = i as f64 / steps as f64;
                let r = a.x + (b.x - a.x) * u;
                let z = a.y + (b.y - a.y) * u;
                // 实际步长可能大于 dz（段长不是 dz 的整数倍）
                gap = gap.max(len / steps as f64);
                if r < 1e-9 {
                    // 轴上的点：一个点就够（转一圈还是它）
                    local.push(DVec3::new(0.0, 0.0, z));
                    reach = reach.max(z.abs());
                    continue;
                }
                let n = ((core::f64::consts::TAU * r / dphi).ceil() as usize).max(4);
                for k in 0..n {
                    let ang = core::f64::consts::TAU * k as f64 / n as f64;
                    local.push(DVec3::new(r * ang.cos(), r * ang.sin(), z));
                }
                gap = gap.max(core::f64::consts::TAU * r / n as f64);
                reach = reach.max((r * r + z * z).sqrt());
            }
        }
        Self { local, gap, reach }
    }

    pub fn len(&self) -> usize {
        self.local.len()
    }
    pub fn is_empty(&self) -> bool {
        self.local.is_empty()
    }
    /// 采样间隙（mm）
    pub fn gap(&self) -> f64 {
        self.gap
    }
    /// 最远采样点到刀尖的距离（推速度上界用）
    pub fn reach(&self) -> f64 {
        self.reach
    }

    /// 采样点集在 `from → to` 扫掠过程中的**世界包围盒**。
    ///
    /// 给上层做粗筛用：刀杆离毛坯十万八千里时，一次盒子比较就能跳过整段窄相。
    /// 只含 `from` 与 `to` 两个位姿——中间位姿是插值出来的，位置在两个端点之间，
    /// 姿态插值也不会把点甩到两端包围盒之外（球面插值的最远点在端点）。
    pub fn aabb(&self, from: &Pose, to: &Pose) -> (DVec3, DVec3) {
        let mut lo = DVec3::splat(f64::INFINITY);
        let mut hi = DVec3::splat(f64::NEG_INFINITY);
        for pose in [from, to] {
            let (e1, e2) = frame(pose.axis);
            for q in &self.local {
                let p = pose.tip + e1 * q.x + e2 * q.y + pose.axis * q.z;
                lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
                hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
            }
        }
        (lo, hi)
    }
}

/// 位姿插值：位置线性、姿态球面。与 `voxel::lerp_pose` 同一个定义。
fn lerp_pose(a: &Pose, b: &Pose, t: f64) -> Pose {
    Pose {
        tip: a.tip.lerp(b.tip, t),
        axis: slerp(a.axis, b.axis, t),
    }
}

/// 由刀轴构造正交基：`axis` 与 `e1 / e2` 右手系。
fn frame(axis: DVec3) -> (DVec3, DVec3) {
    // 选一个与 axis 不平行的参考向量
    let helper = if axis.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    let e1 = axis.cross(helper).normalize();
    let e2 = axis.cross(e1);
    (e1, e2)
}

/// 位姿下**最接近材料的采样点**到材料的距离。
///
/// 注意这是**上界**：采样点是刀具表面的子集，最靠近材料的那个点可能没被采到。
/// 判定"撞没撞"用它的符号——`< 0` 说明确有采样点进了材料，**不会误报**。
pub fn clearance(field: &impl DistanceField, body: &SurfaceBody, pose: &Pose) -> f64 {
    sample_stats(field, body, pose).0
}

/// 一次遍历同时拿到：最小 sdf + 采样点的世界包围盒。
///
/// 两样都要，而且都是 O(n)；分两次遍历等于把最贵的那部分做两遍。
fn sample_stats(
    field: &impl DistanceField,
    body: &SurfaceBody,
    pose: &Pose,
) -> (f64, DVec3, DVec3) {
    let (e1, e2) = frame(pose.axis);
    let mut m = f64::INFINITY;
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for q in &body.local {
        let p = pose.tip + e1 * q.x + e2 * q.y + pose.axis * q.z;
        m = m.min(field.sdf(p));
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (m, lo, hi)
}

/// 距离**下界**：真实距离 ≥ 它。保守推进用它算安全步长。
///
/// 两条下界取大：
///
/// 1. **采样下界**：`min sdf − δ − ε`。1-Lipschitz 给出"真实距离 ≥ 采样最小值 − 间隙"。
/// 2. **包围盒下界**：采样点全在其世界包围盒里，所以真实距离 ≥ 「采样点包围盒」到
///    「场包围盒」的距离。
///
/// 为什么必须加第 2 条：体素场的 `sdf` 在窄带外被钳制成 `+band`（默认 0.4 mm），
/// 而 δ 默认 0.5 mm。于是第 1 条恒为 `−0.1`，**任何位置都"无法保证"**——
/// 保守推进退化成每段都报碰撞（实测：刀在毛坯上方 40 mm 空走也会报）。
///
/// 试过用"刀尖球"（半径 = `reach`，O(1)）兜底：φ6×40 的刀杆要包在半径 40 的球里，
/// 球一靠近就是 0，等于没兜。改成采样点包围盒——同一趟遍历顺手拿到，紧得多。
pub fn clearance_lower(field: &impl DistanceField, body: &SurfaceBody, pose: &Pose) -> f64 {
    stats_of(field, body, pose).1
}

/// 一次遍历拿到**两个**数：`(clearance 上界, clearance_lower 下界)`。
///
/// 分开算会把最贵的遍历做两遍，而且容易出错——早版本在保守推进里用
/// `lower + gap + err` 反推上界，得到的是个**假的正数**（下界被包围盒抬到 0 时，
/// 反推回来变成 +0.5），碰撞深度于是永远报 0。
fn stats_of(field: &impl DistanceField, body: &SurfaceBody, pose: &Pose) -> (f64, f64) {
    let (m, blo, bhi) = sample_stats(field, body, pose);
    let by_sample = m - body.gap - FIELD_ERROR_MM;
    let lower = match field.aabb() {
        Some((lo, hi)) => by_sample.max(aabb_gap(blo, bhi, lo, hi)),
        None => by_sample,
    };
    (m, lower)
}

/// 两个轴对齐盒之间的最短距离（相交时为 0）。
fn aabb_gap(alo: DVec3, ahi: DVec3, blo: DVec3, bhi: DVec3) -> f64 {
    let mut s = 0.0;
    for i in 0..3 {
        let d = (blo.component(i) - ahi.component(i)).max(alo.component(i) - bhi.component(i));
        if d > 0.0 {
            s += d * d;
        }
    }
    s.sqrt()
}

/// 一次碰撞。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    /// 段内参数 0..1
    pub t: f64,
    /// 最接近材料的采样点的距离（**上界**）。`≤ 0` 是确切穿透。
    pub clearance: f64,
    /// 距离下界。它 `≤ 0` 而 `clearance > 0` 表示"保守报碰撞"：
    /// 采样间隙 + 场误差之内无法保证安全。
    pub clearance_lower: f64,
}

impl Hit {
    /// 是否只是"太近，无法保证"，而不是确切穿透。
    pub fn near_miss(&self) -> bool {
        self.clearance > 0.0
    }
}

/// 该段内**任意点**的最大速度上界（mm / 段参数单位）。
///
/// `|Δtip| + θ·r_max`：平移项 + 转动项。转动项不能省——五轴摆动时刀柄末端
/// 的速度可能远大于刀尖（`|ω × r|`），用刀尖速度当上界会漏报。
pub fn speed_bound(from: &Pose, to: &Pose, reach: f64) -> f64 {
    let v = (to.tip - from.tip).length();
    let cos = from.axis.dot(to.axis).clamp(-1.0, 1.0);
    let theta = cos.acos();
    v + theta * reach
}

/// 在 `[0, 1]` 段内找最早碰撞。
///
/// 迭代耗尽时返回 `Some(Hit{t: 当前位置})`——**保守报碰撞**。
/// 返回 `None` 表示这一段安全跨过。
pub fn earliest_collision(
    field: &impl DistanceField,
    body: &SurfaceBody,
    from: &Pose,
    to: &Pose,
) -> Option<Hit> {
    const MAX_STEPS: u32 = 64;
    /// 步长的绝对下限（段参数）：防止"距离恒为 ε、v 很大"时退化成死循环
    const MIN_STEP: f64 = 1e-5;

    let v = speed_bound(from, to, body.reach());
    if v <= 0.0 {
        // 位姿没变：静态判定
        let (c, lo) = stats_of(field, body, from);
        return (lo <= 0.0).then_some(Hit { t: 0.0, clearance: c, clearance_lower: lo });
    }

    let mut t = 0.0f64;
    for _ in 0..MAX_STEPS {
        let pose = lerp_pose(from, to, t);
        // 步长用**下界**：真实距离 ≥ lower，所以 lower/v 之前不可能撞
        let (c, lo) = stats_of(field, body, &pose);
        if lo <= 0.0 {
            return Some(Hit { t, clearance: c, clearance_lower: lo });
        }
        let dt = lo / v;
        if t + dt >= 1.0 {
            return None; // 这一段跨完了
        }
        t += dt.max(MIN_STEP);
    }
    // 迭代耗尽：保守报碰撞
    let pose = lerp_pose(from, to, t);
    let (c, lo) = stats_of(field, body, &pose);
    Some(Hit { t, clearance: c, clearance_lower: lo })
}

/// **反例**：只查两端点的采样器。留着是为了让测试能证明它真的会漏。
pub fn endpoints_only(
    field: &impl DistanceField,
    body: &SurfaceBody,
    from: &Pose,
    to: &Pose,
) -> Option<Hit> {
    for t in [0.0, 1.0] {
        let c = clearance(field, body, &lerp_pose(from, to, t));
        if c <= 0.0 {
            return Some(Hit { t, clearance: c, clearance_lower: c });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_tool::flat_end_mill;

    /// 半空间 `z ≤ 0`（材料在下面）
    struct Floor;
    impl DistanceField for Floor {
        fn sdf(&self, p: DVec3) -> f64 {
            p.z
        }
        fn band(&self) -> f64 {
            f64::INFINITY
        }
    }

    /// 厚 2 mm 的板：`|z| ≤ 1` 是材料，两侧是空气。
    struct Slab;
    impl DistanceField for Slab {
        fn sdf(&self, p: DVec3) -> f64 {
            p.z.abs() - 1.0
        }
        fn band(&self) -> f64 {
            f64::INFINITY
        }
    }

    /// 厚 2 mm 的竖墙：`|x| ≤ 1` 是材料。
    ///
    /// 用它做"高速穿越"的样本：φ6 的刀在 x=±8 处离墙都远，中段才穿过——
    /// 这正是端点采样会漏、而保守推进必须抓到的形状。
    struct Wall;
    impl DistanceField for Wall {
        fn sdf(&self, p: DVec3) -> f64 {
            p.x.abs() - 1.0
        }
        fn band(&self) -> f64 {
            f64::INFINITY
        }
    }

    fn body() -> SurfaceBody {
        // 刀柄：把整把 φ6×20 都当刀柄，方便算
        SurfaceBody::revolve(&flat_end_mill(6.0, 20.0), 1.0, 1.0)
    }

    #[test]
    fn body_covers_the_surface_with_a_known_gap() {
        let b = body();
        assert!(b.len() > 100, "采样点太少：{}", b.len());
        assert!(b.gap() <= 1.05, "间隙应≈1 mm，实际 {}", b.gap());
        assert!(b.reach() > 20.0, "最远点应超过 20 mm");
    }

    #[test]
    fn clearance_is_minus_reach_above_a_floor() {
        // 刀尖在 z=100、轴向上：最低点是刀尖，到地板 100
        let b = body();
        let pose = Pose { tip: DVec3::new(0.0, 0.0, 100.0), axis: DVec3::Z };
        let c = clearance(&Floor, &b, &pose);
        assert!((c - 100.0).abs() < 1e-9, "采样最近点应在刀尖，实际 {c}");
        let lo = clearance_lower(&Floor, &b, &pose);
        assert!((lo - (100.0 - b.gap() - FIELD_ERROR_MM)).abs() < 1e-9, "下界实际 {lo}");
    }

    #[test]
    fn head_on_approach_hits_when_the_tip_reaches_the_floor() {
        let b = body();
        // 从 z=+10 到 z=−10：刀尖在 z=0 处接触 → t = 0.5
        let from = Pose { tip: DVec3::new(0.0, 0.0, 10.0), axis: DVec3::Z };
        let to = Pose { tip: DVec3::new(0.0, 0.0, -10.0), axis: DVec3::Z };
        let hit = earliest_collision(&Floor, &b, &from, &to).expect("必须报碰撞");
        assert!(hit.t > 0.3 && hit.t < 0.6, "接触应在 t≈0.5，实际 {}", hit.t);
    }

    #[test]
    fn fast_pass_through_a_thin_wall_is_caught() {
        // 横穿 2 mm 竖墙：x 从 −8 走到 +8，一步跨 16 mm（100 m/min 的 10 ms）
        let b = body();
        let from = Pose { tip: DVec3::new(-8.0, 0.0, 10.0), axis: DVec3::Z };
        let to = Pose { tip: DVec3::new(8.0, 0.0, 10.0), axis: DVec3::Z };

        assert!(
            endpoints_only(&Wall, &b, &from, &to).is_none(),
            "这个用例的意义就是「端点采样会漏」——如果它报了，说明样本不再有代表性"
        );
        let hit = earliest_collision(&Wall, &b, &from, &to).expect("保守推进必须抓到");
        assert!(hit.t > 0.0 && hit.t < 1.0, "碰撞应在段内，实际 t={}", hit.t);
        // 刀半径 3、墙在 x∈[−1,1]：刀尖走到 x = −4 时表面刚好碰到墙
        assert!((hit.t - (8.0 - 4.0) / 16.0).abs() < 0.12, "接触点应在 t≈0.25，实际 {}", hit.t);
    }

    #[test]
    fn passing_above_the_wall_reports_nothing() {
        let b = body();
        // 刀尖在 z=+3（板顶在 z=+1）横着走：最低点离板 2 mm
        let from = Pose { tip: DVec3::new(-5.0, 0.0, 3.0), axis: DVec3::Z };
        let to = Pose { tip: DVec3::new(5.0, 0.0, 3.0), axis: DVec3::Z };
        let c = clearance(&Slab, &b, &from);
        assert!(c > 0.0, "起点应是安全的，实际 {c}");
        assert!((c - 2.0).abs() < 1e-6, "离板顶应正好 2 mm，实际 {c}");
        // 保守推进会因为保守的下界报"接近"，但不该报穿透
        if let Some(h) = earliest_collision(&Slab, &b, &from, &to) {
            assert!(h.t > 0.0, "不该在起点就报");
        }
    }

    #[test]
    fn already_inside_is_reported_at_t0() {
        let b = body();
        let pose = Pose { tip: DVec3::new(0.0, 0.0, 0.0), axis: DVec3::Z };
        let hit = earliest_collision(&Floor, &b, &pose, &pose).expect("静态穿透要报");
        assert_eq!(hit.t, 0.0);
        assert!(hit.clearance <= 0.0);
    }

    #[test]
    fn speed_bound_includes_rotation_term() {
        // 只转不平移：速度上界必须 > 0，否则保守推进的步长会无穷大
        let a = Pose { tip: DVec3::ZERO, axis: DVec3::Z };
        let b = Pose {
            tip: DVec3::ZERO,
            axis: DVec3::new(0.0, 0.5, 0.866_025_403_784_438_6),
        };
        let v = speed_bound(&a, &b, 20.0);
        assert!(v > 1.0, "纯转动也得有速度上界，实际 {v}");
        // (0, 0.5, 0.866) 距 +Z 是 30°，弧长 = π/6 × 20
        assert!((v - std::f64::consts::FRAC_PI_6 * 20.0).abs() < 1e-9, "实际 {v}");
    }
}
