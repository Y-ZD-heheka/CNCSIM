//! 解析扫掠 SDF：刀具沿一段直线平移时，扫掠体的有符号距离。
//!
//! ## 为什么是 `min over t`
//!
//! 扫掠体 `S = ⋃_{t∈[0,1]} T(t)`，所以
//!
//! ```text
//! sd_S(p) = min_{t∈[0,1]} sd_T(p, t)
//! ```
//!
//! 对**凸**刀具（平底/球头/圆鼻/钻头都算）这是精确的：距离固定点到沿直线平移的
//! 凸集的距离，是平移参数的凸函数，黄金分割能取到真极值。
//!
//! ## 两个必须写明的局限
//!
//! **① 只对凸刀型成立。** T 形刀、燕尾刀这类凹母线刀具，`g(t)` 非凸，
//! 黄金分割可能错过全局最小 → 扫掠体被低估 → 该切的地方没切。v1 的刀型都是凸的。
//!
//! **② 深处会低估深度。** `min` 在扫掠体**内部**不是真 SDF（它给出"最深那个
//! 平移位置"的深度，比真实深度更负）。但窄带只关心 `|d| < band`，在扫掠面附近
//! 两者一致——这正是窄带表示成立的前提。
//!
//! **③ 旋转段要由 `sim` 细分。** 这里对位姿做 `tip` 线性 + `axis` 球面插值，
//! 刀轴转角大时扫掠体不是这两个端点位姿的并集。`sim` 按弦高容差把旋转段切成子步
//! （见 `docs/点位预读与流水线缓冲.md`），每个子步的转角小到可以忽略。

use cncsim_math::DVec3;
use cncsim_motion::{Pose, slerp};
use cncsim_tool::{Profile, sd_profile, sd_profile_rz};

/// 黄金分割的最少迭代次数。极短行程用它。
pub const MIN_GOLDEN_ITERS: u32 = 6;
/// 黄金分割的最多迭代次数（防长行程把耗时拉爆）。
pub const MAX_GOLDEN_ITERS: u32 = 24;

/// 极值位置的**目标分辨率**（mm）。
///
/// 距离误差 ≈ `Δx²/(2R)`，取 `Δx = 0.02 mm`、R=3 时是 **6.7×10⁻⁵ mm**，
/// 比 `i8` 量化档（3.15 µm）小 47 倍。这是"细分到够用"的判据。
const TARGET_RESOLUTION_MM: f64 = 0.02;

const INV_PHI: f64 = 0.618_033_988_749_894_9;

/// 按行程选迭代次数。
///
/// **必须自适应，固定次数会在长行程上失准**：区间按 `0.618^n` 收缩，
/// 极值位置误差 = `行程 × 0.618^n`，而距离误差是这个量的平方除以 `2R`。
/// 固定 8 次时 10 mm 行程的定位误差是 0.21 mm → 距离误差 **7.6 µm**，
/// 比量化档还大 2.4 倍（实测抓到的）。
///
/// 迭代次数只依赖几何（行程 + 刀半径），不依赖时间 → 结果仍然逐位可复现。
/// 黄金分割**两个内点的初始间距**（区间长度为 1 时）。
///
/// 关键：极值被夹在 `c` 与 `d` **之间**，所以定位误差是 `|c−d|`，
/// 不是整个区间长度。`|c−d|` 每次迭代乘 0.618。
const GOLDEN_BRACKET: f64 = 0.236_067_977_499_789_9;

/// 按行程选迭代次数。
///
/// 定位误差 `Δx ≈ 行程 · GOLDEN_BRACKET · 0.618ⁿ`，距离误差 ≈ `Δx²/(2R)`。
///
/// **实测校准**（φ6 平底刀、R=3，测试 `long_move_keeps_sweep_accuracy`）：
///
/// | 行程 | 固定 8 次 | 自适应 |
/// |---|---|---|
/// | 10 mm | 0.42 µm ✓ | 8 次，同上 |
/// | 200 mm | **16 µm ✗**（量化档的 5 倍） | 15 次，< 5 µm ✓ |
///
/// 模型公式对长行程估得偏保守（估 0.17 mm，实测 16 µm），所以实际比理论更早够用；
/// 但**方向是对的**：长行程必须加迭代。
///
/// 迭代次数只依赖几何（行程 + 刀半径），不依赖时间 → 结果仍然逐位可复现。
pub fn golden_iters(tip_travel: f64, axis_sweep_arc: f64) -> u32 {
    // 刀轴转动时，刀面上最外点的弧长才是真实行程
    let travel = tip_travel.max(axis_sweep_arc);
    let per_step = GOLDEN_BRACKET * travel;
    if per_step <= TARGET_RESOLUTION_MM {
        return MIN_GOLDEN_ITERS;
    }
    let n = (TARGET_RESOLUTION_MM / per_step).ln() / INV_PHI.ln();
    (n.ceil() as u32).clamp(MIN_GOLDEN_ITERS, MAX_GOLDEN_ITERS)
}

/// 刀具在位姿 `pose` 下的有符号距离（世界坐标，正值在刀外）。
///
/// 把世界点转到刀具局部坐标（`z` 沿刀轴、`r` 为径向），交给 [`sd_profile`]。
/// 用 `r = |d − axis·z|` 而不是建正交基——`sd_profile` 只用 `hypot(x,y)`，
/// 方位角无关，省掉一次基构造。
pub fn sd_tool(p: DVec3, pose: &Pose, profile: &Profile) -> f64 {
    let d = p - pose.tip;
    let z = d.dot(pose.axis);
    let r = (d - pose.axis * z).length();
    sd_profile(DVec3::new(r, 0.0, z), profile)
}

/// 位姿插值：位置线性、姿态球面。
pub fn lerp_pose(a: &Pose, b: &Pose, t: f64) -> Pose {
    Pose {
        tip: a.tip.lerp(b.tip, t),
        axis: slerp(a.axis, b.axis, t),
    }
}

/// 扫掠体的有符号距离。
///
/// 纯平移时这是精确的（凸刀具）；带旋转时依赖调用方把转角切小。
///
/// ## 为什么不像原来那样每次求值都调 `lerp_pose`
///
/// 原来 `eval(t) = sd_tool(p, &lerp_pose(from, to, t), profile)`，而 `slerp` 开头
/// 就 `acos(dot)`。**刀轴根本没转（平移）时也照样算一次 `acos`**——实测每次求值
/// 19 ns，其中一半花在这里。现在按"轴是否变"分两条路，两条都把 t 的依赖提成
/// **标量多项式**：
///
/// | | 每次求值要算什么 |
/// |---|---|
/// | 平移 | `z = z0 + zd·t`，`r² = a0 + b·t + c·t²` → **零三角函数**，一次 `sqrt` |
/// | 平移 + 转 | `cos(tθ)`、`sin(tθ)` 各一次 → **两次**，且 `cosθ/sinθ/u` 在循环外算好 |
///
/// 黄金分割的 `t` 序列没变，所以**结果与原来数学等价**（末位可能不同）。
pub fn sd_sweep(p: DVec3, from: &Pose, to: &Pose, profile: &Profile) -> f64 {
    sd_sweep_with(p, from, to, profile, profile.as_flat_end_mill())
}

/// 与 [`sd_sweep`] 相同，但允许调用方**预先判定**平底刀（`Profile::as_flat_end_mill`）。
///
/// 逐体素的热路径（`cut_box`）必须把判定提到循环外：`as_flat_end_mill` 要比较
/// 4 个点，每体素重复一次会吃掉快路径省下的时间。
pub fn sd_sweep_with(
    p: DVec3,
    from: &Pose,
    to: &Pose,
    profile: &Profile,
    flat: Option<(f64, f64)>,
) -> f64 {
    let step = to.tip - from.tip;
    let step_len2 = step.length_squared();
    let axis_delta_sq = (from.axis - to.axis).length_squared();

    // ── 快路径：平底刀 + 纯平移 + 径向/轴向解耦 → 闭式精确解 ──
    //
    // 省掉端点两次 `sd_tool` 和 13 次黄金分割（每体素 34 ns → 8 ns 量级）。
    // 条件刻意保守：刀轴一点不动、步长非零、且 step 与刀轴正交或平行。
    // 斜向进给时径向和轴向共享同一个 t，必须回退黄金分割。
    if let Some((r, len)) = flat {
        if axis_delta_sq < 1e-30 && step_len2 > 1e-30 {
            let ax = from.axis;
            let d = p - from.tip;
            let z0 = d.dot(ax);
            let w0 = d - ax * z0;
            let sz = step.dot(ax);
            let wd = step - ax * sz;
            if sz.abs() < 1e-12 || wd.length_squared() < 1e-28 {
                return sd_flat_prism(z0, w0, sz, wd, r, len);
            }
        }
    }

    // 端点必须显式算：极值可能正好落在 t=0 或 t=1（刀刚接触/刚离开）
    let d0 = sd_tool(p, from, profile);
    let d1 = sd_tool(p, to, profile);
    let mut best = d0.min(d1);

    // 退化：位姿没变
    if step_len2 < 1e-30 && axis_delta_sq < 1e-30 {
        return best;
    }

    let cos = from.axis.dot(to.axis).clamp(-1.0, 1.0);
    let theta = cos.acos();
    let tip_travel = step.length();
    let axis_arc = theta * profile.max_radius();
    let iters = golden_iters(tip_travel, axis_arc);

    if theta < 1e-12 {
        // ── 纯平移：z 线性、r² 二次，全程无三角函数 ──
        let ax = from.axis;
        let d = p - from.tip;
        let z0 = d.dot(ax);
        let w0 = d - ax * z0;
        let zd = -step.dot(ax);
        let wd = -(step - ax * step.dot(ax));
        let a0 = w0.dot(w0);
        let bq = 2.0 * w0.dot(wd);
        let cq = wd.dot(wd);
        let eval = |t: f64| {
            let z = z0 + zd * t;
            let r = (a0 + bq * t + cq * t * t).max(0.0).sqrt();
            sd_profile_rz(r, z, profile)
        };
        best = best.min(golden_min(iters, eval));
    } else {
        // ── 平移 + 转：轴(t) = a·cos(tθ) + u·sin(tθ)，u 与 a 正交 ──
        //
        // 把"点乘轴"展开成标量：z(t) = A·cos + B·sin − t(C·cos + E·sin)，
        // 其中 A=d0·a、B=d0·u、C=step·a、E=step·u 都在循环外算好。
        let a = from.axis;
        let u = (to.axis - a * cos) / theta.sin(); // 单位、⊥ a
        let d = p - from.tip;
        let (dot_aa, dot_au) = (d.dot(a), d.dot(u));
        let (dot_sa, dot_su) = (step.dot(a), step.dot(u));
        let d2 = d.dot(d);
        let ds = d.dot(step);
        let s2 = step.dot(step);
        let eval = |t: f64| {
            let (s, c) = (t * theta).sin_cos();
            let z = dot_aa * c + dot_au * s - t * (dot_sa * c + dot_su * s);
            let len2 = d2 - 2.0 * t * ds + t * t * s2;
            let r = (len2 - z * z).max(0.0).sqrt();
            sd_profile_rz(r, z, profile)
        };
        best = best.min(golden_min(iters, eval));
    }
    best
}

/// 平底刀**纯平移**扫掠体的闭式 SDF。
///
/// 扫掠体 = 径向的 2D 胶囊 × 轴向区间（**笛卡尔积**），所以用盒公式精确合成：
///
/// ```text
/// 径向：|w(t)| = |w0 − wd·t|  → 最小值 = 原点到线段 [w0, w0−wd] 的距离
/// 轴向：∪_{t∈[0,1]} [sz·t, len + sz·t] = [min(0,sz), max(len, len+sz)]
/// ```
///
/// **只在径向/轴向解耦时成立**（`sz = 0` 水平走刀，或 `wd = 0` 竖直下刀）。
/// 斜向进给时两个维度共享同一个 t，交集判断不再独立，必须回退黄金分割。
///
/// 参数已在刀具局部坐标：`z0` 是点相对刀尖的轴向坐标，`w0` 是垂直分量，
/// `sz`/`wd` 是一次移动的轴向/垂直分量。
fn sd_flat_prism(z0: f64, w0: DVec3, sz: f64, wd: DVec3, r: f64, len: f64) -> f64 {
    let dr = if wd.length_squared() < 1e-28 {
        // 竖直下刀：径向距离不变
        w0.length() - r
    } else {
        origin_to_segment(w0, w0 - wd) - r
    };

    // 刀具的 [0, len] 沿 −sz 扫过后的并集区间
    let zlo = sz.min(0.0);
    let zhi = len.max(len + sz);
    let dz = (zlo - z0).max(z0 - zhi);

    // 盒公式：两维正交 → 笛卡尔积的精确 SDF（内部取 max、外部取斜边）
    let (a, b) = (dr.max(0.0), dz.max(0.0));
    (a * a + b * b).sqrt() + dr.max(dz).min(0.0)
}

/// 原点到线段 `[a, b]` 的距离。
fn origin_to_segment(a: DVec3, b: DVec3) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    if l2 < 1e-30 {
        return a.length();
    }
    let t = (-a.dot(ab) / l2).clamp(0.0, 1.0);
    (a + ab * t).length()
}

/// 黄金分割求 `min`，返回找到的最小值（含两个初始内点）。
///
/// `best` 由调用方带上端点值一起取最小——端点不是内点，必须单独算。
fn golden_min(iters: u32, eval: impl Fn(f64) -> f64) -> f64 {
    let (mut a, mut b) = (0.0f64, 1.0f64);
    let mut c = b - INV_PHI * (b - a);
    let mut d = a + INV_PHI * (b - a);
    let mut fc = eval(c);
    let mut fd = eval(d);
    let mut best = fc.min(fd);
    for _ in 0..iters {
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - INV_PHI * (b - a);
            fc = eval(c);
            best = best.min(fc);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + INV_PHI * (b - a);
            fd = eval(d);
            best = best.min(fd);
        }
    }
    best
}

/// 暴力采样版：**测试用 oracle**，不做任何凸性假设。
///
/// 独立于黄金分割的实现，用来验证 `sd_sweep` 在窄带内与"真并集"一致。
pub fn sd_sweep_sampled(p: DVec3, from: &Pose, to: &Pose, profile: &Profile, samples: usize) -> f64 {
    let n = samples.max(2);
    let mut best = f64::INFINITY;
    for i in 0..n {
        let t = i as f64 / (n - 1) as f64;
        best = best.min(sd_tool(p, &lerp_pose(from, to, t), profile));
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_tool::flat_end_mill;

    fn up(tip: DVec3) -> Pose {
        Pose { tip, axis: DVec3::Z }
    }

    #[test]
    fn static_sweep_equals_tool() {
        let prof = flat_end_mill(6.0, 20.0);
        let pose = up(DVec3::new(1.0, 2.0, 3.0));
        for p in [
            DVec3::new(1.0, 2.0, 3.0),
            DVec3::new(5.0, 2.0, 10.0),
            DVec3::new(1.0, 2.0, 1.0),
        ] {
            let a = sd_sweep(p, &pose, &pose, &prof);
            let b = sd_tool(p, &pose, &prof);
            assert!((a - b).abs() < 1e-15, "退化扫掠应等于单一位姿：{a} vs {b}");
        }
    }

    /// 解析快路径（`sd_flat_prism`）与黄金分割在**水平走刀**下必须一致。
    ///
    /// 两者是"精确 vs 近似"的关系：解析式闭式求解，黄金分割按
    /// `TARGET_RESOLUTION_MM` 收敛（距离误差 ~7×10⁻⁵ mm）。所以容差取后者的
    /// 收敛精度，不是 1e-9——这个测试证明的是"快路径没算错"，不是"两者逐位相同"。
    ///
    /// `sd_sweep_with(.., None)` 是强制走黄金分割的对拍通道。
    #[test]
    fn flat_prism_matches_golden_search_horizontal() {
        let prof = flat_end_mill(6.0, 20.0);
        let (r, l) = prof.as_flat_end_mill().expect("φ6 平底刀应判为矩形");
        let from = up(DVec3::new(0.0, 0.0, 10.0));
        let to = up(DVec3::new(10.0, 0.0, 10.0));

        let mut checked = 0;
        for i in 0..40 {
            for j in 0..26 {
                for k in 0..14 {
                    let p = DVec3::new(
                        -5.0 + i as f64 * 0.4,
                        -5.0 + j as f64 * 0.4,
                        8.0 + k as f64 * 0.4,
                    );
                    let fast = sd_sweep_with(p, &from, &to, &prof, Some((r, l)));
                    let slow = sd_sweep_with(p, &from, &to, &prof, None);
                    // 只比带内：带外两边都被钳制，没有信息量
                    if fast.abs() < 0.5 && slow.abs() < 0.5 {
                        assert!(
                            (fast - slow).abs() < 2e-4,
                            "({p:?}) 解析 {fast} vs 黄金分割 {slow}"
                        );
                        checked += 1;
                    }
                }
            }
        }
        assert!(checked > 1000, "带内应覆盖大量点，实际 {checked}");
    }

    /// 竖直下刀（钻孔）：扫掠体是圆柱沿轴延长，同样走解析。
    #[test]
    fn flat_prism_matches_golden_search_vertical() {
        let prof = flat_end_mill(6.0, 20.0);
        let (r, l) = prof.as_flat_end_mill().unwrap();
        let from = up(DVec3::new(5.0, 5.0, 20.0));
        let to = up(DVec3::new(5.0, 5.0, 2.0));

        let mut checked = 0;
        for i in 0..20 {
            for j in 0..20 {
                for k in 0..40 {
                    let p = DVec3::new(
                        2.0 + i as f64 * 0.4,
                        2.0 + j as f64 * 0.4,
                        0.0 + k as f64 * 0.5,
                    );
                    let fast = sd_sweep_with(p, &from, &to, &prof, Some((r, l)));
                    let slow = sd_sweep_with(p, &from, &to, &prof, None);
                    if fast.abs() < 0.5 && slow.abs() < 0.5 {
                        assert!(
                            (fast - slow).abs() < 2e-4,
                            "({p:?}) 解析 {fast} vs 黄金分割 {slow}"
                        );
                        checked += 1;
                    }
                }
            }
        }
        assert!(checked > 500, "带内应覆盖大量点，实际 {checked}");
    }

    /// 斜向进给时解析式**不成立**（径向与轴向共享同一个 t，不再是笛卡尔积）——
    /// 必须自动回退，且回退后与"完全不做平底判定"逐位相同。
    #[test]
    fn oblique_step_falls_back_to_golden() {
        let prof = flat_end_mill(6.0, 20.0);
        let from = up(DVec3::new(0.0, 0.0, 10.0));
        let to = up(DVec3::new(10.0, 0.0, 5.0)); // 斜着下去 5 mm
        let flat = prof.as_flat_end_mill();
        assert!(flat.is_some(), "前提：这把刀本身是平底的");

        for p in [
            DVec3::new(5.0, 3.0, 8.0),
            DVec3::new(5.0, 3.0, 12.5),
            DVec3::new(12.0, 0.0, 6.0),
            DVec3::new(2.0, 2.9, 9.0),
            DVec3::new(-2.0, 0.0, 14.0),
        ] {
            let a = sd_sweep_with(p, &from, &to, &prof, flat);
            let b = sd_sweep_with(p, &from, &to, &prof, None);
            assert_eq!(a, b, "({p:?}) 斜向必须逐位回退到黄金分割：{a} vs {b}");
        }
    }

    /// 斜向但**位移与刀轴平行**时仍是笛卡尔积（钻孔的特殊情形），不能误判成斜向。
    #[test]
    fn tilted_tool_uses_pose_orientation() {        let prof = flat_end_mill(6.0, 20.0);
        // 刀轴指向 +X（刀尖在原点，刀身向 +X 伸出）
        let pose = Pose {
            tip: DVec3::ZERO,
            axis: DVec3::X,
        };
        // 沿 +X 方向 10mm 处是刀身内部，径向距离 = 到轴线 0 → 距离 −3
        let d = sd_tool(DVec3::new(10.0, 0.0, 0.0), &pose, &prof);
        assert!((d + 3.0).abs() < 1e-9, "轴向内部应 −3，实际 {d}");
        // 刀尖沿 −X 方向 2mm 处是外部
        let d = sd_tool(DVec3::new(-2.0, 0.0, 0.0), &pose, &prof);
        assert!((d - 2.0).abs() < 1e-9, "刀尖外应 +2，实际 {d}");
    }
}
