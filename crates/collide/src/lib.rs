//! # cncsim-collide
//!
//! 碰撞：GJK 距离、EPA 穿透深度、保守推进、episode 聚类。
//!
//! | 模块 | 管什么 | 精度 |
//! |---|---|---|
//! | [`convex`] | 凸体 × 凸体的**距离**（GJK，解析 support） | 浮点精度，零建模误差 |
//! | [`penetration`] | 凸体 × 凸体的**穿透深度与分离轴**（EPA） | 平面逼近曲面，实测球面 ~2e-5 mm |
//! | [`continuous`] | 扫掠过程的**保守推进**（不漏报） | 由采样间隙决定，默认 42 µm |
//!
//! 毛坯（体素场）那一类走 [`DistanceField`] trait：**本 crate 不依赖 `voxel`**，
//! 依赖方向反过来由上层注入。好处是碰撞能脱离体素单独测试（用解析半空间做假实现），
//! 这正是 docs/ADR-004 §7.7 那条"圆柱 vs 半空间"随机对拍需要的。

pub mod continuous;
pub mod convex;
pub mod penetration;

use cncsim_math::DVec3;

/// 有符号距离场。由 `voxel::SparseGrid` 实现。
///
/// **只问"点在材料内吗"和"多远"**，不问拓扑、不问网格。
pub trait DistanceField {
    /// 有符号距离：正值在材料外，负值在材料内。
    fn sdf(&self, p: DVec3) -> f64;

    /// 窄带半宽（mm）。超出带宽的 `sdf` 返回钳制值。
    fn band(&self) -> f64;

    /// 场覆盖区域的包围盒（世界坐标）。**没有就返回 `None`。**
    ///
    /// 保守推进（[`continuous::earliest_collision`]）用它算一个 O(1) 的距离下界。
    /// 这一条对**体素场**是必需的，不是优化：
    ///
    /// 体素场的窄带只有 `band` 宽（默认 0.4 mm），带外一律返回钳制的 `+band`。
    /// 而刀杆表面采样间隙 `δ` 默认 0.5 mm。于是"采样点最小 sdf − δ" = `−0.1`，
    /// **任何位置都算'无法保证'**——保守推进退化成每段都报碰撞。
    ///
    /// 补一条下界就好了："刀杆所有点都在以刀尖为心、`reach` 为半径的球里，
    /// 所以到包围盒的距离 ≥ |刀尖 − 包围盒| − `reach`"。O(1)、严格、不误报。
    fn aabb(&self) -> Option<(DVec3, DVec3)> {
        None
    }
}

/// 解析半空间 `z ≤ 0` —— 测试用的假 `DistanceField`，真值精确可算。
pub struct HalfSpaceZ;

impl DistanceField for HalfSpaceZ {
    fn sdf(&self, p: DVec3) -> f64 {
        p.z
    }
    fn band(&self) -> f64 {
        f64::INFINITY
    }
}

/// 一次碰撞事件聚类（同一对部件、间隔 < 50 ms 合并）。
///
/// 为什么必须聚类：一次真实碰撞会在连续几十个采样点上反复触发，逐条报出来
/// 是几百行噪声，人眼筛不动。按"同一对部件 + 时间间隔"合并之后，
/// 一个 episode ≈ 一次事故。
///
/// 时间单位是仿真秒。`max_penetration` 来自 [`penetration::Penetration::depth`]。
#[derive(Clone, Debug)]
pub struct CollisionEpisode {
    pub a: String,
    pub b: String,
    pub t_start: f64,
    pub t_end: f64,
    pub max_penetration: f64,
}

impl CollisionEpisode {
    /// 开一个新 episode。
    pub fn new(a: impl Into<String>, b: impl Into<String>, t: f64, depth: f64) -> Self {
        Self {
            a: a.into(),
            b: b.into(),
            t_start: t,
            t_end: t,
            max_penetration: depth,
        }
    }

    /// `t` 时刻同一对部件又撞了一次。间隔 ≤ `gap` 秒就并进来并返回 `true`。
    ///
    /// `false` 表示"这是另一次事故"，调用方该开新 episode。
    /// 默认 `gap = 0.05`（50 ms）——见 [`DEFAULT_EPISODE_GAP`]。
    pub fn absorb(&mut self, t: f64, depth: f64, gap: f64) -> bool {
        if t - self.t_end > gap {
            return false;
        }
        // 时间要单调递增；乱序时保留较大的那个终点，不要让 episode 倒退
        self.t_end = self.t_end.max(t);
        self.max_penetration = self.max_penetration.max(depth);
        true
    }

    /// 是否已经超过 `gap` 没有再碰（可以关账）。
    pub fn expired(&self, t: f64, gap: f64) -> bool {
        t - self.t_end > gap
    }

    /// 持续时长（仿真秒）
    pub fn duration(&self) -> f64 {
        self.t_end - self.t_start
    }
}

/// episode 的默认合并间隔：50 ms。
///
/// 取这个数是因为 100 m/min 快移在 50 ms 里走 83 mm——比任何真实碰撞的
/// 接触尺度都大一个量级，所以同一次碰撞不会被拆成两段；而两次独立事故
/// 之间基本不会在 50 ms 内重复。
pub const DEFAULT_EPISODE_GAP: f64 = 0.05;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn episode_merges_events_within_the_gap() {
        let mut e = CollisionEpisode::new("tool", "fixture", 1.000, 0.02);
        assert!(e.absorb(1.010, 0.05, DEFAULT_EPISODE_GAP), "10 ms 内应合并");
        assert!(e.absorb(1.040, 0.01, DEFAULT_EPISODE_GAP), "距上一次 30 ms，仍应合并");
        assert_eq!(e.max_penetration, 0.05, "取最大穿透，不是最后一次");
        assert!((e.duration() - 0.040).abs() < 1e-12);

        // 距 t_end 超过 50 ms → 是另一次事故
        assert!(!e.absorb(1.100, 0.5, DEFAULT_EPISODE_GAP));
        assert_eq!(e.t_end, 1.040, "被拒的事件不该改终点");
        assert!(e.expired(1.100, DEFAULT_EPISODE_GAP));
    }

    #[test]
    fn episode_gap_is_83mm_of_rapid_traverse() {
        // 50 ms @ 100 m/min = 83 mm——比任何接触尺度都大一个量级，
        // 所以同一次碰撞不会被拆成两段
        let rapid_mm_per_s = 100_000.0 / 60.0;
        assert!((rapid_mm_per_s * DEFAULT_EPISODE_GAP - 83.33).abs() < 0.01);
    }

    #[test]
    fn half_space_sign_is_correct() {
        let h = HalfSpaceZ;
        assert!(h.sdf(DVec3::new(0.0, 0.0, -5.0)) < 0.0, "材料内应为负");
        assert!(h.sdf(DVec3::new(0.0, 0.0, 5.0)) > 0.0, "材料外应为正");
        assert!(h.sdf(DVec3::new(1.0, 2.0, 3.0)) == 3.0, "距离应等于 z");
    }
}
