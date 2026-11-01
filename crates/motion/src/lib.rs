//! # cncsim-motion
//!
//! 运动词汇：位姿、运动段、进给。
//!
//! 这一层**只有数据与约定，没有算法**。它被 `kin`（算位姿）、`gcode`（产出位姿）、
//! `voxel`（沿位姿扫掠）共用，所以放在最底下、只依赖 `math`。
//!
//! ## 全项目唯一的 `Pose` 约定
//!
//! ```text
//! Pose { tip: 刀尖位置, axis: 刀轴方向（单位向量） }
//! ```
//!
//! **`axis` 的方向是「由刀尖指向主轴」**，即刀具"朝上"那一端。
//! 竖直加工时 `axis == (0,0,1)`。
//!
//! 这条约定必须全项目统一——`kin` 的 FK、`voxel` 的扫掠、`collide` 的刀柄判定
//! 全都依赖它。写反的话几何上仍然"能跑"，只是刀具反向，极难发现。
//! 见 [`Pose::UP`] 与测试 `axis_points_from_tip_to_spindle`。

pub mod arc;

use cncsim_math::DVec3;

/// 位姿：刀尖位置 + 刀轴方向。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// 刀尖（刀具最前端）在**工件坐标系**中的位置，单位 mm。
    pub tip: DVec3,
    /// 刀轴单位向量，方向**由刀尖指向主轴**。竖直加工时为 `(0,0,1)`。
    pub axis: DVec3,
}

impl Pose {
    /// 刀轴竖直向上——A/B 轴为 0 时的位姿。
    pub const UP: DVec3 = DVec3::Z;

    pub fn new(tip: DVec3, axis: DVec3) -> Self {
        debug_assert!(
            (axis.length() - 1.0).abs() < 1e-9,
            "Pose::axis 必须是单位向量，实际长度 {}",
            axis.length()
        );
        Self { tip, axis }
    }

    /// 位置线性插值 + 姿态球面插值。
    ///
    /// **姿态必须用 slerp**：`lerp` + 归一化会让角速度两端慢中间快，
    /// 从而把非线性误差算错（见 docs/G代码到轴运动.md §2.2）。
    pub fn lerp_slerp(&self, o: &Pose, t: f64) -> Pose {
        Pose {
            tip: self.tip.lerp(o.tip, t),
            axis: slerp(self.axis, o.axis, t),
        }
    }
}

/// 单位向量的球面线性插值。
pub fn slerp(a: DVec3, b: DVec3, t: f64) -> DVec3 {
    let dot = a.dot(b).clamp(-1.0, 1.0);
    let theta = dot.acos();
    if theta < 1e-9 {
        return a;
    }
    let (sa, sb) = ((1.0 - t) * theta, t * theta);
    (a * sa.sin() + b * sb.sin()) / theta.sin()
}

/// 进给模式（G93 / G94 / G95）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedMode {
    /// G93：F = 1/时间（分钟⁻¹）。五轴常用，能让刀尖速度恒定。
    InverseTime,
    /// G94：F = mm/min。
    UnitsPerMin,
    /// G95：F = mm/rev（需要主轴转速）。
    UnitsPerRev,
}

/// 一条进给指令。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Feed {
    pub value: f64,
    pub mode: FeedMode,
}

impl Default for Feed {
    fn default() -> Self {
        Self { value: 0.0, mode: FeedMode::UnitsPerMin }
    }
}

/// 运动类型。**只描述"怎么动"，不含时间**——时间在 `sim` 里算。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotionKind {
    Rapid,
    Linear,
    /// 圆弧：圆心 + 方向 + 所在平面
    Arc { center: DVec3, clockwise: bool, plane: Plane },
    Dwell { seconds: f64 },
}

/// 圆弧所在平面（G17 / G18 / G19）。决定 I/J/K 的含义。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Plane {
    /// G17：XY 平面，用 I、J
    Xy,
    /// G18：ZX 平面，用 I、K
    Zx,
    /// G19：YZ 平面，用 J、K
    Yz,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::FRAC_PI_2;

    #[test]
    fn axis_points_from_tip_to_spindle() {
        // 竖直加工时刀轴是 +Z（刀尖在下，主轴在上）
        assert_eq!(Pose::UP, DVec3::Z);
        let p = Pose::new(DVec3::new(0.0, 0.0, -5.0), Pose::UP);
        assert!(p.axis.z > 0.0, "刀轴必须朝上（指向主轴）");
    }

    #[test]
    fn slerp_is_uniform_angular() {
        // 相邻等参采样的夹角必须相等——lerp+归一化不满足这一点
        let a = DVec3::Z;
        let b = DVec3::new(0.0, 0.5, 0.866_025_403_784_438_6);
        let mut prev = slerp(a, b, 0.0);
        let mut angles = Vec::new();
        for i in 1..=10 {
            let cur = slerp(a, b, i as f64 / 10.0);
            angles.push(prev.dot(cur).clamp(-1.0, 1.0).acos());
            prev = cur;
        }
        let first = angles[0];
        for (i, w) in angles.windows(2).enumerate() {
            assert!(
                (w[0] - w[1]).abs() < 1e-12,
                "第 {i} 段角步长 {} 与 {} 不等",
                w[0],
                w[1]
            );
        }
        assert!(first > 0.0);
    }

    #[test]
    fn slerp_endpoints_are_exact() {
        let a = DVec3::Z;
        let b = DVec3::new(0.0, 0.5, 0.866_025_403_784_438_6);
        assert!((slerp(a, b, 0.0) - a).length() < 1e-15);
        assert!((slerp(a, b, 1.0) - b).length() < 1e-15);
    }

    #[test]
    fn slerp_half_of_90deg_is_45deg() {
        let a = DVec3::Z;
        let b = DVec3::new(0.0, 1.0, 0.0); // 绕 X 转 90°
        let m = slerp(a, b, 0.5);
        let expected = DVec3::new(0.0, FRAC_PI_2.sin() / 2.0_f64.sqrt() * 1.0, FRAC_PI_2.cos() / 2.0_f64.sqrt() * 1.0);
        // 直接验证夹角 = 45°
        assert!((m.dot(a).clamp(-1.0, 1.0).acos() - FRAC_PI_2 / 2.0).abs() < 1e-12);
        let _ = expected;
    }
}
