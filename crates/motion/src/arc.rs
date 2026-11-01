//! 圆弧离散：把 `G2/G3` 的圆弧变成折线。
//!
//! 体素切削是"刀尖从 A 直线扫到 B"，所以圆弧必须先离散。离散密度由**弦高**
//! 决定，不是固定段数——固定段数在 R=200 的大圆弧上会明显啃出多边形，
//! 在 R=1 的小圆弧上又白算几千段。
//!
//! 弦高与段长的关系：半径 `R` 的圆上，弦长 `L` 对应的弓高
//! `s = R(1 − cos(θ/2)) ≈ L²/(8R)`。反解 `L = √(8R·s)`。
//!
//! 默认弦高取多少见 [`DEFAULT_ARC_SAGITTA`]。

use crate::Plane;
use cncsim_math::DVec3;

/// 默认弦高上限（mm）。
///
/// 取 2 µm：比体素尺寸（0.1 mm）和碰撞检出下限（42 µm）都小一个量级以上，
/// 所以圆弧离散**不会成为误差来源**——剩下的误差全是体素和采样带来的。
pub const DEFAULT_ARC_SAGITTA: f64 = 0.002;

/// 平面内的两个正交基 `(u, v)`，满足 `u × v` = 平面法向。
///
/// 法向决定"顺时针"从哪边看：G17 的 +Z、G18 的 +Y、G19 的 +X。
fn basis(plane: Plane) -> (DVec3, DVec3) {
    match plane {
        Plane::Xy => (DVec3::X, DVec3::Y),
        Plane::Zx => (DVec3::Z, DVec3::X),
        Plane::Yz => (DVec3::Y, DVec3::Z),
    }
}

/// 把圆弧离散成折线点：**不含起点，含终点**。
///
/// `center` 是圆心（与 `from` / `to` 同一个世界坐标系）。`clockwise` 按平面法向
/// 看下去的方向判定（G17 顺时针 = 从 +Z 看）。
///
/// 起终点重合时按整圆处理——这是 `G2/G3` 走整圆的写法，不能返回空。
pub fn sample_arc(
    from: DVec3,
    to: DVec3,
    center: DVec3,
    clockwise: bool,
    plane: Plane,
    max_sagitta: f64,
) -> Vec<DVec3> {
    let (u, v) = basis(plane);
    let a = from - center;
    let b = to - center;
    let (au, av) = (a.dot(u), a.dot(v));
    let r = (au * au + av * av).sqrt();
    if r < 1e-9 {
        // 起点就在圆心上：圆弧退化成点，直接给终点
        return vec![to];
    }

    let th0 = av.atan2(au);
    let th1 = b.dot(v).atan2(b.dot(u));

    // 扫掠角：方向由 `clockwise` 定，绝对值不超过一圈
    let tau = core::f64::consts::TAU;
    let mut d = th1 - th0;
    if clockwise {
        while d >= 0.0 {
            d -= tau;
        }
        while d < -tau {
            d += tau;
        }
    } else {
        while d <= 0.0 {
            d += tau;
        }
        while d > tau {
            d -= tau;
        }
    }

    // 段长按弦高反解；`max_sagitta` 太小会让段数爆，兜个底
    let sag = max_sagitta.max(1e-6);
    let seg_len = (8.0 * r * sag).sqrt();
    let n = ((d.abs() * r / seg_len).ceil() as usize).clamp(1, 8192);

    let mut out = Vec::with_capacity(n);
    for i in 1..=n {
        let t = i as f64 / n as f64;
        let th = th0 + d * t;
        out.push(center + u * (r * th.cos()) + v * (r * th.sin()));
    }
    // 末点强制等于 `to`：解析出来的 `th1` 有浮点误差，别让终点偏一点点
    if let Some(last) = out.last_mut() {
        *last = to;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quarter_circle_in_xy() {
        // 从 (0,10) 逆时针到 (10,0)，圆心原点半径 10
        let pts = sample_arc(
            DVec3::new(0.0, 10.0, 0.0),
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::ZERO,
            false,
            Plane::Xy,
            DEFAULT_ARC_SAGITTA,
        );
        assert!(!pts.is_empty());
        assert_eq!(*pts.last().unwrap(), DVec3::new(10.0, 0.0, 0.0), "末点必须精确等于终点");
        for p in &pts {
            assert!((p.length() - 10.0).abs() < 1e-9, "所有点都在圆上，实际 {:?}", p);
        }
    }

    #[test]
    fn clockwise_goes_the_other_way() {
        // 从 (10,0) 到 (0,10)：逆时针走 90°，顺时针走 270°
        let ccw = sample_arc(
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::new(0.0, 10.0, 0.0),
            DVec3::ZERO,
            false,
            Plane::Xy,
            DEFAULT_ARC_SAGITTA,
        );
        let cw = sample_arc(
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::new(0.0, 10.0, 0.0),
            DVec3::ZERO,
            true,
            Plane::Xy,
            DEFAULT_ARC_SAGITTA,
        );
        let ccw_mid = ccw[ccw.len() / 2];
        let cw_mid = cw[cw.len() / 2];
        assert!(ccw_mid.x > 1.0 && ccw_mid.y > 1.0, "逆时针中点在 +X+Y 象限，实际 {ccw_mid:?}");
        assert!(cw_mid.x < 0.0 && cw_mid.y < 0.0, "顺时针中点在第三象限，实际 {cw_mid:?}");
        assert!(cw.len() > ccw.len() * 2, "顺时针要多走两倍的角度");
    }

    #[test]
    fn full_circle_when_endpoints_coincide() {
        let from = DVec3::new(10.0, 0.0, 0.0);
        let pts = sample_arc(from, from, DVec3::ZERO, false, Plane::Xy, 0.05);
        assert!(pts.len() > 10, "整圈不能退化成空");
        // 中点应在圆的另一侧
        let mid = pts[pts.len() / 2];
        assert!(mid.x < -9.0, "半圈后应到 −X 侧，实际 {mid:?}");
    }

    #[test]
    fn sagitta_controls_the_segment_count() {
        let from = DVec3::new(100.0, 0.0, 0.0);
        let to = DVec3::new(0.0, 100.0, 0.0);
        let coarse = sample_arc(from, to, DVec3::ZERO, false, Plane::Xy, 0.5);
        let fine = sample_arc(from, to, DVec3::ZERO, false, Plane::Xy, 0.002);
        assert!(fine.len() > coarse.len() * 10, "细 250 倍应多约 √250 倍的点：{} vs {}", fine.len(), coarse.len());
        // 弦高真的满足
        for w in fine.windows(2) {
            let chord = (w[1] - w[0]).length();
            let sag = 100.0 - (100.0f64 * 100.0 - chord * chord / 4.0).sqrt();
            assert!(sag <= 0.0021, "弦高 {sag} 超上限，段长 {chord}");
        }
    }

    #[test]
    fn zx_plane_uses_y_as_the_normal() {
        // G18：从 +Y 看，顺时针 = 从 +Z 转向 +X
        let pts = sample_arc(
            DVec3::new(0.0, 0.0, 10.0),
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::ZERO,
            false,
            Plane::Zx,
            DEFAULT_ARC_SAGITTA,
        );
        // 中点约在 (7.07, 0, 7.07)；`len()/2` 是整段索引，允许半段偏差
        let mid = pts[pts.len() / 2];
        assert!((mid.length() - 10.0).abs() < 1e-9, "点必须在圆上，实际 {mid:?}");
        assert!(mid.z > 6.4 && mid.x > 6.4 && mid.y.abs() < 1e-9, "实际 {mid:?}");
    }
}
