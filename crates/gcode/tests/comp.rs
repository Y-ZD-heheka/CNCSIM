//! 刀径补偿（G41/G42）。
//!
//! 核心断言只有一条，但它比任何手算的点都强：**补偿后的每个刀位点，到编程
//! 路径的距离必须正好等于补偿半径**。这就是"刀径补偿"的定义。手动挑几个点
//! 只能证明那几个点对；这条性质一次覆盖整条路径，包括拐角圆弧上的点。

use cncsim_gcode::{InterpConfig, MotionProgram, interpret};
use cncsim_math::DVec3;
use cncsim_motion::MotionKind;

fn cfg() -> InterpConfig {
    InterpConfig {
        tool_radii: vec![0.0, 3.0], // D1 = 半径 3
        ..InterpConfig::default()
    }
}

/// 点到折线的最短距离（XY 平面）
fn dist_to_polyline(p: DVec3, poly: &[(f64, f64)]) -> f64 {
    let mut best = f64::INFINITY;
    for w in poly.windows(2) {
        let (ax, ay) = w[0];
        let (bx, by) = w[1];
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-18 {
            0.0
        } else {
            (((p.x - ax) * dx + (p.y - ay) * dy) / len2).clamp(0.0, 1.0)
        };
        let (qx, qy) = (ax + dx * t, ay + dy * t);
        best = best.min(((p.x - qx).powi(2) + (p.y - qy).powi(2)).sqrt());
    }
    best
}

/// 补偿路径上的点（跳过引入块之前的那些块）
fn comp_points(p: &MotionProgram, from: usize) -> Vec<DVec3> {
    p.blocks[from..].iter().map(|b| b.target.tip).collect()
}

const SQUARE: &[(f64, f64)] = &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)];

/// 逆时针方框，左侧补偿（刀在框内）
fn g41_square() -> MotionProgram {
    interpret(
        "G0 X0 Y0\nG1 G41 D1 X10 Y0 F100\nX10 Y10\nX0 Y10\nX0 Y0\nG40\nG1 X-5 Y0\n",
        &cfg(),
    )
}

#[test]
fn g41_insets_the_ccw_square() {
    let p = g41_square();
    let pts = comp_points(&p, 1);
    // 每个内角都是两条偏移线求交：内缩 3 mm 的方框
    assert!(
        (pts[0] - DVec3::new(7.0, 3.0, 0.0)).length() < 1e-9,
        "第一个内角应在 (7,3)，实际 {:?}",
        pts[0]
    );
    assert!((pts[1] - DVec3::new(7.0, 7.0, 0.0)).length() < 1e-9, "实际 {:?}", pts[1]);
    assert!((pts[2] - DVec3::new(3.0, 7.0, 0.0)).length() < 1e-9, "实际 {:?}", pts[2]);
}

#[test]
fn every_compensated_point_is_exactly_r_from_the_programmed_path() {
    // 这条就是"刀径补偿"的定义：刀心到编程轮廓的距离恒等于半径。
    // 拐角求交算错、圆弧插值算错、法向取反，都会在这里红。
    for src in [
        "G0 X0 Y0\nG1 G41 D1 X10 Y0 F100\nX10 Y10\nX0 Y10\nX0 Y0\nG40\nG1 X-5 Y0\n",
        "G0 X0 Y0\nG1 G42 D1 X10 Y0 F100\nX10 Y10\nX0 Y10\nX0 Y0\nG40\nG1 X-5 Y0\n",
    ] {
        let p = interpret(src, &cfg());
        let n = p.blocks.len();
        for (k, b) in p.blocks.iter().enumerate().skip(1) {
            if k + 1 == n {
                continue; // 引出段回到编程路径，不参与这条性质
            }
            let d = dist_to_polyline(b.target.tip, SQUARE);
            assert!(
                d <= 3.0 + 1e-9,
                "第 {k} 块 {:?} 到编程路径 {d}，不该超过半径 3",
                b.target.tip
            );
            // 收尾那一块（G40 前最后一块）落在闭合角上，它到"已切过的那条边"
            // 的距离会是 0 —— 那是几何事实，不是错。其余点必须正好在 r 上。
            if k + 2 == n {
                continue;
            }
            assert!(
                (d - 3.0).abs() < 1e-9,
                "第 {k} 块 {:?} 到编程路径的距离是 {d}，应为 3",
                b.target.tip
            );
        }
    }
}

#[test]
fn g42_outsets_and_rolls_around_corners() {
    let p = interpret(
        "G0 X0 Y0\nG1 G42 D1 X10 Y0 F100\nX10 Y10\nX0 Y10\nX0 Y0\nG40\nG1 X-5 Y0\n",
        &cfg(),
    );
    let arcs: Vec<_> = p
        .blocks
        .iter()
        .filter(|b| matches!(b.motion, MotionKind::Arc { .. }))
        .collect();
    // 三个中间顶点是外角，各插一段绕顶点的圆弧
    assert_eq!(arcs.len(), 3, "外角应有 3 段拐角弧，实际 {}", arcs.len());
    for a in &arcs {
        if let MotionKind::Arc { center, .. } = a.motion {
            // 弧心就是编程顶点，弧上的点离它正好 r
            let r = (a.target.tip - center).length();
            assert!((r - 3.0).abs() < 1e-9, "拐角弧半径应为 3，实际 {r}");
            let on_corner = SQUARE
                .iter()
                .any(|&(x, y)| (center.x - x).abs() < 1e-9 && (center.y - y).abs() < 1e-9);
            assert!(on_corner, "弧心应在编程顶点上，实际 {center:?}");
        }
    }
}

#[test]
fn g41_on_a_circle_keeps_the_radius_minus_r() {
    // 逆时针整圆（两段半圆），左补偿 = 内侧 → 半径 10 − 3 = 7。
    // 引入段从 (20,0) 走到 (10,0)：先到 (10,−3)，再绕 (10,0) 转一段 90° 的
    // 拐角弧接到半径 7 的圆上 —— 引入线在圆外，工具得从外面进来。
    let p = interpret(
        "G0 X20 Y0\nG1 G41 D1 X10 Y0 F100\nG3 X-10 Y0 I-10 J0\nG3 X10 Y0 I10 J0\nG40\nG1 X20 Y0\n",
        &cfg(),
    );
    let mut checked = 0;
    for b in &p.blocks {
        // 只看绕原点的那两段圆弧（拐角弧的圆心在 (10,0)）
        if let MotionKind::Arc { center, .. } = b.motion
            && center.length() < 1e-9
        {
            let r = b.target.tip.length();
            assert!((r - 7.0).abs() < 1e-9, "圆上补偿点半径应为 7，实际 {r}");
            checked += 1;
        }
    }
    assert!(checked == 2, "两段半圆都该被补偿，实际 {checked}");
}

#[test]
fn collinear_points_do_not_create_corners() {
    let p = interpret(
        "G0 X0 Y0\nG1 G41 D1 X10 Y0 F100\nX20 Y0\nX30 Y0\nG40\nG1 X40 Y0\n",
        &cfg(),
    );
    assert_eq!(
        p.blocks
            .iter()
            .filter(|b| matches!(b.motion, MotionKind::Arc { .. }))
            .count(),
        0,
        "共线点不该产生拐角弧"
    );
    for b in &p.blocks[1..=3] {
        assert!((b.target.tip.y - 3.0).abs() < 1e-9, "应一直偏 3 mm：{:?}", b.target.tip);
    }
}

#[test]
fn g40_returns_to_the_programmed_path() {
    let p = g41_square();
    let last = p.blocks.last().unwrap();
    assert!(
        (last.target.tip - DVec3::new(-5.0, 0.0, 0.0)).length() < 1e-9,
        "G40 之后应回到编程坐标，实际 {:?}",
        last.target.tip
    );
    assert!(matches!(last.motion, MotionKind::Linear));
}

#[test]
fn compensation_does_not_touch_z() {
    let p = interpret(
        "G0 X0 Y0 Z5\nG1 G41 D1 X10 Y0 Z-2 F100\nX10 Y10 Z-2\nG40\n",
        &cfg(),
    );
    assert!((p.blocks[1].target.tip.z + 2.0).abs() < 1e-9);
    assert!((p.blocks[2].target.tip.z + 2.0).abs() < 1e-9);
}

#[test]
fn unknown_d_register_is_an_error() {
    // D9 没登记 → 半径 0 → 路径会整整差一个刀半径，必须报错而不是当 0
    let p = interpret("G0 X0 Y0\nG1 G41 D9 X10 Y0 F100\nG40\n", &cfg());
    assert!(p.issues.iter().any(|i| i.id == "GCODE-011"), "应报 GCODE-011");
    assert!(
        matches!(p.trust, cncsim_gcode::Trust::Unreliable { .. }),
        "实际 {:?}",
        p.trust
    );
}

#[test]
fn arc_as_the_lead_in_is_reported() {
    // 圆弧引入要新建半径，本版不做——报出来，而不是悄悄按直线引入
    let p = interpret(
        "G0 X10 Y0\nG2 G41 D1 X-10 Y0 I-10 J0 F100\nX10 Y0 I10 J0\nG40\n",
        &cfg(),
    );
    assert!(
        p.issues.iter().any(|i| i.id == "GCODE-005"),
        "圆弧引入应报不支持"
    );
}

#[test]
fn comp_without_g40_at_the_end_still_works() {
    let p = interpret("G0 X0 Y0\nG1 G41 D1 X10 Y0 F100\nX10 Y10\n", &cfg());
    let last = p.blocks.last().unwrap();
    // 最后一个顶点是内角，补偿点 (7,3)... 最后一段的终点是 (10,10) 的偏移
    assert!((last.target.tip.y - 10.0).abs() < 0.5, "实际 {:?}", last.target.tip);
}

#[test]
fn changing_side_mid_program_starts_a_new_group() {
    // G41 → G42 没写 G40：按两组处理，各自从当前点重新引入
    let p = interpret(
        "G0 X0 Y0\nG1 G41 D1 X10 Y0 F100\nG42 X20 Y0\nG40\n",
        &cfg(),
    );
    // 不该 panic，也不该产生 NaN
    for b in &p.blocks {
        assert!(b.target.tip.x.is_finite() && b.target.tip.y.is_finite());
    }
}

#[test]
fn no_comp_no_change() {
    // 没有 G41/G42 时逐位不变（后处理是空操作）
    let src = "G0 X0 Y0\nG1 X10 Y0\nX10 Y10\n";
    let p = interpret(src, &cfg());
    let q = interpret(src, &InterpConfig::default());
    assert_eq!(p.blocks.len(), q.blocks.len());
    for (a, b) in p.blocks.iter().zip(&q.blocks) {
        assert_eq!(a.target.tip, b.target.tip);
    }
}
