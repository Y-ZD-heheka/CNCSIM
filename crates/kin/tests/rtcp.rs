//! RTCP（刀尖点跟随）不变量测试。
//!
//! RTCP 的定义：**程序给的 X/Y/Z 是工件坐标系里的刀尖位置，与两个旋转轴的角度无关。**
//! 也就是说，固定刀尖位置、把 A 从 −60° 摆到 +60°，每一组解算出的机床轴都必须让刀尖
//! 回到同一个点。
//!
//! 这条测试是"摆长基准错"的唯一可靠检出手段：摆长错 50mm 时画面看起来完全正常，
//! 但刀尖会随 A 摆动而漂移几十毫米。

use cncsim_kin::{forward, rtcp_inverse, MachineTopology};
use cncsim_math::DVec3;

/// RTCP 不变量：刀尖固定，A 摆动，刀尖必须纹丝不动。
#[test]
fn rtcp_keeps_tip_fixed_while_a_swings() {
    let m = MachineTopology::ac_trunnion();
    let target = DVec3::new(30.0, 20.0, -5.0);

    let mut max_drift: f64 = 0.0;
    let mut a = -60.0;
    while a <= 60.0 {
        let q = rtcp_inverse(&m, target, &m.q(&[("A", a), ("C", 0.0)]))
            .unwrap_or_else(|| panic!("A={a} 应可解"));

        let back = forward(&m, &q);
        let drift = back.tip.abs_diff(target);
        max_drift = max_drift.max(drift);

        assert!(
            drift < 1e-6,
            "A={a} 时刀尖漂移 {drift} mm（阈值 1 µm）\n机床轴：{:?}",
            q
        );
        a += 3.0;
    }
    assert!(max_drift < 1e-6);
}

/// 同上，但 C 也一起转——四象限都要过。
#[test]
fn rtcp_keeps_tip_fixed_across_c_quadrants() {
    let m = MachineTopology::ac_trunnion();
    let target = DVec3::new(-45.0, 60.0, -12.5);

    for c in [0.0, 90.0, 180.0, 270.0, -135.0] {
        for a in [-90.0, -45.0, -15.0, 15.0, 45.0, 90.0] {
            let q = rtcp_inverse(&m, target, &m.q(&[("A", a), ("C", c)]))
                .unwrap_or_else(|| panic!("A={a} C={c} 应可解"));
            let drift = forward(&m, &q).tip.abs_diff(target);
            assert!(drift < 1e-6, "A={a} C={c} 漂移 {drift} mm");
        }
    }
}

/// **补偿必须是真的在动**——防止"返回常数"这种假通过。
///
/// 刀尖在台面中心、A 从 0 摆到 110°，Y 要负向走 169mm、Z 要下沉 242mm
/// （见 docs/机床建模-AC双转台.md §6.1）。
#[test]
fn rtcp_compensation_actually_moves_the_axes() {
    let m = MachineTopology::ac_trunnion();
    let center = DVec3::ZERO; // 台面中心

    let q0 = rtcp_inverse(&m, center, &m.q(&[("A", 0.0)])).unwrap();
    let q110 = rtcp_inverse(&m, center, &m.q(&[("A", 110.0)])).unwrap();

    let (iy, iz) = (m.index_of("Y").unwrap(), m.index_of("Z").unwrap());
    let dy = q110[iy] - q0[iy];
    let dz = q110[iz] - q0[iz];

    assert!(
        (dy + 169.1).abs() < 0.5,
        "A=110° 时 Y 应约 −169.1 mm 的补偿，实际 {dy:.2}"
    );
    assert!(
        (dz + 241.6).abs() < 1.0,
        "A=110° 时 Z 应约 −241.6 mm 的补偿，实际 {dz:.2}"
    );
}

/// `rtcp_inverse` 与 `inverse` 必须一致：给定 A/C，由 FK 得到位姿，
/// 两条路径应解出同一组机床轴。
#[test]
fn rtcp_agrees_with_full_inverse() {
    let m = MachineTopology::ac_trunnion();
    let q0 = m.q(&[("X", 80.0), ("Y", -35.0), ("Z", -60.0), ("A", 40.0), ("C", -70.0)]);
    let pose = forward(&m, &q0);

    let q_rtcp = rtcp_inverse(&m, pose.tip, &q0).unwrap();
    let q_full = cncsim_kin::inverse(&m, &pose, &q0).unwrap();

    for name in ["C", "A", "X", "Y", "Z"] {
        let i = m.index_of(name).unwrap();
        assert!(
            (q_rtcp[i] - q_full[i]).abs() < 1e-6,
            "{name}: rtcp {:.9} vs full {:.9}",
            q_rtcp[i],
            q_full[i]
        );
    }
}

/// 超出行程时**仍然返回解**，但由 `violated_joints` 报出来。
///
/// 这是刻意的设计：调用方要能区分
/// - `None` = **运动学上无解**（雅可比退化 / Newton 不收敛）
/// - `Some(q)` + 限位报警 = **解得出来，但机床走不到**
///
/// 两者对用户的含义完全不同，不能混成一个 `None`。
#[test]
fn out_of_travel_returns_solution_and_flags_limit() {
    let m = MachineTopology::ac_trunnion();
    let q = rtcp_inverse(&m, DVec3::new(5000.0, 0.0, 0.0), &m.q(&[("A", 0.0)]))
        .expect("运动学上有解（只是机床走不到）");

    let bad = m.violated_joints(&q);
    assert_eq!(bad.len(), 1, "应只有 X 超程，实际 {:?}", bad);
    assert_eq!(m.chain[bad[0]].name, "X");
    assert!(!m.within_limits(&q));
}

/// 正常可达的位姿不应报限位。
#[test]
fn reachable_pose_has_no_limit_violation() {
    let m = MachineTopology::ac_trunnion();
    let q = m.q(&[("X", 100.0), ("Y", 50.0), ("Z", -20.0), ("A", 30.0), ("C", 45.0)]);
    assert!(m.within_limits(&q), "正常位姿不该报限位：{:?}", m.violated_joints(&q));
    // 解出来的也该在限位内
    let q2 = rtcp_inverse(&m, forward(&m, &q).tip, &q).unwrap();
    assert!(m.within_limits(&q2), "{:?}", m.violated_joints(&q2));
}
