//! 运动学绝对锚点 —— **本项目的第一个红测试**。
//!
//! 这三组数来自 `testdata/golden/ac_trunnion_anchors.csv` 的前三行，
//! 是**手推 + 独立齐次矩阵连乘**双重确认的，独立于 `forward()` 的实现。
//!
//! 为什么要锚点而不只靠往返测试：往返测试抓不到"FK 和 IK 一起错但自洽"。
//! 比如把刀轴方向整体取反，往返仍然完美通过，但仿真结果全错。
//!
//! **注意 `q` 的顺序是链顺序 `[C, A, X, Y, Z]`，不是 `[X,Y,Z,A,C]`。**
//! 用 `m.q(&[("X", ..), ..])` 按名字构造，别手写数组。

use cncsim_kin::{forward, MachineTopology};
use cncsim_math::DVec3;

fn assert_pose(got: cncsim_motion::Pose, tip: [f64; 3], axis: [f64; 3], tol: f64) {
    let t = DVec3::new(tip[0], tip[1], tip[2]);
    let a = DVec3::new(axis[0], axis[1], axis[2]);
    assert!(
        got.tip.abs_diff(t) < tol,
        "刀尖错：得 {:?}，应 {:?}（差 {}）",
        got.tip,
        t,
        got.tip.abs_diff(t)
    );
    assert!(
        got.axis.abs_diff(a) < tol,
        "刀轴错：得 {:?}，应 {:?}",
        got.axis,
        a
    );
}

#[test]
fn anchor_1_all_zero() {
    // (X,Y,Z,A,C) = (0,0,0,0,0)
    // 刀尖在工件原点下方 L=180；刀轴竖直向上
    let m = MachineTopology::ac_trunnion();
    let p = forward(&m, &m.q(&[("X", 0.0), ("Y", 0.0), ("Z", 0.0), ("A", 0.0), ("C", 0.0)]));
    assert_pose(p, [0.0, 0.0, -180.0], [0.0, 0.0, 1.0], 1e-9);
}

#[test]
fn anchor_2_tilt_90_deg() {
    // (X,Y,Z,A,C) = (0,0,-50,90,0)
    // 台面翻 90°，刀尖在工件系里表现为 Y=−50；刀轴变成 +Y
    let m = MachineTopology::ac_trunnion();
    let p = forward(&m, &m.q(&[("X", 0.0), ("Y", 0.0), ("Z", -50.0), ("A", 90.0), ("C", 0.0)]));
    assert_pose(p, [0.0, -50.0, -180.0], [0.0, 1.0, 0.0], 1e-9);
}

#[test]
fn anchor_3_general_pose() {
    // (X,Y,Z,A,C) = (100,50,-20,30,45) —— 非平凡位姿
    let m = MachineTopology::ac_trunnion();
    let p = forward(
        &m,
        &m.q(&[("X", 100.0), ("Y", 50.0), ("Z", -20.0), ("A", 30.0), ("C", 45.0)]),
    );
    assert_pose(
        p,
        [94.258_232_091_579, -47.163_124_145_731, -222.320_508_075_689],
        [0.353_553_390_593, 0.353_553_390_593, 0.866_025_403_784],
        1e-9,
    );
}

#[test]
fn axis_is_always_unit_length() {
    // 刀轴必须恒为单位向量——否则 sd_profile 的距离会有系统性偏差
    let m = MachineTopology::ac_trunnion();
    for a in [-110.0, -45.0, 0.0, 30.0, 90.0, 110.0] {
        for c in [-180.0, 0.0, 45.0, 270.0] {
            let p = forward(&m, &m.q(&[("A", a), ("C", c)]));
            assert!(
                (p.axis.length() - 1.0).abs() < 1e-12,
                "A={a} C={c} 时刀轴长度 {}",
                p.axis.length()
            );
        }
    }
}

#[test]
fn q_builder_uses_chain_order() {
    // 链顺序是 [C, A, X, Y, Z]——按名字构造才能避免记错
    let m = MachineTopology::ac_trunnion();
    assert_eq!(m.index_of("C"), Some(0));
    assert_eq!(m.index_of("A"), Some(1));
    assert_eq!(m.index_of("X"), Some(2));
    assert_eq!(m.index_of("Z"), Some(4));
    let q = m.q(&[("Z", -50.0), ("A", 90.0)]);
    assert_eq!(q, vec![0.0, 90.0, 0.0, 0.0, -50.0]);
}
