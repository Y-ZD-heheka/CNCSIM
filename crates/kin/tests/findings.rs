//! 运动学诊断：奇异点与轴限位。
//!
//! **奇异的定义**：两个旋转轴对刀轴的影响趋于共线，雅可比退化——
//! 此时 C 轴要转很大角度才能让刀轴动一点，需求转速爆炸。
//!
//! 用**条件数** `κ = σ_max/σ_min` 度量，而不是硬编码 `|sin A| < eps`：
//! 前者对任何「2 旋转」构型都成立，后者是摇篮式的专用公式。

use cncsim_kin::{check, rotary_condition, KinFinding, MachineTopology};

/// 摇篮式的解析值：κ = 1/|sin A|。
#[test]
fn kappa_matches_analytic_for_trunnion() {
    let m = MachineTopology::ac_trunnion();
    for a in [5.0, 15.0, 30.0, 45.0, 60.0, 90.0] {
        let q = m.q(&[("A", a)]);
        let kappa = rotary_condition(&m, &q).expect("摇篮式是 2 旋转");
        let expect = 1.0 / a.to_radians().sin();
        assert!(
            (kappa - expect).abs() < 1e-3,
            "A={a}° 时 κ 应约 {expect:.3}，实际 {kappa:.3}"
        );
    }
}

/// A=0 是真奇异：κ 无穷大。
#[test]
fn kappa_is_infinite_at_a_zero() {
    let m = MachineTopology::ac_trunnion();
    let kappa = rotary_condition(&m, &m.q(&[("A", 0.0)])).unwrap();
    assert!(kappa.is_infinite(), "A=0 时 κ 应无穷大，实际 {kappa}");
}

/// A=90° 时最"健康"：κ = 1。
#[test]
fn kappa_is_one_at_a_ninety() {
    let m = MachineTopology::ac_trunnion();
    let kappa = rotary_condition(&m, &m.q(&[("A", 90.0)])).unwrap();
    assert!((kappa - 1.0).abs() < 1e-6, "A=90° 时 κ 应为 1，实际 {kappa}");
}

/// κ 与 C 无关——摇篮式的奇异只由 A 决定。
#[test]
fn kappa_is_independent_of_c() {
    let m = MachineTopology::ac_trunnion();
    let base = rotary_condition(&m, &m.q(&[("A", 30.0), ("C", 0.0)])).unwrap();
    for c in [45.0, 90.0, 180.0, 270.0] {
        let k = rotary_condition(&m, &m.q(&[("A", 30.0), ("C", c)])).unwrap();
        assert!((k - base).abs() < 1e-6, "C={c} 时 κ 变了：{k} vs {base}");
    }
}

/// 阈值 100 意味着 |sin A| < 0.01，即 A < 0.57°。
#[test]
fn near_singular_reported_below_threshold() {
    let m = MachineTopology::ac_trunnion();
    let limit = 100.0;

    // A=0.5° → κ ≈ 114.6 > 100，应报
    let f = check(&m, &m.q(&[("A", 0.5)]), limit);
    assert!(
        f.iter().any(|x| matches!(x, KinFinding::NearSingular { .. })),
        "A=0.5° 应报近奇异，实际 {f:?}"
    );

    // A=1° → κ ≈ 57.3 < 100，不报
    let f = check(&m, &m.q(&[("A", 1.0)]), limit);
    assert!(
        !f.iter().any(|x| matches!(x, KinFinding::NearSingular { .. })),
        "A=1° 不该报近奇异，实际 {f:?}"
    );
}

/// 正常位姿不该有任何 finding。
#[test]
fn healthy_pose_has_no_findings() {
    let m = MachineTopology::ac_trunnion();
    let q = m.q(&[("X", 100.0), ("Y", 50.0), ("Z", -20.0), ("A", 30.0), ("C", 45.0)]);
    let f = check(&m, &q, 100.0);
    assert!(f.is_empty(), "正常位姿不该有诊断，实际 {f:?}");
}

/// 超程要报出**具体哪个关节、当前值、限位**。
#[test]
fn axis_limit_reports_joint_and_values() {
    let m = MachineTopology::ac_trunnion();
    let q = m.q(&[("X", 500.0), ("A", 30.0)]); // X 行程 ±400
    let f = check(&m, &q, 100.0);

    let lim: Vec<_> = f
        .iter()
        .filter_map(|x| match x {
            KinFinding::AxisLimit { joint, value, limit } => Some((*joint, *value, *limit)),
            _ => None,
        })
        .collect();
    assert_eq!(lim.len(), 1, "应只报 X 超程，实际 {f:?}");
    let (joint, value, limit) = lim[0];
    assert_eq!(m.chain[joint].name, "X");
    assert_eq!(value, 500.0);
    assert_eq!(limit, (-400.0, 400.0));
}

/// 编号是稳定契约——测试断言编号，不断言消息文本。
#[test]
fn finding_ids_are_stable() {
    let m = MachineTopology::ac_trunnion();
    let f = check(&m, &m.q(&[("A", 0.0), ("X", 999.0)]), 100.0);
    let ids: Vec<_> = f.iter().map(|x| x.id()).collect();
    assert!(ids.contains(&"KIN-001"), "奇异点编号应为 KIN-001，实际 {ids:?}");
    assert!(ids.contains(&"KIN-002"), "限位编号应为 KIN-002，实际 {ids:?}");
}
