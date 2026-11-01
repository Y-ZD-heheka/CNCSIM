//! G 代码 → 位姿 → 逆解 → 正解：**程序能不能在这台机床上执行**。
//!
//! 在此之前，`gcode` 产出位姿、`kin` 验证机床，两条线各测各的——没有任何东西
//! 证明"解释器算出的刀路，这台机床跟得上"。这一片把它们接上（放 `sim` 是因为
//! 装配层才允许同时碰这两个 crate，`kin → gcode` 会违反依赖方向）：
//!
//! 1. 每个块的位姿逆解必须收敛（机床能到达）；
//! 2. 正解必须回到同一个位姿（RTCP 闭环）；
//! 3. 轴不超限位 / 奇异点要被 `check` 报出来。
//!
//! 第 1/2 条抓的是**约定不一致**：解释器算的刀轴方向和机床运动学对不上，
//! 在这里表现为逆解不收敛或正解回不到原点。

use cncsim_gcode::{InterpConfig, RotaryConvention, interpret};
use cncsim_kin::{MachineTopology, forward, inverse};
use cncsim_math::DVec3;

fn topo() -> MachineTopology {
    MachineTopology::ac_trunnion()
}

fn ic() -> InterpConfig {
    InterpConfig::default()
}

/// 逆解整个程序：返回每块的正解位姿（顺便验证收敛与闭环）。
fn run_inverse(
    prog: &cncsim_gcode::MotionProgram,
    topo: &MachineTopology,
) -> Vec<cncsim_motion::Pose> {
    let n = topo.chain.len();
    let mut q = vec![0.0; n];
    let mut out = Vec::new();
    for (i, b) in prog.blocks.iter().enumerate() {
        let Some(q_new) = inverse(topo, &b.target, &q) else {
            panic!(
                "第 {i} 块（第 {} 行）逆解不收敛：机床到不了这个位姿。tip={:?} axis={:?}",
                b.line_no, b.target.tip, b.target.axis
            );
        };
        let back = forward(topo, &q_new);
        let tip_err = (back.tip - b.target.tip).length();
        let axis_err = (back.axis - b.target.axis).length();
        assert!(
            tip_err < 1e-9 && axis_err < 1e-9,
            "第 {i} 块 RTCP 闭环失败：刀尖差 {tip_err:e}，刀轴差 {axis_err:e}\n  q={q_new:?}"
        );
        q = q_new;
        out.push(back);
    }
    out
}

#[test]
fn three_axis_program_round_trips() {
    // 纯三轴：A/C 都是 0。这条最容易在"角度约定"上翻车——归零后必须正好是 +Z
    let prog = interpret("G0 X0 Y0 Z100\nG1 X50 Y0 Z80 F500\nG2 X80 Y30 I30 J0\n", &ic());
    let poses = run_inverse(&prog, &topo());
    for (i, p) in poses.iter().enumerate() {
        assert!(
            (p.axis - DVec3::Z).length() < 1e-9,
            "第 {i} 块刀轴应是 +Z，实际 {:?}（解释器与机床的 A/C 归零约定不一致）",
            p.axis
        );
    }
}

#[test]
fn five_axis_program_round_trips() {
    // A/C 都动：这是"解释器的 Rᵀ·Z 与机床的 R⁻¹·Z 是同一个东西"的证明
    let src = "G0 X0 Y0 Z120 A0. C0.\
               \nG1 X50 Y0 Z80 A20. F500\
               \nG2 X80 Y30 I30 J0 A20. C45.\
               \nG1 X80 Y60 A-10. C90.\
               \nG1 X40 Y60 A-10. C135.";
    let prog = interpret(src, &ic());
    assert!(prog.blocks.len() >= 5, "应解析出 5 块，实际 {}", prog.blocks.len());
    run_inverse(&prog, &topo());
}

#[test]
fn incremental_angle_mode_round_trips() {
    // G91 的 A/C 是增量角，终角要和绝对写法一致
    let abs = interpret("G0 X0 Y0 Z120 A20. C45.\nG1 X50 Y0 Z80 F500\n", &ic());
    let inc = interpret(
        "G0 X0 Y0 Z120 A0. C0.\nG91\nG1 X0 A20. C45.\nG90\nG1 X50 Y0 Z80 F500\n",
        &ic(),
    );
    let t = topo();
    let a = run_inverse(&abs, &t);
    let b = run_inverse(&inc, &t);
    let last = |v: &[cncsim_motion::Pose]| *v.last().unwrap();
    assert!(
        (last(&a).tip - last(&b).tip).length() < 1e-12
            && (last(&a).axis - last(&b).axis).length() < 1e-12,
        "增量角与绝对角算出的刀轴不同：{:?} vs {:?}",
        last(&a).axis,
        last(&b).axis
    );
}

#[test]
fn dual_head_convention_still_inverts() {
    // 双摆头：刀轴方向反过来（A30 → (0,−sin,cos)）。机床拓扑没变，
    // 只是刀轴在工件系里的表达变了——约定切换不能让逆解炸掉。
    let ic = InterpConfig {
        rotary: RotaryConvention::Head,
        ..InterpConfig::default()
    };
    let prog = interpret("G0 X0 Y0 Z120 A20. C45.\nG1 X50 Y0 Z80 F500\n", &ic);
    run_inverse(&prog, &topo());
}

#[test]
fn near_singular_pose_is_reported_by_check() {
    // 摇篮式在 A=0 处奇异（C 转多少刀轴都不动）。逆解器在这种点上仍能收敛
    // （它解的是刀轴，A=0 是合法解），但 `check` 必须把 κ 超限报出来——
    // 实际加工时 C 轴会疯转。
    let topo = topo();
    let prog = interpret("G0 X0 Y0 Z120 A0. C0.\nG1 X50 Y0 Z80 F500\n", &ic());
    let n = topo.chain.len();
    let mut q = vec![0.0; n];
    let mut findings = 0;
    for b in &prog.blocks {
        let q_new = inverse(&topo, &b.target, &q).expect("A=0 应可解");
        findings += cncsim_kin::check(&topo, &q_new, 50.0).len();
        q = q_new;
    }
    assert!(findings > 0, "A=0 的摇篮机床应当报近奇异");
}
