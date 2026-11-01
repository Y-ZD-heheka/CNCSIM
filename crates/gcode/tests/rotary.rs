//! 五轴相关：A/B/C 轴、倾斜工作平面（G68.2）、程序停止。
//!
//! 这三件事的共同点：**都影响后续所有块的几何**，而且"看起来对、其实反了"的
//! 错误特别多。所以每条都配一个手算得出来的数。

use cncsim_gcode::{InterpConfig, RotaryConvention, StopKind, interpret};
use cncsim_math::DVec3;

fn cfg() -> InterpConfig {
    InterpConfig::default()
}

fn cfg_head() -> InterpConfig {
    InterpConfig {
        rotary: RotaryConvention::Head,
        ..InterpConfig::default()
    }
}

fn tip(prog: &cncsim_gcode::MotionProgram, i: usize) -> DVec3 {
    prog.blocks[i].target.tip
}

fn axis(prog: &cncsim_gcode::MotionProgram, i: usize) -> DVec3 {
    prog.blocks[i].target.axis
}

fn close(a: DVec3, b: DVec3, tol: f64) -> bool {
    (a - b).length() < tol
}

// ─────────────────────────── A/B/C 轴 ───────────────────────────

#[test]
fn a_axis_tilts_the_tool_on_both_conventions() {
    let src = "G0 X0 Y0 Z10 A30.\n";

    // 双转台：工件绕 X 转 +30°，刀轴在工件坐标系里是 Rx(−30)·Z
    let table = interpret(src, &cfg());
    let a_t = axis(&table, 0);
    assert!(
        close(a_t, DVec3::new(0.0, 0.5, 0.8660254), 1e-6),
        "双转台 A30 的刀轴应是 (0, 0.5, 0.866)，实际 {a_t:?}"
    );

    // 双摆头：刀具绕 X 转 +30°，刀轴是 Rx(+30)·Z —— Y 分量反号
    let head = interpret(src, &cfg_head());
    let a_h = axis(&head, 0);
    assert!(
        close(a_h, DVec3::new(0.0, -0.5, 0.8660254), 1e-6),
        "双摆头 A30 的刀轴应是 (0, −0.5, 0.866)，实际 {a_h:?}"
    );

    // 两者的 Y 分量必须反号 —— 这是"转工件 vs 转刀具"唯一的可见差别
    assert!((a_t.y + a_h.y).abs() < 1e-9);
}

#[test]
fn angles_are_modal_and_do_not_disturb_xyz() {
    let src = "G0 X1 Y2 Z3 A45.\nG1 X10 F100\n";
    let prog = interpret(src, &cfg());
    assert_eq!(prog.blocks.len(), 2);
    // 第二个块没有 A 字，角度沿用 45°
    assert!((prog.blocks[1].angles[0] - 45f64.to_radians()).abs() < 1e-12);
    // XYZ 不受角度影响
    assert!(close(tip(&prog, 1), DVec3::new(10.0, 2.0, 3.0), 1e-12));
}

#[test]
fn incremental_angles_accumulate() {
    // 每个块都要有 XYZ：`G0 A10.` 只有角度字，不产生运动块
    let src = "G0 X1 A10.\nG91 X1 A5.\nG90 X1 A0.\n";
    let prog = interpret(src, &cfg());
    let deg = |i: usize| prog.blocks[i].angles[0].to_degrees();
    assert!((deg(0) - 10.0).abs() < 1e-9, "实际 {}", deg(0));
    assert!((deg(1) - 15.0).abs() < 1e-9, "增量应累加到 15°，实际 {}", deg(1));
    assert!((deg(2) - 0.0).abs() < 1e-9, "G90 之后是绝对值，实际 {}", deg(2));
}

#[test]
fn angle_word_order_within_a_block_does_not_matter() {
    // `G91 A30 G1 X10` 与 `G1 X10 G91 A30` 必须完全等价
    let a = interpret("G0 X0\nG91 A30 G1 X10\n", &cfg());
    let b = interpret("G0 X0\nG1 X10 G91 A30\n", &cfg());
    assert_eq!(a.blocks.len(), b.blocks.len());
    let (a1, b1) = (&a.blocks[1], &b.blocks[1]);
    assert!((a1.angles[0] - b1.angles[0]).abs() < 1e-15);
    assert!(close(a1.target.tip, b1.target.tip, 1e-15));
    assert!(close(a1.target.axis, b1.target.axis, 1e-15));
    // 而且 X 必须是增量（10 而不是绝对 10）
    assert!(close(a1.target.tip, DVec3::new(10.0, 0.0, 0.0), 1e-12));
}

#[test]
fn c_axis_does_not_tilt_the_tool_but_changes_nothing_in_z() {
    // C 绕 Z 转：Z 轴不受影响，刀轴仍是 +Z
    let prog = interpret("G0 C90.\n", &cfg());
    assert!(
        close(axis(&prog, 0), DVec3::new(0.0, 0.0, 1.0), 1e-12),
        "C 轴转动不该改变刀轴，实际 {:?}",
        axis(&prog, 0)
    );
    assert!((prog.blocks[0].angles[2] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
}

#[test]
fn b_and_c_compose_in_zyx_order() {
    // B90 C90：R = Rz(90)·Ry(90)
    // 刀轴 = Rᵀ·Z = Ry(−90)·Rz(−90)·Z = Ry(−90)·Z = (−1, 0, 0)
    let prog = interpret("G0 B90. C90.\n", &cfg());
    assert!(
        close(axis(&prog, 0), DVec3::new(-1.0, 0.0, 0.0), 1e-9),
        "B90 C90 的刀轴应是 (−1,0,0)，实际 {:?}",
        axis(&prog, 0)
    );
}

// ─────────────────────── 倾斜工作平面 G68.2 ───────────────────────

#[test]
fn twp_rotates_subsequent_moves() {
    // 绕 X 转 90° 的倾斜平面：程序里的 +Z 变成工件坐标的 −Y
    let src = "G68.2 X0 Y0 Z0 I0 J90 K0\nG0 Z10\n";
    let prog = interpret(src, &cfg());
    let p = tip(&prog, 0);
    assert!(
        close(p, DVec3::new(0.0, -10.0, 0.0), 1e-9),
        "程序 Z10 在绕 X 转 90° 的平面里应落在 (0,−10,0)，实际 {p:?}"
    );
}

#[test]
fn twp_keeps_unspecified_axes_in_program_space() {
    // 只给 X 时，Y/Z 必须保持**程序坐标**，不能被拽回平面外
    let src = "G68.2 I0 J90 K0\nG0 Z10\nG0 X5\n";
    let prog = interpret(src, &cfg());
    assert!(close(tip(&prog, 0), DVec3::new(0.0, -10.0, 0.0), 1e-9));
    assert!(
        close(tip(&prog, 1), DVec3::new(5.0, -10.0, 0.0), 1e-9),
        "只给 X 时 Y 应保持 −10（= 程序 Y0 + Z10 的投影），实际 {:?}",
        tip(&prog, 1)
    );
}

#[test]
fn twp_incremental_moves_are_rotated_too() {
    let src = "G68.2 I0 J90 K0\nG91\nG0 Z10\n";
    let prog = interpret(src, &cfg());
    // 增量 Z10 在倾斜平面里也是 +Z 方向 → 工件坐标 −Y
    assert!(
        close(tip(&prog, 0), DVec3::new(0.0, -10.0, 0.0), 1e-9),
        "实际 {:?}",
        tip(&prog, 0)
    );
}

#[test]
fn twp_offsets_the_plane_origin() {
    let src = "G68.2 X1 Y2 Z3 I0 J90 K0\nG0 X0 Y0 Z0\n";
    let prog = interpret(src, &cfg());
    assert!(
        close(tip(&prog, 0), DVec3::new(1.0, 2.0, 3.0), 1e-9),
        "程序原点应落在倾斜平面原点上，实际 {:?}",
        tip(&prog, 0)
    );
}

#[test]
fn g69_restores_the_unrotated_frame() {
    // G69 之后重新给全 XYZ：倾斜期间的物理位置是 (0,−10,0)，
    // 只给 Z 的话 X/Y 会保持在 −10（那本身是对的，但测不出"恢复了坐标系"）
    let src = "G68.2 I0 J90 K0\nG0 Z10\nG69\nG0 X0 Y0 Z20\n";
    let prog = interpret(src, &cfg());
    assert!(close(tip(&prog, 0), DVec3::new(0.0, -10.0, 0.0), 1e-9), "倾斜时应偏转");
    assert!(close(tip(&prog, 1), DVec3::new(0.0, 0.0, 20.0), 1e-9), "G69 之后应恢复");
}

#[test]
fn twp_rotates_the_tool_axis_too() {
    // 倾斜平面绕 X 转 90° → 刀轴（程序 +Z）在工件坐标里是 −Y
    let prog = interpret("G68.2 I0 J90 K0\nG0 X0 Y0 Z0\n", &cfg());
    assert!(
        close(axis(&prog, 0), DVec3::new(0.0, -1.0, 0.0), 1e-9),
        "刀轴也应跟着倾斜，实际 {:?}",
        axis(&prog, 0)
    );
}

#[test]
fn g53_is_not_affected_by_twp() {
    // G53 走机床坐标，是绝对物理位置
    let mut c = cfg();
    c.machine_origin = DVec3::new(100.0, 0.0, 0.0);
    // 没给的轴保持当前位置，所以三个都显式给
    let prog = interpret("G68.2 I0 J90 K0\nG53 G0 X0 Y0 Z10\n", &c);
    assert!(
        close(tip(&prog, 0), DVec3::new(100.0, 0.0, 10.0), 1e-9),
        "G53 应绕过倾斜平面、且相对机床原点，实际 {:?}",
        tip(&prog, 0)
    );
}

// ─────────────────────────── 程序停止 ───────────────────────────

#[test]
fn program_end_is_recorded_as_a_stop_point() {
    let prog = interpret("G0 X10\nG1 X20 F100\nM30\n", &cfg());
    assert_eq!(prog.blocks.len(), 2);
    assert_eq!(prog.stops.len(), 1);
    let s = prog.stops[0];
    assert_eq!(s.kind, StopKind::ProgramEndReset);
    assert_eq!(s.block, 2, "M30 在最后一块之后");
    assert_eq!(s.line_no, 3);
}

#[test]
fn m0_mid_program_stops_at_the_right_block() {
    let prog = interpret("G0 X10\nM0\nG1 X20 F100\n", &cfg());
    assert_eq!(prog.blocks.len(), 2);
    assert_eq!(prog.stops.len(), 1);
    assert_eq!(prog.stops[0].kind, StopKind::ProgramStop);
    assert_eq!(prog.stops[0].block, 1, "M0 之后的第一块是第 1 号块");
}

#[test]
fn m1_is_optional_stop() {
    let prog = interpret("G0 X10\nM1\n", &cfg());
    assert_eq!(prog.stops[0].kind, StopKind::OptionalStop);
}

#[test]
fn a_program_without_stops_has_none() {
    let prog = interpret("G0 X10\nG1 X20 F100\n", &cfg());
    assert!(prog.stops.is_empty());
}
