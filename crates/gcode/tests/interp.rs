//! 解释器集成测试。
//!
//! 这些测试专门覆盖**组合块**——单个字都对、组合起来才错的情形。
//! 每个断言都能区分"做对了"和"做错了"，不是 `is_some()` 这类空断言。

use cncsim_gcode::{DistanceMode, InterpConfig, Severity, Trust, interpret};
use cncsim_math::DVec3;
use cncsim_motion::MotionKind;

fn cfg() -> InterpConfig {
    InterpConfig::default()
}

/// **最重要的一条**：LinuxCNC `overview.adoc` §Item order 明文——
/// words 在行内可以任意重排而不改变含义。
///
/// 一个"按字的位置顺序处理"的实现会让这两个程序产生不同结果。
#[test]
fn word_order_within_a_block_does_not_matter() {
    let a = interpret("G0 X100\nG91 G1 X10 F100\n", &cfg());
    let b = interpret("G0 X100\nG1 X10 G91 F100\n", &cfg());

    assert_eq!(a.len(), 2, "两份都应产出 2 块");
    assert_eq!(a.len(), b.len());
    for (x, y) in a.blocks.iter().zip(&b.blocks) {
        assert_eq!(x.motion, y.motion, "第 {} 块运动类型不同", x.line_no);
        assert_eq!(
            x.target.tip, y.target.tip,
            "第 {} 块目标点不同：{:?} vs {:?}（说明实现依赖了字在行内的位置）",
            x.line_no, x.target.tip, y.target.tip
        );
        assert_eq!(x.tool_length, y.tool_length, "第 {} 块刀长不同", x.line_no);
    }
}

/// G91 必须在**本块运动之前**生效。
///
/// 若实现先读状态的 `distance` 再处理本行的 G91，会算成绝对坐标 X=10。
#[test]
fn incremental_mode_applies_to_the_same_block() {
    let p = interpret("G0 X100\nG91 G1 X10 F100\n", &cfg());
    let b = &p.blocks[1];
    assert!(
        (b.target.tip.x - 110.0).abs() < 1e-12,
        "G91 应增量：100+10=110，实际 {}",
        b.target.tip.x
    );
}

#[test]
fn absolute_mode_restores_after_incremental() {
    let p = interpret("G0 X100\nG91 G1 X10\nG90 G1 X5\n", &cfg());
    assert!((p.blocks[1].target.tip.x - 110.0).abs() < 1e-12);
    assert!(
        (p.blocks[2].target.tip.x - 5.0).abs() < 1e-12,
        "G90 回到绝对：X5，实际 {}",
        p.blocks[2].target.tip.x
    );
}

/// G43 必须在**本块运动之前**生效——否则第一刀偏一个刀长。
#[test]
fn tool_length_is_applied_before_motion() {
    let mut c = cfg();
    c.tool_lengths[7] = 100.0;
    let p = interpret("G43 H7 G1 Z-5 F100\n", &c);
    let b = &p.blocks[0];
    assert_eq!(b.tool_length, 100.0, "本行 G43 H7 必须在本行生效");
    assert!(
        (b.target.tip.z - (-5.0)).abs() < 1e-12,
        "G43 改的是机床补偿，不改程序坐标：Z 应仍是 -5，实际 {}",
        b.target.tip.z
    );
}

/// 同一行换 H 号，本块必须用新的刀长——不能沿用上一块的。
#[test]
fn tool_length_is_not_stale() {
    let mut c = cfg();
    c.tool_lengths[1] = 100.0;
    c.tool_lengths[2] = 200.0;
    let p = interpret("G43 H1 G1 Z-5 F100\nG43 H2 G1 Z-6\n", &c);
    assert_eq!(p.blocks[0].tool_length, 100.0);
    assert_eq!(
        p.blocks[1].tool_length, 200.0,
        "第二块的 H2 必须立即生效，不能还是 H1 的 100"
    );
}

#[test]
fn g49_clears_tool_length() {
    let mut c = cfg();
    c.tool_lengths[1] = 100.0;
    let p = interpret("G43 H1 G1 Z-5\nG49 G1 Z-6\n", &c);
    assert_eq!(p.blocks[0].tool_length, 100.0);
    assert_eq!(p.blocks[1].tool_length, 0.0, "G49 取消刀长");
}

/// G20 必须在**本块**把 X1 解释成 1 英寸。
#[test]
fn units_apply_to_the_same_block() {
    let p = interpret("G20 G1 X1 F10\n", &cfg());
    assert!(
        (p.blocks[0].target.tip.x - 25.4).abs() < 1e-9,
        "G20 下 X1 = 25.4 mm，实际 {}",
        p.blocks[0].target.tip.x
    );
}

#[test]
fn units_are_modal() {
    let p = interpret("G21 G1 X1\nG1 X2\n", &cfg());
    assert!((p.blocks[0].target.tip.x - 1.0).abs() < 1e-12);
    assert!((p.blocks[1].target.tip.x - 2.0).abs() < 1e-12, "G21 是模态的");
}

#[test]
fn plane_is_modal_for_arcs() {
    let p = interpret("G0 X10 Y0 Z0\nG18\nG2 X0 Z10 I-10 K0\n", &cfg());
    match p.blocks[1].motion {
        MotionKind::Arc { plane, clockwise, .. } => {
            assert_eq!(plane, cncsim_motion::Plane::Zx, "G18 选中 ZX 平面");
            assert!(clockwise, "G2 是顺时针");
        }
        other => panic!("应为圆弧，实际 {other:?}"),
    }
}

/// R 格式圆弧：从 (10,0) 到 (0,10)、R10、逆时针 → 圆心必须是 (0,0)。
#[test]
fn arc_r_format_center_is_correct() {
    let p = interpret("G0 X10 Y0 Z0\nG3 X0 Y10 R10 F100\n", &cfg());
    match p.blocks[1].motion {
        MotionKind::Arc { center, clockwise, .. } => {
            assert!(!clockwise, "G3 是逆时针");
            assert!(
                center.abs_diff(DVec3::new(0.0, 0.0, 0.0)) < 1e-9,
                "圆心应为 (0,0,0)，实际 {center:?}"
            );
        }
        other => panic!("应为圆弧，实际 {other:?}"),
    }
}

/// 组 0 轴字代码（G28）会**挂起**组 1 的运动——不能多走一段。
#[test]
fn group_zero_suspends_motion() {
    let p = interpret("G0 X10 Y0 Z0\nG28 X0 Y0\n", &cfg());
    assert_eq!(p.len(), 1, "第二行不应产出运动块");
    assert!(
        !p.issues_with("GCODE-010").is_empty(),
        "应报「组 1 被挂起」，实际问题：{:?}",
        p.issues.iter().map(|i| i.id).collect::<Vec<_>>()
    );
}

/// 组 0 与组 1 同行都用轴字是错误。
#[test]
fn group_zero_and_motion_axis_words_conflict() {
    let p = interpret("G1 X10 G28 X0\n", &cfg());
    assert_eq!(p.len(), 0, "冲突行不应产出运动块");
    assert!(!p.issues_with("GCODE-004").is_empty(), "应报组 0/1 冲突");
    assert!(matches!(p.trust, Trust::Unreliable { .. }), "结果不可信");
}

/// 刀补不做，但必须标不可信——路径会偏一个刀半径。
#[test]
fn cutter_comp_marks_unreliable() {
    let p = interpret("G41 D1 G1 X10 F100\n", &cfg());
    let issues = p.issues_with("GCODE-005");
    assert!(!issues.is_empty(), "G41 应报不支持");
    assert!(issues[0].severity >= Severity::Error);
    assert!(issues[0].hint.is_some(), "每条报错必须有 hint");
    assert!(
        matches!(p.trust, Trust::Unreliable { first_bad_line: 1 }),
        "刀补被跳过会让路径偏一个刀半径，必须标不可信；实际 {:?}",
        p.trust
    );
}

#[test]
fn dwell_block_is_emitted() {
    let p = interpret("G4 P1.5\n", &cfg());
    assert_eq!(p.len(), 1);
    match p.blocks[0].motion {
        MotionKind::Dwell { seconds } => assert!((seconds - 1.5).abs() < 1e-12),
        other => panic!("应为停延，实际 {other:?}"),
    }
}

/// WCS 偏置在解析时吸收：G55 下 X0 在统一坐标系里是 offset。
#[test]
fn wcs_offset_is_absorbed() {
    let mut c = cfg();
    c.work_offsets.wcs[1] = DVec3::new(100.0, 0.0, 0.0); // G55
    let p = interpret("G55 G1 X0 Y0 Z0 F100\n", &c);
    let b = &p.blocks[0];
    assert!((b.target.tip.x - 100.0).abs() < 1e-12, "G55 的偏置应已吸收");
    assert_eq!(b.wcs, 2, "wcs 字段仅供显示：G55 = 2");
}

#[test]
fn wcs_switch_keeps_physical_position() {
    let mut c = cfg();
    c.work_offsets.wcs[0] = DVec3::ZERO; // G54
    c.work_offsets.wcs[1] = DVec3::new(100.0, 0.0, 0.0); // G55
    // 在 G54 里走到 X10（统一坐标 10），切 G55 后物理位置不变，
    // 所以 G55 下不写轴字的 G1 目标仍是统一坐标 10。
    let p = interpret("G1 X10 F100\nG55 G1 X0\n", &c);
    assert!((p.blocks[0].target.tip.x - 10.0).abs() < 1e-12);
    assert!(
        (p.blocks[1].target.tip.x - 100.0).abs() < 1e-12,
        "G55 下 X0 = 统一坐标 100，实际 {}",
        p.blocks[1].target.tip.x
    );
}

#[test]
fn modal_state_persists_across_blank_and_comment_lines() {
    let p = interpret("G91\n(注释)\n\nG1 X10 F100\n", &cfg());
    assert_eq!(p.len(), 1);
    assert!((p.blocks[0].target.tip.x - 10.0).abs() < 1e-12, "G91 跨空行/注释仍生效");
}

#[test]
fn omitted_axes_keep_their_value() {
    let p = interpret("G1 X10 Y20 Z30 F100\nG1 X40\n", &cfg());
    let b = &p.blocks[1];
    assert!((b.target.tip.x - 40.0).abs() < 1e-12);
    assert!((b.target.tip.y - 20.0).abs() < 1e-12, "省略的 Y 保持 20");
    assert!((b.target.tip.z - 30.0).abs() < 1e-12, "省略的 Z 保持 30");
}

#[test]
fn unknown_gcode_is_reported_with_hint() {
    let p = interpret("G123 X1\n", &cfg());
    let i = p.issues_with("GCODE-001");
    assert!(!i.is_empty(), "未知 G 代码应报 GCODE-001");
    assert!(i[0].hint.is_some());
    assert_eq!(i[0].line, 1);
}

#[test]
fn feed_mode_g93_is_inverse_time() {
    let p = interpret("G93 G1 X10 F0.5\n", &cfg());
    assert_eq!(
        p.blocks[0].feed.mode,
        cncsim_motion::FeedMode::InverseTime,
        "G93 下 F 是倒数时间"
    );
    assert!((p.blocks[0].feed.value - 0.5).abs() < 1e-12);
}

#[test]
fn g53_uses_machine_origin() {
    let mut c = cfg();
    c.machine_origin = DVec3::new(-500.0, 0.0, 0.0);
    let p = interpret("G53 G1 X10 F100\n", &c);
    assert!(
        (p.blocks[0].target.tip.x - (-490.0)).abs() < 1e-12,
        "G53 是机床坐标：-500+10=-490，实际 {}",
        p.blocks[0].target.tip.x
    );
}

#[test]
fn rtcp_flag_is_carried_on_block() {
    let p = interpret("G43.4 H1 G1 X10 F100\n", &cfg());
    assert!(p.blocks[0].rtcp, "G43.4 打开 RTCP，Block 必须带上这个标志");
}

#[test]
fn distance_mode_enum_is_reachable() {
    // 保证类型确实导出，供 sim 判断插补空间
    let _ = DistanceMode::Absolute;
    let _ = DistanceMode::Incremental;
}

// ─────────────────────── 隐式模态运动 ───────────────────────

#[test]
fn implicit_modal_motion_runs_without_a_g_word() {
    // 真实后处理的绝大多数行是这样的：只有轴字，没有 G 字。
    // 不认这条，整份程序只有"带 G 的行"会动，而且不报任何错。
    let p = interpret("G1 X10 F100\nX20\nY5\n", &cfg());
    assert_eq!(p.blocks.len(), 3, "三行都该产生运动，实际 {} 块", p.blocks.len());
    assert!((p.blocks[1].target.tip.x - 20.0).abs() < 1e-12);
    assert!((p.blocks[2].target.tip.y - 5.0).abs() < 1e-12);
    // 模态是 G1，不是 G0
    for b in &p.blocks {
        assert!(matches!(b.motion, cncsim_motion::MotionKind::Linear));
    }
}

#[test]
fn implicit_motion_uses_the_current_modal_kind() {
    // G0 之后只给轴字 → 快移
    let p = interpret("G0 X10\nX20\n", &cfg());
    assert_eq!(p.blocks.len(), 2);
    for b in &p.blocks {
        assert!(matches!(b.motion, cncsim_motion::MotionKind::Rapid));
    }
}

#[test]
fn implicit_motion_is_not_emitted_for_group0_axis_words() {
    // G92 也用轴字，但它不是运动
    let p = interpret("G1 X10 F100\nG92 X0\n", &cfg());
    assert_eq!(p.blocks.len(), 1, "G92 不该产生运动块");
}

#[test]
fn implicit_motion_survives_incremental_mode() {
    // G91 + 隐式运动：每一步都是相对上一步
    let p = interpret("G91\nG1 X10 F100\nX10\nX10\n", &cfg());
    assert_eq!(p.blocks.len(), 3);
    assert!((p.blocks[0].target.tip.x - 10.0).abs() < 1e-12);
    assert!((p.blocks[1].target.tip.x - 20.0).abs() < 1e-12);
    assert!((p.blocks[2].target.tip.x - 30.0).abs() < 1e-12);
}

#[test]
fn implicit_motion_does_not_fire_after_g80() {
    // G80 取消固定循环后，模态运动不是 G0–G3，只给轴字的行不该凭空走
    let p = interpret("G80\nX10\n", &cfg());
    assert!(p.blocks.is_empty(), "G80 之后没有模态运动，实际 {} 块", p.blocks.len());
}
