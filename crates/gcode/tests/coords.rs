//! 坐标数据：G10 L2/L20、G52、G92.x。
//!
//! 这三条**不动刀**，只改"程序坐标怎么映射到机床位置"。它们一旦算错，
//! 后续**所有**绝对坐标都会静默偏移——每一块的形状都对，只是整体挪了位置。
//! 所以每个用例都同时断言"位置读对了"和"刀没动"。

use cncsim_gcode::{InterpConfig, MotionProgram, interpret};
use cncsim_math::DVec3;

fn cfg() -> InterpConfig {
    InterpConfig::default()
}

fn tip(p: &MotionProgram, i: usize) -> DVec3 {
    p.blocks[i].target.tip
}

fn last(p: &MotionProgram) -> DVec3 {
    p.blocks.last().expect("应有块").target.tip
}

#[test]
fn g10_l2_sets_the_wcs_origin() {
    // G54 原点设到机床 (10, 0, 5)：之后 X0Y0Z0 就落在那里
    let p = interpret("G10 L2 P1 X10 Y0 Z5\nG0 X0 Y0 Z0\n", &cfg());
    assert_eq!(p.blocks.len(), 1, "G10 本身不动刀");
    assert!((tip(&p, 0) - DVec3::new(10.0, 0.0, 5.0)).length() < 1e-12);
}

#[test]
fn g10_l2_leaves_unspecified_axes_alone() {
    let p = interpret(
        "G10 L2 P1 X10 Y20 Z30\nG10 L2 P1 X0\nG0 X0 Y0 Z0\n",
        &cfg(),
    );
    // 只改了 X，Y/Z 保持上次的 20/30
    assert!((last(&p) - DVec3::new(0.0, 20.0, 30.0)).length() < 1e-12, "实际 {:?}", last(&p));
}

#[test]
fn g10_l2_incremental_adds_to_the_existing_offset() {
    let p = interpret("G10 L2 P1 X10\nG91\nG10 L2 P1 X5\nG90\nG0 X0\n", &cfg());
    assert!((last(&p).x - 15.0).abs() < 1e-12, "实际 {}", last(&p).x);
}

#[test]
fn g10_l2_p0_targets_the_current_system() {
    // 切到 G55（索引 2）再写 P0，改的应该是 G55，G54 不动
    let p = interpret("G55\nG10 L2 P0 X7\nG54\nG0 X0\n", &cfg());
    assert!((last(&p).x - 0.0).abs() < 1e-12, "G54 不该被改：{}", last(&p).x);
    let q = interpret("G55\nG10 L2 P0 X7\nG55\nG0 X0\n", &cfg());
    assert!((last(&q).x - 7.0).abs() < 1e-12, "G55 应被改为 7：{}", last(&q).x);
}

#[test]
fn g10_l20_makes_the_current_point_read_as_given() {
    // 对刀的标准流程：碰到工件表面，然后说"这里就是 Z0"
    let p = interpret("G0 X50 Y20 Z30\nG10 L20 P1 X0 Y0 Z0\nG0 X0 Y0 Z0\n", &cfg());
    assert_eq!(p.blocks.len(), 2, "G10 不产生运动");
    // 两次的目标点是**同一个物理位置**
    assert!((tip(&p, 0) - DVec3::new(50.0, 20.0, 30.0)).length() < 1e-12);
    assert!(
        (last(&p) - DVec3::new(50.0, 20.0, 30.0)).length() < 1e-12,
        "原点移到当前位置之后，X0Y0Z0 应回到同一点，实际 {:?}",
        last(&p)
    );
}

#[test]
fn g10_l20_only_touches_given_axes() {
    let p = interpret(
        "G0 X50 Y20 Z30\nG10 L20 P1 Z0\nG0 X0 Y0 Z0\n",
        &cfg(),
    );
    // 只把 Z 归零：X/Y 的原点仍在机床 0，所以 X0Y0 落在 (0,0)，Z0 落在 30
    assert!(
        (last(&p) - DVec3::new(0.0, 0.0, 30.0)).length() < 1e-12,
        "实际 {:?}",
        last(&p)
    );
}

#[test]
fn g10_l20_accounts_for_g52_and_g92() {
    // 先叠一个 G52，再对刀：原点必须把 G52 减掉，否则会差一个 G52 的量
    let p = interpret(
        "G52 X5\nG0 X10\nG10 L20 P1 X0\nG52 X0\nG0 X0\n",
        &cfg(),
    );
    // G52=5 → 程序 X10 落在物理 15。对刀（X0）后原点 = 15 − 0 − 5 = **10**；
    // 取消 G52 之后 X0 落在 10。若不减 G52，这里会是 15 —— 差一个 G52 的量。
    assert!((last(&p).x - 10.0).abs() < 1e-12, "实际 {}", last(&p).x);
}

#[test]
fn g10_tool_table_is_unsupported() {
    for l in [1, 10, 11] {
        let p = interpret(&format!("G10 L{l} P1 R3.0\n"), &cfg());
        assert!(
            p.issues.iter().any(|i| i.id == "GCODE-005"),
            "G10 L{l} 是刀表，应该明确报不支持"
        );
    }
}

#[test]
fn g52_shifts_and_cancels() {
    let p = interpret("G0 X10\nG52 X5\nG0 X0\nG52 X0\nG0 X0\n", &cfg());
    assert!((tip(&p, 0).x - 10.0).abs() < 1e-12);
    assert!((tip(&p, 1).x - 5.0).abs() < 1e-12, "G52 X5 之后 X0 落在 5");
    assert!((tip(&p, 2).x - 0.0).abs() < 1e-12, "G52 X0 取消后回到 0");
}

#[test]
fn g52_is_layered_on_top_of_the_wcs() {
    let p = interpret("G10 L2 P1 X100\nG52 X5\nG0 X0\n", &cfg());
    assert!((last(&p).x - 105.0).abs() < 1e-12, "实际 {}", last(&p).x);
}

#[test]
fn g92_makes_the_current_point_read_as_zero() {
    let p = interpret("G0 X50 Z50\nG92 X0 Z0\nG0 X10 Z10\n", &cfg());
    assert!((tip(&p, 0) - DVec3::new(50.0, 0.0, 50.0)).length() < 1e-12);
    assert!(
        (last(&p) - DVec3::new(60.0, 0.0, 60.0)).length() < 1e-12,
        "G92 之后 X10 应到物理 60，实际 {:?}",
        last(&p)
    );
}

#[test]
fn g92_1_cancels_and_g92_2_suspends() {
    let p = interpret(
        "G0 X50\nG92 X0\nG92.1\nG0 X0\n",
        &cfg(),
    );
    assert!((last(&p).x - 0.0).abs() < 1e-12, "G92.1 应清零，实际 {}", last(&p).x);

    let q = interpret(
        "G0 X50\nG92 X0\nG92.2\nG0 X0\nG92.3\nG0 X0\n",
        &cfg(),
    );
    assert!((q.blocks[1].target.tip.x - 0.0).abs() < 1e-12, "G92.2 挂起后 X0 落在 0，实际 {}", q.blocks[1].target.tip.x);
    assert!((q.blocks[2].target.tip.x - 50.0).abs() < 1e-12, "G92.3 恢复后 X0 落回 50，实际 {}", q.blocks[2].target.tip.x);
}

#[test]
fn g92_is_layered_on_top_of_g52() {
    let p = interpret("G52 X5\nG0 X10\nG92 X0\nG0 X0\n", &cfg());
    // G52=5 → 程序 X10 落在物理 15。G92 把当前位置读作 X0，
    // 所以再走 X0 应该**原地不动**（15）。这是 G92 的定义，不是巧合。
    assert!((last(&p).x - 15.0).abs() < 1e-12, "实际 {}", last(&p).x);
}

#[test]
fn coordinate_changes_do_not_move_the_tool() {
    // G10 / G52 / G92 全都不该产生运动块
    let p = interpret("G0 X10\nG10 L2 P1 X3\nG52 X1\nG92 X2\nG0 X10\n", &cfg());
    assert_eq!(p.blocks.len(), 2, "只有两条 G0 产生运动");
}

#[test]
fn interp_config_stays_clean_across_runs() {
    // G10 改的是解释器内部的副本。同一份 cfg 跑两遍必须给出一样的结果，
    // 否则第二遍的坐标就已经被上一遍污染了 —— 这类 bug 只在跑第二个程序时出现
    let ic = cfg();
    let src = "G10 L2 P1 X100\nG0 X0\n";
    let a = interpret(src, &ic);
    let b = interpret(src, &ic);
    assert_eq!(a.blocks.len(), b.blocks.len());
    assert_eq!(a.blocks[0].target.tip, b.blocks[0].target.tip);
    assert_eq!(ic.work_offsets.get(1), DVec3::ZERO, "cfg 被改写了");
}

#[test]
fn g10_keeps_the_program_trusted() {
    // 支持的坐标改写不该把整份程序标成不可信
    let p = interpret("G10 L2 P1 X10\nG0 X0\n", &cfg());
    assert!(
        matches!(p.trust, cncsim_gcode::Trust::Reliable),
        "实际 {:?}",
        p.trust
    );
}
