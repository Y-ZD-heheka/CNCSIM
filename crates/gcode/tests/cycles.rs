//! 固定循环（G73 / G76 / G81–G89）。
//!
//! 这一组最容易出的错不是"算错"，而是**少钻一个孔而报告一切正常**：
//! 循环是模态的，孔阵列的后处理每行只写坐标。所以测试的重点是
//! "每个孔都展开了"和"每个孔展开成什么形状"，而不是单个孔对不对。

use cncsim_gcode::{InterpConfig, MotionProgram, interpret};
use cncsim_motion::MotionKind;

fn cfg() -> InterpConfig {
    InterpConfig::default()
}

fn zs(p: &MotionProgram) -> Vec<f64> {
    p.blocks.iter().map(|b| b.target.tip.z).collect()
}

fn xs(p: &MotionProgram) -> Vec<f64> {
    p.blocks.iter().map(|b| b.target.tip.x).collect()
}

#[test]
fn g81_drills_one_hole() {
    // 起始 z=10，R=2，孔底 −8：快移下到 2 → 进给到 −8 → 快移回 10
    let p = interpret("G0 Z10\nG81 Z-8 R2 F100\n", &cfg());
    // 第 0 块是 `G0 Z10` 自己；循环展开成 ②③④ 三块
    assert_eq!(p.blocks.len(), 4, "实际 {} 块：{:?}", p.blocks.len(), zs(&p));
    assert_eq!(zs(&p), vec![10.0, 2.0, -8.0, 10.0]);
    assert!(matches!(p.blocks[1].motion, MotionKind::Rapid), "下到 R 平面是快速");
    assert!(matches!(p.blocks[2].motion, MotionKind::Linear), "下扎必须是进给");
    assert!(matches!(p.blocks[3].motion, MotionKind::Rapid), "退刀是快速");
    // 进给率要落在下扎那一块上
    assert!((p.blocks[2].feed.value - 100.0).abs() < 1e-12);
}

#[test]
fn g81_is_modal_across_holes() {
    // 孔阵列：后两行没有 G 字，只有坐标
    let p = interpret("G0 Z10\nG81 Z-8 R2 F100\nX20\nX40\n", &cfg());
    let xs_got = p.blocks.iter().filter(|b| b.target.tip.z == -8.0).count();
    assert_eq!(xs_got, 3, "应钻三个孔，实际 {} 个：{:?}", xs_got, xs(&p));
    // 每个孔两次经过起始高度：进孔时横移、出孔后退刀。
    // 少了任何一次都意味着"贴着工件横移"或"没退出来"
    let back = p.blocks.iter().filter(|b| b.target.tip.z == 10.0).count();
    assert_eq!(back, 6, "3 个孔 × 2 次");
    assert!(!p.blocks.is_empty());
    assert_eq!(
        p.blocks
            .iter()
            .filter(|b| matches!(b.motion, MotionKind::Linear))
            .count(),
        3,
        "每个孔一次进给"
    );
}

#[test]
fn g80_cancels_the_cycle() {
    let p = interpret("G0 Z10\nG81 Z-8 R2 F100\nX20\nG80\nX60\n", &cfg());
    // 只有两个孔；G80 之后的 X60 不该产生任何块
    let holes = p
        .blocks
        .iter()
        .filter(|b| b.target.tip.z == -8.0)
        .count();
    assert_eq!(holes, 2, "G80 之后不该再钻孔");
    assert!(
        p.blocks.iter().all(|b| b.target.tip.x != 60.0),
        "G80 之后只给 X 不该有运动：{:?}",
        xs(&p)
    );
}

#[test]
fn any_group1_code_cancels_the_cycle() {
    // G1 也是组 1，会取消循环（Fanuc 语义）。之后的 X60 是一条普通 G1
    let p = interpret("G0 Z10\nG81 Z-8 R2 F100\nG1 X30 F200\nX60\n", &cfg());
    let last = p.blocks.last().expect("应有块");
    assert_eq!(last.target.tip.x, 60.0);
    assert_eq!(last.target.tip.z, 10.0, "G1 之后 Z 保持退刀高度");
    assert!(matches!(last.motion, MotionKind::Linear));
    // 不该在 X60 处钻孔
    assert!(!p
        .blocks
        .iter()
        .any(|b| b.target.tip.x == 60.0 && b.target.tip.z < 0.0));
}

#[test]
fn g83_pecks_and_fully_retracts() {
    // R=2、Z=−8、Q=3：每次 3 mm → 孔底依次 −1、−4、−7、−8
    // 每刀之间完全退回到 R 平面（2）排屑
    let p = interpret("G0 Z10\nG83 Z-8 R2 Q3 F100\n", &cfg());
    let bottoms: Vec<f64> = p
        .blocks
        .iter()
        .filter(|b| matches!(b.motion, MotionKind::Linear))
        .map(|b| b.target.tip.z)
        .collect();
    assert_eq!(bottoms, vec![-1.0, -4.0, -7.0, -8.0], "每次 3 mm 分四次到底");
    // 中间每刀之后都要回到 R 平面
    let to_r = p.blocks.iter().filter(|b| b.target.tip.z == 2.0).count();
    assert!(to_r >= 3, "每刀之间应退回 R 平面，实际 {to_r} 次");
}

#[test]
fn g73_only_chips_out() {
    // G73 只在孔内抬一点，不退出到 R 平面
    let p = interpret("G0 Z10\nG73 Z-8 R2 Q3 F100\n", &cfg());
    let bottoms: Vec<f64> = p
        .blocks
        .iter()
        .filter(|b| matches!(b.motion, MotionKind::Linear))
        .map(|b| b.target.tip.z)
        .collect();
    assert_eq!(bottoms, vec![-1.0, -4.0, -7.0, -8.0]);
    // 断屑退刀的最高点必须低于 R 平面（否则就成 G83 了）
    let between: f64 = p
        .blocks
        .iter()
        .filter(|b| b.target.tip.z > -8.0 && b.target.tip.z < 2.0)
        .map(|b| b.target.tip.z)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(between < 2.0, "G73 不该退到 R 平面，实际最高 {between}");
}

#[test]
fn g98_retracts_to_initial_and_g99_to_r_plane() {
    let g98 = interpret("G0 Z20\nG81 Z-8 R2 F100\n", &cfg());
    assert_eq!(*zs(&g98).last().unwrap(), 20.0, "G98 默认退回起始 Z");

    let g99 = interpret("G0 Z20\nG99\nG81 Z-8 R2 F100\n", &cfg());
    assert_eq!(*zs(&g99).last().unwrap(), 2.0, "G99 退回 R 平面");
}

#[test]
fn g99_saves_travel_between_holes() {
    // 一排孔：G99 每次只退到 R 平面，往返距离比 G98 短得多
    let src = "G0 Z20\n{retract}G81 Z-8 R2 F100\nX20\nX40\nX60\n";
    let travel = |prog: &MotionProgram| -> f64 {
        let mut prev = 20.0;
        let mut sum = 0.0;
        for b in &prog.blocks {
            sum += (b.target.tip.z - prev).abs();
            prev = b.target.tip.z;
        }
        sum
    };
    let a = interpret(&src.replace("{retract}", "G98\n"), &cfg());
    let b = interpret(&src.replace("{retract}", "G99\n"), &cfg());
    assert!(
        travel(&b) < travel(&a) * 0.6,
        "G99 应该省很多空行程：G98 {:.1} / G99 {:.1}",
        travel(&a),
        travel(&b)
    );
}

#[test]
fn boring_cycles_feed_out() {
    // G85 / G89 是进给退（主轴还在切削），G86 是快速退
    for (g, last_is_rapid) in [("G85", false), ("G89", false), ("G86", true)] {
        let p = interpret(&format!("G0 Z10\n{g} Z-8 R2 F100\n"), &cfg());
        let last = p.blocks.last().expect("应有块");
        assert_eq!(
            matches!(last.motion, MotionKind::Rapid),
            last_is_rapid,
            "{g} 的退刀方式不对"
        );
        assert_eq!(last.target.tip.z, 10.0);
    }
}

#[test]
fn dwell_comes_from_p_with_fanuc_units() {
    // P 的单位看小数点：P0.3 = 0.3 秒，P300 = 300 毫秒
    let a = interpret("G0 Z10\nG82 Z-8 R2 P0.3 F100\n", &cfg());
    let bottom = a.blocks.iter().find(|b| b.target.tip.z == -8.0).unwrap();
    assert!((bottom.dwell - 0.3).abs() < 1e-12, "实际 {}", bottom.dwell);

    let b = interpret("G0 Z10\nG82 Z-8 R2 P300 F100\n", &cfg());
    let bottom = b.blocks.iter().find(|b| b.target.tip.z == -8.0).unwrap();
    assert!((bottom.dwell - 0.3).abs() < 1e-12, "P300 是 300 ms，实际 {}", bottom.dwell);

    // G81 不吃 P（不暂停）
    let c = interpret("G0 Z10\nG81 Z-8 R2 P0.3 F100\n", &cfg());
    let bottom = c.blocks.iter().find(|b| b.target.tip.z == -8.0).unwrap();
    assert_eq!(bottom.dwell, 0.0);
}

#[test]
fn incremental_cycle_measures_z_from_the_r_plane() {
    // G91：R 相对起始平面（10 + (−8) = 2），Z 相对 R 平面（2 + (−20) = −18）
    let p = interpret("G0 Z10\nG91\nG81 Z-20 R-8 F100\n", &cfg());
    assert_eq!(zs(&p), vec![10.0, 2.0, -18.0, 10.0]);
}

#[test]
fn cycles_outside_g17_are_reported_not_guessed() {
    let p = interpret("G18\nG81 Z-8 R2 F100\n", &cfg());
    assert!(
        p.issues.iter().any(|i| i.id == "GCODE-005"),
        "G18 下的钻孔方向是 Y，应该报不支持而不是假装沿 Z 钻"
    );
    assert!(p.blocks.is_empty(), "不该展开成沿 Z 的孔");
}

#[test]
fn unsupported_cycle_downgrades_trust() {
    // 报 Error 还不够，整个程序的可信度必须降级——否则上层会拿错结果去切
    let p = interpret("G18\nG81 Z-8 R2 F100\n", &cfg());
    assert!(
        matches!(p.trust, cncsim_gcode::Trust::Unreliable { .. }),
        "实际 {:?}",
        p.trust
    );
}

#[test]
fn missing_r_is_reported_but_still_drills() {
    let p = interpret("G0 Z10\nG81 Z-8 F100\n", &cfg());
    assert!(
        p.issues.iter().any(|i| i.id == "GCODE-009"),
        "缺 R 应该说一声"
    );
    // R 退化成当前高度 10：没有快移下刀那一段，直接进给到 −8
    assert_eq!(zs(&p), vec![10.0, -8.0, 10.0]);
}

#[test]
fn g87_back_boring_is_unsupported() {
    let p = interpret("G87 Z-8 R2 F100\n", &cfg());
    assert!(p.issues.iter().any(|i| i.id == "GCODE-005"));
    assert!(p.blocks.is_empty());
}

#[test]
fn peck_without_q_is_reported() {
    let p = interpret("G0 Z10\nG83 Z-8 R2 F100\n", &cfg());
    assert!(
        p.issues.iter().any(|i| i.id == "GCODE-009"),
        "啄钻缺 Q 必须报出来（会退化成一次扎到底）"
    );
    // 仍然要钻：报错不等于不切
    assert!(p.blocks.iter().any(|b| b.target.tip.z == -8.0));
}

#[test]
fn cycle_without_z_is_an_error() {
    let p = interpret("G0 Z10\nG81 R2 F100\n", &cfg());
    assert!(p.issues.iter().any(|i| i.id == "GCODE-009"));
    assert_eq!(p.blocks.len(), 1, "只有前面那条 G0，循环不该展开");
}

#[test]
fn cycle_does_not_duplicate_implicit_motion() {
    // 带 G 字的那一行不该再补一个隐式运动
    let p = interpret("G0 Z10\nG81 X5 Z-8 R2 F100\n", &cfg());
    // G0 → 快移到孔位 → 下到 R → 进给 → 退刀 = 5 块
    assert_eq!(p.blocks.len(), 5, "实际 {:?}", zs(&p));
    assert_eq!(p.blocks[1].target.tip.x, 5.0);
}

#[test]
fn hole_positions_follow_the_program() {
    let p = interpret("G0 Z10\nG81 Z-8 R2 F100\nX20 Y5\nX40 Y5\n", &cfg());
    let holes: Vec<(f64, f64)> = p
        .blocks
        .iter()
        .filter(|b| b.target.tip.z == -8.0)
        .map(|b| (b.target.tip.x, b.target.tip.y))
        .collect();
    assert_eq!(holes, vec![(0.0, 0.0), (20.0, 5.0), (40.0, 5.0)]);
    // 孔间横移必须发生在非负高度上（不能贴着工件或孔底横移）
    let mut prev = p.blocks[0].target.tip;
    for b in &p.blocks[1..] {
        let xy_moved = (b.target.tip.x - prev.x).abs() > 1e-12
            || (b.target.tip.y - prev.y).abs() > 1e-12;
        if xy_moved {
            assert!(
                b.target.tip.z >= 0.0,
                "横移发生在 z={}，太低了：{b:?}",
                b.target.tip.z
            );
        }
        prev = b.target.tip;
    }
}
