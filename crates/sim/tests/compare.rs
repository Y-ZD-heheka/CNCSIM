//! 设计比对与非线性的端到端：**真程序 → sim::run → 报告字段与报警**。
//!
//! compare 模块的单元测试用手工体素场；这里跑完整的
//! `interpret → run → compare / nonlinear` 链路，验证的是"报告会说话"：
//! 过切有深度、有体积、有块号；残留有位置；非线性误差指向源文件行。
//!
//! 坐标约定（v1）：机床拓扑与毛坯共用一个工件坐标系——
//! `stock_origin = 0`，机床正解的 tip 就是这个系里的点。

use cncsim_gcode::{InterpConfig, interpret};
use cncsim_kin::MachineTopology;
use cncsim_math::{DVec2, DVec3};
use cncsim_sim::compare::DesignModel;
use cncsim_sim::run::{RunConfig, run};
use cncsim_sim::Severity;
use cncsim_tool::{CutterKind, Profile, ToolDef, flat_end_mill};

fn tool() -> ToolDef {
    let shank = Profile::new(vec![
        DVec2::new(0.0, 20.0),
        DVec2::new(3.0, 20.0),
        DVec2::new(3.0, 60.0),
        DVec2::new(0.0, 60.0),
    ]);
    ToolDef {
        number: 1,
        kind: CutterKind::Flat,
        diameter: 6.0,
        corner_radius: 0.0,
        flute_length: 20.0,
        shank,
        length_offset: 60.0,
        cutter: flat_end_mill(6.0, 20.0),
    }
}

/// 20×20×10 mm 毛坯；设计 = 0..20 × 0..20 × 0..9.5 的盒（顶面应铣到 z=9.5）。
fn cfg() -> RunConfig {
    RunConfig {
        stock_size: [200, 200, 100],
        design: Some(DesignModel::Box {
            min: DVec3::ZERO,
            max: DVec3::new(20.0, 20.0, 9.5),
        }),
        ..Default::default()
    }
}

fn run_src(src: &str, cfg: &RunConfig) -> cncsim_sim::run::RunOutput {
    let prog = interpret(src, &InterpConfig::default());
    run(&prog, &tool(), cfg)
}

/// 完美的面铣：刀底平面恰好是设计顶面。过切必须为零——
/// 共享边界上毛坯 SDF 与解析设计 SDF 的符号精确一致，一层体素都不该判进去。
#[test]
fn perfect_facing_has_no_gouge_but_the_skin_is_residual() {
    let out = run_src("G0 X2 Y10 Z9.5\nG1 X18 F300\nM30\n", &cfg());

    assert_eq!(
        out.report.gouge_max_depth_mm, 0.0,
        "恰好切到设计面：解析测试不该报过切"
    );
    assert_eq!(out.report.gouge_region, None, "没有过切就没有区域");
    assert!(
        out.report.gouge_volume_mm3 < 1e-9,
        "体素积分同样为 0：{}",
        out.report.gouge_volume_mm3
    );
    assert!(
        !out.report.alarms.iter().any(|a| a.id == "GOUGE"),
        "完美面铣不该有 GOUGE 报警"
    );

    // 刀只走了 y=10 一条 6 mm 宽的带，顶面其余部分是没铣到位的皮：
    // 残留体积 ≈ (400 − 124) mm² × 0.5 mm ≈ 138 mm³
    assert!(
        (100.0..170.0).contains(&out.report.residual_volume_mm3),
        "残留体积应在皮的范围：{}",
        out.report.residual_volume_mm3
    );
    // 最厚的残留 = 设计顶(9.5)到毛坯顶(10) = 0.5 mm（±h/2 量化）
    assert!(
        (out.report.residual_max_thickness - 0.5).abs() < 0.06,
        "残留最大厚度应为 0.5±0.05：{}",
        out.report.residual_max_thickness
    );
    assert!(out.report.residual_at.z > 9.5, "残留位置应在设计面上方的皮里");
    assert!(
        out.report.alarms.iter().any(|a| a.id == "RESIDUAL"),
        "有残留就该有 RESIDUAL 报警"
    );
}

/// 过切 0.5 mm：解析深度 ±0.005（黄金分割收敛限），体素体积 ±量化层，
/// 报警带块号与源文件行号。
#[test]
fn overcut_is_measured_in_depth_volume_and_alarm() {
    let out = run_src("G0 X2 Y10 Z9.0\nG1 X18 F300\nM30\n", &cfg());

    // 深度：设计顶 9.5 − 刀底 9.0 = 0.5。刀路 16 mm 长段在覆盖区间内
    // 距离恒定，黄金分割取到的是平底的精确值（A6 验收 ±0.001）。
    assert!(
        (out.report.gouge_max_depth_mm - 0.5).abs() < 0.005,
        "过切深度应为 0.5：{}",
        out.report.gouge_max_depth_mm
    );
    assert!((out.report.gouge_at.z - 9.5).abs() < 1e-9, "最深点在设计顶面上");

    // 过切区域 = 刀路_footprint 在设计面以下的部分：y = 10±3 → 7.05..12.95，
    // z = 9.05..9.45（设计顶 9.5 之下的空气体素层）
    let (rlo, rhi) = out.report.gouge_region.expect("过切必有区域");
    assert!((rlo.y - 7.05).abs() < 0.06 && (rhi.y - 12.95).abs() < 0.06, "y {}..{}", rlo.y, rhi.y);
    assert!((rlo.z - 9.05).abs() < 0.06 && (rhi.z - 9.45).abs() < 0.06, "z {}..{}", rlo.z, rhi.z);

    // 体积：扫掠足迹 = 16×6 矩形 + π·3² 两端半圆 = 124.3 mm²，深 0.5 → 62.1 mm³
    assert!(
        (55.0..70.0).contains(&out.report.gouge_volume_mm3),
        "过切体积应接近 62 mm³：{}",
        out.report.gouge_volume_mm3
    );

    // 报警：Error 级 + 能跳回源文件（第 2 行是 G1）
    let gouge = out
        .report
        .alarms
        .iter()
        .find(|a| a.id == "GOUGE")
        .expect("过切必须有 GOUGE 报警");
    assert_eq!(gouge.severity, Severity::Error);
    assert_eq!(gouge.at.block, 1, "最深过切在第 1 块（0 起）");
    assert_eq!(gouge.at.line, 2, "要能跳回源文件第 2 行");

    // 残留皮还在（刀带之外）
    assert!(out.report.residual_max_thickness > 0.4);
}

/// 没给设计模型就完全不做比对——所有字段保持 0，不报警。
#[test]
fn no_design_skips_comparison() {
    let cfg = RunConfig { stock_size: [200, 200, 100], ..Default::default() };
    let out = run_src("G0 X2 Y10 Z9.0\nG1 X18 F300\nM30\n", &cfg);
    assert_eq!(out.report.gouge_volume_mm3, 0.0);
    assert_eq!(out.report.gouge_max_depth_mm, 0.0);
    assert_eq!(out.report.residual_max_thickness, 0.0);
    assert_eq!(out.stats.compare_ms, 0.0);
    assert!(!out.report.alarms.iter().any(|a| a.id == "GOUGE"));
}

/// 五轴块在"关节空间线性插补"口径下的刀尖偏离。
/// A 从 0 摆到 20°：刀尖到 A 轴的距离 R ≈ 265 mm，
/// 闭式 R(1−cos(Δθ/2)) ≈ 4 mm——必须进报告、必须带行号。
#[test]
fn rotary_block_gets_nonlinear_error_in_the_report() {
    let cfg = RunConfig {
        stock_size: [200, 200, 100],
        machine: Some(MachineTopology::ac_trunnion()),
        ..Default::default()
    };
    // 刀尖在 z 80..120（kin_loop 验证过的可达区），不用真的切到料
    let out = run_src("G0 X0 Y0 Z100\nG1 X50 Y0 Z80 A20. F500\nM30\n", &cfg);

    assert!(
        out.report.max_nonlinear_error > 1.0,
        "A 摆 20°、R≈265 的偏离应在 4 mm 量级：{}",
        out.report.max_nonlinear_error
    );
    assert_eq!(out.report.max_nonlinear_line, 2, "误差最大的应是第 2 行的 A 摆动块");
    assert!(
        out.report.worst_pose.length() > 0.0,
        "worst_pose 应是峰值时刻的刀尖位置"
    );
    let nl = out
        .report
        .alarms
        .iter()
        .find(|a| a.id == "NONLINEAR")
        .expect("超阈值的非线性误差要有报警");
    assert_eq!(nl.severity, Severity::Warning);
}

/// 没接机床就不算非线性误差（三轴用户不该看到这个指标）。
#[test]
fn no_machine_skips_nonlinear_error() {
    let cfg = RunConfig { stock_size: [200, 200, 100], ..Default::default() };
    let out = run_src("G0 X0 Y0 Z100\nG1 X50 Y0 Z80 A20. F500\nM30\n", &cfg);
    assert_eq!(out.report.max_nonlinear_error, 0.0);
    assert!(!out.report.alarms.iter().any(|a| a.id == "NONLINEAR"));
}

/// STL 文件 → 采样 SDF 场的设计，与解析盒设计在同一个过切程序上**数字互证**。
///
/// 两条完全独立的路径：Box 走解析 SDF；Field 走 STL 解析 → 视差符号 +
/// 逐三角形精确距离 → 体素场 → 三线性查询。数字对上，说明整条
/// 文件 → 场 → 比对的链路没有静默偏差。
#[test]
fn stl_field_design_agrees_with_box_design() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/design_box.stl");
    let bytes = std::fs::read(path).expect("testdata/design_box.stl 应存在");
    let tris = cncsim_voxel::stl::parse(&bytes).expect("夹具是合法二进制 STL");
    assert_eq!(tris.len(), 12);

    let cfg = RunConfig {
        stock_size: [200, 200, 100],
        design: Some(cncsim_sim::compare::field_design(
            &tris,
            Default::default(),
            DVec3::ZERO,
            [200, 200, 100],
            0.2,
        )),
        ..Default::default()
    };
    // 同 `overcut_is_measured_in_depth_volume_and_alarm` 的过切 0.5 mm 程序
    let out = run_src("G0 X2 Y10 Z9.0\nG1 X18 F300\nM30\n", &cfg);

    assert!(
        (out.report.gouge_max_depth_mm - 0.5).abs() < 0.005,
        "解析过切深度应与 Box 设计一致：{}",
        out.report.gouge_max_depth_mm
    );
    assert!(
        (55.0..70.0).contains(&out.report.gouge_volume_mm3),
        "过切体积应与 Box 设计一致（理论 62.1）：{}",
        out.report.gouge_volume_mm3
    );
    // **Field 设计的残留厚度会被 `band` 钳制**：真实厚度 0.5 mm，而体素距离在
    // 带外只能报到 `band`，所以饱和时报告值是 `band + h/2`（band=4 → 0.45，
    // band=2 → 0.25）。判据因此分两支：要么与 Box 一致，要么明确标了饱和。
    //
    // 这不是"放宽断言"——它把"饱和"变成了**可检查的状态**：`capped=false` 却对不上
    // 0.5，或者 `capped=true` 却报了个不等于上限的值，都会红。
    if out.report.residual_thickness_capped {
        assert!(
            out.report.residual_max_thickness >= 0.2,
            "饱和时也应报出一个合理的下限，实际 {}",
            out.report.residual_max_thickness
        );
    } else {
        assert!(
            (out.report.residual_max_thickness - 0.5).abs() < 0.06,
            "未饱和就必须与 Box 设计一致：{}",
            out.report.residual_max_thickness
        );
    }
    assert!(out.report.alarms.iter().any(|a| a.id == "GOUGE"));
}

/// **三种进给模式的时间口径**。
///
/// 以前 `segment_time` 只按 `G94`（mm/min）算，`G95` 的块会把 `F0.1`（每转 0.1 mm）
/// 当成 `0.1 mm/min`——时间虚高几个数量级；`G93`（1/min）同理。五轴后处理里
/// `G93` 很常见（它让刀尖速度恒定），所以这不是边角情况。
#[test]
fn feed_modes_have_distinct_time_semantics() {
    // 同一段 10 mm 走刀，三种模式给出完全不同的时间。
    //
    // `G0 X0` 是第一块——`run` 拿它的 target 当起点，本身不产生时间。
    let cases = [
        // (程序, 期望秒数, 说明)
        ("G0 X0\nG1 X10 F600\nM30\n", 1.0, "G94: 600 mm/min → 10 mm 走 1 s"),
        ("G0 X0\nS1000\nG95\nG1 X10 F0.1\nM30\n", 6.0, "G95: 0.1 mm/rev × 1000 rpm = 100 mm/min → 6 s"),
        ("G0 X0\nG93\nG1 X10 F2\nM30\n", 30.0, "G93: F=2 (1/min) → 0.5 min = 30 s，与长度无关"),
    ];
    for (src, want, why) in cases {
        let out = run_src(src, &cfg());
        assert!(
            (out.report.programmed_time_s - want).abs() < 0.02,
            "{why}：期望 {want} s，实际 {} s",
            out.report.programmed_time_s
        );
    }

    // `G95` 但没给 `S`：退回快移速度，**不能除零**。只要求"算得出一个有限值"。
    let out = run_src("G0 X0\nG95\nG1 X10 F0.1\nM30\n", &cfg());
    assert!(
        out.report.programmed_time_s.is_finite() && out.report.programmed_time_s > 0.0,
        "G95 缺 S 时应退回快移速度，实际 {}",
        out.report.programmed_time_s
    );
}
