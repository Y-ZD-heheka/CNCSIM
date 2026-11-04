//! RTCP 位姿自检：**不跑仿真**，只看"解释出来的刀轴是不是朝外、刀尖是不是落在轮廓上"。
//!
//! 这一层能抓住的错，仿真里要看几千块体素才看得出来：刀轴反了的话，球心的位置
//! 会从"轮廓外 R"变成"轮廓内 R"，切出来整体过切 2R——五轴 RTCP 最典型的翻车方式。
//!
//! 用圆锥当试件：`r(z) = r0 + k·z`，外法向解析可写，A/C 也有闭式解。

use cncsim_cam::rotary::{A_LIMIT_DEG, RotaryJob, generate_rotary};
use cncsim_cam::{Stock, ToolSpec, Tri};
use cncsim_math::{DVec2, DVec3};

/// 圆锥台：底面半径 r0，顶面 r0 + k*h，封闭（上下两个盖）。
fn cone(r0: f64, k: f64, z0: f64, z1: f64) -> Vec<Tri> {
    let n = 96;
    let mut t = Vec::new();
    let ring = |z: f64, r: f64| -> Vec<DVec3> {
        (0..n)
            .map(|i| {
                let a = i as f64 / n as f64 * std::f64::consts::TAU;
                DVec3::new(r * a.cos(), r * a.sin(), z)
            })
            .collect()
    };
    let lo = ring(z0, r0);
    let hi = ring(z1, r0 + k * (z1 - z0));
    for i in 0..n {
        let j = (i + 1) % n;
        t.push([lo[i], lo[j], hi[j]]);
        t.push([lo[i], hi[j], hi[i]]);
        let c0 = DVec3::new(0.0, 0.0, z0);
        let c1 = DVec3::new(0.0, 0.0, z1);
        t.push([c0, lo[j], lo[i]]);
        t.push([c1, hi[i], hi[j]]);
    }
    t
}

#[test]
fn rtcp_axis_points_outward_and_tip_sits_on_the_profile() {
    // k=0.25 → 外法向 A≈104°，落在 110° 行程内。取 0.4 时 A=111.8°，
    // 现在会被"超行程跳过"整段略过，测试就测不到东西了。
    let (r0, k) = (20.0, 0.25);
    let mesh = cone(r0, k, 0.0, 30.0);
    let stock = Stock::from_size(DVec3::new(60.0, 60.0, 36.0));
    let tool = ToolSpec::ball(1, 4.0, 30.0);
    let job = RotaryJob {
        mesh: &mesh,
        stock,
        center: DVec2::new(0.0, 0.0),
        tool: tool.clone(),
        step_deg: 6.0,
        step_z_mm: 3.0,
        feed_mm_min: 800.0,
        plunge_mm_min: 200.0,
        spindle_rpm: 9000,
        a_limit_deg: A_LIMIT_DEG,
        tilt_deg: 0.0,
        roughing: None,
    };
    let prog = generate_rotary(&job).unwrap();
    let ic = cncsim_gcode::InterpConfig {
        tool_radii: vec![0.0, tool.radius()],
        rotary: cncsim_gcode::RotaryConvention::Head,
        ..Default::default()
    };
    let program = cncsim_gcode::interpret(&prog.gcode, &ic);
    assert!(!program.has_errors(), "解释器报错：{:?}", program.issues);

    // 圆锥侧面的解析外法向（去掉 z 方向单位化）：n ∝ (e_r − k·e_z)
    let rad = tool.radius();
    let mut checked = 0usize;
    let mut near_axis = 0usize;
    let mut total = 0usize;
    let mut worst_outward = f64::INFINITY;
    for b in &program.blocks {
        // 只查侧面的切削块：下刀段/快移（离轮廓远）不算
        let tip = b.target.tip;
        let z = tip.z;
        if z <= 0.5 || z >= 29.5 {
            continue; // 上下盖那两圈另算
        }
        let r_tip = tip.x.hypot(tip.y);
        if r_tip < 5.0 {
            near_axis += 1;
            continue;
        }
        total += 1;
        let want = r0 + k * z;
        if (r_tip - want).abs() > 0.5 {
            continue; // 进刀/退刀段
        }
        // 球心（tip + R·axis）必须在**外侧**：径向离轴线更远
        let c = tip + b.target.axis * rad;
        let r_c = c.x.hypot(c.y);
        worst_outward = worst_outward.min(r_c - r_tip);
        checked += 1;
    }
    assert!(checked > 100, "侧向切削块太少：{checked}");
    // 不再断言"九成刀点在轮廓上"：A 超行程点改成**跳过 + 垂直下刀**以后，
    // 每个跳过后面都有几块"抬到安全高度 / 移到正上方"的过渡块，它们本来就不在
    // 轮廓上。真正要守的是下面两条：切削块的刀尖落在轮廓上、球心在轮廓外侧。
    let _ = total;
    let _ = near_axis;
    assert!(
        worst_outward > 0.5,
        "球心不在外侧！最差只外移 {worst_outward:.3} mm（应当 ≥ 刀半径一半）"
    );
}
