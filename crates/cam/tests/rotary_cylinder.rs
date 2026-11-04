//! 回转工序的**闭环**验收：圆柱 → 刀路 → 解释器 → 仿真 → 反量半径。
//!
//! 圆柱的回转面是解析的（`r` 与 θ、z 无关），所以"切完以后量出来的半径"可以
//! 直接和 10 mm 比。这一层专门抓 RTCP 的位姿/扫掠错误：位姿错了圆柱会整体
//! 胖一圈或瘦一圈，而那在花瓶上是看不出来的。

use cncsim_cam::rotary::{A_LIMIT_DEG, RotaryJob, generate_rotary};
use cncsim_cam::{Stock, ToolSpec, Tri};
use cncsim_math::{DVec2, DVec3};

/// 封闭圆柱（侧面 128 段 + 上下底），中心在原点。
fn cylinder(r: f64, z0: f64, z1: f64) -> Vec<Tri> {
    let n = 128;
    let mut t = Vec::new();
    let ring = |z: f64| -> Vec<DVec3> {
        (0..n)
            .map(|i| {
                let a = i as f64 / n as f64 * std::f64::consts::TAU;
                DVec3::new(r * a.cos(), r * a.sin(), z)
            })
            .collect()
    };
    let lo = ring(z0);
    let hi = ring(z1);
    let (c0, c1) = (DVec3::new(0.0, 0.0, z0), DVec3::new(0.0, 0.0, z1));
    for i in 0..n {
        let j = (i + 1) % n;
        t.push([lo[i], lo[j], hi[j]]);
        t.push([lo[i], hi[j], hi[i]]);
        t.push([c0, lo[j], lo[i]]);
        t.push([c1, hi[i], hi[j]]);
    }
    t
}

#[test]
fn machined_cylinder_keeps_its_radius() {
    let (r0, z0, z1) = (10.0, 0.0, 24.0);
    let mesh = cylinder(r0, z0, z1);
    // 毛坯要**居中**：`Stock::from_size` 的角点在原点，圆柱轴线在原点会被切掉一半
    let stock = Stock {
        min: DVec3::new(-16.0, -16.0, -2.0),
        max: DVec3::new(16.0, 16.0, 26.0),
    };
    let tool = ToolSpec::ball(1, 4.0, 30.0);
    let job = RotaryJob {
        mesh: &mesh,
        stock,
        center: DVec2::new(0.0, 0.0),
        tool: tool.clone(),
        step_deg: 6.0,
        step_z_mm: 1.5,
        feed_mm_min: 900.0,
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

    let h = std::env::var("CNCSIM_H").ok().and_then(|v| v.parse().ok()).unwrap_or(0.5);
    let cfg = cncsim_sim::run::RunConfig {
        grid: cncsim_voxel::GridConfig {
            h,
            band_voxels: 4,
            value_type: cncsim_voxel::ValueType::I8,
        },
        stock_origin: stock.min,
        stock_size: [
            (stock.size().x / h) as i32,
            (stock.size().y / h) as i32,
            (stock.size().z / h) as i32,
        ],
        design: None,
        collide: false,
        creator_sub: 1,
        ..Default::default()
    };
    let tools = cncsim_tool::ToolTable::new(vec![tool.to_tool_def()]);
    let out = cncsim_sim::run::run_tools(&program, &tools, &cfg);
    eprintln!(
        "块 {} 切削段 {} 去料 {:.0} mm³  毛坯 {} mm³",
        out.stats.blocks,
        out.stats.cut_segments,
        out.report.removed_volume_mm3,
        out.stats.volume_before
    );
    for (r, z) in [(8.0, 1.0), (12.0, 1.0), (8.0, 12.0), (40.0, 12.0)] {
        eprintln!("  sdf(r={r}, z={z}) = {:+.3}", out.grid.sdf(DVec3::new(r, 0.0, z)));
    }

    // 沿半径从外往里找 sdf 的零点，得到真实半径
    let mut worst = 0.0f64;
    let mut n = 0usize;
    for iz in 1..23 {
        let z = z0 + iz as f64;
        for k in 0..12 {
            let th = k as f64 / 12.0 * std::f64::consts::TAU + 0.13;
            let mut hit = None;
            let mut prev: Option<f64> = None;
            let mut rr = r0 + 3.0;
            while rr > r0 - 3.0 {
                let p = DVec3::new(rr * th.cos(), rr * th.sin(), z);
                if out.grid.sdf(p) < 0.0 {
                    if let Some(rp) = prev {
                        // 二分
                        let (mut lo, mut hi) = (rr, rp);
                        for _ in 0..20 {
                            let m = 0.5 * (lo + hi);
                            let q = DVec3::new(m * th.cos(), m * th.sin(), z);
                            if out.grid.sdf(q) < 0.0 {
                                lo = m;
                            } else {
                                hi = m;
                            }
                        }
                        hit = Some(0.5 * (lo + hi));
                    }
                    break;
                }
                prev = Some(rr);
                rr -= 0.05;
            }
            let got = hit.unwrap_or_else(|| panic!("z={z:.1} θ={th:.2} 整条射线没有材料"));
            worst = worst.max((got - r0).abs());
            n += 1;
        }
    }
    assert!(n >= 200, "采样太少：{n}");
    assert!(
        worst < 0.12,
        "圆柱被切走了 {worst:.3} mm（应 < 0.12：球头 R2、步距 1.5 的残留）"
    );

    // ── 换"按设计面打点"的口径再量一遍 ──
    //
    // 上面那条是沿半径二分的**单点剖面**：它只沿一条射线找零点，射线之间
    // （6° 一圈 ≈ 1 mm 弧长）的小坑看不见。这里改成在名义圆柱面上铺点，
    // 逐点查有符号距离——和三坐标检测同一个做法，也给 `surface_dev` 兜底。
    let mut pts = Vec::new();
    for iz in 0..24 {
        let z = z0 + 0.5 + iz as f64;
        for it in 0..48 {
            let th = it as f64 / 48.0 * std::f64::consts::TAU + 0.017;
            pts.push(DVec3::new(r0 * th.cos(), r0 * th.sin(), z));
        }
    }
    let dev = cncsim_sim::surface_dev::signed_deviation(&out.grid, &pts);
    let st = cncsim_sim::surface_dev::stats(&dev, 0.15, 4.0 * h);
    eprintln!(
        "名义曲面：{} 点 / 平均|d| {:.3} / P95 {:.3} / 合格 {:.1}% / 过切 {:.1}% / 残料 {:.1}% / 最大过切 {:+.3}",
        st.points,
        st.mean_abs,
        st.p95_abs,
        st.ok_frac() * 100.0,
        st.gouge_frac() * 100.0,
        st.residual_frac() * 100.0,
        st.max_gouge
    );
    assert_eq!(st.points, pts.len());
    assert!(
        st.p95_abs < 0.25,
        "名义圆柱面的 P95 偏差 {:.3} mm，超过 0.25（R2 球头 + 1.5 mm 步距）",
        st.p95_abs
    );
    assert!(
        st.gouge_frac() < 0.05,
        "过切点占 {:.1}%：刀尖应当贴着圆柱面走，不该切进去",
        st.gouge_frac() * 100.0
    );
    assert_eq!(st.saturated, 0, "没有点该跑到窄带外面");
}

/// **翘刀轴不改变切出来的曲面**：球心恒在 `p + R·n` 上，翘的只是刀颈。
///
/// 用已知答案的圆柱做行为验收：tilt=0 与 tilt=15° 切出来的偏差必须一致。
/// 这条性质是"用 tilt 躲开锥颈"和"用 tilt 减少 A 超程"的前提。
#[test]
fn tilting_the_axis_does_not_change_the_cut() {
    let (r0, z0, z1) = (10.0, 0.0, 20.0);
    let mesh = cylinder(r0, z0, z1);
    let stock = Stock {
        min: DVec3::new(-15.0, -15.0, -2.0),
        max: DVec3::new(15.0, 15.0, 22.0),
    };
    let tool = ToolSpec::tapered(1, 2.0, 10.0, 5.0);
    let h = 0.5;
    let mut stats = Vec::new();
    for tilt in [0.0f64, 15.0] {
        let job = RotaryJob {
            mesh: &mesh,
            stock,
            center: DVec2::new(0.0, 0.0),
            tool: tool.clone(),
            step_deg: 6.0,
            step_z_mm: 1.5,
            feed_mm_min: 900.0,
            plunge_mm_min: 200.0,
            spindle_rpm: 9000,
            a_limit_deg: A_LIMIT_DEG,
            tilt_deg: tilt,
            roughing: None,
        };
        let prog = generate_rotary(&job).unwrap();
        let ic = cncsim_gcode::InterpConfig {
            tool_radii: vec![0.0, tool.radius()],
            rotary: cncsim_gcode::RotaryConvention::Head,
            ..Default::default()
        };
        let program = cncsim_gcode::interpret(&prog.gcode, &ic);
        let cfg = cncsim_sim::run::RunConfig {
            grid: cncsim_voxel::GridConfig {
                h,
                band_voxels: 4,
                value_type: cncsim_voxel::ValueType::I8,
            },
            stock_origin: stock.min,
            stock_size: [60, 60, 48],
            design: None,
            collide: false,
            creator_sub: 1,
            ..Default::default()
        };
        let tools = cncsim_tool::ToolTable::new(vec![tool.to_tool_def()]);
        let out = cncsim_sim::run::run_tools(&program, &tools, &cfg);
        let mut pts = Vec::new();
        for iz in 0..16 {
            let z = 1.5 + iz as f64;
            for it in 0..36 {
                let th = it as f64 / 36.0 * std::f64::consts::TAU + 0.03;
                pts.push(DVec3::new(r0 * th.cos(), r0 * th.sin(), z));
            }
        }
        let dev = cncsim_sim::surface_dev::signed_deviation(&out.grid, &pts);
        let st = cncsim_sim::surface_dev::stats(&dev, 0.15, 4.0 * h);
        eprintln!("tilt={tilt}: 平均|d| {:.4} P95 {:.4} 合格 {:.1}%", st.mean_abs, st.p95_abs, st.ok_frac() * 100.0);
        stats.push(st);
    }
    assert!(
        (stats[0].mean_abs - stats[1].mean_abs).abs() < 0.02,
        "翘 15° 后平均偏差变了：{:.4} → {:.4}",
        stats[0].mean_abs,
        stats[1].mean_abs
    );
    assert!(
        (stats[0].p95_abs - stats[1].p95_abs).abs() < 0.02,
        "翘 15° 后 P95 变了：{:.4} → {:.4}",
        stats[0].p95_abs,
        stats[1].p95_abs
    );
}
