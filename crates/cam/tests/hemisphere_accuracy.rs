//! **解析基准**：半球（球面精确可算）跑完整回转链路，按所需 A 角分组量偏差。
//!
//! 目的有两个：
//!
//! 1. **定性**——花瓶上平均 0.31 mm 的偏差，到底是刀路产生的，还是体素/扫掠
//!    产生的？半球面是解析的，任何偏差都能直接归因：如果半球本身就偏 0.2~0.3，
//!    那再优化刀路也是白费。
//! 2. **定位**——两组只差 `tilt_deg`（0° vs 35°）。锥颈在中等摆角扫到相邻壁
//!    （文献里的 rear gouging）会表现为"倾斜后 A 60..90° 段明显变好"；如果两组
//!    一样，说明偏差与刀颈无关，得回去查落刀点。
//!
//! 半球的一切都是解析的：点位 `p = R·(sinφcosθ, sinφsinθ, cosφ)`，法向就是
//! 径向，所以"所需 A 角"恰好等于极角 φ。

use cncsim_cam::rotary::{A_LIMIT_DEG, RotaryJob, Roughing, generate_rotary};
use cncsim_cam::{Stock, ToolSpec, Tri};
use cncsim_math::{DVec2, DVec3};

/// 半球：R=15，球心在原点，圆顶朝 +Z，底面在 z=0 用扇形封口。
fn hemisphere(r: f64, n_theta: usize, n_phi: usize) -> Vec<Tri> {
    let mut t = Vec::new();
    let pt = |i: usize, j: usize| -> DVec3 {
        let th = i as f64 / n_theta as f64 * std::f64::consts::TAU;
        let ph = j as f64 / n_phi as f64 * std::f64::consts::FRAC_PI_2;
        DVec3::new(
            r * ph.sin() * th.cos(),
            r * ph.sin() * th.sin(),
            r * ph.cos(),
        )
    };
    for j in 0..n_phi {
        for i in 0..n_theta {
            let (i2, j2) = ((i + 1) % n_theta, j + 1);
            let (a, b, c, d) = (pt(i, j), pt(i2, j), pt(i2, j2), pt(i, j2));
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    // 底面：从 (0,0,0) 扇形封口
    let c0 = DVec3::new(0.0, 0.0, 0.0);
    for i in 0..n_theta {
        let i2 = (i + 1) % n_theta;
        t.push([c0, pt(i2, 0), pt(i, 0)]);
    }
    t
}

/// 半球面上的采样点（θ 均匀、φ 均匀，面积加权靠 φ 的 sin 权重在统计里补）。
fn surface_points(r: f64, n_theta: usize, n_phi: usize) -> Vec<(DVec3, f64)> {
    let mut v = Vec::new();
    for j in 1..n_phi {
        let ph = j as f64 / n_phi as f64 * std::f64::consts::FRAC_PI_2;
        for i in 0..n_theta {
            let th = (i as f64 + 0.37) / n_theta as f64 * std::f64::consts::TAU;
            let p = DVec3::new(
                r * ph.sin() * th.cos(),
                r * ph.sin() * th.sin(),
                r * ph.cos(),
            );
            v.push((p, ph.to_degrees())); // 极角 = 所需 A 角
        }
    }
    v
}

struct Run {
    tilt: f64,
    dev: Vec<f64>,
    a_deg: Vec<f64>,
    seconds: f64,
}

fn run(r: f64, tilt: f64) -> Run {
    let mesh = hemisphere(r, 96, 32);
    let stock = Stock {
        min: DVec3::new(-r - 1.0, -r - 1.0, -1.0),
        max: DVec3::new(r + 1.0, r + 1.0, r + 1.0),
    };
    let tool = ToolSpec::tapered(2, 2.0, 10.0, 5.0); // 与花瓶同款：尖端 R1、10° 锥颈
    let rough = ToolSpec::flat(1, 12.0, 20.0);
    let job = RotaryJob {
        mesh: &mesh,
        stock,
        center: DVec2::new(0.0, 0.0),
        tool: tool.clone(),
        step_deg: 1.25,
        step_z_mm: 0.6,
        feed_mm_min: 1200.0,
        plunge_mm_min: 300.0,
        spindle_rpm: 10_000,
        a_limit_deg: A_LIMIT_DEG,
        tilt_deg: tilt,
        roughing: Some(Roughing {
            tool: rough.clone(),
            step_mm: 7.0,
            pitch_mm: 7.0,
            step_deg: 12.0,
            allowance_mm: 0.5,
        }),
    };
    let prog = generate_rotary(&job).unwrap();
    let ic = cncsim_gcode::InterpConfig {
        tool_radii: vec![0.0, rough.radius(), tool.radius()],
        rotary: cncsim_gcode::RotaryConvention::Head,
        ..Default::default()
    };
    let program = cncsim_gcode::interpret(&prog.gcode, &ic);
    assert!(!program.has_errors(), "解释器报错：{:?}", program.issues);

    let h = 0.5;
    let cfg = cncsim_sim::run::RunConfig {
        grid: cncsim_voxel::GridConfig {
            h,
            band_voxels: 4,
            value_type: cncsim_voxel::ValueType::I8,
        },
        stock_origin: stock.min,
        stock_size: [
            (stock.size().x / h).round() as i32,
            (stock.size().y / h).round() as i32,
            (stock.size().z / h).round() as i32,
        ],
        design: None,
        collide: false,
        creator_sub: 1,
        drop_chips: true,
        ..Default::default()
    };
    let tools = cncsim_tool::ToolTable::new(vec![rough.to_tool_def(), tool.to_tool_def()]);

    let t0 = std::time::Instant::now();
    let out = cncsim_sim::run::run_tools(&program, &tools, &cfg);
    let seconds = t0.elapsed().as_secs_f64();

    let pts = surface_points(r, 72, 36);
    let positions: Vec<DVec3> = pts.iter().map(|(p, _)| *p).collect();
    let dev = cncsim_sim::surface_dev::signed_deviation(&out.grid, &positions);
    let a_deg = pts.iter().map(|(_, a)| *a).collect();
    Run {
        tilt,
        dev,
        a_deg,
        seconds,
    }
}

/// 按 A 角分组：`(下限, 点数, 平均 |d|, 合格率)`
fn bins(r: &Run) -> Vec<(u32, usize, f64, f64)> {
    let edges = [0.0f64, 30.0, 50.0, 65.0, 80.0, 91.0];
    let mut out = Vec::new();
    for w in edges.windows(2) {
        let mut n = 0usize;
        let mut sum = 0.0;
        let mut ok = 0usize;
        for (d, a) in r.dev.iter().zip(r.a_deg.iter()) {
            if *a >= w[0] && *a < w[1] {
                n += 1;
                sum += d.abs();
                if d.abs() <= 0.15 {
                    ok += 1;
                }
            }
        }
        if n > 0 {
            out.push((w[0] as u32, n, sum / n as f64, ok as f64 / n as f64 * 100.0));
        }
    }
    out
}

#[test]
fn hemisphere_accuracy_by_tilt() {
    let r = 15.0;
    let a = run(r, 0.0);
    let b = run(r, 35.0);
    for x in [&a, &b] {
        let st = cncsim_sim::surface_dev::stats(&x.dev, 0.15, 2.0);
        eprintln!(
            "\n== tilt {:.0}° == {:.1} s  点 {}  平均|d| {:.3}  P95 {:.3}  合格 {:.1}%  过切 {:.1}%  残料 {:.1}%",
            x.tilt,
            x.seconds,
            st.points,
            st.mean_abs,
            st.p95_abs,
            st.ok_frac() * 100.0,
            st.gouge_frac() * 100.0,
            st.residual_frac() * 100.0
        );
        for (lo, n, mean, ok) in bins(x) {
            eprintln!("   A {lo:2}..：{n:5} 点  平均|d| {mean:.3}  合格 {ok:.1}%");
        }
    }
    // 半球是解析面：只要链路是干净的，平均偏差就该明显小于体素 0.5 mm。
    // 这里不锁死 0.05（那要等 drop-cutter 落地），先把上界钉住，防止回归。
    let st_a = cncsim_sim::surface_dev::stats(&a.dev, 0.15, 2.0);
    let st_b = cncsim_sim::surface_dev::stats(&b.dev, 0.15, 2.0);
    assert!(
        st_a.mean_abs < 0.55 && st_b.mean_abs < 0.55,
        "半球平均偏差 {:.3}/{:.3} mm —— 超过体素尺度，先查仿真不是查刀路",
        st_a.mean_abs,
        st_b.mean_abs
    );
}
