//! 端到端验收：**五轴曲面法向点铣** —— 高度场切不到的竖壁，必须真的切到。
//!
//! 这个测试断言的是**物理量**，不是文本：把生成的五轴程序交给现有解释器
//! 与仿真器，再拿设计面点到"刀具扫掠体并集"的有符号距离去量。
//!
//! 为什么需要这条链路：`generate`（3 轴高度场）每列只认最上面那一层，
//! 竖直侧壁只会被啃出一条 R 的圆角，剩下的料永远留着（`docs/项目记忆.md` §40）。
//! 五轴道不经过高度场：球心放在 `p + R·n`，刀轴沿 `n`，球与设计面在 `p` 相切。

use cncsim_cam::surface::{SurfaceOpts, generate_surface};
use cncsim_cam::{Stock, Tri};
use cncsim_gcode::{InterpConfig, interpret};
use cncsim_math::DVec3;
use cncsim_sim::run::{RunConfig, run_tools};
use cncsim_tool::ToolTable;

fn box_mesh(lo: DVec3, hi: DVec3) -> Vec<Tri> {
    let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
    let c = [
        p(lo.x, lo.y, lo.z),
        p(hi.x, lo.y, lo.z),
        p(hi.x, hi.y, lo.z),
        p(lo.x, hi.y, lo.z),
        p(lo.x, lo.y, hi.z),
        p(hi.x, lo.y, hi.z),
        p(hi.x, hi.y, hi.z),
        p(lo.x, hi.y, hi.z),
    ];
    let faces = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    faces.iter().map(|f| [c[f[0]], c[f[1]], c[f[2]]]).collect()
}

/// 竖壁上一片目标点：`x = WALL`，法向 `(-1,0,0)`（刀轴水平摆 90°）。
fn wall_targets(nz: usize, ny: usize) -> Vec<(DVec3, DVec3)> {
    let mut v = Vec::new();
    for k in 0..nz {
        for j in 0..ny {
            let y = 4.0 + j as f64 * 0.5;
            let z = 2.0 + k as f64 * 0.5;
            v.push((DVec3::new(6.0, y, z), DVec3::new(-1.0, 0.0, 0.0)));
        }
    }
    v
}

const SAFE_Z: f64 = 30.0;

#[test]
fn five_axis_pass_actually_cuts_a_vertical_wall() {
    // 一块 6..16 × 2..10 × 0..10 的料；机器链空运行（dry），只看扫掠体
    let mesh = box_mesh(
        DVec3::new(6.0, 2.0, 0.0),
        DVec3::new(16.0, 10.0, 10.0),
    );
    let st = Stock::from_size(DVec3::new(20.0, 12.0, 12.0));
    let tool = cncsim_cam::ToolSpec::ball(1, 2.0, 12.0);
    let targets = wall_targets(12, 12);

    let mut o = SurfaceOpts::new(tool.clone(), 1, st);
    o.safe_z_mm = SAFE_Z;
    o.out_step_mm = 0.0; // 逐点出块，便于逐点对账
    let out = generate_surface(&mesh, st, &targets, &o).expect("五轴道应能生成");
    assert_eq!(
        out.stats.a_rejected, 0,
        "法向 (−1,0,0) 的 A 角是 90°，不该超程"
    );
    assert!(
        out.stats.accepted >= targets.len() * 9 / 10,
        "至少 90% 的点该通过闸门：{} / {}",
        out.stats.accepted,
        targets.len()
    );
    assert!(out.gcode.contains("G43.4 H1"), "必须开 RTCP 才谈得上五轴");
    assert!(
        out.gcode.contains("A90.000"),
        "竖壁的 A 角必须是 90°：\n{}",
        &out.gcode[..out.gcode.len().min(800)]
    );

    // ── 解释 + 仿真（不切料，只收集扫掠段）──
    let mut g = String::from("G21 G90 G17 G94\n");
    g.push_str(&out.gcode);
    g.push_str("M5\nM30\n");
    let ic = InterpConfig {
        tool_radii: {
            let mut r = vec![0.0; 8];
            r[1] = tool.diameter * 0.5;
            r
        },
        rotary: cncsim_gcode::RotaryConvention::Head,
        ..Default::default()
    };
    let prog = interpret(&g, &ic);
    let table = ToolTable::new(vec![tool.to_tool_def()]);
    // `cut_segs` 只在"有设计场要对账"时才收集（那是运行时的用途），所以这里
    // 按 app 里的方式建一个小设计场——顺便让断言跑的是同一条链路。
    let grid = cncsim_voxel::GridConfig {
        h: 0.5,
        band_voxels: 4,
        value_type: cncsim_voxel::ValueType::I8,
    };
    let domain = [40, 24, 24];
    let design = cncsim_sim::compare::field_design(&mesh, grid, st.min, domain, 0.3);
    let cfg = RunConfig {
        grid,
        stock_origin: st.min,
        stock_size: domain,
        design: Some(design),
        dry_run: true,
        ..Default::default()
    };
    let run = run_tools(&prog, &table, &cfg);

    // ── 量：设计面点到"刀具扫掠体并集"的有符号距离 ──
    let pts: Vec<DVec3> = targets.iter().map(|(p, _)| *p).collect();
    let d = cncsim_sim::compare::sweep_deviation(&run.cut_segs, &table, &pts, 1.0);
    let ok = d.iter().filter(|v| v.abs() <= 0.15).count();
    let gouge = d.iter().filter(|v| **v < -0.15).count();
    let worst = d.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let deepest = d.iter().cloned().fold(f64::INFINITY, f64::min);
    println!(
        "竖壁 {} 点：合格 {ok} / 过切 {gouge} / 最深 {deepest:.3} / 最浅 {worst:.3}",
        pts.len()
    );
    assert!(
        ok * 100 >= pts.len() * 90,
        "至少 90% 的竖壁点该被切到（±0.15）：{ok} / {}",
        pts.len()
    );
    assert!(
        deepest > -0.15,
        "不该啃进设计面：最深 {deepest:.3} mm"
    );
}

/// 收尾必须自己退刀：**刀尖还歪着时竖直一抬**，会从工件里穿出去。
///
/// 这条是实测踩出来的（桌腿件 0.67 mm）。仿真里 `G0` 不算切削，但
/// `G53 Z<安全高度>` 这条轴向移动会被当成 G1 处理——82 mm 的长扫掠把整段料扫了。
#[test]
fn program_ends_with_the_tool_outside_the_part() {
    let mesh = box_mesh(
        DVec3::new(6.0, 2.0, 0.0),
        DVec3::new(16.0, 10.0, 10.0),
    );
    let st = Stock::from_size(DVec3::new(20.0, 12.0, 12.0));
    let tool = cncsim_cam::ToolSpec::ball(1, 2.0, 12.0);
    let mut o = SurfaceOpts::new(tool, 1, st);
    o.safe_z_mm = SAFE_Z;
    let out = generate_surface(&mesh, st, &wall_targets(6, 6), &o).expect("应能生成");
    let tail: Vec<&str> = out.gcode.lines().rev().take(6).collect();
    let joined = tail.iter().rev().cloned().collect::<Vec<_>>().join(" | ");
    assert!(
        joined.contains("G0 A0.000 C0.000"),
        "收尾要把刀摆回竖直：{joined}"
    );
    assert!(
        joined.contains(&format!("G0 Z{SAFE_Z:.3}")),
        "收尾要抬到安全高度：{joined}"
    );
}

/// 进给自适应：凹的地方 F 要降下来，**切出来的点一个都不许动**。
///
/// 两件事一起验：
///
/// * **几何不变**：开/关自适应各跑一遍，把所有 `F` 字从 G 代码里剥掉之后，
///   两份程序必须逐行相同——F 只改速度，不改刀尖去哪。
/// * **该降的地方降了**：正弦波纹的波谷（`x ≈ 6`，凹、曲率半径 0.91）的 F
///   明显低于名义值；波峰（`x ≈ 0`，凸，凹度必须为 0）仍是名义值。
///   只看"F 变过"会被噪声骗过去，必须钉到位置上。
#[test]
fn adaptive_feed_slows_in_a_groove_without_moving_a_point() {
    // z = A·cos(kx)，沿 Y 拉伸。波谷曲率半径 ρ = 1/(A·k²) = 0.91 > R0.5（球塞得下）
    let (amp, k) = (4.0f64, std::f64::consts::TAU / 12.0);
    let zc = 7.0;
    let zof = |x: f64| zc + amp * (k * x).cos();
    let zdev = |x: f64| -amp * k * (k * x).sin(); // dz/dx
    let (x0, x1, y0, y1) = (0.2f64, 7.8f64, 3.0f64, 9.0f64);
    let (nx, ny) = (38usize, 13usize);
    let px = |ix: usize| x0 + (x1 - x0) * ix as f64 / nx as f64;
    let py = |iy: usize| y0 + (y1 - y0) * iy as f64 / ny as f64;
    let mut mesh = Vec::new();
    for ix in 0..nx {
        for iy in 0..ny {
            let (xa, xb) = (px(ix), px(ix + 1));
            let (ya, yb) = (py(iy), py(iy + 1));
            let (za, zb) = (zof(xa), zof(xb));
            let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
            mesh.push([p(xa, ya, za), p(xb, ya, zb), p(xb, yb, zb)]);
            mesh.push([p(xa, ya, za), p(xb, yb, zb), p(xa, yb, za)]);
        }
    }
    // 目标点带真法向：n ∝ (−dz/dx, 0, 1)
    let mut targets = Vec::new();
    for ix in 0..=nx {
        let x = px(ix);
        for iy in 0..=ny {
            targets.push((
                DVec3::new(x, py(iy), zof(x)),
                DVec3::new(-zdev(x), 0.0, 1.0).normalize(),
            ));
        }
    }

    let st = Stock::from_size(DVec3::new(20.0, 16.0, 14.0));
    let tool = cncsim_cam::ToolSpec::ball(1, 1.0, 1.2);
    let mut o = SurfaceOpts::new(tool, 1, st);
    o.out_step_mm = 0.0; // 逐点出块：F 与点的对应关系才干净
    o.safe_z_mm = SAFE_Z;

    o.feed.adaptive = false;
    let off = generate_surface(&mesh, st, &targets, &o).expect("关闭自适应也要能生成");
    o.feed.adaptive = true;
    let on = generate_surface(&mesh, st, &targets, &o).expect("开启自适应也要能生成");

    assert_eq!(
        off.stats.accepted, on.stats.accepted,
        "F 不该改变通过闸门的点数"
    );
    assert!(
        on.stats.accepted >= targets.len() * 8 / 10,
        "波纹面至少 80% 该通过：{} / {}",
        on.stats.accepted,
        targets.len()
    );

    // ── 几何逐行相同（剥掉 F 字后）──
    let strip = |g: &str| -> Vec<String> {
        g.lines()
            .map(|l| {
                l.split_whitespace()
                    .filter(|w| !w.starts_with('F'))
                    .collect::<Vec<_>>()
                    .join(" ")
            })
            .collect()
    };
    let (a, b) = (strip(&off.gcode), strip(&on.gcode));
    assert_eq!(a.len(), b.len(), "行数不该变");
    for (i, (x, y)) in a.iter().zip(&b).enumerate() {
        assert_eq!(x, y, "第 {i} 行的几何被改动了：\n{x}\n{y}");
    }

    // ── F 随位置：波谷慢、波峰满速 ──
    let mut cur_f = 0.0f64;
    let (mut f_trough, mut f_crest) = (f64::INFINITY, 0.0f64);
    for l in on.gcode.lines() {
        if !l.starts_with("G1 ") {
            continue;
        }
        let mut x = 0.0;
        for w in l.split_whitespace() {
            let (c, rest) = w.split_at(1);
            let Ok(v) = rest.parse::<f64>() else { continue };
            match c {
                "X" => x = v,
                "F" => cur_f = v,
                _ => {}
            }
        }
        if (x - 6.0).abs() < 0.3 {
            f_trough = f_trough.min(cur_f);
        } else if x < 1.0 {
            f_crest = f_crest.max(cur_f);
        }
    }
    let nom = o.feed_mm_min;
    println!("波谷最低 F{f_trough:.0} / 波峰最高 F{f_crest:.0} / 名义 F{nom:.0}");
    assert!(f_trough.is_finite(), "波谷没有采到切削块");
    assert!(
        f_trough < 0.85 * nom,
        "波谷该减速：F{f_trough:.0} 应低于 {:.0}",
        0.85 * nom
    );
    assert!(
        f_crest > 0.95 * nom,
        "波峰是凸的，不该减速：F{f_crest:.0}"
    );
    assert!(
        on.stats.feed.time_s > off.stats.feed.time_s,
        "降速必然让切削时间变长：{:.1}s → {:.1}s",
        off.stats.feed.time_s,
        on.stats.feed.time_s
    );
    assert!(
        (off.stats.feed.min_vf - nom).abs() < 1e-9,
        "关闭自适应时必须全程名义值：{}",
        off.stats.feed.min_vf
    );
}
