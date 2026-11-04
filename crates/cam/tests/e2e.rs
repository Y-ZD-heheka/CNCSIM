//! 端到端验收：**CAM 生成 → 现有解释器 → 现有仿真 → 设计比对**。
//!
//! 零件是一个封闭的"高场"凸包：24×24×2 的板 + 中间 1.5 mm 高的光滑凸包。
//! 它满足 CAM 的工艺假设（3 轴、+Z、无倒扣），所以过切/残留能直接量。
//!
//! 断言的是**物理量**，不是文本：过切深度、残留厚度、凸包顶点高度。
//! 这些量全部由 `cncsim-sim` 的体素积分 + 解析扫掠测试给出。

use cncsim_cam::{Job, Process, Stock, ToolSpec, Tri, generate};
use cncsim_gcode::{InterpConfig, interpret};
use cncsim_math::DVec3;
use cncsim_sim::compare::field_design;
use cncsim_sim::run::{RunConfig, run_tools};
use cncsim_tool::ToolTable;
use cncsim_voxel::{GridConfig, ValueType};

const SIZE: f64 = 16.0;
const BASE_Z: f64 = 2.0;
const BUMP: f64 = 1.2;

/// 光滑凸包：中心 (12,12)，σ²=40；边界处几乎回到 BASE_Z。
fn height(x: f64, y: f64) -> f64 {
    let r2 = (x - SIZE / 2.0).powi(2) + (y - SIZE / 2.0).powi(2);
    BASE_Z + BUMP * (-r2 / 18.0).exp()
}

/// 封闭网格：顶面高度场 + 四面竖壁 + 底面。
fn relief_mesh(n: usize) -> Vec<Tri> {
    let step = SIZE / n as f64;
    let v = |i: usize, j: usize| {
        DVec3::new(i as f64 * step, j as f64 * step, height(i as f64 * step, j as f64 * step))
    };
    let b = |i: usize, j: usize| DVec3::new(i as f64 * step, j as f64 * step, 0.0);
    let mut tris = Vec::new();

    // 顶面
    for j in 0..n {
        for i in 0..n {
            tris.push([v(i, j), v(i + 1, j), v(i + 1, j + 1)]);
            tris.push([v(i, j), v(i + 1, j + 1), v(i, j + 1)]);
        }
    }
    // 底面（法向 -Z）
    for j in 0..n {
        for i in 0..n {
            tris.push([b(i, j), b(i + 1, j + 1), b(i + 1, j)]);
            tris.push([b(i, j), b(i, j + 1), b(i + 1, j + 1)]);
        }
    }
    // 四面竖壁
    for k in 0..n {
        // x = 0（外法向 -X）
        tris.push([v(0, k), v(0, k + 1), b(0, k + 1)]);
        tris.push([v(0, k), b(0, k + 1), b(0, k)]);
        // x = SIZE（+X）
        tris.push([v(n, k), b(n, k), b(n, k + 1)]);
        tris.push([v(n, k), b(n, k + 1), v(n, k + 1)]);
        // y = 0（-Y）
        tris.push([v(k, 0), b(k, 0), b(k + 1, 0)]);
        tris.push([v(k, 0), b(k + 1, 0), v(k + 1, 0)]);
        // y = SIZE（+Y）
        tris.push([v(k, n), v(k + 1, n), b(k + 1, n)]);
        tris.push([v(k, n), b(k + 1, n), b(k, n)]);
    }
    tris
}

#[test]
fn relief_mesh_is_machined_without_gouge_and_with_bounded_residual() {
    let mesh = relief_mesh(32);
    let stock = Stock::from_size(DVec3::new(SIZE, SIZE, 3.5));
    let rough = ToolSpec::flat(1, 4.0, 20.0);
    let finish = ToolSpec::ball(2, 3.0, 20.0);
    let process = Process {
        stock_allowance_mm: 0.2,
        stepdown_mm: 0.8,
        rough_stepover_mm: 2.0,
        finish_stepover_mm: 0.8,
        sample_mm: 0.25,
        max_finish_slope_deg: 80.0,
        ..Default::default()
    };
    let job = Job {
        mesh: &mesh,
        stock,
        rough: Some(rough.clone()),
        finish: finish.clone(),
        process: process.clone(),
    };
    let prog = generate(&job).expect("CAM 生成失败");
    println!(
        "G 代码：{} 行 / {} 层 / 粗 {} 段 / 精 {} 段 / 切 {} mm / 快移 {} mm",
        prog.stats.lines,
        prog.stats.levels,
        prog.stats.rough_moves,
        prog.stats.finish_moves,
        prog.stats.cut_mm as i64,
        prog.stats.rapid_mm as i64
    );

    // ── 现有解释器 ──
    let ic = InterpConfig {
        tool_radii: vec![0.0, rough.radius(), finish.radius()],
        ..Default::default()
    };
    let program = interpret(&prog.gcode, &ic);
    assert!(!program.has_errors(), "解释器报错：{:?}", program.issues);
    assert_eq!(program.stops.len(), 1, "应以 M30 结束");

    // ── 现有仿真 + 设计比对 ──
    let cfg_grid = GridConfig {
        h: 0.25,
        band_voxels: 4,
        value_type: ValueType::I8,
    };
    let domain = [
        (SIZE / cfg_grid.h) as i32,
        (SIZE / cfg_grid.h) as i32,
        (3.5 / cfg_grid.h) as i32,
    ];
    let design = field_design(&mesh, cfg_grid, DVec3::ZERO, domain, 0.25);
    let cfg = RunConfig {
        grid: cfg_grid,
        stock_origin: DVec3::ZERO,
        stock_size: domain,
        design: Some(design),
        design_step: 0.25,
        collide: false,
        creator_sub: 1,
        ..Default::default()
    };
    let tools = ToolTable::new(vec![rough.to_tool_def(), finish.to_tool_def()]);
    let out = run_tools(&program, &tools, &cfg);
    let r = &out.report;
    println!(
        "仿真：{} 块 / {} 段 / 去料 {:.1} mm³",
        out.stats.blocks, out.stats.cut_segments, r.removed_volume_mm3
    );
    println!(
        "过切 {:.3} mm³ / {:.4} mm；残留 {:.3} mm³ / {:.4} mm @ {:?}",
        r.gouge_volume_mm3,
        r.gouge_max_depth_mm,
        r.residual_volume_mm3,
        r.residual_max_thickness,
        r.residual_at
    );

    assert!(
        r.gouge_max_depth_mm < 0.03,
        "过切太深：{:.4} mm @ {:?}",
        r.gouge_max_depth_mm,
        r.gouge_at
    );
    assert!(
        r.residual_max_thickness < 0.25,
        "残留太厚：{:.4} mm @ {:?}",
        r.residual_max_thickness,
        r.residual_at
    );

    // ── 形状本身：凸包顶点被切到 3.5±0.1，中心列下方是料、上方是气 ──
    let at = |x: f64, y: f64, z: f64| {
        out.grid.get([
            (x / cfg_grid.h).floor() as i32,
            (y / cfg_grid.h).floor() as i32,
            (z / cfg_grid.h).floor() as i32,
        ])
    };
    assert!(at(SIZE / 2.0, SIZE / 2.0, 3.35) >= 0, "凸包顶点没切到：3.35 处仍是料");
    assert!(at(SIZE / 2.0, SIZE / 2.0, 3.05) < 0, "凸包顶点切过头：3.05 处已经是气");
    assert!(at(SIZE / 2.0, SIZE / 2.0, 2.10) < 0, "底部余量没削掉：2.10 处仍是料");
    assert!(at(1.5, 1.5, 2.20) >= 0, "平坦区被切过头：2.20 处已是气");
    assert!(at(1.5, 1.5, 1.90) < 0, "平坦区没切到 2.0");
}
