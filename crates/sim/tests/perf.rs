//! 毛坯碰撞的成本基准（默认跳过）。
//!
//! ```bash
//! cargo test --release -p cncsim-sim --test perf -- --ignored --nocapture
//! ```

use cncsim_collide::continuous::SurfaceBody;
use cncsim_math::{DVec2, DVec3};
use cncsim_motion::Pose;
use cncsim_sim::stock::{shank_vs_stock_continuous, shank_vs_stock_voxels};
use cncsim_tool::{CutterKind, Profile, ToolDef, flat_end_mill};
use cncsim_voxel::{GridConfig, SparseGrid};

fn tool() -> ToolDef {
    let shank = Profile::new(vec![
        DVec2::new(0.0, 20.0),
        DVec2::new(4.5, 20.0),
        DVec2::new(4.5, 80.0),
        DVec2::new(0.0, 80.0),
    ]);
    ToolDef {
        number: 1,
        kind: CutterKind::Flat,
        diameter: 6.0,
        corner_radius: 0.0,
        flute_length: 20.0,
        shank,
        length_offset: 80.0,
        cutter: flat_end_mill(6.0, 20.0),
    }
}

/// 200×200×100 mm 毛坯（h=0.1）
fn stock() -> SparseGrid<i8> {
    let mut g = SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO);
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    g
}

#[test]
#[ignore = "性能基准，默认不跑"]
fn cost_of_both_directions() {
    let g = stock();
    let t = tool();
    let body = SurfaceBody::revolve(&t.shank, 0.5, 0.5);
    println!("刀杆采样点：{}，间隙 {:.2} mm", body.len(), body.gap());

    // 场景 1：刀尖在毛坯顶面（z=100），刀杆在 z∈[120,180] —— 完全在空中
    // 场景 2：刀尖在 z=50，刀杆 z∈[70,130] —— 扎进毛坯（顶面 100）
    for (name, tip_z, n) in [("空中（刀杆在毛坯上方 20 mm）", 100.0, 200), ("扎进毛坯", 50.0, 50)] {
        let mut a_ms = 0.0;
        let mut b_ms = 0.0;
        let mut a_hits = 0;
        let mut b_hits = 0;
        for i in 0..n {
            let x0 = i as f64 * 0.5;
            let x1 = x0 + 0.5;
            let a = Pose { tip: DVec3::new(x0, 100.0, tip_z), axis: DVec3::Z };
            let b = Pose { tip: DVec3::new(x1, 100.0, tip_z), axis: DVec3::Z };

            let t0 = std::time::Instant::now();
            if shank_vs_stock_voxels(&g, &a, &b, &t.shank).is_some() {
                a_hits += 1;
            }
            a_ms += t0.elapsed().as_secs_f64() * 1e3;

            let t0 = std::time::Instant::now();
            if shank_vs_stock_continuous(&g, &body, &a, &b).is_some() {
                b_hits += 1;
            }
            b_ms += t0.elapsed().as_secs_f64() * 1e3;
        }
        println!(
            "{name}：方向 A {:.2} ms/段（命中 {a_hits}/{n}），方向 B {:.3} ms/段（命中 {b_hits}/{n}）",
            a_ms / n as f64,
            b_ms / n as f64
        );
    }
}

/// 方向 A 的遍历量：刀杆扎进毛坯时到底扫了多少体素
#[test]
#[ignore = "性能基准，默认不跑"]
fn voxel_sweep_volume() {
    let g = stock();
    let t = tool();
    let a = Pose { tip: DVec3::new(100.0, 100.0, 50.0), axis: DVec3::Z };
    let b = Pose { tip: DVec3::new(100.0, 100.0, 50.0), axis: DVec3::Z };
    let hit = shank_vs_stock_voxels(&g, &a, &b, &t.shank);
    match hit {
        Some(h) => println!("命中 {:?}，深度 {:.2} mm，遍历 {} 个材料体素", h.ijk, h.depth, h.visited),
        None => println!("未命中"),
    }
}
