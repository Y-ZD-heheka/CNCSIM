//! LOD 提取里"逐顶点取切割色"的开销。
//!
//! `extract_surface_lod` 对**每个顶点**调一次 `creator_of_voxel`，而那是
//! "哈希查根 + 逐级下降"找叶——比 `extract_surface`（按叶批量取）贵得多。
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test lod_color_perf -- --ignored --nocapture
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, SparseGrid, ValueType};

fn grid(sub: u8, stamp: bool) -> SparseGrid<i8> {
    let cfg = GridConfig { h: 0.1, band_voxels: 4, value_type: ValueType::I8 };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.set_creator_sub(sub);
    g.fill_box_exact([0, 0, 0], [300, 300, 120], cfg.band_mm());
    if stamp {
        // 面铣十刀，制造真实的分色面
        let prof = flat_end_mill(6.0, 20.0);
        g.set_creator(1);
        for i in 0..10 {
            let y = 1.5 + i as f64 * 3.0;
            g.cut_segment(
                &Pose { tip: DVec3::new(-10.0, y, 11.0), axis: DVec3::Z },
                &Pose { tip: DVec3::new(40.0, y, 11.0), axis: DVec3::Z },
                &prof,
            );
        }
    }
    g
}

#[test]
#[ignore = "性能基准，手动跑"]
fn lod_color_cost() {
    let mut best = f64::INFINITY;
    let mut verts = 0usize;
    for _ in 0..5 {
        let g = grid(4, true);
        let t0 = std::time::Instant::now();
        let m = g.extract_surface_lod(3);
        best = best.min(t0.elapsed().as_secs_f64() * 1e3);
        verts = m.positions.len();
    }
    println!();
    println!("LOD k=3 提取：{best:.1} ms，{verts} 顶点 → {:.1} ns/顶点", best * 1e6 / verts as f64);

    // 对照：sub=1（每叶一个值）也走同一条逐顶点查找，先看它是不是同一量级
    let mut best1 = f64::INFINITY;
    for _ in 0..5 {
        let g = grid(1, true);
        let t0 = std::time::Instant::now();
        let _ = g.extract_surface_lod(3);
        best1 = best1.min(t0.elapsed().as_secs_f64() * 1e3);
    }
    println!("LOD k=3 提取（sub=1 对照）：{best1:.1} ms");

    // ── 单独量"逐顶点找叶取色"这一步 ──
    // 顶点坐标从真实提取结果里取，保证访问模式和实际一致。
    let g = grid(4, true);
    let m = g.extract_surface_lod(3);
    let probes: Vec<[i32; 3]> = m
        .positions
        .iter()
        .map(|p| {
            [
                (p[0] as f64 / 0.1).round() as i32,
                (p[1] as f64 / 0.1).round() as i32,
                (p[2] as f64 / 0.1).round() as i32,
            ]
        })
        .collect();
    let mut sink = 0u64;
    let mut bl = f64::INFINITY;
    for _ in 0..5 {
        let t0 = std::time::Instant::now();
        for &p in &probes {
            sink += g.creator_of_voxel(p) as u64;
        }
        bl = bl.min(t0.elapsed().as_secs_f64() * 1e6);
    }
    println!(
        "  其中 creator_of_voxel：{:.2} ms（{:.1} ns/次）  sink={sink}",
        bl / 1e3,
        bl / probes.len() as f64
    );

    // 带游标的版本：相邻探针大多落在同一片叶里，省掉哈希查根
    let mut sink2 = 0u64;
    let mut bc = f64::INFINITY;
    for _ in 0..5 {
        let mut cur = cncsim_voxel::tree::CreatorCursor::default();
        let t0 = std::time::Instant::now();
        for &p in &probes {
            sink2 += g.creator_of_voxel_cached(p, &mut cur) as u64;
        }
        bc = bc.min(t0.elapsed().as_secs_f64() * 1e6);
    }
    println!(
        "  带游标：{:.2} ms（{:.1} ns/次）  结果一致={}",
        bc / 1e3,
        bc / probes.len() as f64,
        sink == sink2
    );
}

/// 给 VTune 用的长跑：同一张网格反复提取 LOD。
///
/// 提取一次只有 ~28 ms，采样间隔 5 ms 只够几十个样本，必须重复。
#[test]
#[ignore = "VTune 采集用（需提权）"]
fn profile_lod() {
    let g = grid(4, true);
    let mut sink = 0usize;
    for _ in 0..120 {
        let m = g.extract_surface_lod(3);
        sink += m.positions.len();
    }
    println!("profile_lod sink={sink}");
}
