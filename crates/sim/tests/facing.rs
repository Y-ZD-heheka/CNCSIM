//! 面铣一层之后，切面必须**一条带子一种颜色**，而且十条带子把顶面铺满。
//!
//! 这是"每切一条换一种颜色"的定量判据。光看截图只能说"看起来有条纹"，
//! 说不清三件事：
//! · 是不是**每**条都有各自的颜色（少一条就说明有两刀撞色）；
//! · 带子宽度是不是等于**实际新切到的宽度**（不是刀径，是前一刀留下的边）；
//! · 十条加起来是不是把 30×30 的顶面**盖满**（有缝就说明某刀没切到）。

use cncsim_gcode::{InterpConfig, interpret};
use cncsim_sim::demo;
use cncsim_sim::run::{RunConfig, run};
use cncsim_voxel::color_bands;

fn run_facing() -> cncsim_voxel::MeshData {
    let ic = InterpConfig { tool_radii: vec![0.0, 3.0], ..Default::default() };
    let prog = interpret(&demo::facing_gcode(), &ic);
    let cfg = RunConfig {
        stock_size: [300, 300, 120],
        // 不做设计比对：这里只关心切面颜色，比对会让运行慢一倍
        design: None,
        ..Default::default()
    };
    let out = run(&prog, &demo::tool(), &cfg);
    // k=1（不降采样）：带子边界要看清，LOD 会把边界量化到 0.3 mm
    out.grid.extract_surface()
}

#[test]
fn facing_gives_one_band_per_pass() {
    let mesh = run_facing();
    let bands = color_bands(&mesh);
    let cut: Vec<_> = bands.iter().filter(|b| b.index != 0).collect();

    assert_eq!(
        cut.len(),
        demo::FACING_PASSES,
        "应有 {} 条带子（每刀一条），实际 {}：{:?}",
        demo::FACING_PASSES,
        cut.len(),
        cut.iter().map(|b| b.index).collect::<Vec<_>>()
    );
    // 色号 = 刀具号 × 10 段 + 进度段。面铣没有 T 字（刀号 0 → 按第 1 把算），
    // 所以 10 刀正好落在 1..=10，每刀一段。
    let mut idx: Vec<u8> = cut.iter().map(|b| b.index).collect();
    idx.sort_unstable();
    assert_eq!(idx[0], 1, "第一刀应是第 1 段（色号 1），实际 {}", idx[0]);
    assert_eq!(
        *idx.last().unwrap(),
        10,
        "最后一刀应是第 10 段（色号 10），实际 {}",
        idx.last().unwrap()
    );
    assert!(
        idx.windows(2).all(|w| w[0] < w[1]),
        "每刀的编码必须严格递增（有重复说明两刀撞了同一档）：{idx:?}"
    );
}

/// 每条带子都是**平的**（同一个 Z 平面），且十条把 30×30 顶面盖满。
#[test]
fn facing_bands_are_flat_and_cover_the_face() {
    let mesh = run_facing();
    let bands = color_bands(&mesh);
    let mut cut: Vec<_> = bands.iter().filter(|b| b.index != 0).copied().collect();
    cut.sort_by(|a, b| a.lo[1].partial_cmp(&b.lo[1]).unwrap());

    let stock_w = demo::STOCK_MM[1] as f32;
    let step = demo::FACING_STEP_MM as f32;

    for b in &cut {
        // Z 跨度**不能**要求 ≤ 量化档：侧壁上有一圈（最多一个叶宽 8h = 0.8 mm）
        // 会被叶级溯源溢出染上同一档颜色，那一圈在 z 上比平面低 0.8 mm。
        // 所以"平不平"要看 `area_up` 的占比，而不是看包围盒。
        let dz = b.hi[2] - b.lo[2];
        assert!(dz <= 0.9, "第 {} 条带子的 Z 跨度 {dz:.2} mm，超过一个叶宽", b.index);
        let up_frac = b.area_up_mm2 / b.area_mm2.max(1e-9);
        assert!(
            up_frac > 0.75,
            "第 {} 条带子只有 {:.0}% 是朝上的平面（其余是侧壁溢出）",
            b.index,
            up_frac * 100.0
        );
        let w = b.hi[1] - b.lo[1];
        // 带宽上限取 **1.6 个步距**，不是 1 个：第一条带子没人替它切，
        // 名义宽度就是 1.5 步距（刀心 y=1.5 覆盖 0..4.5）。
        // "两刀被并成一条"由 `facing_gives_one_band_per_pass` 的带子数断言守，
        // 这里只是防单条异常膨胀。
        assert!(
            w <= 1.6 * step,
            "第 {} 条带子太宽（{w:.2} mm，步距 {step:.1}）",
            b.index
        );
    }

    // 首尾要盖到毛坯两侧
    assert!(cut[0].lo[1] <= 1.0, "最靠前的一条没盖到 y=0：{}", cut[0].lo[1]);
    assert!(
        cut[cut.len() - 1].hi[1] >= stock_w - 1.0,
        "最后一条没盖到 y={stock_w}：{}",
        cut[cut.len() - 1].hi[1]
    );
    // 相邻带子不能有缝（允许一个叶宽 + 量化的错位）
    for w in cut.windows(2) {
        let gap = w[1].lo[1] - w[0].hi[1];
        assert!(
            gap <= 0.9,
            "第 {} 条与第 {} 条之间有 {gap:.2} mm 的缝（没被任何一刀盖住）",
            w[0].index,
            w[1].index
        );
    }
}

/// 切掉的总面积必须等于 30×30 的顶面——这是"均匀切掉一层"的直接检验。
///
/// 面积按三角形累加，不受带子划分影响，所以这条**不依赖**着色对不对；
/// 它和上一条一起才说明"颜色分对了，而且分的是一整块面"。
#[test]
fn facing_removes_exactly_the_top_layer() {
    let mesh = run_facing();
    let bands = color_bands(&mesh);
    // 只看**朝上**的面积：侧壁上那一圈叶级溢出（周长 120 × 一个叶宽 0.8
    // ≈ 96 mm²）会把总面积撑大 6%，用总面积做判据会误判成"多切了"。
    let cut_area: f64 = bands.iter().filter(|b| b.index != 0).map(|b| b.area_up_mm2).sum();
    let want = demo::STOCK_MM[0] * demo::STOCK_MM[1];
    assert!(
        (cut_area - want).abs() / want < 0.03,
        "切面（朝上部分）面积 {cut_area:.1} mm²，应约为 {want:.1} mm²（顶面 30×30）"
    );

    // 原始毛坯面 = 四个侧壁 + 底面 = 4×30×11 + 30×30
    let stock_area: f64 = bands.iter().filter(|b| b.index == 0).map(|b| b.area_mm2).sum();
    let want0 = 4.0 * 30.0 * 11.0 + 30.0 * 30.0;
    assert!(
        (stock_area - want0).abs() / want0 < 0.05,
        "未切面面积 {stock_area:.1} mm²，应约为 {want0:.1} mm²（侧壁 + 底面）"
    );
}

/// 带子边界上的"混色三角形"必须很少——多到一定程度就说明着色比网格还碎，
/// 那就不是"一条带子一色"而是噪点了。
#[test]
fn band_boundaries_are_not_noisy() {
    let mesh = run_facing();
    let bands = color_bands(&mesh);
    let mixed: usize = bands.iter().map(|b| b.mixed_triangles).sum();
    let total = mesh.indices.len() / 3;
    // 边界有 10 条，每条按 30 mm / 0.1 mm 体素 ≈ 300 个三角形跨过去，
    // 取 10% 作为上限已经非常宽松。
    assert!(
        (mixed as f64) < total as f64 * 0.10,
        "混色三角形 {mixed} / {total}，超过 10%：着色边界太碎"
    );
}
