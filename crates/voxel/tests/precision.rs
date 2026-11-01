//! 体素精度极限实测：量化误差如何随 `h` 与存储类型变化。
//!
//! 理论：`to_value` 把 `[−band, +band]` 均分给 `steps` 档，所以
//!
//! ```text
//! 量化步长 = 2 · band_voxels · h / steps
//! 表面位置最坏误差 = 步长 / 2
//! ```
//!
//! 也就是说**相对精度只由 `band_voxels / steps` 决定，与 `h` 无关**；
//! 减小 `h` 只是把同样的相对误差缩到更小的物理尺度上。
//!
//! 为了把 `h` 隔离出来，这里让**一切都按 `h` 归一化**：毛坯恒为 40×40×20 体素、
//! 刀具半径恒为 3h、槽底恒在 13.3h（**故意落在体素边界中间**，让量化误差最大化）。
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test precision -- --nocapture
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, Scalar, SparseGrid, ValueType};

/// 切一条槽，二分求槽底的实测位置。
///
/// 全部长度按 `h` 归一化：毛坯 40×40×20 体素、刀半径 3h、刀尖 13.3h。
fn measure_floor<S: Scalar>(h: f64, vt: ValueType) -> f64 {
    let cfg = GridConfig { h, band_voxels: 4, value_type: vt };
    let band = cfg.band_mm();
    let mut g = SparseGrid::<S>::new(cfg, DVec3::ZERO);
    g.fill_box_exact([0, 0, 0], [40, 40, 20], band);

    let tip_z = 13.3 * h; // 非整数体素 → 量化误差最大化
    let prof = flat_end_mill(6.0 * h, 20.0 * h);
    g.cut_segment(
        &Pose { tip: DVec3::new(5.0 * h, 20.0 * h, tip_z), axis: DVec3::Z },
        &Pose { tip: DVec3::new(35.0 * h, 20.0 * h, tip_z), axis: DVec3::Z },
        &prof,
    );

    // 沿 z 二分（三线性插值后的连续场，单调跨过槽底）
    let f = |z: f64| g.sdf(DVec3::new(20.0 * h, 20.0 * h, z));
    let (mut lo, mut hi) = (tip_z - 2.0 * h, tip_z + 2.0 * h);
    assert!(f(lo) < 0.0 && f(hi) > 0.0, "槽底两侧符号不对");
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// **I8：误差与 h 成正比**（相对精度恒定）。
#[test]
fn i8_error_scales_linearly_with_h() {
    println!("\n【I8 / band=4 体素】槽底理论位置 13.3h");
    println!("{:>8} {:>12} {:>12} {:>12} {:>12}", "h(mm)", "实测(mm)", "误差(µm)", "理论步长(µm)", "误差/h");
    let mut ratios = Vec::new();
    for h in [0.1_f64, 0.05, 0.02, 0.01] {
        let got = measure_floor::<i8>(h, ValueType::I8);
        let expect = 13.3 * h;
        let err_um = (got - expect).abs() * 1000.0;
        let step_um = ValueType::I8.step_mm(&GridConfig { h, band_voxels: 4, value_type: ValueType::I8 }) * 1000.0;
        println!(
            "{h:>8.3} {got:>12.6} {err_um:>12.2} {step_um:>12.2} {:>12.4}",
            err_um / 1000.0 / h
        );
        // 量化误差不可能超过半步长
        assert!(
            err_um <= step_um / 2.0 + 1e-6,
            "h={h}: 误差 {err_um:.2} µm 超过半量化步长 {:.2} µm",
            step_um / 2.0
        );
        ratios.push(err_um / 1000.0 / h);
    }
    // 相对误差应在同一量级（常数 × h），不是随 h 漂移
    let (mn, mx) = (
        ratios.iter().cloned().fold(f64::INFINITY, f64::min),
        ratios.iter().cloned().fold(0.0, f64::max),
    );
    assert!(
        mx < mn * 20.0 + 1e-9,
        "误差/h 应大致恒定（同一相对精度），实际 {mn:.5} … {mx:.5}"
    );
}

/// **I16：同样 `h` 下精度高 258 倍**（65534/254）。
#[test]
fn i16_is_two_orders_more_precise() {
    println!("\n【I16 / band=4 体素】槽底理论位置 13.3h");
    let mut i8_err = Vec::new();
    let mut i16_err = Vec::new();
    for h in [0.1_f64, 0.02] {
        let e8 = (measure_floor::<i8>(h, ValueType::I8) - 13.3 * h).abs();
        let e16 = (measure_floor::<i16>(h, ValueType::I16) - 13.3 * h).abs();
        println!("h={h:.3}: I8 误差 {:.3} µm，I16 误差 {:.4} µm", e8 * 1000.0, e16 * 1000.0);
        i8_err.push(e8);
        i16_err.push(e16);
    }
    for (e8, e16) in i8_err.iter().zip(&i16_err) {
        assert!(
            *e16 < *e8 * 0.05,
            "I16 应比 I8 精确至少 20 倍：I8 {:.3} µm vs I16 {:.4} µm",
            e8 * 1000.0,
            e16 * 1000.0
        );
    }
}

/// **band 越小越准**：步长正比于 band_voxels。
#[test]
fn narrower_band_improves_precision() {
    println!("\n【I8，h=0.05】band 宽度对精度的影响");
    let mut last = f64::INFINITY;
    for band_voxels in [8u8, 4, 2] {
        let cfg = GridConfig { h: 0.05, band_voxels, value_type: ValueType::I8 };
        let band = cfg.band_mm();
        let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
        g.fill_box_exact([0, 0, 0], [40, 40, 20], band);
        let tip_z = 13.3 * 0.05;
        let prof = flat_end_mill(6.0 * 0.05, 20.0 * 0.05);
        g.cut_segment(
            &Pose { tip: DVec3::new(0.25, 1.0, tip_z), axis: DVec3::Z },
            &Pose { tip: DVec3::new(1.75, 1.0, tip_z), axis: DVec3::Z },
            &prof,
        );
        let f = |z: f64| g.sdf(DVec3::new(1.0, 1.0, z));
        let (mut lo, mut hi) = (tip_z - 0.1, tip_z + 0.1);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if f(mid) < 0.0 { lo = mid } else { hi = mid }
        }
        let err = ((0.5 * (lo + hi)) - tip_z).abs() * 1000.0;
        println!("band={band_voxels} 体素: 步长 {:.2} µm，误差 {err:.2} µm", ValueType::I8.step_mm(&cfg) * 1000.0);
        assert!(err < last + 1e-9, "band 变窄误差应不增大");
        last = err;
    }
}

/// **表面提取（Surface Nets）的精度**：最终看到的 mesh 顶点也要落在真值上。
///
/// 这一步比 SDF 查询多了一层"单元格内 8 角加权"，所以单独量一次。
#[test]
fn mesh_vertices_land_on_the_true_surface() {
    println!("\n【I8 / band=4】mesh 顶点位置误差（槽底 13.3h）");
    for h in [0.1_f64, 0.02] {
        let cfg = GridConfig { h, band_voxels: 4, value_type: ValueType::I8 };
        let band = cfg.band_mm();
        let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
        g.fill_box_exact([0, 0, 0], [40, 40, 20], band);
        let tip_z = 13.3 * h;
        let prof = flat_end_mill(6.0 * h, 20.0 * h);
        g.cut_segment(
            &Pose { tip: DVec3::new(5.0 * h, 20.0 * h, tip_z), axis: DVec3::Z },
            &Pose { tip: DVec3::new(35.0 * h, 20.0 * h, tip_z), axis: DVec3::Z },
            &prof,
        );

        let mesh = g.extract_surface();
        // 槽底中段（避开两端和槽壁）
        let zs: Vec<f64> = mesh
            .positions
            .iter()
            .filter(|p| {
                let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
                x > 15.0 * h
                    && x < 25.0 * h
                    && y > 18.0 * h
                    && y < 22.0 * h
                    && (z - tip_z).abs() < 1.5 * h
            })
            .map(|p| p[2] as f64)
            .collect();
        assert!(!zs.is_empty(), "h={h}: 槽底附近应有顶点");
        let mean = zs.iter().sum::<f64>() / zs.len() as f64;
        let err = (mean - tip_z).abs() * 1000.0;
        let step = ValueType::I8.step_mm(&cfg) * 1000.0;
        println!("h={h:.2}: {} 个顶点，平均 z={mean:.6}，误差 {err:.2} µm（步长 {step:.2} µm）", zs.len());
        assert!(err < step, "h={h}: mesh 误差 {err:.2} µm 应小于一个量化步长 {step:.2} µm");
    }
}
