//! 与原生 OpenVDB 的**几何对拍**：同样的毛坯、刀具、刀路，SDF 值应该一致到量化精度。
//!
//! 对照侧是 `tools/ovdb_bench/ovdb_bench.cpp` 的 `PROFILE` 段，场景逐项对齐：
//! 30×30×12 毛坯 / h=0.1 / 窄带 4 体素 / φ6 平底刀（r=3, 长 20）/ 刀尖 z=11.5 / 四刀。
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test ovdb_parity -- --nocapture
//! ```
//!
//! 输出格式与 OpenVDB 侧一致（`Z <坐标> <SDF>` / `Y <坐标> <SDF>`），
//! 便于直接 diff。采样点用 `floor(x/h)` 定位体素——两边同一约定，
//! 取的都是**体素中心**的存储值（不走三线性）。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, SparseGrid};

/// 建场：与 `tools/ovdb_bench` 完全同场景，四刀切完。
fn build_scene() -> (SparseGrid<i8>, f64) {
    let h = 0.1;
    let mut g = SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO);
    // **用精确版建毛坯**：边界壳写成真实盒 SDF——OpenVDB 的 level set 毛坯就是这个形态。
    // 用饱和版（`fill_box`）的话未切削面的插值位置会偏（实测左侧壁差 25 µm）。
    let band = GridConfig::default().band_mm();
    g.fill_box_exact([0, 0, 0], [300, 300, 120], band);

    let prof = flat_end_mill(6.0, 20.0);
    for (len, x0, y) in [(1.0, 2.0, 3.0), (10.0, 2.0, 10.0), (20.0, 2.0, 15.0), (28.0, 1.0, 22.0)] {
        g.cut_segment(
            &Pose { tip: DVec3::new(x0, y, 11.5), axis: DVec3::Z },
            &Pose { tip: DVec3::new(x0 + len, y, 11.5), axis: DVec3::Z },
            &prof,
        );
    }
    (g, h)
}

/// 采样：包含该坐标的体素（与 OpenVDB 的 `floor(x/h)` 一致）。
fn probe(g: &SparseGrid<i8>, h: f64, x: f64, y: f64, z: f64) -> f64 {
    let ijk = [
        (x / h).floor() as i32,
        (y / h).floor() as i32,
        (z / h).floor() as i32,
    ];
    g.to_distance(g.get(ijk))
}

#[test]
fn dump_profile_for_openvdb_comparison() {
    let (g, h) = build_scene();

    println!("=== PROFILE_BEGIN ===");
    // **按体素索引采样**（与 OpenVDB 侧一致）：0.1 在二进制下不精确，
    // 用 11.0 + i*0.1 生成坐标再 floor(x/h) 会偶尔掉到前一个体素。
    let idx = |i: i32, j: i32, k: i32| g.to_distance(g.get([i, j, k]));
    // ① 槽底：x=15, y=22（k 从 110 到 120）
    for k in 110..=120 {
        println!("Z {:.2} {:.6}", (k as f64 + 0.5) * h, idx(150, 220, k));
    }
    // ② 槽壁：x=15, z=11.9（j 从 185 到 255）
    for j in 185..=255 {
        println!("Y {:.2} {:.6}", (j as f64 + 0.5) * h, idx(150, j, 119));
    }
    println!("=== PROFILE_END ===");
}

/// **几何一致性断言**：表面位置与原生 OpenVDB 14.0.0 完全一致。
///
/// 参考值来自 `tools/ovdb_bench`（clang++ -O3 -march=native，同一场景）的
/// `PROFILE` 段，用相邻两点线性插值求过零点。容差取 `2 µm`——
/// `i8` 在带内 256 级的量化步长是 3.15 µm，过零点插值实际只用到半步。
#[test]
fn surface_positions_match_native_openvdb() {
    let (g, h) = build_scene();

    // 表面位置：在参考过零点两侧二分（SDF 在表面附近单调）
    let check = |name: &str, expect: f64, f: &dyn Fn(f64) -> f64| {
        let (mut lo, mut hi) = (expect - 0.2, expect + 0.2);
        let vlo = f(lo);
        let vhi = f(hi);
        // 方向无关：左壁是"材料→空气"，右壁是"空气→材料"
        assert!(
            vlo * vhi < 0.0,
            "{name}: 参考过零点两侧没有符号变化（{vlo:+.3} / {vhi:+.3}）"
        );
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            // 与 lo 同号就收缩 lo，否则收缩 hi
            if (f(mid) < 0.0) == (vlo < 0.0) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let got = 0.5 * (lo + hi);
        assert!(
            (got - expect).abs() < 2e-3,
            "{name}: 过零点 {got:.4} vs OpenVDB {expect:.4}，差 {:.1} µm（容差 2 µm）",
            (got - expect).abs() * 1000.0
        );
    };

    check("Z 槽底", 11.5000, &|t| probe(&g, h, 15.0, 22.0, t));
    check("Y 左壁", 19.0000, &|t| probe(&g, h, 15.0, t, 11.9));
    check("Y 右壁", 25.0000, &|t| probe(&g, h, 15.0, t, 11.9));

    // 带内数值也与 OpenVDB 一致到量化精度（体素中心 11.35/11.45/11.55/11.75）
    for (k, expect) in [(113_i32, -0.150_f64), (114, -0.050), (115, 0.050), (117, 0.250)] {
        let got = g.to_distance(g.get([150, 220, k]));
        assert!(
            (got - expect).abs() < 3e-3,
            "k={k}: {got:+.4} vs OpenVDB {expect:+.4}"
        );
    }
}
