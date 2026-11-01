//! 一刀的切削微基准（默认跳过）。
//!
//! "切一刀要多久"有两个口径，这里都量：
//! - **仿真耗时**：`cut_segment` 从调用到返回（含 AABB 裁剪、带内求值、并行落盘）
//! - **程序时间**：这段走刀在实际机床上要多久（进给率决定，与仿真无关）
//!
//! 想知道求值/落盘各占多少，设 `CNCSIM_PROF=1` 跑任意切削基准（见 `src/cut.rs`）。
//! （把求值 / 落盘 / prune 三段分开计时）。
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test cut_perf -- --ignored --nocapture
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, SparseGrid};

fn up(x: f64, y: f64, z: f64) -> Pose {
    Pose {
        tip: DVec3::new(x, y, z),
        axis: DVec3::Z,
    }
}

/// 30×30×12 mm 实心毛坯（300×300×120 体素 @ h=0.1）——与演示场景同尺寸。
fn stock() -> SparseGrid<i8> {
    let mut g = SparseGrid::<i8>::new(GridConfig::default(), DVec3::ZERO);
    g.fill_box([0, 0, 0], [300, 300, 120], -127);
    g
}

/// 一刀 = 一次 G1 切削走（x0 → x0+len，y 固定）。每刀换 y，互不重叠，
/// 避免"前面切过所以后面更快"的假象。
///
/// 数据（i7-13700K / 24 线程，release）。**不含转 mesh、不含渲染**：
///
/// | 长度 | 切深 | 纯 SDF 求值 | 落盘+prune | 合计 | 遍历 | @F300 |
/// |---|---|---|---|---|---|---|
/// | 1 mm | 0.5 | 0.4 ms | 0.8 ms | 1.6 ms | 61 k | 0.20 s |
/// | 10 mm | 0.5 | 1.2 ms | 1.6 ms | 2.0 ms | 149 k | 2.00 s |
/// | 20 mm | 0.5 | 1.2 ms | 2.3 ms | 3.4 ms | 254 k | 4.00 s |
/// | 28 mm | 0.5 | 1.4 ms | 3.0 ms | 4.2 ms | 311 k | 5.60 s |
/// | 20 mm | 2.0 | — | — | 6.1 ms | 479 k | 4.00 s |
/// | 20 mm | 空刀 | — | — | 0.7 ms | 103 k | 4.00 s |
///
/// 关键观察：
/// - 求值占 34%（每体素 **~5 ns**）、落盘 55%、prune 10%。
/// - 耗时 ≈ 遍历体素数 × 常数，**与"走了多长"只是间接相关**（AABB 决定一切）。
///   所以 1 mm 的刀也要 1.6 ms——刀径和 band 撑起来的 AABB 有 6×1.4 mm 截面。
/// - 空刀很便宜（0.7 ms）：值没变，落盘全跳过；求值仍要跑。
/// - 短刀并行效率差：1 mm 段只有 14 层可切，8× 掉到 4.6×。
/// - 首次一刀的 `changed` 大多是**从饱和值 −127 变精确窄带值**，不是变空气。
#[test]
#[ignore = "微基准，手动跑"]
fn one_pass_timing() {
    let prof = flat_end_mill(6.0, 20.0);
    let mut g = stock();

    println!("\n【单刀耗时】30×30×12 毛坯，h=0.1，φ6 平底刀");
    println!(
        "{:<9} {:<7} {:>9} {:>11} {:>11} {:>9}",
        "长度", "切深", "耗时ms", "遍历体素", "改动体素", "@F300"
    );

    // (刀长, 起点 x, y, 刀尖 z)——毛坯顶 11.5，z=11.0 即切深 0.5
    let cases = [
        (1.0, 2.0, 3.0, 11.0),
        (5.0, 2.0, 6.0, 11.0),
        (10.0, 2.0, 10.0, 11.0),
        (20.0, 2.0, 15.0, 11.0),
        (28.0, 1.0, 22.0, 11.0),
        (20.0, 2.0, 25.0, 9.5),
    ];

    for (len, x0, y, z) in cases {
        let depth = 11.5 - z;
        let from = up(x0, y, z);
        let to = up(x0 + len, y, z);
        let t0 = std::time::Instant::now();
        let st = g.cut_segment(&from, &to, &prof);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        println!(
            "{:<9} {:<7} {:>9.3} {:>11} {:>11} {:>8.2}s",
            format!("{len} mm"),
            format!("{depth} mm"),
            ms,
            st.visited,
            st.changed,
            len / (300.0 / 60.0)
        );
    }

    // 空刀：位置已经切过，值不变但仍要逐体素比较
    let from = up(2.0, 10.0, 11.0);
    let to = up(12.0, 10.0, 11.0);
    let t0 = std::time::Instant::now();
    let st = g.cut_segment(&from, &to, &prof);
    println!(
        "\n空刀（重复切同一刀，20 mm 等效 10 mm 段）：{:.3} ms，遍历 {}，改动 {}",
        t0.elapsed().as_secs_f64() * 1e3,
        st.visited,
        st.changed
    );
}
