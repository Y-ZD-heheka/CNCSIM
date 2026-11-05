//! **切割颜色开关**：关掉染色必须「零开销、零影响」。
//!
//! 验收两条，都是能证伪的：
//! 1. 几何逐体素一致：同一段切削，开/关染色后 `get()` 完全一样；
//! 2. 关掉后 `creator` 数组**一个字节都不分配**，读出来全是 0（= 单色）。
//!
//! 如果哪天有人把 `grow_creator` 的短路去掉、或者把 `set_colorize` 变成
//! "只影响渲染"，第 2 条会立刻失败。

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::{ball_end_mill, flat_end_mill};
use cncsim_voxel::{GridConfig, SparseGrid, ValueType};

const H: f64 = 0.25;
const S: f64 = 12.0;

fn n(mm: f64) -> i32 {
    (mm / H).round() as i32
}

fn stock(colorize: bool) -> SparseGrid<i8> {
    let cfg = GridConfig {
        h: H,
        band_voxels: 4,
        value_type: ValueType::I8,
    };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.set_colorize(colorize);
    g.set_creator_sub(8); // 每个体素一个 creator：最能暴露"有没有分配"
    g.fill_box_exact([0, 0, 0], [n(S), n(S), n(S)], cfg.band_mm());
    g
}

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose {
        tip: DVec3::new(x, y, z),
        axis: DVec3::Z,
    }
}

/// 一条通槽 + 一个球头坑，两把刀、两段，覆盖"盖章"和"整叶填空气"两条路径。
fn cut(mut g: SparseGrid<i8>) -> SparseGrid<i8> {
    let flat = flat_end_mill(6.0, 20.0);
    g.set_creator(3);
    g.cut_segment(&pose(-2.0, 6.0, 4.0), &pose(14.0, 6.0, 4.0), &flat);
    let ball = ball_end_mill(4.0, 12.0, 24);
    g.set_creator(17);
    g.cut_segment(&pose(6.0, 2.0, 8.0), &pose(6.0, 10.0, 8.0), &ball);
    g
}

#[test]
fn geometry_is_identical_with_color_off() {
    let on = cut(stock(true));
    let off = cut(stock(false));
    assert_eq!(on.colorize(), true);
    assert_eq!(off.colorize(), false);

    let total = (n(S) * n(S) * n(S)) as usize;
    let mut solid = 0usize;
    for z in 0..n(S) {
        for y in 0..n(S) {
            for x in 0..n(S) {
                let p = [x, y, z];
                let (a, b) = (on.get(p), off.get(p));
                assert_eq!(a, b, "体素 {p:?} 的值不同：开 {a} / 关 {b}");
                if a < 0 {
                    solid += 1;
                }
            }
        }
    }
    assert!(solid > 1000, "切削几乎没生效：只剩 {solid} 个实体体素");
    // 顺带确认"确实切掉了东西"
    assert!(solid < total, "一点料都没去掉");
}

#[test]
fn color_off_allocates_nothing() {
    let on = cut(stock(true));
    let off = cut(stock(false));

    assert!(
        on.creator_bytes() > 0,
        "开着染色却不分配 creator，测试就失去意义了"
    );
    assert_eq!(
        off.creator_bytes(),
        0,
        "关掉染色还分配了 {} 字节",
        off.creator_bytes()
    );

    // 读出来必须全是 0（= 单一材质色），而不是垃圾值
    let mut nonzero = 0usize;
    for z in 0..n(S) {
        for y in 0..n(S) {
            for x in 0..n(S) {
                if off.creator_of_voxel([x, y, z]) != 0 {
                    nonzero += 1;
                }
            }
        }
    }
    assert_eq!(nonzero, 0, "关掉染色后仍有 {nonzero} 个体素带颜色标记");
}

/// 开/关染色的**切削耗时**必须在同一个量级：关掉时 `changed_mask` 直接
/// 走 `count_changed`，不做子块位图。这里用 200 刀做一次粗粒度的对照。
#[test]
fn color_off_does_not_add_work() {
    let cut_n = |colorize: bool| {
        let mut g = stock(colorize);
        let flat = flat_end_mill(4.0, 20.0);
        let t0 = std::time::Instant::now();
        for k in 0..200 {
            let y = 1.0 + (k % 20) as f64 * 0.5;
            let z = 1.0 + (k / 20) as f64 * 0.5;
            g.set_creator((k % 20 + 1) as u8);
            g.cut_segment(&pose(-2.0, y, z), &pose(14.0, y, z), &flat);
        }
        (t0.elapsed().as_secs_f64(), g.creator_bytes())
    };
    // 并行跑测试时单次计时抖得厉害，取 5 次最小值再比——这比调大阈值可靠
    let (mut on, mut off) = (f64::INFINITY, f64::INFINITY);
    let mut bytes_on = 0usize;
    for _ in 0..5 {
        let (t, b) = cut_n(true);
        on = on.min(t);
        bytes_on = b;
        off = off.min(cut_n(false).0);
    }
    assert!(bytes_on > 0, "开着染色却不分配 creator");
    assert_eq!(cut_n(false).1, 0, "关掉染色仍分配 creator");
    assert!(
        off <= on * 1.10,
        "关掉染色反而更慢：开 {:.3}s / 关 {:.3}s",
        on,
        off
    );
    println!(
        "200 刀（5 次取最小）：开 {:.1} ms（{bytes_on} B）/ 关 {:.1} ms",
        on * 1e3,
        off * 1e3
    );
}
