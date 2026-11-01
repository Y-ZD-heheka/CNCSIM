//! 溯源开销 A/B：同一时间窗里**交替**跑，量新增的位图/盖章/取色各值多少。
//!
//! 为什么必须交替：这台机器连续跑几十轮会降频，铣光逐刀 47 → 56–65 ms
//! （见 `docs/项目记忆.md` §7）。**单次测量的绝对值不可信**，只有同一窗口里
//! 交替跑的相对值可信。
//!
//! 四组配置：
//!
//! | 组 | `creator_sub` | 盖章 | 对应 |
//! |---|---|---|---|
//! | `off` | 4 | 不盖（`pending_creator = 0`） | 旧代码在"不着色"时的路径 |
//! | `s1` | 1 | 盖 | 旧代码的粒度 |
//! | `s4` | 4 | 盖 | **现在的默认** |
//! | `s8` | 8 | 盖 | 最细 |
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test creator_perf -- --ignored --nocapture
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, SparseGrid, ValueType};

const H: f64 = 0.2;
const SX: f64 = 50.0;
const SY: f64 = 50.0;
const SZ: f64 = 100.0;
const DOC: f64 = 5.0;
const PASSES: usize = 10;
const ROUNDS: usize = 5;

fn n(mm: f64) -> i32 {
    (mm / H).round() as i32
}

fn program() -> Vec<(Pose, Pose)> {
    let r = 3.0;
    let mut cuts = Vec::with_capacity(200);
    let n_layers = (SZ / DOC).round() as usize;
    let mut layer_top = SZ;
    for _ in 0..n_layers {
        let tip_z = (layer_top - DOC).max(0.0);
        for i in 0..PASSES {
            let y = r + (SY - 2.0 * r) * i as f64 / (PASSES - 1) as f64;
            let (x0, x1) = if i % 2 == 0 { (-r, SX + r) } else { (SX + r, -r) };
            cuts.push((
                Pose { tip: DVec3::new(x0, y, tip_z), axis: DVec3::Z },
                Pose { tip: DVec3::new(x1, y, tip_z), axis: DVec3::Z },
            ));
        }
        layer_top -= DOC;
    }
    cuts
}

fn fresh(sub: u8) -> SparseGrid<i8> {
    let cfg = GridConfig { h: H, band_voxels: 4, value_type: ValueType::I8 };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.set_creator_sub(sub);
    g.fill_box_exact([0, 0, 0], [n(SX), n(SY), n(SZ)], cfg.band_mm());
    g
}

/// 跑一遍 200 刀，返回 (切削 ms, creator 字节)
fn run_cuts(sub: u8, stamp: bool) -> (f64, usize) {
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = program();
    let mut g = fresh(sub);
    if stamp {
        g.set_creator(1);
    }
    let t = std::time::Instant::now();
    for (from, to) in &cuts {
        g.cut_segment(from, to, &prof);
    }
    let ms = t.elapsed().as_secs_f64() * 1e3;
    (ms, g.creator_bytes())
}

#[test]
#[ignore = "性能基准，手动跑"]
fn creator_overhead() {
    let groups: [(&str, u8, bool); 4] =
        [("off", 4, false), ("s1", 1, true), ("s4", 4, true), ("s8", 8, true)];
    let mut best = vec![f64::INFINITY; groups.len()];
    let mut bytes = vec![0usize; groups.len()];

    // 预热一轮（第一次跑要吃 page fault / 分支预测冷启动）
    for (_, sub, stamp) in groups {
        let _ = run_cuts(sub, stamp);
    }
    // 交替跑：第 r 轮按顺序跑四组，每组只保留最好成绩（噪声只会让它变慢）
    for _ in 0..ROUNDS {
        for (i, (_, sub, stamp)) in groups.iter().enumerate() {
            let (ms, b) = run_cuts(*sub, *stamp);
            best[i] = best[i].min(ms);
            bytes[i] = b;
        }
    }

    let base = best[0];
    println!();
    println!("50×50×100 铣光 200 刀，5 轮取最好（同一时间窗交替）");
    println!("{:>5}  {:>10}  {:>9}  {:>9}  {:>12}", "组", "切削 ms", "每刀 ms", "相对", "creator B");
    for (i, (name, _, _)) in groups.iter().enumerate() {
        println!(
            "{name:>5}  {:>10.1}  {:>9.3}  {:>8.2}×  {:>12}",
            best[i],
            best[i] / 200.0,
            best[i] / base,
            bytes[i]
        );
    }
}
