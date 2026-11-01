//! 带颜色切割的**缓存**与**多线程**分析。
//!
//! 分两部分，因为它们要的驱动方式不同：
//!
//! - [`working_set_sweep`]：一次进程内跑完。扫毛坯尺寸，看 `ns/求值体素` 随工作集
//!   增长的变化——曲线在某个尺寸上抬头就是超出某一级缓存。
//! - [`thread_scaling`]：一次进程只测一个线程数（线程池是进程级单例，
//!   `CNCSIM_THREADS` 只在建池时读一次），所以由外部脚本换着跑。
//!
//! ```bash
//! # 工作集扫描
//! cargo test --release -p cncsim-voxel --test creator_analysis -- --ignored --nocapture
//! # 线程扩展性（每行一组）
//! for t in 1 2 4 8 16 24; do
//!   CNCSIM_THREADS=$t cargo test --release -p cncsim-voxel --test creator_analysis \
//!     thread_scaling -- --ignored --nocapture 2>/dev/null | grep THREADS
//! done
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{CutStats, GridConfig, SparseGrid, ValueType};

const H: f64 = 0.1;

fn n(mm: f64) -> i32 {
    (mm / H).round() as i32
}

/// `mm × mm × mm/2` 的实心毛坯
fn stock(mm: f64, sub: u8) -> SparseGrid<i8> {
    let cfg = GridConfig { h: H, band_voxels: 4, value_type: ValueType::I8 };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.set_creator_sub(sub);
    g.fill_box_exact([0, 0, 0], [n(mm), n(mm), n(mm / 2.0)], cfg.band_mm());
    g
}

/// 50×50×100 mm 实心毛坯（与 `millout` 基准同尺寸）
fn tall_stock(sub: u8) -> SparseGrid<i8> {
    let cfg = GridConfig { h: 0.2, band_voxels: 4, value_type: ValueType::I8 };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.set_creator_sub(sub);
    g.fill_box_exact([0, 0, 0], [250, 250, 500], cfg.band_mm());
    g
}

/// 铣光 200 刀（20 层 × 10 刀），与 `millout` 基准同一个程序
fn millout_program() -> Vec<(Pose, Pose)> {
    let r = 3.0;
    let (sx, sy, sz, doc): (f64, f64, f64, f64) = (50.0, 50.0, 100.0, 5.0);
    let passes = 10;
    let mut cuts = Vec::with_capacity(200);
    let mut layer_top = sz;
    for _ in 0..(sz / doc).round() as usize {
        let tip_z = (layer_top - doc).max(0.0);
        for i in 0..passes {
            let y = r + (sy - 2.0 * r) * i as f64 / (passes - 1) as f64;
            let (x0, x1) = if i % 2 == 0 { (-r, sx + r) } else { (sx + r, -r) };
            cuts.push((
                Pose { tip: DVec3::new(x0, y, tip_z), axis: DVec3::Z },
                Pose { tip: DVec3::new(x1, y, tip_z), axis: DVec3::Z },
            ));
        }
        layer_top -= doc;
    }
    cuts
}

/// 一条贯穿的直槽，切在毛坯半高——`visited` 随毛坯footprint 线性增长
fn sweep(mm: f64) -> Vec<(Pose, Pose)> {
    let r = 3.0;
    let y = mm / 2.0;
    let z = mm / 4.0;
    vec![(
        Pose { tip: DVec3::new(-r, y, z), axis: DVec3::Z },
        Pose { tip: DVec3::new(mm + r, y, z), axis: DVec3::Z },
    )]
}

fn best_of(rounds: usize, mut f: impl FnMut() -> (f64, CutStats)) -> (f64, CutStats) {
    let mut best = (f64::INFINITY, CutStats::default());
    for _ in 0..rounds {
        let r = f();
        if r.0 < best.0 {
            best = r;
        }
    }
    best
}

/// 工作集扫描：`ns / 求值体素` 随毛坯尺寸的变化。
///
/// 三个配置同窗交替（每轮按 off → s1 → s4 跑一遍），取最好成绩。
/// 若 `s4` 相对 `off` 的**增量**随尺寸抬头，说明 creator 数组的访问开始掉出缓存。
#[test]
#[ignore = "分析用，手动跑"]
fn working_set_sweep() {
    let prof = flat_end_mill(6.0, 6.0);
    println!();
    println!(
        "{:>7}  {:>12}  {:>9}  {:>9}  {:>9}  {:>10}  {:>10}",
        "毛坯mm", "creator MB", "off ns/体素", "s1", "s4", "s4-off", "s4/off"
    );
    for mm in [8.0, 12.0, 20.0, 30.0, 50.0, 80.0, 120.0] {
        let cuts = sweep(mm);
        let mut t = [f64::INFINITY; 3];
        let mut visited = 0usize;
        let mut cbytes = 0usize;
        for _ in 0..3 {
            for (i, (sub, stamp)) in [(4u8, false), (1, true), (4, true)].iter().enumerate() {
                let mut g = stock(mm, *sub);
                if *stamp {
                    g.set_creator(1);
                }
                let (ms, st) = best_of(1, || {
                    let t0 = std::time::Instant::now();
                    let s = g.cut_batch(&cuts, &prof);
                    (t0.elapsed().as_secs_f64() * 1e3, s)
                });
                t[i] = t[i].min(ms);
                visited = st.visited.max(1);
                cbytes = g.creator_bytes();
            }
        }
        let per = |ms: f64| ms * 1e6 / visited as f64;
        println!(
            "{mm:>7.0}  {:>12.2}  {:>9.2}  {:>9.2}  {:>9.2}  {:>10.2}  {:>10.2}",
            cbytes as f64 / 1e6,
            per(t[0]),
            per(t[1]),
            per(t[2]),
            per(t[2]) - per(t[0]),
            per(t[2]) / per(t[0])
        );
    }
}

/// 线程扩展性：一次进程只测一个线程数，输出一行给外部脚本收集。
#[test]
#[ignore = "分析用，手动跑（由外部脚本换 CNCSIM_THREADS）"]
fn thread_scaling() {
    let threads = cncsim_voxel::par::worker_count();
    let prof = flat_end_mill(6.0, 6.0);
    // **用铣光 200 刀的大批量**，不是单刀：单刀只有 1–3 ms，一次 fork-join 的
    // 固定开销就占掉大半，测出来的是线程池启动成本不是扩展性。
    let cuts = millout_program();
    let mut out = String::new();
    for (name, sub, stamp) in [("off", 4u8, false), ("s1", 1, true), ("s4", 4, true)] {
        let mut best = f64::INFINITY;
        for _ in 0..5 {
            let mut g = tall_stock(sub);
            if stamp {
                g.set_creator(1);
            }
            let t0 = std::time::Instant::now();
            g.cut_batch(&cuts, &prof);
            best = best.min(t0.elapsed().as_secs_f64() * 1e3);
        }
        out.push_str(&format!(" {name}={best:.2}"));
    }
    println!("THREADS={threads}{out}");
}

/// 给硬件计数器（VTune）用的长跑：同一批 200 刀重复 N 次，跑够采样时间。
///
/// `SUB=4 STAMP=1` 控制配置；`ROUNDS` 控制轮数。
#[test]
#[ignore = "分析用（VTune 采集）"]
fn profile_run() {
    let sub: u8 = std::env::var("SUB").ok().and_then(|s| s.parse().ok()).unwrap_or(4);
    let stamp = std::env::var("STAMP").map(|s| s == "1").unwrap_or(true);
    let rounds: usize = std::env::var("ROUNDS").ok().and_then(|s| s.parse().ok()).unwrap_or(25);
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = millout_program();
    let mut sink = 0usize;
    for _ in 0..rounds {
        let mut g = tall_stock(sub);
        if stamp {
            g.set_creator(1);
        }
        let st = g.cut_batch(&cuts, &prof);
        sink += st.changed;
    }
    println!("profile_run: sub={sub} stamp={stamp} rounds={rounds} sink={sink}");
}

/// 相位拆分：`cut` 只有两段——**并行求值**（`cut_box`，含位图计算）和
/// **串行落盘**（`write_parts`，含盖章与 prune）。位图/盖章的开销落在哪一段，
/// 决定了它能不能被多线程摊薄。
///
/// 需要 `CNCSIM_PROF=1`（计数器是进程级、`swap` 取值）。
#[test]
#[ignore = "分析用，需 CNCSIM_PROF=1"]
fn phase_breakdown() {
    if std::env::var_os("CNCSIM_PROF").is_none() {
        println!("跳过：需要 CNCSIM_PROF=1");
        return;
    }
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = millout_program();
    println!();
    println!("{:>4}  {:>9}  {:>9}  {:>8}  {:>8}  {:>7}", "组", "求值ms", "落盘ms", "求值%", "落盘%", "合计");
    for (name, sub, stamp) in [("off", 4u8, false), ("s1", 1, true), ("s4", 4, true)] {
        let mut best: Option<(f64, f64)> = None;
        for _ in 0..5 {
            let mut g = tall_stock(sub);
            if stamp {
                g.set_creator(1);
            }
            let _ = cncsim_voxel::prof_take();
            let _ = cncsim_voxel::prof_take_write();
            g.cut_batch(&cuts, &prof);
            if let Some(w5) = cncsim_voxel::prof_take_write() {
                println!("    {name} 落盘细分 [whole, collect, sort, blit, prune] = {w5:?}");
            }
            if let Some((e, w)) = cncsim_voxel::prof_take() {
                let tot = e + w;
                if best.is_none_or(|(be, bw)| tot < be + bw) {
                    best = Some((e, w));
                }
            }
        }
        if let Some((e, w)) = best {
            println!(
                "{name:>4}  {e:>9.1}  {w:>9.1}  {:>7.0}%  {:>7.0}%  {:>7.1}",
                e / (e + w) * 100.0,
                w / (e + w) * 100.0,
                e + w
            );
        }
    }
}

/// 提取阶段：**每顶点读一次 creator**，粒度越细数组越大。
///
/// 这是整个特性里唯一"按顶点"的访存点——切削侧是按叶（64 B 一条 line）。
/// 如果细粒度会拖慢提取，就说明 creator 的读开始掉出缓存。
#[test]
#[ignore = "分析用，手动跑"]
fn extract_vs_granularity() {
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = millout_program();
    println!();
    println!(
        "{:>4}  {:>11}  {:>11}  {:>9}  {:>11}",
        "sub", "creator MB", "提取 ms", "顶点数", "ns/顶点"
    );
    for sub in [1u8, 2, 4, 8] {
        let mut g = tall_stock(sub);
        g.set_creator(1);
        // **只切一层**（前 10 刀）：`millout` 那 200 刀是把整块料切光，
        // 切完一个面都不剩，提取出来 0 顶点（第一版就是这么量到 inf 的）。
        g.cut_batch(&cuts[..10], &prof);
        let mut best = f64::INFINITY;
        let mut verts = 0usize;
        for _ in 0..3 {
            let t0 = std::time::Instant::now();
            let m = g.extract_surface();
            best = best.min(t0.elapsed().as_secs_f64() * 1e3);
            verts = m.positions.len();
        }
        println!(
            "{sub:>4}  {:>11.2}  {best:>11.1}  {verts:>9}  {:>11.2}",
            g.creator_bytes() as f64 / 1e6,
            best * 1e6 / verts as f64
        );
    }
}

/// 固定配置的长跑（给提权的 VTune 采集用）。
///
/// 为什么按配置拆成三个测试函数、而不是读环境变量：提权进程**不继承外部环境**，
/// 而用 `Start-Process` 拉起解释器又被安全策略拦了。测试名是唯一能穿过
/// `vtune -- <exe> <filter> --ignored` 这条命令行的配置通道。
fn profile_fixed(sub: u8, stamp: bool) {
    // 100 轮 ≈ 3–4 s：VTune 的 `memory-access` 采样间隔 5 ms，太短的运行
    // 只有几十个样本，它会自己报 "metrics data may be unreliable"。
    let rounds: usize = 100;
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = millout_program();
    let mut sink = 0usize;
    for _ in 0..rounds {
        let mut g = tall_stock(sub);
        if stamp {
            g.set_creator(1);
        }
        sink += g.cut_batch(&cuts, &prof).changed;
    }
    println!("profile: sub={sub} stamp={stamp} rounds={rounds} sink={sink}");
}

#[test]
#[ignore = "VTune 采集用（需提权）"]
fn profile_s4() {
    profile_fixed(4, true);
}

#[test]
#[ignore = "VTune 采集用（需提权）"]
fn profile_s1() {
    profile_fixed(1, true);
}

#[test]
#[ignore = "VTune 采集用（需提权）"]
fn profile_off() {
    profile_fixed(4, false);
}
