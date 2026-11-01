//! 50×50×100 mm 毛坯被 φ6 立铣刀**分层铣光**——粗加工全切除的性能基准。
//!
//! 参数（与 `tools/ovdb_bench` 的同名场景逐项对齐）：
//! · 毛坯 50×50×100 mm，`h = 0.2`（250×250×500 = 3125 万体素）
//! · φ6 平底刀，**刃长 6 mm**（`cutter` 只含刃长，刀杆不参与切削几何）
//! · 每层切深 5 mm（< 刃长），步距 5 mm（< 直径，留 1 mm 重叠）
//! · 20 层 × 每层 10 刀 = **200 刀**，把整块料切光
//!
//! `h=0.2` 而不是默认的 0.1：50×50×100 @ 0.1 是 2.5 亿体素，建场就要几十秒，
//! 收益全被初始化吃掉。粗加工本来也不需要 0.1 的精度。
//!
//! 两个入口都测：
//! · `cut_segment` 逐刀切 —— 仿真主循环的用法（每刀之后要查碰撞）；
//! · `cut_batch` 攒批切 —— 只看最终形状的用法，验证"少遍历几遍"值多少。
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test millout -- --ignored --nocapture
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;
use cncsim_voxel::{GridConfig, SparseGrid, ValueType};

const H: f64 = 0.2;
const SX: f64 = 50.0;
const SY: f64 = 50.0;
const SZ: f64 = 100.0;

/// 每层切深
const DOC: f64 = 5.0;
/// 每层刀数（步距 = (SY − 2r)/(passes−1)）
const PASSES: usize = 10;

fn n(mm: f64) -> i32 {
    (mm / H).round() as i32
}

/// 分层铣光的刀路：20 层 × 10 刀，往复走刀。
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

fn fresh_stock() -> SparseGrid<i8> {
    let cfg = GridConfig { h: H, band_voxels: 4, value_type: ValueType::I8 };
    let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
    g.fill_box_exact([0, 0, 0], [n(SX), n(SY), n(SZ)], cfg.band_mm());
    g
}

#[test]
#[ignore = "大场景基准，手动跑"]
fn mill_out_50x50x100() {
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = program();
    let feed = 300.0; // mm/min，只用于换算程序时间
    let prog_mm: f64 = cuts.iter().map(|(a, b)| (b.tip - a.tip).length()).sum();

    // ── ① 逐刀切（仿真主循环的用法）──
    let t_build = std::time::Instant::now();
    let mut g = fresh_stock();
    let build_ms = t_build.elapsed().as_secs_f64() * 1e3;

    let (mut visited, mut changed, mut box_vox) = (0usize, 0usize, 0usize);
    let t_cut = std::time::Instant::now();
    for (from, to) in &cuts {
        let st = g.cut_segment(from, to, &prof);
        visited += st.visited;
        changed += st.changed;
        box_vox += (st.box_size[0] * st.box_size[1] * st.box_size[2]).max(0) as usize;
    }
    let cut_ms = t_cut.elapsed().as_secs_f64() * 1e3;

    // ── ② 攒批切（只看最终形状的用法）──
    let mut gb = fresh_stock();
    let t_batch_all = std::time::Instant::now();
    let st_all = gb.cut_batch(&cuts, &prof);
    let batch_all_ms = t_batch_all.elapsed().as_secs_f64() * 1e3;

    // 按层攒批（每批 10 刀）——真实刀路不会一次性攒满 200 刀
    let mut gl = fresh_stock();
    let t_batch_layer = std::time::Instant::now();
    let mut layer_changed = 0usize;
    for chunk in cuts.chunks(PASSES) {
        layer_changed += gl.cut_batch(chunk, &prof).changed;
    }
    let batch_layer_ms = t_batch_layer.elapsed().as_secs_f64() * 1e3;

    // 抽查：切光之后整个体积内不应再有材料
    let mut solid = 0usize;
    let mut checked = 0usize;
    let mut mismatch = 0usize;
    for k in (2..n(SZ) - 2).step_by(17) {
        for j in (2..n(SY) - 2).step_by(13) {
            for i in (2..n(SX) - 2).step_by(13) {
                checked += 1;
                let v = g.get([i, j, k]);
                if g.to_distance(v) < -0.05 {
                    solid += 1;
                }
                // 批量与逐刀必须逐位相同（抽查，全量比一次要 3000 万次）
                if v != gb.get([i, j, k]) || v != gl.get([i, j, k]) {
                    mismatch += 1;
                }
            }
        }
    }

    println!("\n【铣光 50×50×100】h={H} mm，φ6 刃长 6，切深 5，步距 5");
    println!("  毛坯 {}×{}×{} 体素，建场 {build_ms:.0} ms", n(SX), n(SY), n(SZ));
    println!(
        "  逐刀切：{} 刀，{cut_ms:.0} ms = 每刀 {:.2} ms",
        cuts.len(),
        cut_ms / cuts.len() as f64
    );
    println!(
        "  整批切：200 刀一次 = {batch_all_ms:.0} ms（{:.2}× 于逐刀，改动 {} M）",
        cut_ms / batch_all_ms,
        st_all.changed as f64 / 1e6
    );
    println!(
        "  分层攒批：20 批 × 10 刀 = {batch_layer_ms:.0} ms（{:.2}× 于逐刀，改动 {} M）",
        cut_ms / batch_layer_ms,
        layer_changed as f64 / 1e6
    );
    println!(
        "  程序时间 {:.1} s @F{feed:.0}（{:.0} m 刀路）→ 仿真实时加速 {:.0}×",
        prog_mm / (feed / 60.0),
        prog_mm / 1000.0,
        prog_mm / (feed / 60.0) / (cut_ms / 1e3)
    );
    println!("  抽查 {checked} 点，残留材料 {solid} 个（应为 0），批量不一致 {mismatch} 个（应为 0）");
    println!(
        "  AABB 共 {:.1} M 体素，实际求值 {:.1} M，改动 {:.1} M（改动/求值 {:.0}%）",
        box_vox as f64 / 1e6,
        visited as f64 / 1e6,
        changed as f64 / 1e6,
        changed as f64 / visited.max(1) as f64 * 100.0
    );

    assert_eq!(solid, 0, "切光后不应残留材料");
    assert_eq!(mismatch, 0, "批量与逐刀结果必须逐位相同");
    assert_eq!(cuts.len(), 200, "应为 20 层 × 10 刀");
}

/// **剖析专用**：把"逐刀切"重复若干轮，给 VTune 一个足够长的采集窗口。
///
/// 单跑一次 `mill_out_50x50x100` 只有 1 秒，user-mode 采样（默认 ~10 ms 一次）
/// 只能拿到几十个样本——热点表里全是线程池等待，看不出真正在算什么。
/// 这里重复 20 轮（约 1.2 秒切削 × 20），样本量够到能读行号。
///
/// ```bash
/// cargo test --release -p cncsim-voxel --test millout --no-run
/// vtune -collect hotspots -result-dir=C:/dev/vt-mill -- \
///   target/release/deps/millout-<hash>.exe --ignored --nocapture perf_cut_loop
/// ```
#[test]
#[ignore = "剖析专用（重复 20 轮），只给 VTune 用"]
fn perf_cut_loop() {
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = program();
    let rounds = 20;
    let t0 = std::time::Instant::now();
    let mut sink = 0u64;
    for r in 0..rounds {
        let mut g = fresh_stock();
        for (from, to) in &cuts {
            let st = g.cut_segment(from, to, &prof);
            sink += st.changed as u64;
        }
        sink += g.leaf_count() as u64 + r as u64;
    }
    let dt = t0.elapsed().as_secs_f64();
    println!(
        "剖析基准：{rounds} 轮 × {} 刀，合计 {:.0} ms（每轮 {:.1} ms）；sink={sink}",
        cuts.len(),
        dt * 1e3,
        dt * 1e3 / rounds as f64
    );
    // `CNCSIM_PROF=1` 时给零开销的 eval / write 拆分（原子累加，跑完一次读）
    if let Some((e, w)) = cncsim_voxel::prof_take() {
        let total = dt * 1e3;
        println!(
            "  拆分：eval {e:.1} ms（{:.0}%）  write {w:.1} ms（{:.0}%）  其余 {:.1} ms",
            e / total * 100.0,
            w / total * 100.0,
            total - e - w
        );
        let b = cncsim_voxel::prof_take_box();
        let bn = cncsim_voxel::PROF_BOX_N.swap(0, std::sync::atomic::Ordering::Relaxed);
        println!(
            "  建场内部：eval {:.1}  write {:.1}  prune {:.1}（写入 {bn} 体素）",
            b[0], b[1], b[2]
        );
        if let Some(w5) = cncsim_voxel::prof_take_write() {
            let names = ["whole", "collect", "sort", "blit", "prune"];
            let parts: Vec<String> = names
                .iter()
                .zip(w5)
                .map(|(n, v)| format!("{n} {v:.1}"))
                .collect();
            println!("  write 内部：{}", parts.join("  "));
            use std::sync::atomic::Ordering::Relaxed;
            let nl = cncsim_voxel::PROF_NEW_LEAF.swap(0, Relaxed);
            let nb = cncsim_voxel::PROF_N_BLOCK.swap(0, Relaxed);
            println!("  块统计：{nb} 块 / {nl} 新建叶（新建率 {:.0}%）", nl as f64 / nb.max(1) as f64 * 100.0);
        }
    }
}

/// **分片数扫描**：并行粒度是不是最优？
///
/// 铣光一刀的 AABB 是 284×34×29 体素，`cut_sweeps` 按"叶层最多的轴"（x，36 个叶层）
/// 切成 `min(2×线程数, 36) = 36` 片。每片只有 7.8K 体素、约 2.8 µs——和 rayon
/// worker 的唤醒延迟（15–50 µs）同量级。片数是不是太多？这里直接扫一遍。
#[test]
#[ignore = "分片数扫描，手动跑"]
fn slice_count_sweep() {
    let prof = flat_end_mill(6.0, 6.0);
    let cuts = program();
    // 预热：让线程池起来，别把首次唤醒算进去
    {
        let mut g = fresh_stock();
        for (from, to) in cuts.iter().take(20) {
            g.cut_segment(from, to, &prof);
        }
    }
    println!("\n【分片数扫描】50×50×100 铣光 200 刀逐刀（0 = 自动）");
    for slices in [0usize, 1, 2, 4, 8, 12, 18, 24, 36] {
        let mut best = f64::MAX;
        for _ in 0..3 {
            let mut g = fresh_stock();
            let t = std::time::Instant::now();
            for (from, to) in &cuts {
                g.cut_segment_with(from, to, &prof, slices);
            }
            best = best.min(t.elapsed().as_secs_f64() * 1e3);
        }
        println!("  slices={slices:2}  {best:5.1} ms");
    }
}
