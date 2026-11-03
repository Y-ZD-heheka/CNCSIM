//! # cncsim-app
//!
//! 装配层：窗口、CLI、文件 I/O。**不含业务逻辑**。
//!
//! 目前做两件事：加载机床配置并跑一遍运动学自检，证明
//! 「配置文件 → 拓扑 → 正解/逆解」这条链是通的。
//!
//! 用法：
//! ```text
//! cargo run                              # 默认 assets/machines/ac_trunnion.machine.ron
//! cargo run -- --machine path/to/x.ron
//! ```

use cncsim_config::MachineConfig;

/// 默认配置路径：相对 `crates/app/` 往上两级。
///
/// 用 `CARGO_MANIFEST_DIR` 而不是 cwd，这样从任何目录 `cargo run` 都能找到。
/// 装成正式程序后这里要换成「可执行文件旁边 / 用户配置目录」的查找顺序。
const DEFAULT_MACHINE: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/machines/ac_trunnion.machine.ron");

/// 窗口里分块重建的块边长（mm）。
///
/// **这个值直接决定增量更新有没有收益**：`TileCache::update` 会把脏区
/// 再外扩 ±2 块（要带上四边形连到邻块的顶点），所以实际重建范围是
/// 「这一刀 + 4 个块边长」。30 mm 毛坯实测：
///
/// | 块边长 | 块数 | 逐刀更新 | 对照：全量重提 |
/// |---|---|---|---|
/// | 8 mm | 60 | 14.3 ms | 10 ms |
/// | 4 mm | 220 | 9.3 ms | 10 ms |
/// | **2 mm** | 806 | **3.9 ms** | 10 ms |
///
/// 取 4 mm 是折中：块再小更新更快，但实体数/绘制调用线性涨。
#[cfg(feature = "window")]
const TILE_MM: f64 = 4.0;
/// 窗口网格的 LOD 因子（h_c = k·h）。
#[cfg(feature = "window")]
const LOD_K: u32 = 4;

fn main() {
    println!("CNCSIM — 五轴联动 CNC 切割仿真");
    println!();

    let path = machine_path_from_args();
    let cfg = match cncsim_config::load(&path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("加载机床配置失败\n  路径：{}\n  原因：{e}", path.display());
            std::process::exit(1);
        }
    };

    // `--window`：开 Bevy 窗口（需 `--features window` 编译）
    // `--screenshot` 隐含开窗：它是"开窗 → 截图 → 退出"的无人值守模式
    if std::env::args()
        .skip(1)
        .any(|a| a == "--window" || a == "--screenshot" || a.starts_with("--screenshot="))
    {
        run_window_demo();
        return;
    }

    // `--export <path>`：跑一个演示程序，把切完的毛坯写成自包含 HTML 预览
    if let Some(out) = export_path_from_args() {
        export_preview(&out);
        return;
    }

    print_layers();
    print_machine(&cfg);
    kinematics_self_check(&cfg);
    gcode_self_check();
    interp_self_check();
    voxel_self_check();
    sim_self_check();
}

/// 解释器的三件新能力：A/B/C 轴、倾斜工作平面、隐式模态运动。
fn interp_self_check() {
    let ic = cncsim_gcode::InterpConfig::default();
    println!();
    println!("  解释器：旋转轴 / 倾斜平面 / 隐式运动");

    // 同一个「A30.」在两种机床构型下的刀轴 —— Y 分量必须反号
    let table = cncsim_gcode::interpret("G0 X0 A30.", &ic);
    let head = cncsim_gcode::interpret(
        "G0 X0 A30.",
        &cncsim_gcode::InterpConfig {
            rotary: cncsim_gcode::RotaryConvention::Head,
            ..ic.clone()
        },
    );
    let fmt = |v: cncsim_math::DVec3| format!("({:>6.3}, {:>6.3}, {:>6.3})", v.x, v.y, v.z);
    println!(
        "    A30 双转台 → 刀轴 {}    双摆头 → {}",
        fmt(table.blocks[0].target.axis),
        fmt(head.blocks[0].target.axis)
    );

    // G68.2 绕 X 转 90°：程序里的 +Z 落到工件坐标的 −Y
    let twp = cncsim_gcode::interpret("G68.2 I0 J90 K0
G0 Z10", &ic);
    println!(
        "    G68.2 I0 J90 K0 之后 G0 Z10 → {}（程序 Z 变成工件 −Y）",
        fmt(twp.blocks[0].target.tip)
    );

    // 隐式模态运动：真实后处理绝大多数行不带 G 字
    let imp = cncsim_gcode::interpret("G1 X10 F100
X20
Y5", &ic);
    println!(
        "    「G1 X10 / X20 / Y5」→ {} 块（后两行是隐式 G1）",
        imp.blocks.len()
    );

    // 固定循环：一个 G83 + 两行坐标 = 三个孔 × 四次啄钻
    let cyc = cncsim_gcode::interpret("G0 Z10
G83 Z-8 R2 Q3 F100
X20
X40", &ic);
    let pecks = cyc
        .blocks
        .iter()
        .filter(|b| matches!(b.motion, cncsim_motion::MotionKind::Linear))
        .count();
    println!(
        "    「G83 Q3 + X20 / X40」→ {} 块，{} 次下扎（3 个孔 × 4 刀啄钻）",
        cyc.blocks.len(),
        pecks
    );

    // 对刀：把当前位置读作 Z0，之后 Z0 就落在原地
    let probe = cncsim_gcode::interpret("G0 Z50
G10 L20 P1 Z0
G0 Z0", &ic);
    println!(
        "    「G0 Z50 / G10 L20 P1 Z0 / G0 Z0」→ 目标 z={:.1}（对刀后原地不动）",
        probe.blocks.last().expect("应有块").target.tip.z
    );

    // 刀径补偿：10×10 的方框、刀半径 3、G41 内侧 → 刀心缩到 [3,7]
    let cc = cncsim_gcode::InterpConfig {
        tool_radii: vec![0.0, 3.0],
        ..ic.clone()
    };
    let comp = cncsim_gcode::interpret(
        "G0 X0 Y0
G1 G41 D1 X10 Y0 F100
X10 Y10
X0 Y10
X0 Y0
G40",
        &cc,
    );
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    // 跳过第一块（G0 到起点，还没开始补偿）
    for b in &comp.blocks[1..] {
        lo = lo.min(b.target.tip.x);
        hi = hi.max(b.target.tip.x);
    }
    println!(
        "    G41 D3 走 10×10 方框 → 刀心 x ∈ [{lo:.1}, {hi:.1}]（内缩一个刀半径）"
    );
}

/// 端到端：G 代码 → 位姿 → 切削 + 碰撞 → 报告。
///
/// 这一段是"整条链能跑"的证明：前面几个自检各管一段，这里把它们串起来。
fn sim_self_check() {
    use cncsim_math::DVec2;
    use cncsim_tool::{CutterKind, Profile, ToolDef, flat_end_mill};

    // 一把 φ6 平底刀，刃长 20、全长 60 → 刀杆从 z=20 起
    let tool = ToolDef {
        number: 1,
        kind: CutterKind::Flat,
        diameter: 6.0,
        corner_radius: 0.0,
        flute_length: 20.0,
        shank: Profile::new(vec![
            DVec2::new(0.0, 20.0),
            DVec2::new(3.0, 20.0),
            DVec2::new(3.0, 60.0),
            DVec2::new(0.0, 60.0),
        ]),
        length_offset: 60.0,
        cutter: flat_end_mill(6.0, 20.0),
    };

    // 20×20×10 mm 毛坯。程序里三条槽：两条正常，第三条扎穿到刀杆
    let src = "G0 X4 Y4 Z9.5
G1 X16 Y4 F600
G0 X4 Y10
G1 X16 Y10 F600
G0 X4 Y16 Z-25
G1 X16 Y16 F600
M30
";
    let prog = cncsim_gcode::interpret(src, &cncsim_gcode::InterpConfig::default());
    let cfg = cncsim_sim::run::RunConfig {
        stock_size: [200, 200, 100],
        ..Default::default()
    };
    let out = cncsim_sim::run::run(&prog, &tool, &cfg);

    println!();
    println!("  端到端仿真（20×20×10 mm 毛坯，三条槽，第三条刀杆扎穿）");
    println!(
        "    {} 块 → {} 段切削 + {} 段快移",
        out.stats.blocks, out.stats.cut_segments, out.stats.rapid_segments
    );
    println!(
        "    去除材料 {:.1} mm³（{:.1} → {:.1}），程序时间 {:.2} s",
        out.report.removed_volume_mm3,
        out.stats.volume_before,
        out.stats.volume_after,
        out.report.programmed_time_s
    );
    println!(
        "    切削 {:.0} ms，碰撞检查 {:.0} ms，墙钟 {:.2} s",
        out.stats.cut_ms, out.stats.collide_ms, out.report.actual_time_s
    );
    if out.collisions.is_empty() {
        println!("    未检出碰撞（三条槽都正常）");
    } else {
        for c in &out.collisions {
            println!(
                "    碰撞：{} × {} 最深 {:.2} mm，{:.3}–{:.3} s，首次在第 {} 块（第 {} 行）",
                c.episode.a,
                c.episode.b,
                c.episode.max_penetration,
                c.episode.t_start,
                c.episode.t_end,
                c.first_block,
                c.first_line
            );
        }
    }
    for a in &out.report.alarms {
        println!("    [{}] {}", a.id, a.message);
    }

    // ── 回放时间轴 + 后台任务 ──
    // 两条契约在这里现场验证：回放总长与报告节拍逐位一致；后台跑完与同步 run 一致。
    let play = cncsim_sim::playback::Playback::from_program(&prog, &cfg);
    let same_time = play.total_time() == out.report.programmed_time_s;
    let job = cncsim_sim::job::SimJob::spawn(prog.clone(), tool.clone(), cfg.clone());
    let mut progress_events = 0usize;
    let mut async_out = None;
    while let Some(e) = job.recv_timeout(std::time::Duration::from_secs(30)) {
        match e {
            cncsim_sim::job::JobEvent::Progress(_) => progress_events += 1,
            cncsim_sim::job::JobEvent::Done(o) => {
                async_out = Some(o);
                break;
            }
            cncsim_sim::job::JobEvent::Failed(m) => {
                println!("    后台任务失败：{m}");
                break;
            }
            cncsim_sim::job::JobEvent::Started { .. } => {}
        }
    }
    let same_result = async_out
        .as_ref()
        .map(|o| {
            o.report.programmed_time_s == out.report.programmed_time_s
                && o.report.removed_volume_mm3 == out.report.removed_volume_mm3
                && o.stats.cut_segments == out.stats.cut_segments
        })
        .unwrap_or(false);
    println!();
    println!("  回放时间轴 + 后台任务");
    println!(
        "    {} 个块区间，总长 {:.2} s，与报告节拍逐位一致={}",
        play.spans().len(),
        play.total_time(),
        same_time
    );
    println!(
        "    后台跑完：{progress_events} 条进度事件，结果与同步一致={same_result}"
    );
}

/// 稀疏树自检：证明 tile 把实心区域吃掉了，以及 prune 会把切空的叶折回来。
fn voxel_self_check() {
    println!();
    println!("稀疏体素树自检");

    let cfg = cncsim_voxel::GridConfig::default();
    let mut g = cncsim_voxel::SparseGrid::<i8>::new(cfg, cncsim_math::DVec3::ZERO);

    // 200×200×100 mm 毛坯 @ h=0.1 → 2000×2000×1000 体素 = 4e9
    let t0 = std::time::Instant::now();
    g.fill_box([0, 0, 0], [2000, 2000, 1000], -127);
    let ms = t0.elapsed().as_secs_f64() * 1e3;

    let dense_bytes = 2000.0 * 2000.0 * 1000.0;
    println!("  毛坯 200×200×100 mm @ h=0.1 → {:.0} 亿体素", dense_bytes / 1e8);
    println!("  分层填充耗时 {ms:.1} ms");
    println!(
        "  root 条目 {}   叶 {}   值占用 {:.1} MB   （稠密需 {:.1} GB）",
        g.root_len(),
        g.leaf_count(),
        g.value_bytes() as f64 / 1e6,
        dense_bytes / 1e9
    );

    // 切空一个叶，验证 prune 折叠
    for i in 0..8 {
        for j in 0..8 {
            for k in 0..8 {
                g.set([i, j, k], 127);
            }
        }
    }
    let before = matches!(g.leaf_at([0, 0, 0]), Some(cncsim_voxel::Leaf::Dense(_)));
    g.prune();
    let after = matches!(g.leaf_at([0, 0, 0]), Some(cncsim_voxel::Leaf::Constant(_)));
    println!("  切空一个叶：prune 前密集={before}，prune 后常量={after}");

    // 量化与插值
    let inside = g.sdf(cncsim_math::DVec3::new(100.0, 100.0, 50.0));
    let outside = g.sdf(cncsim_math::DVec3::new(300.0, 100.0, 50.0));
    println!("  sdf 内部 {inside:+.3} mm（负=材料内，钳制到 −band）");
    println!("  sdf 外部 {outside:+.3} mm（钳制到 +band）");
    println!(
        "  量化每档 {:.2} µm（i8）/ {:.1} nm（i16）",
        cfg.value_type.step_mm(&cfg) * 1e3,
        cncsim_voxel::ValueType::I16.step_mm(&cfg) * 1e6
    );

    // ── 切一刀，实测耗时（这是对「0.06 ms/段」估算的核对） ──
    let prof = cncsim_tool::flat_end_mill(6.0, 20.0);
    let from = cncsim_motion::Pose {
        tip: cncsim_math::DVec3::new(0.0, 100.0, 99.0),
        axis: cncsim_math::DVec3::Z,
    };
    let to = cncsim_motion::Pose {
        tip: cncsim_math::DVec3::new(200.0, 100.0, 99.0),
        axis: cncsim_math::DVec3::Z,
    };
    let t0 = std::time::Instant::now();
    let st = g.cut_segment(&from, &to, &prof);
    let cut_ms = t0.elapsed().as_secs_f64() * 1e3;
    println!();
    println!("  切削一段 200mm 直槽（φ6 平底刀，刀尖 z=99；{} 线程）", cncsim_voxel::par::worker_count());
    println!(
        "    AABB {}×{}×{} 体素，遍历 {}，改动 {}，耗时 {cut_ms:.1} ms",
        st.box_size[0], st.box_size[1], st.box_size[2], st.visited, st.changed
    );
    if st.visited > 0 {
        println!(
            "    每体素 {:.2} µs（含 {}-{} 次 sd_profile 求值）",
            cut_ms * 1e3 / st.visited as f64,
            cncsim_voxel::MIN_GOLDEN_ITERS,
            cncsim_voxel::golden_iters(200.0, 0.0)
        );
    }
    let d = g.sdf(cncsim_math::DVec3::new(100.0, 100.0, 99.0));
    println!("    槽底 z=99 处 sdf {d:+.4} mm（应≈0）");

    // ── 真实切削的段长是 0.05mm 量级，不是 200mm ──
    let short = cncsim_motion::Pose {
        tip: cncsim_math::DVec3::new(0.0, 50.0, 99.0),
        axis: cncsim_math::DVec3::Z,
    };
    let short2 = cncsim_motion::Pose {
        tip: cncsim_math::DVec3::new(0.05, 50.0, 99.0),
        axis: cncsim_math::DVec3::Z,
    };
    let t0 = std::time::Instant::now();
    let st2 = g.cut_segment(&short, &short2, &prof);
    let short_ms = t0.elapsed().as_secs_f64() * 1e3;
    println!();
    println!("  切削一段 0.05mm（真实段长）");
    println!(
        "    AABB {}×{}×{} 体素，遍历 {}，耗时 {short_ms:.2} ms",
        st2.box_size[0], st2.box_size[1], st2.box_size[2], st2.visited
    );
    println!(
        "    冷启动外推 10 万段：{:.1} 分钟（连续切削时每段约 0.2 ms → 约 0.3 分钟，\
         短段大头是固定成本：FlatSweep 特化 + blind 标脏 + 缓冲预留之后已压到最低）",
        short_ms * 1e5 / 6e4
    );

    // ── 提取表面网格 ──
    //
    // 注意：这里用 12×12×8 mm 的小块，不是上面那个 200mm 毛坯。
    // 200mm 毛坯 @ h=0.1 的表面有 3200 万个三角形、834 MB、耗时 43 秒——
    // 那是离屏基准，不该塞进自检；自检看的是"粗细两档的比例对不对"。
    let mut small = cncsim_voxel::SparseGrid::<i8>::new(cfg, cncsim_math::DVec3::ZERO);
    small.fill_box([0, 0, 0], [120, 120, 80], -127);
    small.cut_segment(
        &cncsim_motion::Pose {
            tip: cncsim_math::DVec3::new(0.0, 6.0, 7.0),
            axis: cncsim_math::DVec3::Z,
        },
        &cncsim_motion::Pose {
            tip: cncsim_math::DVec3::new(12.0, 6.0, 7.0),
            axis: cncsim_math::DVec3::Z,
        },
        &prof,
    );
    let t0 = std::time::Instant::now();
    let mesh = small.extract_surface();
    let ex_ms = t0.elapsed().as_secs_f64() * 1e3;
    // 同一份数据强制单块跑一遍：**并行只改怎么算，不改算出什么**
    let t0 = std::time::Instant::now();
    let serial = small.extract_surface_chunks(1);
    let ser_ms = t0.elapsed().as_secs_f64() * 1e3;
    let same = mesh.positions == serial.positions && mesh.indices == serial.indices;
    println!();
    println!("  提取等值面（Surface Nets，12×12×8 mm 块 + 一条槽）");
    println!(
        "    细格 h=0.1：顶点 {}  三角形 {}  丢洞 {}",
        mesh.positions.len(),
        mesh.indices.len() / 3,
        mesh.skipped_quads
    );
    println!(
        "    并行 {:.0} ms（{} 线程） / 串行 {:.0} ms（单块） → {:.1}×   逐位一致={same}",
        ex_ms,
        cncsim_voxel::par::worker_count(),
        ser_ms,
        ser_ms / ex_ms.max(0.001)
    );

    // ── 显示 LOD：粗格重采样（不是网格抽稀）──
    println!();
    println!();
    println!(
        "  显示 LOD（粗格重采样；k 上限 = 窄带宽度 {} 体素；{} 线程）",
        small.max_lod_factor(),
        cncsim_voxel::par::worker_count()
    );
    let tri_fine = mesh.indices.len() / 3;
    for k in [1u32, 2, 4] {
        let t0 = std::time::Instant::now();
        let m = small.extract_surface_lod(k);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        let tri = m.indices.len() / 3;
        println!(
            "    k={k}  h_c={:.1} mm  顶点 {:>6}  三角形 {tri:>7}  ({:.0}%)  耗时 {ms:>5.1} ms  丢洞 {}",
            cfg.h * k as f64,
            m.positions.len(),
            tri as f64 / tri_fine as f64 * 1e2,
            m.skipped_quads
        );
    }
    println!("    k=1 与上面那行逐位一致（测试锁）；k 超过 band 会 panic，不静默产出错网格");

    // ── 按预算选 k：200mm 毛坯 3200 万面 → 200 万面 ──
    let budget = 2_000_000;
    let k = cncsim_voxel::choose_lod_factor(32_000_000, budget, small.max_lod_factor());
    println!(
        "  预算 {} 万面：200mm 毛坯（3200 万面）→ k = {k}，h_c = {:.1} mm，预计 {:.0} 万面",
        budget / 10_000,
        cfg.h * k as f64,
        32_000_000.0 / (k * k) as f64 / 10_000.0
    );
}

/// 一段刻意写得"难"的小程序：同一行混多个模态字、组 0 挂起、刀补不支持。
///
/// 目的是让人一眼看出解释器的行为，而不只是测试通过。
const DEMO: &str = "\
G21 G90 G54 G17 G94
G0 X0 Y0 Z50
G43 H1 G1 Z-5 F200
G1 X50 Y0 G91
G1 X10
G90 G1 X100
G41 D1 G1 X120
G2 X140 Y20 R20
G28 X0 Y0
M30
";

fn gcode_self_check() {
    println!();
    println!("G 代码解释自检");

    let mut c = cncsim_gcode::InterpConfig::default();
    c.tool_lengths[1] = 120.0; // H1 = 120 mm

    let p = cncsim_gcode::interpret(DEMO, &c);

    println!(
        "  {:>4}  {:<8} {:>9} {:>9} {:>9}  {:>6} {:>6}",
        "行", "运动", "X", "Y", "Z", "进给", "刀长"
    );
    for b in &p.blocks {
        let kind = match b.motion {
            cncsim_motion::MotionKind::Rapid => "G0",
            cncsim_motion::MotionKind::Linear => "G1",
            cncsim_motion::MotionKind::Arc { clockwise: true, .. } => "G2",
            cncsim_motion::MotionKind::Arc { .. } => "G3",
            cncsim_motion::MotionKind::Dwell { .. } => "G4",
        };
        println!(
            "  {:>4}  {:<8} {:>9.3} {:>9.3} {:>9.3}  {:>6.1} {:>6.1}",
            b.line_no, kind, b.target.tip.x, b.target.tip.y, b.target.tip.z, b.feed.value, b.tool_length
        );
    }

    println!();
    println!("  问题（{} 条）", p.issues.len());
    for i in &p.issues {
        println!(
            "    {:>4}  {:<9} {:?}  {}",
            i.line, i.id, i.severity, i.message
        );
    }
    println!("  可信度：{:?}", p.trust);
}

/// `--export <path>`：导出 HTML 预览而不是跑自检。
fn export_path_from_args() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--export" {
            return args.next().map(Into::into);
        }
        if let Some(p) = a.strip_prefix("--export=") {
            return Some(p.into());
        }
    }
    None
}

/// 演示场景（`--export` 与 `--window` 共用）。
///
/// 默认是**面铣一层**（`--demo facing`）：Z 向均匀切掉 1 mm，十刀直来直往，
/// 每刀一条带子、一条一个颜色——这是"切完之后逐条看颜色"的样板。
/// `--demo pocket` 换回旧程序（面铣 + G41 方槽 + 4 个 G83 孔），
/// 用来跑刀径补偿与固定循环的端到端路径。
struct DemoScene {
    prog: cncsim_gcode::MotionProgram,
    /// 刀库。面铣/型腔只有一把；半球凸台是 T1 粗 + T2 精。
    tools: Vec<cncsim_tool::ToolDef>,
    cfg: cncsim_sim::run::RunConfig,
    /// 预览/窗口的 LOD 因子。**大毛坯要更大**：50×50×100 的表面是 30 mm
    /// 毛坯的 8 倍，k=3 的 HTML 会到 18 MB，k=4 砍到 8 MB。
    lod_k: u32,
    title: &'static str,
}

fn demo_program() -> DemoScene {
    // `bowl` 是旧名（半球坑那一版），留着当别名不吃亏
    if matches!(demo_kind().as_str(), "dome" | "bowl") {
        return dome_scene();
    }
    let tool = cncsim_sim::demo::tool();
    let src = match demo_kind().as_str() {
        "pocket" => cncsim_sim::demo::pocket_gcode(),
        _ => cncsim_sim::demo::facing_gcode(),
    };

    let ic = cncsim_gcode::InterpConfig {
        tool_radii: vec![0.0, 3.0],
        ..Default::default()
    };
    let prog = cncsim_gcode::interpret(&src, &ic);
    // 设计成品：`--design path.stl` 用网格设计（STL → 采样 SDF），
    // 缺省用"顶面铣到 z=11 的盒"。面铣恰好切到设计面（不该报过切）；
    // 方槽（z=8）和四个孔（z=3）都切进了设计体——报告会把它们点出来。
    let design = match design_path_from_args() {
        Some(path) => {
            let bytes = match std::fs::read(&path) {
                Ok(b) => b,
                Err(e) => {
                    eprintln!("读设计模型失败\n  路径：{}\n  原因：{e}", path.display());
                    std::process::exit(1);
                }
            };
            let tris = match cncsim_voxel::stl::parse(&bytes) {
                Ok(t) => t,
                Err(e) => {
                    eprintln!("解析 STL 失败：{e}");
                    std::process::exit(1);
                }
            };
            println!(
                "设计模型：{}（{} 条三角形 → 采样 SDF 场）",
                path.display(),
                tris.len()
            );
            // **设计场的域 = 设计模型 ∪ 毛坯**（各外扩 pad）。
            //
            // 原来硬编码 `[300, 300, 120]`（正好等于毛坯域），78×79×87 mm 的模型会被
            // 裁掉大半。而 `voxel_compare` 只在**毛坯域**内查 `design.sdf(p)`——设计域
            // 覆盖不到的地方返回背景值（空气），会把"设计要料"误判成"设计气区"，
            // 残留体积虚高。
            //
            // 反过来说：**设计模型必须与毛坯在同一世界坐标系里**，两者不重叠时比对
            // 没有意义（会报满残留）。这里不拦，只在下面把域尺寸打出来。
            const STOCK: [i32; 3] = [300, 300, 120];
            let h = 0.1;
            let pad = 4.0 * h;
            let (mut dlo, mut dhi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for t in &tris {
                for v in t {
                    dlo[0] = dlo[0].min(v.x);
                    dlo[1] = dlo[1].min(v.y);
                    dlo[2] = dlo[2].min(v.z);
                    dhi[0] = dhi[0].max(v.x);
                    dhi[1] = dhi[1].max(v.y);
                    dhi[2] = dhi[2].max(v.z);
                }
            }
            let mut lo = [0.0f64; 3];
            let mut hi = [0.0f64; 3];
            let mut domain = [0i32; 3];
            for i in 0..3 {
                lo[i] = dlo[i].min(0.0) - pad;
                hi[i] = dhi[i].max(STOCK[i] as f64 * h) + pad;
                domain[i] = ((hi[i] - lo[i]) / h).ceil() as i32;
            }
            println!(
                "  模型包围盒 {:.1}×{:.1}×{:.1} mm → 设计场 {}×{}×{} 体素（{:.1}×{:.1}×{:.1} mm）",
                dhi[0] - dlo[0],
                dhi[1] - dlo[1],
                dhi[2] - dlo[2],
                domain[0],
                domain[1],
                domain[2],
                hi[0] - lo[0],
                hi[1] - lo[1],
                hi[2] - lo[2]
            );

            // 设计模型用 **band = 2 体素**（切削用 4，那需要精确的带外距离）。
            //
            // 体素化的成本 ∝ `(三角形尺寸 + 2·band)³`，band 减半 AABB 体积降到 1/3：
            // 190 万三角形、h=0.1 实测 **41 → 15 s**。量化档 `2·band·h/254` 也跟着
            // 减半，体积误差反而从 −0.05% 降到 −0.01%。
            //
            // **代价**：`residual_max_thickness` 依赖设计气区距离，而那个距离在带外
            // 被钳到 ±band——band=2 时厚度上限从 0.45 降到 0.25。这个代价现在被
            // **显式标记**（`report.residual_thickness_capped`），报告里会写
            // 「已饱和：真实更厚」，不会被误当成真值。
            Some(cncsim_sim::compare::field_design(
                &tris,
                cncsim_voxel::GridConfig {
                    h,
                    band_voxels: 2,
                    ..Default::default()
                },
                cncsim_math::DVec3::new(lo[0], lo[1], lo[2]),
                domain,
                cncsim_sim::compare::DEFAULT_DESIGN_STEP_MM,
            ))
        }
        None => {
            let (lo, hi) = cncsim_sim::demo::design_box();
            Some(cncsim_sim::compare::DesignModel::Box {
                min: cncsim_math::DVec3::new(lo[0], lo[1], lo[2]),
                max: cncsim_math::DVec3::new(hi[0], hi[1], hi[2]),
            })
        }
    };
    let cfg = cncsim_sim::run::RunConfig {
        stock_size: [300, 300, 120],
        design,
        ..Default::default()
    };
    DemoScene {
        prog,
        tools: vec![tool],
        cfg,
        lod_k: 3,
        title: "CNCSIM 切削预览",
    }
}

/// `--demo dome`：50×50×100 毛坯铣出一个 R15 半球**凸台**，
/// T1 平底刀粗加工 → T2 锥度球头精加工。
///
/// 这是**换刀**这条链路的端到端样件：同一份程序、一条时间轴、一把刀切完换另一把。
/// 几何生成与不变量都在 `cncsim_sim::hemisphere`，这里只装配。
fn dome_scene() -> DemoScene {
    let src = cncsim_sim::hemisphere::gcode();
    let ic = cncsim_gcode::InterpConfig {
        // D 值按刀号取半径：T1 φ6、T2 R0.5
        tool_radii: vec![0.0, 3.0, cncsim_sim::hemisphere::FINISH_TIP_R],
        ..Default::default()
    };
    let prog = cncsim_gcode::interpret(&src, &ic);
    println!(
        "程序：{} 行 → {} 块（T1 粗加工 + T2 精加工）",
        src.lines().count(),
        prog.blocks.len()
    );
    let cfg = cncsim_sim::run::RunConfig {
        stock_size: [
            (cncsim_sim::hemisphere::STOCK_MM[0] / 0.1) as i32,
            (cncsim_sim::hemisphere::STOCK_MM[1] / 0.1) as i32,
            (cncsim_sim::hemisphere::STOCK_MM[2] / 0.1) as i32,
        ],
        design: Some(cncsim_sim::hemisphere::design()),
        // 设计面采样：球面 + 顶面环带，0.4 mm 步距 ≈ 24k 点 × 每段 AABB 预筛
        design_step: 0.4,
        ..Default::default()
    };
    DemoScene {
        prog,
        tools: vec![
            cncsim_sim::hemisphere::rough_tool(),
            cncsim_sim::hemisphere::finish_tool(),
        ],
        cfg,
        lod_k: 4,
        title: "CNCSIM 半球凸台 R15（粗加工 + 锥度球头精加工）",
    }
}

/// 刀库 → `ToolTable`（渲染/仿真共用的那份顺序）。
fn tool_table(tools: &[cncsim_tool::ToolDef]) -> cncsim_tool::ToolTable {
    cncsim_tool::ToolTable::new(tools.to_vec())
}

/// 打印切面颜色分布：**每种索引占多大一片、在哪里**。
///
/// 这是"切完之后颜色对不对"的定量回答——光看截图只能定性说"有渐变"，
/// 说不清"每条带子宽度是不是等于步距""平面度是不是量化档那么大"。
///
/// 0 号是原始毛坯面（没被切到的侧壁），1.. 是第 n 刀。
fn print_color_bands(mesh: &cncsim_voxel::MeshData) {
    let bands = cncsim_voxel::color_bands(mesh);
    if bands.is_empty() {
        return;
    }
    println!();
    println!("切面颜色分布（索引 0 = 原始毛坯面，1.. = 第 n 刀）");
    println!(
        "  {:>3}  {:>7}  {:>7}  {:>9}  {:>9}  {:>16}  {:>16}  {:>5}",
        "idx", "顶点", "三角形", "面积", "其中朝上", "Z 范围", "Y 范围", "混色"
    );
    // **一刀一色的场景不可能逐行打**：半球精加工是 2900 刀，按 22 一刀的
    // 调色板周期循环，逐行打会刷屏。超过 16 行就只打首尾各 4 行。
    const MAX_ROWS: usize = 16;
    let many = bands.len() > MAX_ROWS;
    let mut cut_area = 0.0f64;
    let mut cut_up = 0.0f64;
    let mut cut_widths: Vec<f64> = Vec::new();
    let mut cut_z: Vec<f64> = Vec::new();
    for (bi, b) in bands.iter().enumerate() {
        if many && bi >= 4 && bi + 4 < bands.len() {
            if bi == 4 {
                println!("  …  中间 {} 个色带省略（逐刀渐变，看首尾两截即可）", bands.len() - 8);
            }
            continue;
        }
        println!(
            "  {:>3}  {:>7}  {:>7}  {:>9.1}  {:>9.1}  {:>7.2}..{:<7.2}  {:>7.2}..{:<7.2}  {:>5}",
            b.index,
            b.vertices,
            b.triangles,
            b.area_mm2,
            b.area_up_mm2,
            b.lo[2],
            b.hi[2],
            b.lo[1],
            b.hi[1],
            b.mixed_triangles
        );
        if b.index != 0 {
            cut_area += b.area_mm2;
            cut_up += b.area_up_mm2;
            cut_widths.push((b.hi[1] - b.lo[1]) as f64);
            cut_z.push((b.hi[2] - b.lo[2]) as f64);
        }
    }
    let n_cut = bands.iter().filter(|b| b.index != 0).count();
    println!(
        "  ── 切割色 {n_cut} 种，合计面积 {cut_area:.1} mm²（其中朝上 {cut_up:.1} mm²）"
    );
    if !cut_widths.is_empty() {
        let wmin = cut_widths.iter().cloned().fold(f64::INFINITY, f64::min);
        let wmax = cut_widths.iter().cloned().fold(0.0f64, f64::max);
        let zmax = cut_z.iter().cloned().fold(0.0f64, f64::max);
        let covered: f64 = cut_widths.iter().sum();
        println!(
            "  ── 每条 Y 跨度 {wmin:.2}..{wmax:.2} mm（合计 {covered:.2} mm），单条 Z 跨度 ≤ {zmax:.2} mm"
        );
        println!(
            "  ── 解读：Y 跨度 ≈ 步距 → 一条带子一色；朝上面积 ≈ 顶面 → 十条盖满；"
        );
        println!(
            "            Z 跨度比量化档大是侧壁上那一圈叶级溢出（最多一个叶宽 8h）"
        );
    }
}

/// 跑一段演示程序，把切完的毛坯提取成网格并导出 HTML。
///
/// 用的是**和自检完全一样的那条链**：interpret → sim::run → extract_surface_lod
/// → render::preview。所以预览里看到的每一块材料，都是仿真真的切出来的。
fn export_preview(out: &std::path::Path) {
    let scene = demo_program();
    let tools = tool_table(&scene.tools);
    let t0 = std::time::Instant::now();
    let run = cncsim_sim::run::run_tools(&scene.prog, &tools, &scene.cfg);
    let sim_ms = t0.elapsed().as_secs_f64() * 1e3;

    // LOD k=3（h_c=0.3 mm）：面数压到十万以内，肉眼几乎看不出差别。
    // k=2 是 19 万三角形、6 MB 的 HTML，浏览器打开会明显卡一下；
    // k 的上限是窄带宽度（4 体素），所以 3 还在约束内。
    let t0 = std::time::Instant::now();
    let mesh = run.grid.extract_surface_lod(scene.lod_k);
    let ex_ms = t0.elapsed().as_secs_f64() * 1e3;

    print_color_bands(&mesh);
    let tw = std::time::Instant::now();
    let stats = match cncsim_render::preview::write_html(out, &mesh, scene.title) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("写预览失败：{e}");
            std::process::exit(1);
        }
    };
    println!("  写 HTML {:.1} ms（{:.1} MB）", tw.elapsed().as_secs_f64() * 1e3, stats.bytes as f64 / 1e6);
    println!(
        "仿真空跑完成（{} 块，{} 切削段 / {} 快移段，去料 {:.1} mm³）",
        run.stats.blocks, run.stats.cut_segments, run.stats.rapid_segments, run.report.removed_volume_mm3
    );
    if run.report.gouge_max_depth_mm > 0.0 {
        let region = run
            .report
            .gouge_region
            .map(|(lo, hi)| {
                format!(
                    "，区域 x {:.1}..{:.1} y {:.1}..{:.1} z {:.1}..{:.1}",
                    lo.x, hi.x, lo.y, hi.y, lo.z, hi.z
                )
            })
            .unwrap_or_default();
        println!(
            "  过切 {:.1} mm³，设计面最大侵入 {:.3} mm @ ({:.2}, {:.2}, {:.2}){}",
            run.report.gouge_volume_mm3,
            run.report.gouge_max_depth_mm,
            run.report.gouge_at.x,
            run.report.gouge_at.y,
            run.report.gouge_at.z,
            region
        );
    }
    if run.report.residual_volume_mm3 > 0.0 {
        // 残留厚度**只有 `DesignModel::Field` 会饱和**（体素距离被钳到 `band`），
        // 解析盒是精确距离、没有上限。这个标记由 `voxel_compare` 直接算，
        // 不在打印时猜（早期版本用"厚度是否恰好等于上限"猜，是个脆弱的启发式）。
        let note = match (run.report.residual_thickness_capped, run.report.residual_thickness_cap) {
            (true, Some(cap)) => format!(
                "（**已饱和**：设计场 band 的上限 {:.3} = band + h/2，真实更厚）",
                cap + run.grid.config.h * 0.5
            ),
            (true, None) => "（**已饱和**：真实更厚）".to_string(),
            _ => String::new(),
        };
        println!(
            "  残留 {:.2} mm³，最厚 {:.3} mm{note} @ ({:.1}, {:.1}, {:.1})",
            run.report.residual_volume_mm3,
            run.report.residual_max_thickness,
            run.report.residual_at.x,
            run.report.residual_at.y,
            run.report.residual_at.z
        );
    }
    println!(
        "  仿真 {sim_ms:.0} ms（比对 {:.0} ms，切削 {:.0} ms = 每刀 {:.2} ms），提取 {ex_ms:.0} ms（LOD k={}）→ {} 顶点 / {} 三角形",
        run.stats.compare_ms,
        run.stats.cut_ms,
        run.stats.cut_ms / run.stats.cut_segments.max(1) as f64,
        scene.lod_k,
        stats.vertices,
        stats.triangles
    );
    println!(
        "  溯源数组 {} MB（粒度 sub={}，每叶 {} B）",
        run.grid.creator_bytes() / (1024 * 1024),
        scene.cfg.creator_sub,
        run.grid.creator_bytes() / run.grid.leaf_count().max(1)
    );
    if run.stats.tool_changes > 0 {
        // 每把刀的时段——截图/回放要靠它取时刻（"粗加工结束"就是 T2 第一段的时刻）
        for n in scene.tools.iter().map(|t| t.number) {
            let segs: Vec<_> = run.cut_segs.iter().filter(|s| s.tool == n).collect();
            if let (Some(a), Some(b)) = (segs.first(), segs.last()) {
                println!(
                    "    T{n}：{} 段切削，{:.1}–{:.1} s",
                    segs.len(),
                    a.t_start,
                    b.t_start
                );
            }
        }
        println!(
            "  换刀 {} 次：{}",
            run.stats.tool_changes,
            scene
                .tools
                .iter()
                .map(|t| format!("T{} {:?} φ{:.2}", t.number, t.kind, t.diameter))
                .collect::<Vec<_>>()
                .join(" → ")
        );
    }
    println!("  已写出 {}", out.display());
    println!("  用浏览器打开即可旋转查看（单文件、无外链）");
}

/// `--screenshot <path>` / `--screenshot=<path>`：截一张图写盘后退出。
#[cfg(feature = "window")]
///
/// 用途：**没有显示器的环境下验证渲染层**。否则 `--window` 只能证明"跑起来不崩"，
/// 看不出画面对不对——而"画面对不对"正是渲染层唯一要验证的东西。
fn screenshot_path() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--screenshot" {
            return args.next().map(Into::into);
        }
        if let Some(p) = a.strip_prefix("--screenshot=") {
            return Some(p.into());
        }
    }
    None
}

fn machine_path_from_args() -> std::path::PathBuf {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--machine" {
            if let Some(p) = args.next() {
                return p.into();
            }
        } else if let Some(p) = a.strip_prefix("--machine=") {
            return p.into();
        }
    }
    DEFAULT_MACHINE.into()
}

/// `--window`：算一遍演示场景，把网格/报告/时间轴交给 Bevy 外壳。
#[cfg(feature = "window")]
fn run_window_demo() {
    let scene = demo_program();
    let tools = tool_table(&scene.tools);
    let t0 = std::time::Instant::now();
    let out = cncsim_sim::run::run_tools(&scene.prog, &tools, &scene.cfg);
    let mesh = out.grid.extract_surface_lod(scene.lod_k);
    print_color_bands(&mesh);
    let playback = cncsim_sim::playback::Playback::from_program(&scene.prog, &scene.cfg);
    // 窗口用**活毛坯**：从空料开始，按回放时间一刀刀切，分块增量重建。
    // 切削列表直接取仿真的 `cut_segs`——圆弧已经按同一份 `arc_sagitta` 离散过，
    // 自己再离散一次就会和刀痕/溯源色对不上。
    // 活毛坯也带刀库：粗加工用 φ6 扫、精加工用 R0.5 扫。用错母线不是"不好看"，
    // 是把料切坏——所以 `cut_segs` 里每一段都带着自己的刀号。
    let live = cncsim_render::shell::LiveCut::new(
        &scene.cfg,
        scene.tools.iter().map(|t| (t.number, t.cutter.clone())).collect(),
        out.cut_segs.iter().map(|s| (s.t_start, s.from, s.to, s.tool)).collect(),
        TILE_MM,
        LOD_K,
    );
    // 报告先灌进 HMI，面板的横幅/报警列才有东西可显示——顺序反了就是空面板。
    let mut hmi = cncsim_hmi::Hmi::default();
    hmi.ingest(&out.report);
    println!(
        "仿真 {:.0} ms → {} 顶点 / {} 三角形，{} 刀；时间轴 {:.2} s；状态「{}」；打开窗口…",
        t0.elapsed().as_secs_f64() * 1e3,
        mesh.positions.len(),
        mesh.indices.len() / 3,
        live.cut_count(),
        playback.total_time(),
        cncsim_ui::UiView { hmi: &hmi }.status_banner()
    );
    cncsim_render::shell::run_window(cncsim_render::shell::ShellScene {
        live,
        report: out.report,
        playback,
        hmi,
        title: "CNCSIM".into(),
        screenshot: screenshot_path(),
        // 刀库：**与仿真、碰撞用的是同一份** `ToolDef`。渲染单独造一把
        // "看起来像"的刀，就会出现"画面上没撞、报告说撞了"。
        tools: scene.tools.clone(),
    });
}

#[cfg(not(feature = "window"))]
fn run_window_demo() {
    eprintln!("窗口外壳未编译：用 `cargo run --release --features window -- --window` 重跑");
    std::process::exit(1);
}

/// `--demo <facing|pocket>`：选演示程序，缺省 `facing`（面铣一层）。
///
/// 面铣是"逐条看颜色"的样板：十刀直来直往，一刀一条带子。
/// `pocket` 是旧程序，跑 G41 刀径补偿与 G83 固定循环。
fn demo_kind() -> String {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--demo" {
            return args.next().unwrap_or_else(|| "facing".into());
        }
        if let Some(v) = a.strip_prefix("--demo=") {
            return v.into();
        }
    }
    "facing".into()
}

/// `--design <path.stl>`：设计模型（STL）。缺省 = 内置盒。
fn design_path_from_args() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--design" {
            if let Some(p) = args.next() {
                return Some(p.into());
            }
        } else if let Some(p) = a.strip_prefix("--design=") {
            return Some(p.into());
        }
    }
    None
}

fn print_layers() {
    println!("  层  crate            状态");
    println!("  ──────────────────────────────────────────");
    println!("  L0  cncsim-math      f64 线性代数（零依赖）");
    println!("  L0  cncsim-motion    Pose / MotionSegment");
    println!("  L0  cncsim-tool      母线 profile / sd_profile");
    println!("  L0  cncsim-config    .machine.ron → MachineTopology");
    println!("  L1  cncsim-kin       运动学（FK / IK / RTCP / 诊断）");
    println!("  L1  cncsim-gcode     解释器（骨架）");
    println!("  L2  cncsim-voxel     稀疏 SDF（骨架）");
    println!("  L2  cncsim-collide   GJK / 保守推进（骨架）");
    println!("  L3  cncsim-sim       编排（骨架）");
    println!("  L3  cncsim-hmi       控制状态机");
    println!("  L4  cncsim-render    Bevy 插件（待接 Bevy）");
    println!("  L4  cncsim-ui        bevy_egui（待接 Bevy）");
    println!();
}

fn print_machine(cfg: &MachineConfig) {
    let m = &cfg.topology;
    println!("机床：{}（{} 个关节）", m.name, m.joint_count());
    for j in &m.chain {
        let kind = match j.kind {
            cncsim_kin::JointKind::Linear(a) => format!("直线 {a:?}"),
            cncsim_kin::JointKind::Rotary(a) => format!("旋转 {a:?}"),
        };
        // offset 只在非零时打出来——摇篮式的 L 就藏在 A 的 offset 里
        let off = if j.offset == cncsim_math::DVec3::ZERO {
            String::new()
        } else {
            format!("  offset=({:.1}, {:.1}, {:.1})", j.offset.x, j.offset.y, j.offset.z)
        };
        println!(
            "  {:<2} {:<10} sign={:+.0}  limit=({}, {}){off}",
            j.name, kind, j.sign, j.limit.0, j.limit.1
        );
    }
    println!();
}

fn kinematics_self_check(cfg: &MachineConfig) {
    let m = &cfg.topology;
    println!("运动学自检");

    // ── 锚点：与 testdata/golden 第 3 行一致（L=180, X100 Y50 Z-20 A30 C45）──
    let q = m.q(&[("A", 30.0), ("C", 45.0), ("X", 100.0), ("Y", 50.0), ("Z", -20.0)]);
    let p = cncsim_kin::forward(m, &q);
    println!(
        "  FK   (X100 Y50 Z-20 A30 C45) → 刀尖 ({:.3}, {:.3}, {:.3})  刀轴 ({:.3}, {:.3}, {:.3})",
        p.tip.x, p.tip.y, p.tip.z, p.axis.x, p.axis.y, p.axis.z
    );

    // ── 逆解往返：拿正解结果回代，必须回到同一组关节值 ──
    match cncsim_kin::inverse(m, &p, &q) {
        Some(q2) => {
            let err = q.iter().zip(&q2).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
            println!("  IK   往返最大偏差 {err:.2e}（应 < 1e-9）");
        }
        None => println!("  IK   无解（不应发生）"),
    }

    // ── 奇异：κ = 1/|sin A| ──
    print!("  κ    ");
    for a in [0.0, 1.0, 30.0, 90.0] {
        match cncsim_kin::rotary_condition(m, &m.q(&[("A", a)])) {
            Some(k) if k.is_infinite() => print!("A={a:>4.1}°→∞  "),
            Some(k) => print!("A={a:>4.1}°→{k:>7.2}  "),
            None => print!("—  "),
        }
    }
    println!("（κ 越大越接近奇异）");

    // ── RTCP：刀尖固定在台面中心，A 摆动，直线轴必须补偿 ──
    let target = cncsim_math::DVec3::new(0.0, 0.0, 0.0);
    print!("  RTCP 刀尖固定在台面中心，A 摆动 → 补偿量 ");
    for a in [0.0, 90.0] {
        let q = cncsim_kin::rtcp_inverse(m, target, &m.q(&[("A", a)])).unwrap();
        let (iy, iz) = (m.index_of("Y").unwrap(), m.index_of("Z").unwrap());
        print!("A={a:>4.0}°: Y={:>7.1} Z={:>7.1}   ", q[iy], q[iz]);
    }
    println!();

    // ── 诊断编号 ──
    let f = cncsim_kin::check(m, &m.q(&[("A", 0.0)]), 100.0);
    println!("  check(A=0°) → {:?}", f.iter().map(|x| x.id()).collect::<Vec<_>>());
}
