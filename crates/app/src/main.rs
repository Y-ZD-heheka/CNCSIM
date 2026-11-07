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

    // `--cam <model.stl>`：跑 CAM 生成 → 解释器 → 仿真，不依赖机床配置。
    if let Some(model) = cam_path_from_args() {
        run_cam(&model);
        return;
    }

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

/// `--cam <model.stl>`：CAM 生成 → 解释器 → 仿真，一条链跑完。
///
/// STL 是三角网格，直接进；STEP 是 B-rep，先三角化（错误信息里写清楚）。
/// 生成的 G 代码默认写到模型旁边的 `.nc`，可 `--gcode <path>` 覆盖。
fn run_cam(model: &std::path::Path) {
    use cncsim_math::DVec3;

    let ext = model
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    if matches!(ext.as_str(), "stp" | "step") {
        eprintln!("{}", cncsim_cam::CamError::StepNeedsTessellation);
        eprintln!("  文件：{}", model.display());
        std::process::exit(1);
    }
    let bytes = match std::fs::read(model) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("读模型失败\n  路径：{}\n  原因：{e}", model.display());
            std::process::exit(1);
        }
    };
    let tris = match cncsim_voxel::stl::parse(&bytes) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("解析 STL 失败：{e}\n  文件：{}", model.display());
            std::process::exit(1);
        }
    };
    let (mut lo, mut hi) = (
        DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for t in &tris {
        for v in t {
            lo = DVec3::new(lo.x.min(v.x), lo.y.min(v.y), lo.z.min(v.z));
            hi = DVec3::new(hi.x.max(v.x), hi.y.max(v.y), hi.z.max(v.z));
        }
    }
    println!(
        "模型：{} 个三角形，包围盒 ({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})",
        tris.len(),
        lo.x,
        lo.y,
        lo.z,
        hi.x,
        hi.y,
        hi.z
    );

    let mut stock = match cam_stock_from_args() {
        Some(size) => {
            let c = (lo + hi) * 0.5;
            cncsim_cam::Stock {
                min: DVec3::new(c.x - size.x * 0.5, c.y - size.y * 0.5, lo.z),
                max: DVec3::new(c.x + size.x * 0.5, c.y + size.y * 0.5, lo.z + size.z),
            }
        }
        None => cncsim_cam::Stock {
            min: lo - DVec3::new(1.0, 1.0, 0.0),
            max: hi + DVec3::new(1.0, 1.0, 1.0),
        },
    };
    println!(
        "毛坯：({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})",
        stock.min.x, stock.min.y, stock.min.z, stock.max.x, stock.max.y, stock.max.z
    );

    // ── 工件包络：装不下就**主动缩小**，而不是硬着头皮生成一份跑不了的刀路 ──
    //
    // 包络来自机床配置（`work: table_diameter / max_height`），默认 Ø400 × 高 340
    // （AC 双转台摇篮：台面 Ø400；Z 上限 520 − 台面 180）。
    // 缩放是**等比**的，中心取（毛坯 XY 中心，底面）——件还是坐在台面上、
    // 绕 C 轴扫的还是同一个圆。比例会打印出来，不偷偷改件。
    let mut tris = tris;
    {
        let env = match cncsim_config::load(machine_path_from_args()) {
            Ok(c) => c.work,
            Err(_) => cncsim_config::WorkEnvelope::default(),
        };
        let pivot = DVec3::new(
            0.5 * (stock.min.x + stock.max.x),
            0.5 * (stock.min.y + stock.max.y),
            stock.min.z,
        );
        let (r, h) = stock_envelope(&stock);
        match shrink_for_envelope(r, h, &env) {
            Some(k) if !args_flag("--no-shrink") => {
                for t in tris.iter_mut() {
                    for p in t.iter_mut() {
                        *p = pivot + (*p - pivot) * k;
                    }
                }
                let size = stock.size();
                stock = cncsim_cam::Stock {
                    min: pivot,
                    max: pivot + size * k,
                };
                println!(
                    "工件包络：件 Ø{:.0}×{:.0} 超出 台面 Ø{:.0} / 高 {:.0} → 等比缩小 **{:.3}×** → Ø{:.0}×{:.0}",
                    2.0 * r,
                    h,
                    env.table_diameter,
                    env.max_height,
                    k,
                    2.0 * r * k,
                    h * k
                );
            }
            Some(_) => println!(
                "工件包络：件 Ø{:.0}×{:.0} 超出 台面 Ø{:.0} / 高 {:.0}，`--no-shrink` 保持原尺寸（**这份程序在这台机床上装不下**）",
                2.0 * r, h, env.table_diameter, env.max_height
            ),
            None => println!(
                "工件包络：Ø{:.0}×{:.0} ⊆ 台面 Ø{:.0} / 高 {:.0} ✓",
                2.0 * r, h, env.table_diameter, env.max_height
            ),
        }
    }

    // `--axis x,y,z`：**定轴 3+2**——把旋转轴锁到该方向，再走普通三轴刀路。
    // `--multi`：多方向开粗 + 精加工（+Z / ±X / ±Y），一份程序。
    if let Some(dir) = axis_from_args() {
        println!(
            "定轴 3+2：刀轴锁定 ({:.2},{:.2},{:.2})",
            dir.x, dir.y, dir.z
        );
        run_cam_multi(model, &tris, stock, vec![dir]);
        return;
    }
    if args_flag("--multi") {
        run_cam_multi(model, &tris, stock, cncsim_cam::multi::default_directions());
        return;
    }

    // ── 工艺选择：先判断"这件是不是回转体"，不靠命令行的 `--rotary` 猜 ──
    //
    // 回转（车铣）只对回转体有意义：方形支架、手机支架这种件，绕 Z 的轮廓根本
    // 描述不了它，硬跑只会得到一堆"残料/过切"。判据是面积加权吻合度。
    let rot = cncsim_cam::rotary::rotationality(&tris);
    println!("回转体吻合度：{:.1}%", rot * 100.0);
    if rotary_from_args() || rot >= 0.85 {
        if rot < 0.85 {
            eprintln!("  注意：吻合度偏低，回转工艺只能做外形近似（其余靠跳过）");
        }
        run_cam_rotary(model, &tris, stock);
        return;
    }
    if rot < 0.85 {
        println!("  不是回转体 → 走 3 轴高度场（单值上表面）");
    }

    // 刀径按毛坯短边缩放：小件用小刀，大件用 φ6。
    let short = stock.size().x.min(stock.size().y);
    let d_rough = 6.0f64.min(0.35 * short).max(1.0);
    let d_finish = 4.0f64.min(d_rough * 0.7).max(0.8);
    let rough = cncsim_cam::ToolSpec::flat(1, d_rough, 20.0).with_shank(shank_d());
    let finish = cncsim_cam::ToolSpec::ball(2, d_finish, 20.0).with_shank(shank_d());
    let sample = (short / 200.0).clamp(0.2, 0.6);
    let process = cncsim_cam::Process {
        stock_allowance_mm: 0.2,
        stepdown_mm: 1.0,
        rough_stepover_mm: d_rough * 0.6,
        finish_stepover_mm: (short / 120.0).clamp(0.4, 1.2),
        sample_mm: sample,
        max_finish_slope_deg: 75.0,
        ..Default::default()
    };
    let job = cncsim_cam::Job {
        mesh: &tris,
        stock,
        rough: Some(rough.clone()),
        finish: finish.clone(),
        process: process.clone(),
        rest: None,
    };
    let t0 = std::time::Instant::now();
    let prog = match cncsim_cam::generate(&job) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("CAM 生成失败：{e}");
            std::process::exit(1);
        }
    };
    println!(
        "刀路：{} 行 / {} 层 / 粗 {} 段 / 精 {} 段 / 切 {:.0} mm（{:.2} s）",
        prog.stats.lines,
        prog.stats.levels,
        prog.stats.rough_moves,
        prog.stats.finish_moves,
        prog.stats.cut_mm,
        t0.elapsed().as_secs_f64()
    );

    let nc = gcode_path_from_args().unwrap_or_else(|| model.with_extension("nc"));
    if let Err(e) = std::fs::write(&nc, &prog.gcode) {
        eprintln!("写 G 代码失败：{e}");
    } else {
        println!("G 代码：{}", nc.display());
    }

    // ── 现有解释器 ──
    let ic = cncsim_gcode::InterpConfig {
        tool_radii: vec![0.0, rough.radius(), finish.radius()],
        ..Default::default()
    };
    let program = cncsim_gcode::interpret(&prog.gcode, &ic);
    if program.has_errors() {
        eprintln!("解释器报错：{:?}", program.issues);
        std::process::exit(1);
    }
    println!(
        "解释：{} 块 / {} 条 issue",
        program.len(),
        program.issues.len()
    );

    // ── 现有仿真 + 设计比对 ──
    let h = 0.25;
    let size = stock.size();
    let domain = [
        (size.x / h).round().max(1.0) as i32,
        (size.y / h).round().max(1.0) as i32,
        (size.z / h).round().max(1.0) as i32,
    ];
    let grid = cncsim_voxel::GridConfig {
        h,
        band_voxels: 4,
        value_type: cncsim_voxel::ValueType::I8,
    };
    let design = cncsim_sim::compare::field_design(&tris, grid, stock.min, domain, 0.3);
    let cfg = cncsim_sim::run::RunConfig {
        grid,
        stock_origin: stock.min,
        stock_size: domain,
        // 相似度要在仿真**之后**用设计场，所以这里克隆一份（Field 是 i8 稀疏场，
        // 桌腿件 h=0.6 时约 0.5 MB，代价可以忽略）
        design: Some(design.clone()),
        design_step: 0.3,
        collide: false,
        creator_sub: 4,
        colorize: args_flag("--color"),
        drop_chips: true,
        ..Default::default()
    };
    let tools = cncsim_tool::ToolTable::new(vec![rough.to_tool_def(), finish.to_tool_def()]);
    let t0 = std::time::Instant::now();
    let out = cncsim_sim::run::run_tools(&program, &tools, &cfg);
    let r = &out.report;
    println!(
        "仿真：{:.1} s / {} 块 / {} 段 / 去料 {:.1} mm³ / 丢碎屑 {} 格",
        t0.elapsed().as_secs_f64(),
        out.stats.blocks,
        out.stats.cut_segments,
        r.removed_volume_mm3,
        r.chip_voxels
    );
    println!(
        "  切割染色：{}（creator {:.1} KB）",
        if cfg.colorize { "开" } else { "关" },
        out.grid.creator_bytes() as f64 / 1024.0
    );
    println!(
        "  过切 {:.3} mm³ / {:.4} mm；残留 {:.3} mm³ / {:.4} mm",
        r.gouge_volume_mm3, r.gouge_max_depth_mm, r.residual_volume_mm3, r.residual_max_thickness
    );

    // ── 逐列比较：加工后的顶面 vs 设计顶面 ──
    let design = cfg.design.as_ref().expect("上面刚设置 design");
    let (dz, diff) = height_diff(&out.grid, design, &cfg, stock);
    println!(
        "高度差：{} 列 / 平均 |Δz| {:.3} mm / P95 {:.3} mm / 残料 {:+.3} mm / 过切 {:+.3} mm / |Δz|≤0.1 mm {:.1}%",
        diff.columns,
        diff.mean_abs,
        diff.p95,
        diff.max_residual,
        diff.max_gouge,
        diff.within_0_1 * 100.0
    );
    let diff_path = diff_path_from_args().unwrap_or_else(|| model.with_extension("diff.html"));
    let title = model.file_name().unwrap_or_default().to_string_lossy();
    match write_diff_html(&diff_path, &dz, cfg.stock_size[0], cfg.stock_size[1], &diff, &title) {
        Ok(()) => println!("偏差图：{}", diff_path.display()),
        Err(e) => eprintln!("写偏差图失败：{e}"),
    }

    if let Some(path) = export_path_from_args() {
        let mesh = out.grid.extract_surface_lod(4);
        let title = format!(
            "CAM：{}",
            model.file_name().unwrap_or_default().to_string_lossy()
        );
        match cncsim_render::preview::write_html(&path, &mesh, &title) {
            Ok(s) => println!(
                "预览：{}（{} 顶点 / {} 三角形 / {} MB）",
                path.display(),
                s.vertices,
                s.triangles,
                s.bytes / (1 << 20)
            ),
            Err(e) => eprintln!("写预览失败：{e}"),
        }
    }
}

#[derive(Default)]
struct HeightDiff {
    columns: usize,
    mean_abs: f64,
    p95: f64,
    max_residual: f64,
    max_gouge: f64,
    within_0_1: f64,
}

/// 逐列比较「加工后的顶面」与「设计顶面」。
///
/// 两边都取**从上往下第一个材料体素的中心 z**，所以体素量化误差在差值里抵消；
/// 差值为正 = 加工面高于设计面（残料），为负 = 切过头（过切）。
fn height_diff(
    grid: &cncsim_voxel::SparseGrid<i8>,
    design: &cncsim_sim::compare::DesignModel,
    cfg: &cncsim_sim::run::RunConfig,
    stock: cncsim_cam::Stock,
) -> (Vec<f32>, HeightDiff) {
    let h = cfg.grid.h;
    let [nx, ny, nz] = cfg.stock_size;
    let mut dz = vec![f32::NAN; (nx * ny) as usize];
    let mut vals: Vec<f64> = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            let x = stock.min.x + (i as f64 + 0.5) * h;
            let y = stock.min.y + (j as f64 + 0.5) * h;
            let mut zm = None;
            for k in (0..nz).rev() {
                if grid.get([i, j, k]) < 0 {
                    zm = Some(stock.min.z + (k as f64 + 0.5) * h);
                    break;
                }
            }
            let mut zd = None;
            for k in (0..nz).rev() {
                let z = stock.min.z + (k as f64 + 0.5) * h;
                if design.sdf(cncsim_math::DVec3::new(x, y, z)) < 0.0 {
                    zd = Some(z);
                    break;
                }
            }
            if let (Some(a), Some(b)) = (zm, zd) {
                let d = a - b;
                dz[(j * nx + i) as usize] = d as f32;
                vals.push(d);
            }
        }
    }
    if vals.is_empty() {
        return (dz, HeightDiff::default());
    }
    let n = vals.len();
    let mut sorted = vals.clone();
    sorted.sort_by(f64::total_cmp);
    let mut s = HeightDiff {
        columns: n,
        mean_abs: vals.iter().map(|v| v.abs()).sum::<f64>() / n as f64,
        p95: sorted[((n as f64 * 0.95) as usize).min(n - 1)],
        max_residual: sorted[n - 1],
        max_gouge: sorted[0],
        within_0_1: vals.iter().filter(|v| v.abs() <= 0.1).count() as f64 / n as f64,
    };
    s.p95 = s.p95.abs();
    (dz, s)
}

/// 偏差热力图：顶视图，红 = 残料（加工面高），蓝 = 过切（低），绿 = ±0.1 mm。
fn write_diff_html(
    path: &std::path::Path,
    dz: &[f32],
    nx: i32,
    ny: i32,
    stats: &HeightDiff,
    title: &str,
) -> std::io::Result<()> {
    let mut arr = String::with_capacity(dz.len() * 7);
    for (i, v) in dz.iter().enumerate() {
        if i > 0 {
            arr.push(',');
        }
        if v.is_finite() {
            arr.push_str(&format!("{v:.2}"));
        } else {
            arr.push_str("null");
        }
    }
    let html = format!(
        r#"<!DOCTYPE html><html><head><meta charset="utf-8"><title>{title} 偏差</title>
<style>
body{{background:#0b0e12;color:#d7dde5;font:13px/1.5 ui-monospace,Consolas,monospace;margin:24px}}
h1{{font-size:16px;font-weight:600;margin:0 0 8px}}
.k{{color:#8b98a8}} canvas{{image-rendering:pixelated;border:1px solid #2a3340;background:#222}}
.legend{{display:flex;gap:16px;margin:10px 0}} .sw{{display:inline-block;width:12px;height:12px;margin-right:4px;vertical-align:-1px}}
</style></head><body>
<h1>{title} — 加工结果 vs 原始模型（顶视高度差）</h1>
<div class="k">列数 {columns} · 平均 |Δz| {mean:.3} mm · P95 {p95:.3} mm · 残料最大 {res:.3} mm · 过切最大 {gouge:.3} mm · |Δz|≤0.1 mm {within:.1}%</div>
<div class="legend"><span><i class="sw" style="background:#3cb45a"></i>±0.1 mm</span><span><i class="sw" style="background:#c83c3c"></i>残料（红）</span><span><i class="sw" style="background:#3c64c8"></i>过切（蓝）</span><span><i class="sw" style="background:#333"></i>无数据</span></div>
<canvas id="c" width="{nx}" height="{ny}" style="width:640px;height:640px"></canvas>
<script>
const NX={nx}, NY={ny}, DZ=[{arr}];
const cv=document.getElementById('c'), ctx=cv.getContext('2d');
const img=ctx.createImageData(NX,NY);
for(let j=0;j<NY;j++){{for(let i=0;i<NX;i++){{
  const d=DZ[(NY-1-j)*NX+i], o=(j*NX+i)*4;
  let r,g,b;
  if(d===null){{r=g=b=45;}}
  else if(Math.abs(d)<=0.1){{r=60;g=180;b=90;}}
  else if(d>0){{const t=Math.min(1,(d-0.1)/0.4);r=200;g=Math.round(120*(1-t));b=60;}}
  else{{const t=Math.min(1,(-d-0.1)/0.4);r=60;g=Math.round(120*(1-t));b=200;}}
  img.data[o]=r;img.data[o+1]=g;img.data[o+2]=b;img.data[o+3]=255;
}}}}
ctx.putImageData(img,0,0);
</script></body></html>"#,
        title = title,
        columns = stats.columns,
        mean = stats.mean_abs,
        p95 = stats.p95,
        res = stats.max_residual,
        gouge = stats.max_gouge,
        within = stats.within_0_1 * 100.0,
        nx = nx,
        ny = ny,
        arr = arr,
    );
    std::fs::write(path, html)
}

/// `--cam <model.stl> --multi`：多方向开粗 + 精加工，串成一份程序。
///
/// 这是"从 XYZ 任意方向开粗"的实现：每道工序用自己的 `Frame` 把模型转进该
/// 方向的局部系跑同一条 3 轴链路，输出时再用 `G43.4 H1` + A/C 把刀轴摆过去。
/// 单道做不了（该方向多层/倒扣）就跳过并报原因，不拖垮整份计划。
fn run_cam_multi(
    model: &std::path::Path,
    tris: &[cncsim_cam::Tri],
    stock: cncsim_cam::Stock,
    mut dirs: Vec<cncsim_math::DVec3>,
) {
    // `--no-side`：**扔掉 ±X/±Y 这两道**。
    //
    // 实测桌腿件（h=0.6，清根 + 翻面 + 曲面道）：
    //
    //   11 道（含 ±X/±Y）：97.8% / 过切 **1.2%** / 残料 1.0%
    //    7 道（去 ±X/±Y）：97.6% / 过切 **0.1%** / 残料 2.3%
    //
    // 侧向工序是**定轴高度场**，每列只认最上面那层，遇到倒扣就多啃 0.3 mm
    // （过切归因 T6/T16/T14 全在这几道）。五轴曲面道已经把它们活干了，
    // 而且干得准。合格率只差 0.2 个点，过切差 10 倍——**过切是废品，残料不是**。
    //
    // **但别当默认**：叶轮 1 去掉侧向道是 94.7%（原来是 98.2%）——叶片通道的
    // 大块料还得靠侧向工序开出来，五轴曲面道只做面不做量。所以这条只对
    // "侧向道会啃倒扣、而曲面道覆盖得动"的件（桌腿件这类）成立。
    if !args_flag("--no-surface") && args_flag("--no-side") {
        let before = dirs.len();
        dirs.retain(|d| d.z.abs() > 0.5);
        if dirs.len() != before {
            println!(
                "侧向工序：--no-side 去掉 {} 道 ±X/±Y（五轴曲面道覆盖它们，过切少一个量级）",
                before - dirs.len()
            );
        }
    }
    use cncsim_cam::multi::{Pass, generate_plan};

    // 工艺不设限的前提是"知道每条路能走多远"：回转件上 RTCP（刀轴逐点跟法向）
    // 比"多道定轴"更好——既能覆盖到更陡的面，接触点曲率也是恒定的，光洁度更高。
    // 这里如实把判断摆出来，而不是偷偷只用其中一条。
    let rot = cncsim_cam::rotary::rotationality(tris);
    if rot >= 0.85 {
        println!(
            "工艺提示：回转体吻合度 {:.1}% → 这件更适合 `--rotary`（五轴 RTCP，刀轴逐点跟法向）；\
             当前按多道定轴编排",
            rot * 100.0
        );
    } else {
        println!(
            "工艺提示：回转体吻合度 {:.1}% → 回转 RTCP 描述不了它，走多道定轴（3 轴 + 3+2）",
            rot * 100.0
        );
    }

    let short = stock.size().x.min(stock.size().y);
    let d_rough = 6.0f64.min(0.35 * short).max(1.0);
    // ── 精加工刀径：**这一项决定形状保真度的天花板** ──
    //
    // 球头在凹角处必然留下 `R(1/sin(θ/2)−1)` 的残料（90° 角 = 0.41R），这是几何
    // 决定的，不是刀路能绕过去的。桌腿件 h=0.6 的实测（解析口径，与 h 无关）：
    //
    //   Ø3.0 → 合格 85.0% / 残料 14.4% / 切长 45 m
    //   Ø1.5 → 合格 91.6% / 残料  7.2% / 切长 84 m
    //   Ø1.0 → 合格 94.1% / 残料  4.0% / 切长 123 m
    //
    // 残料面积 ∝ √R（残高 >tol 的带宽 ∝ √(2R·tol)），所以**刀径是唯一的杠杆**，
    // 行距/坡度都不是。代价是切长 ∝ 1/D，按工件尺度取下限：大件用 Ø1 跑出来的
    // 程序是天文数字。`--finish-d` 可显式覆盖。
    let d_finish = finish_d_from_args()
        .unwrap_or_else(|| (short / 18.0).clamp(0.8, 3.0).min(d_rough * 0.6))
        .clamp(0.4, d_rough);
    let rough = cncsim_cam::ToolSpec::flat(0, d_rough, 20.0).with_shank(shank_d());

    // ── 开粗参数：按**最大材料去除率**定，同时留够精加工余量 ──
    //
    // 材料去除率 MRR = ap(层深) × ae(行距) × vf(进给)。平底刀开粗的三项取满：
    //   ap = 1.0×D（满刀深，平底刀每层切出一个平面，这是它相对球头的优势）
    //   ae = 0.7×D（大于 0.7D 中心会留下未切芯，反而要跑第二遍）
    //   vf = 进给，按刀径缩放（小刀跑不快）
    // 留量 `allowance_mm` 是**法向**余量：粗加工把每个面留在真实面之外这么多，
    // 精加工只削这一层。0.3 mm ≥ 精加工行距带来的残高，粗加工不会侵占设计面。
    let allowance = 0.3f64;
    let ap = d_rough.min(8.0);
    let ae = d_rough * 0.7;
    let vf = (600.0 + d_rough * 120.0).clamp(700.0, 1500.0);
    let mrr = ap * ae * vf / 1000.0; // cm³/min

    let process = cncsim_cam::Process {
        stock_allowance_mm: allowance,
        stepdown_mm: ap,
        rough_stepover_mm: ae,
        finish_stepover_mm: finish_step_from_args().unwrap_or((d_finish * 0.4).clamp(0.3, 1.2)),
        sample_mm: (short / 250.0).clamp(0.2, 0.5),
        // 陡壁的坡度门槛。**drop-cutter 不会因为坡陡而啃进去**（刀心取刀盘内最高点，
        // 几何上保证不侵入），所以门槛放宽只多走路、不会多切。留 70° 是把"接近
        // 竖直但还没到竖直"的那一段也纳进来；竖直面交给侧向工序。
        max_finish_slope_deg: slope_deg(),
        feed_mm_min: vf,
        plunge_mm_min: vf * 0.35,
        // 多方向/定轴是"尽最大努力"：孔、内腔、倒扣的背面切不到就留料，不拒收
        allow_undercut: true,
        ..Default::default()
    };
    // 球头精加工用 `--flat-finish` 换成平底刀（走 drop-cutter，见 cam 的 finish_pass）
    let finish = if args_flag("--flat-finish") {
        cncsim_cam::ToolSpec::flat(0, d_finish, 20.0).with_shank(shank_d())
    } else {
        cncsim_cam::ToolSpec::ball(0, d_finish, 20.0).with_shank(shank_d())
    };

    println!(
        "开粗：ap {ap:.1}（{:.1}×D）× ae {ae:.1}（0.7×D）× vf {vf:.0} → MRR {mrr:.1} cm³/min，留量 {allowance:.2} mm",
        ap / d_rough
    );
    // ── 网格与设计场：可达性分析要用，所以提前到"定工序"之前 ──
    let size = stock.size();
    let vol = size.x * size.y * size.z;
    let h = h_from_args().unwrap_or((vol / 3.0e7).cbrt().clamp(0.3, 1.0));
    let domain = [
        (size.x / h).round().max(1.0) as i32,
        (size.y / h).round().max(1.0) as i32,
        (size.z / h).round().max(1.0) as i32,
    ];
    let grid = cncsim_voxel::GridConfig {
        h,
        band_voxels: 4,
        value_type: cncsim_voxel::ValueType::I8,
    };
    println!(
        "网格：h = {h:.2} mm（{}×{}×{}）",
        domain[0], domain[1], domain[2]
    );
    let design = cncsim_sim::compare::field_design(tris, grid, stock.min, domain, 0.3);
    // 设计面采样（带法向和面积）：可达性分析用，比体素细一级没意义
    let samples = {
        let mut step = 1.5f64;
        let mut smp = cncsim_voxel::meshvol::surface_samples(tris, step);
        while smp.len() > 160_000 && step < 6.0 {
            step *= 1.5;
            smp = cncsim_voxel::meshvol::surface_samples(tris, step);
        }
        smp
    };

    let mut passes: Vec<Pass> = dirs
        .iter()
        .map(|d| Pass {
            dir: *d,
            rough: Some(rough.clone()),
            finish: finish.clone(),
            rest: None,
            process: None,
        })
        .collect();
    // 轴向工序用 45° 坡度门槛，覆盖的是"法向与某根轴夹角 ≤45°"的面；剩下的洞在
    // **体对角线**附近。补 4 条 54.7° 的斜向道，只精加工（粗料已被轴向道去掉）。
    let mut n_oblique = 0usize;
    let mut n_kinematic = 0usize;
    if dirs.len() > 1 && !args_flag("--no-tilt-finish") {
        // 默认 4 条体对角线；`--tilt <度> --tilt-n <条数>` 可给任意角度。
        // 只受**机床运动学**约束：A 轴行程之外的方向直接丢，并如实报数。
        let (tilt, n) = tilt_from_args().unwrap_or((54.735_610_229_728_65, 4));
        let lim = cncsim_cam::rotary::A_LIMIT_DEG;
        for d in cncsim_cam::multi::oblique_directions(tilt, n) {
            let a = d.z.clamp(-1.0, 1.0).acos().to_degrees();
            if a > lim + 1e-9 {
                n_kinematic += 1; // A 轴够不到
                continue;
            }
            passes.push(Pass {
                dir: d,
                rough: None,
                finish: finish.clone(),
                rest: None,
                process: None,
            });
            n_oblique += 1;
        }
        println!(
            "斜向精加工：{n_oblique} 道（倾角 {tilt:.1}°）{}",
            if n_kinematic > 0 {
                format!("，A>{lim:.0}° 运动学挡掉 {n_kinematic} 道")
            } else {
                String::new()
            }
        );
    }
    // ── 工序编排：**不限定加工方式**，让覆盖面积决定要几道、往哪摆 ──
    //
    // 3 轴（+Z）、3+2（任意倾角定轴）在本实现里是同一套高度场，只是工序方向不同；
    // 真正的五轴 RTCP（刀轴逐点跟法向）只在回转件上有实现（`--rotary`，判据是
    // `rotationality ≥ 0.85`）。所以这里把**整个可达半球**当候选集，用带权集合覆盖
    // 贪心地挑方向：每次挑"新覆盖面积最大"的那一个，收益低于阈值就停。
    //
    // 这样"用三轴还是 3+2"不再是命令行开关，而是几何和机床行程共同决定的结论：
    // 平面件会被 +Z 一道吃满（贪心直接停），有侧壁的自然长出侧向/斜向道。
    let rc = cncsim_sim::reach::ReachConfig::default();
    let mut cand: Vec<cncsim_math::DVec3> = passes.iter().map(|p| p.dir).collect();
    {
        // 候选只在**机床摆得到**的范围内加：A = acos(dir.z) ≤ 110°
        let lim = cncsim_cam::rotary::A_LIMIT_DEG;
        for (tilt, n) in [(75.0f64, 8usize), (101.0, 8)] {
            for d in cncsim_cam::multi::oblique_directions(tilt, n) {
                if d.z.clamp(-1.0, 1.0).acos().to_degrees() > lim + 1e-9 {
                    continue;
                }
                if cand.iter().any(|c| c.dot(d) > 0.999) {
                    continue;
                }
                cand.push(d);
            }
        }
    }
    let vis = cncsim_sim::reach::visibility(&design, &samples, &cand, &rc);
    let base_mask = if passes.is_empty() {
        0
    } else {
        (1u64 << passes.len().min(63)) - 1
    };
    let mut reach = cncsim_sim::reach::report_with(&vis, base_mask);
    let mut final_mask = base_mask;
    let mut n_auto = 0usize;
    if !args_flag("--no-auto-pass") {
        let extra = vis.greedy(base_mask, 6, 0.004);
        if extra.is_empty() {
            println!(
                "工序编排：候选 {} 个方向 → 现有 {} 道已覆盖 {:.1}%，再加没有收益",
                vis.dirs.len(),
                passes.len(),
                reach.coverage() * 100.0
            );
        } else {
            println!(
                "工序编排：候选 {} 个方向（A≤{:.0}°）→ 现有 {} 道覆盖 {:.1}%",
                vis.dirs.len(),
                cncsim_cam::rotary::A_LIMIT_DEG,
                passes.len(),
                reach.coverage() * 100.0
            );
            let mut m = base_mask;
            // 面朝过但一个候选方向都不通的面积：这些是工艺天花板，加方向也救不回来
            let (mut blocked_area, mut back_area) = (0.0f64, 0.0f64);
            for i in 0..vis.masks.len() {
                if vis.masks[i] != 0 {
                    continue;
                }
                if vis.facing[i] != 0 {
                    blocked_area += vis.areas[i];
                } else {
                    back_area += vis.areas[i];
                }
            }
            if blocked_area + back_area > 0.0 {
                println!(
                    "  ├ 上限 {:.1}%：另有 {:.0} mm² 被遮挡 + {:.0} mm² 是背面，任何方向都够不到",
                    (1.0 - (blocked_area + back_area) / vis.total_area_mm2.max(1e-9)) * 100.0,
                    blocked_area,
                    back_area
                );
            }
            for k in &extra {
                let prev = m;
                m |= 1u64 << *k;
                let d = vis.dirs[*k];
                let a = d.z.clamp(-1.0, 1.0).acos().to_degrees();
                let c = d.x.atan2(-d.y).to_degrees();
                println!(
                    "  ├ +1 道 --axis={:.2},{:.2},{:.2}（A{a:.0}° C{c:.0}°）覆盖 {:.1}% → {:.1}%（新增 {:.0} mm²）",
                    d.x,
                    d.y,
                    d.z,
                    vis.coverage(prev) * 100.0,
                    vis.coverage(m) * 100.0,
                    (vis.coverage(m) - vis.coverage(prev)) * vis.total_area_mm2
                );
                passes.push(Pass {
                    dir: d,
                    rough: None,
                    finish: finish.clone(),
                    rest: None,
                    process: None,
                });
                n_auto += 1;
            }
            final_mask = m;
            reach = cncsim_sim::reach::report_with(&vis, m);
        }
    }

    println!(
        "工序：{} 道（轴向 {} + 斜向 {n_oblique} + 编排 {n_auto}）· 粗 Ø{d_rough:.1} 平底 + 精 Ø{d_finish:.1} {}",
        passes.len(),
        dirs.len(),
        if args_flag("--flat-finish") {
            "平底（立铣刀）"
        } else {
            "球头"
        }
    );

    // ── 凹角小刀清根：先按现有工序体检一遍，再把残料圈补一道小刀 ──
    //
    // 球头在凹角处必然留下 `R(1/sin(θ/2)−1)`（90° 角 = 0.41R），换小刀只补
    // 残料那一圈，切长只涨几个百分点；全局换小刀是 2.7 倍切长（见 §39）。
    //
    // 判据用**解析口径**（与 h 无关）：只补 0.15 < d ≤ 1.0 的点。d > 1 mm 的是
    // "刀根本没接近过"——换小刀也进不去，那是翻面或五轴的活，补了只是白跑。
    // ── 刀路体检：**只走刀、不切料**，拿到"哪里的设计面还没被切到" ──
    //
    // 清根（小刀补残料圈）和五轴曲面道（补高度场看不见的面）都要这份数据，
    // 所以只跑一次。代价 = 一次纯粹的刀路走位（桌腿件 22 道 ~51 s）。
    let want_rest = !args_flag("--no-rest");
    let want_surface = !args_flag("--no-surface");
    let mut probe: Option<(cncsim_sim::run::RunOutput, cncsim_tool::ToolTable)> = None;
    if want_rest || want_surface {
        let t0 = std::time::Instant::now();
        let plan0 = match generate_plan(tris, stock, &process, &passes) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("刀路体检：初版计划生成失败：{e}");
                std::process::exit(1);
            }
        };
        let table0 = cncsim_tool::ToolTable::new(plan0.tools.clone());
        let prog0 =
            cncsim_gcode::interpret(&plan0.program.gcode, &interp_config_for(&plan0.tools));
        let cfg0 = cncsim_sim::run::RunConfig {
            grid,
            stock_origin: stock.min,
            stock_size: domain,
            design: Some(design.clone()),
            dry_run: true,
            drop_chips: false,
            ..Default::default()
        };
        let out0 = cncsim_sim::run::run_tools(&prog0, &table0, &cfg0);
        println!(
            "刀路体检（只走刀不切料）：{:.1} s / {} 段",
            t0.elapsed().as_secs_f64(),
            out0.cut_segs.len()
        );
        probe = Some((out0, table0));
    }

    if want_rest && let Some((out0, table0)) = &probe {
        // 清根刀径：**由公差反推**。球头在 90° 凹角的残高是 `R(1/sin45°−1)=0.414R`，
        // 要落进 ±0.15 就必须 R ≤ 0.36（Ø0.72）。默认取这个上限与 0.6×精刀 的较小者，
        // `--rest-d` 可覆盖。
        let d_rest = rest_d_from_args().unwrap_or_else(|| {
            let by_tol = 2.0 * 0.15 / (1.0 / (std::f64::consts::FRAC_PI_4).sin() - 1.0);
            (d_finish * 0.6).min(by_tol * 1.02).clamp(0.5, 1.5)
        });
        let d0 = cncsim_sim::compare::sweep_deviation(&out0.cut_segs, table0, &vis.points, 1.0);
        // 残料点的**下标**（后面要按方向筛：每个方向只补它自己看得见的那批）
        let targets: Vec<usize> = d0
            .iter()
            .enumerate()
            .filter(|(_, d)| d.is_finite() && **d > 0.15 && **d <= 1.0)
            .map(|(i, _)| i)
            .collect();
        println!(
            "残料清根：{} 点残料（0.15–1.0 mm）→ 换 Ø{d_rest:.1} 补刀（体检程序 {} 段）",
            targets.len(),
            out0.cut_segs.len()
        );
        if targets.is_empty() {
            println!("  └ 没有落在这个区间的残料，不加清根道");
        } else {
            let mut rp = process.clone();
            rp.finish_stepover_mm = (d_rest * 0.4).clamp(0.2, 0.6);
            rp.stock_allowance_mm = 0.0;
            let n0 = passes.len();
            // **每个方向只补它自己看得见的残料**（`vis.masks` 的第 k 位）。
            // 11 个方向各补一遍同一批点是 2.1 倍切长——实测下来还不如全局换小刀，
            // 那就白做了。覆盖不到 3% 的方向直接不出道。
            let mut added = 0usize;
            for k in 0..n0.min(63) {
                let bit = 1u64 << k;
                let pts: Vec<cncsim_math::DVec3> = targets
                    .iter()
                    .filter(|i| vis.masks[**i] & bit != 0)
                    .map(|i| vis.points[*i])
                    .collect();
                if pts.len() * 33 < targets.len() {
                    continue; // <3%：这道补不了多少东西
                }
                passes.push(Pass {
                    dir: passes[k].dir,
                    rough: None,
                    // 刃长 3 mm：清根刀是**长颈小刀**，不是 20 mm 刃的常规球头——
                    // 刃长写 20 会让刀杆检查把"悬伸下面 20 mm 处的料"也判成碰撞，
                    // 实测一次就误杀 1474 个清根点。
                    finish: cncsim_cam::ToolSpec::ball(0, d_rest, 3.0).with_shank(shank_d()),
                    rest: Some(pts),
                    process: Some(rp.clone()),
                });
                added += 1;
            }
            println!(
                "  └ 追加 {added} 道 Ø{d_rest:.1} 清根（每道只走自己看得见的残料，行距 {:.2}）",
                rp.finish_stepover_mm
            );
        }
    }

    // ── 翻面装夹（第二套）：把件翻过来，从**下面**再加工一遍 ──
    //
    // 一次装夹里够不到的面，绝大多数是朝下的（`reach` 报"纯背面，建议 --axis=0,0,-1
    // （A180°）"）。A 轴摆不到 180°，但**把件翻过来**是等价的：翻面后刀还是竖直的，
    // 相对运动恰好等于"从 −Z 加工"。所以仿真里就是一道 `dir = -Z` 的工序，
    // 世界坐标不变、网格不用转——只有出厂程序要按翻面后的坐标系另出一份。
    let flip = args_flag("--flip");
    let flip_idx = if flip {
        let i = passes.len();
        passes.push(Pass {
            dir: cncsim_math::DVec3::new(0.0, 0.0, -1.0),
            // 翻面后要开粗：凹腔/悬伸下面的料整块还在
            rough: Some(rough.clone()),
            finish: finish.clone(),
            rest: None,
            process: None,
        });
        println!(
            "翻面装夹：追加第 {} 道（dir -Z，A 轴摆不到 180°，实机是把件翻过来重新装夹）",
            i + 1
        );
        Some(i)
    } else {
        None
    };

    // ── 五轴曲面法向点铣：把"高度场看不见"的面补上 ──
    //
    // 高度场每列只认最上面那一层，所以叶片侧壁在任何方向的列里都不是最高面——
    // `docs/项目记忆.md` §40 里叶轮 2 可达 86.3%、可达面却只有 15.4% 合格。
    // 这条道**不经过高度场**：球心放在 `p + R·n`，刀轴沿 `n`，球在 `p` 相切。
    //
    // 目标点用**密采**（`surface_samples`，步距 = 0.4×精刀径）：`reach` 的
    // 1.5 mm 采样只够判"看不看得见"，不够定刀路——1.5 mm 步距在 R0.5 的球上
    // 会留下 0.56 mm 的刀花。
    let mut surf_frag = String::new();
    let mut surf_tool: Option<cncsim_cam::ToolSpec> = None;
    if want_surface && let Some((out0, table0)) = &probe {
        let step = (d_finish * 0.4).clamp(0.25, 0.8);
        let smp = cncsim_voxel::meshvol::surface_samples(tris, step);
        let pts: Vec<cncsim_math::DVec3> = smp.iter().map(|(p, _, _)| *p).collect();
        let d = cncsim_sim::compare::sweep_deviation(&out0.cut_segs, table0, &pts, 1.0);
        let targets: Vec<(cncsim_math::DVec3, cncsim_math::DVec3)> = smp
            .iter()
            .zip(&d)
            .filter(|(_, dd)| dd.is_finite() && **dd > 0.15)
            .map(|((p, n, _), _)| (*p, *n))
            .collect();
        println!(
            "五轴曲面：设计面密采 {} 点（{:.2} mm）→ 残料 {} 点（{:.1}%）",
            smp.len(),
            step,
            targets.len(),
            targets.len() as f64 / smp.len().max(1) as f64 * 100.0
        );
        if !targets.is_empty() {
            // 曲面道刀径：**默认 Ø0.8**（≤ 精刀）。§44 时 Ø1.0 最优，但那是
            // 等径柄口径；Ø6 柄 + 分段碰撞之后重测（§51）：
            //
            //   Ø1.0 颈0 → 96.5% / Ø0.8 颈0 → 96.6% / **Ø0.8 颈9 → 98.0%**
            //
            // 小球能把球面闸门的拒绝从 3309 降到 214（进得去窄槽），
            // 配合长颈把 Ø6 柄挡在通道外。
            let ds = surface_d_from_args().unwrap_or(d_finish.min(0.8));
            // 刃长 = 刀能"伸进去"的深度。给长一点能进深腔，但**刃部圆柱也要
            // 真的进得去**（见 `shank_hits_axis`）——深窄槽里长刃反而全被拒。
            let flute = surface_flute_from_args()
                .unwrap_or_else(|| (0.4 * stock.size().z).clamp(4.0, 16.0));
            let tn = (2 * passes.len() + 1) as u16;
            // **刀号必须写进 `ToolSpec`**：G 代码里的 `T23` 只是文本，仿真按
            // `ToolTable::by_number` 找母线。刀号写成 0 会让仿真用错刀（退到第一把
            // Ø6 平底刀），实测把过切从 1.2% 抬到 6.8%、最深 2.92 mm。
            let tool = cncsim_cam::ToolSpec::ball(tn, ds, flute)
                .with_shank(shank_d())
                // 避空颈：让 Ø6 柄**晚点介入**。默认 = 工件高度的 22%（叶轮
                // 41 mm → 9.0），实测桌腿件 +12 无害（瓶颈不在刀杆）、叶轮
                // +9 把刀杆撞从 7516 降到 1178、合格率 96.5 → 98.0%。
                .with_neck(surface_neck_from_args()
                    .unwrap_or_else(|| (0.22 * stock.size().z).clamp(0.0, 12.0)));
            let mut o = cncsim_cam::surface::SurfaceOpts::new(tool.clone(), tn, stock);
            o.follow_flow = args_flag("--flow");
            o.max_tilt_deg = surface_tilt_from_args().unwrap_or(60.0);
            o.link_mm = (2.0 * step).clamp(0.5, 3.0);
            o.out_step_mm = surface_step_from_args()
                .unwrap_or_else(|| (1.2 * tool.radius()).clamp(0.3, 1.2));
            o.feed_mm_min = 800.0;
            match cncsim_cam::surface::generate_surface(tris, stock, &targets, &o) {
                Ok(sx) => {
                    let st = sx.stats;
                    println!(
                        "  └ Ø{ds:.1} 长颈球头（刃长 {flute:.0}）：通过 {} 点（其中 {} 点靠歪刀轴）/ A超程 {} / 球面塞不下 {} / 刀杆撞 {} / 切入啃 {} → {} 条链 / {} 块 / 切 {:.1} m / 空移 {:.1} m / 链间擦料 {} 处",
                        st.accepted,
                        st.axis_retried,
                        st.a_rejected,
                        st.gouge_rejected,
                        st.shank_rejected,
                        st.plunge_rejected,
                        st.runs,
                        st.moves,
                        st.cut_mm / 1000.0,
                        st.rapid_mm / 1000.0,
                        st.link_unsafe
                    );
                    surf_frag = sx.gcode;
                    surf_tool = Some(tool);
                }
                Err(e) => println!("  └ 五轴曲面道生成失败：{e}"),
            }
        }
    }

    let t0 = std::time::Instant::now();
    let mut plan = match generate_plan(tris, stock, &process, &passes) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("多方向 CAM 失败：{e}");
            std::process::exit(1);
        }
    };

    // 把五轴曲面道插进程序：**翻面道之前**（它在第一套装夹里，目标是翻面前的残料），
    // 没有翻面时就插在收尾（G53 回退）之前。
    if !surf_frag.is_empty() {
        let at = match flip_idx {
            Some(fi) => plan
                .program
                .gcode
                .find(&format!("(PASS {} dir", fi + 1))
                .unwrap_or(plan.program.gcode.len()),
            None => plan
                .program
                .gcode
                .rfind("(G53:")
                .unwrap_or(plan.program.gcode.len()),
        };
        plan.program.gcode.insert_str(at, &surf_frag);
        if let Some(t) = surf_tool {
            plan.tools.push(t.to_tool_def());
        }
    }
    println!(
        "刀路：{} 行 / {} 层 / 粗 {} 段 / 精 {} 段 / 切 {:.0} mm（{:.2} s）",
        plan.program.stats.lines,
        plan.program.stats.levels,
        plan.program.stats.rough_moves,
        plan.program.stats.finish_moves,
        plan.program.stats.cut_mm,
        t0.elapsed().as_secs_f64()
    );
    println!(
        "完成 {} 道 / 跳过 {} 道 / 刀杆避让 {} 点 / 球面避让（啃悬伸）{} 点",
        plan.passes,
        plan.skipped.len(),
        plan.program.stats.shank_skips,
        plan.program.stats.ball_skips
    );
    for (i, why) in &plan.skipped {
        let d = passes[*i].dir;
        println!(
            "  ├ 跳过 #{i} dir ({:.0},{:.0},{:.0})：{why}",
            d.x, d.y, d.z
        );
    }

    let nc = gcode_path_from_args().unwrap_or_else(|| model.with_extension("multi.nc"));
    // 有翻面道时：出厂程序要拆成两套——程序里那道 `A180` 机床摆不到，实机是
    // **把件翻过来重新装夹**，第二套的坐标要按翻面后的坐标系给（G55）。
    if let Some(fi) = flip_idx {
        let mark = format!("(PASS {} dir", fi + 1);
        match plan.program.gcode.find(&mark) {
            Some(pos) => {
                let mut s1 = String::from(&plan.program.gcode[..pos]);
                s1.push_str("(SETUP 1 END: 松开压板，把件翻面)
");
                s1.push_str(&format!("G49
G53 Z{:.3}
M5
M30
", stock.max.z + 30.0));
                if let Err(e) = std::fs::write(&nc, &s1) {
                    eprintln!("写 G 代码失败：{e}");
                } else {
                    println!("G 代码（第 1 套）：{}", nc.display());
                }
                // 第二套：翻面坐标系（+Z 就是机床的竖直方向），另出一份
                let f = cncsim_cam::Frame::from_dir(cncsim_math::DVec3::new(0.0, 0.0, -1.0));
                let flip_mesh: Vec<cncsim_cam::Tri> = tris
                    .iter()
                    .map(|t| [f.unapply(t[0]), f.unapply(t[1]), f.unapply(t[2])])
                    .collect();
                let mut lo2 = cncsim_math::DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut hi2 = cncsim_math::DVec3::new(
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                );
                for t in &flip_mesh {
                    for v in t {
                        lo2 = cncsim_math::DVec3::new(lo2.x.min(v.x), lo2.y.min(v.y), lo2.z.min(v.z));
                        hi2 = cncsim_math::DVec3::new(hi2.x.max(v.x), hi2.y.max(v.y), hi2.z.max(v.z));
                    }
                }
                let pad = cncsim_math::DVec3::new(1.0, 1.0, 0.0);
                let fstock = cncsim_cam::Stock {
                    min: lo2 - pad,
                    max: hi2 + pad,
                };
                let fpass = Pass {
                    dir: cncsim_math::DVec3::new(0.0, 0.0, 1.0),
                    rough: Some(rough.clone()),
                    finish: finish.clone(),
                    rest: None,
                    process: None,
                };
                match generate_plan(&flip_mesh, fstock, &process, &[fpass]) {
                    Ok(p2) => {
                        let p2p = nc.with_extension("setup2.nc");
                        let txt = p2.program.gcode.replace("G54", "G55");
                        let txt = format!(
                            "(SETUP 2: 翻面装夹 / G55 / 原点在翻面后的毛坯上表面中心)
{txt}"
                        );
                        if let Err(e) = std::fs::write(&p2p, &txt) {
                            eprintln!("写第二套 G 代码失败：{e}");
                        } else {
                            println!(
                                "G 代码（第 2 套，翻面 / G55）：{}（{} 行 / 工具 T1/T2）",
                                p2p.display(),
                                txt.lines().count()
                            );
                        }
                    }
                    Err(e) => eprintln!("第二套（翻面）CAM 失败：{e}"),
                }
            }
            None => {
                eprintln!("翻面道在程序里找不到，出厂程序按整份写出（含 A180，**实机跑不了**）");
                let _ = std::fs::write(&nc, &plan.program.gcode);
            }
        }
    } else if let Err(e) = std::fs::write(&nc, &plan.program.gcode) {
        eprintln!("写 G 代码失败：{e}");
    } else {
        println!("G 代码：{}", nc.display());
    }

    // ── 解释：A/C 按 Head（双摆头）约定生成，必须配套 ──
    let ic = interp_config_for(&plan.tools);
    let program = cncsim_gcode::interpret(&plan.program.gcode, &ic);
    if program.has_errors() {
        eprintln!("解释器报错：{:?}", program.issues);
        std::process::exit(1);
    }
    println!(
        "解释：{} 块 / {} 条 issue",
        program.len(),
        program.issues.len()
    );

    let dry = args_flag("--dry");
    let cfg = cncsim_sim::run::RunConfig {
        grid,
        stock_origin: stock.min,
        stock_size: domain,
        // 相似度要在仿真**之后**用设计场，所以这里克隆一份（Field 是 i8 稀疏场，
        // 桌腿件 h=0.6 时约 0.5 MB，代价可以忽略）
        design: Some(design.clone()),
        design_step: 0.3,
        collide: false,
        creator_sub: 4,
        colorize: args_flag("--color"),
        drop_chips: !dry,
        dry_run: dry,
        ..Default::default()
    };
    let table = cncsim_tool::ToolTable::new(plan.tools.clone());
    let t0 = std::time::Instant::now();
    let out = cncsim_sim::run::run_tools(&program, &table, &cfg);
    let r = &out.report;
    if dry {
        println!(
            "刀路体检（--dry，不切材料）：{:.1} s / {} 块 / {} 段",
            t0.elapsed().as_secs_f64(),
            out.stats.blocks,
            out.stats.cut_segments
        );
    } else {
        println!(
            "仿真：{:.1} s / {} 块 / {} 段 / 去料 {:.1} mm³ / 丢碎屑 {} 格",
            t0.elapsed().as_secs_f64(),
            out.stats.blocks,
            out.stats.cut_segments,
            r.removed_volume_mm3,
            r.chip_voxels
        );
        println!(
            "  切割染色：{}（creator {:.1} KB）",
            if cfg.colorize { "开" } else { "关" },
            out.grid.creator_bytes() as f64 / 1024.0
        );
        // 墙钟拆成"切削 / 比对 / 其它"。**其它**里是解释、逐段位姿、线段表、
        // 观察者回调这些**串行**开销——多线程利用率不高时，先看这一项占多少。
        let wall = t0.elapsed().as_secs_f64();
        let cut = out.stats.cut_ms / 1e3;
        let cmp = out.stats.compare_ms / 1e3;
        println!(
            "  ├ 切削 {cut:.1} s（{:.0}%）/ 比对 {cmp:.1} s / 其它 {:.1} s（{:.0}%）/ 线程 {} / 单批并行片数 {}",
            cut / wall * 100.0,
            (wall - cut - cmp).max(0.0),
            (wall - cut - cmp).max(0.0) / wall * 100.0,
            cncsim_voxel::par::worker_count(),
            out.stats.max_par_slices
        );
        println!(
            "  过切 {:.3} mm³ / {:.4} mm @ ({:.2},{:.2},{:.2})；残留 {:.3} mm³ / {:.4} mm @ ({:.2},{:.2},{:.2})",
            r.gouge_volume_mm3,
            r.gouge_max_depth_mm,
            r.gouge_at.x,
            r.gouge_at.y,
            r.gouge_at.z,
            r.residual_volume_mm3,
            r.residual_max_thickness,
            r.residual_at.x,
            r.residual_at.y,
            r.residual_at.z
        );
        if let Some((lo, hi)) = r.gouge_region {
            println!(
                "  过切区域 ({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})",
                lo.x, lo.y, lo.z, hi.x, hi.y, hi.z
            );
        }
    }

    // ── 可达性报告：够不到的面单独列出来，并给出建议方向 ──
    println!(
        "可达性：{} 点 / {:.0} mm² → 可达 {:.1}% / 遮挡 {:.1}% / 背面 {:.1}%",
        reach.points,
        reach.total_area_mm2,
        reach.reachable_area_mm2 / reach.total_area_mm2.max(1e-9) * 100.0,
        reach.occluded_area_mm2 / reach.total_area_mm2.max(1e-9) * 100.0,
        reach.backfacing_area_mm2 / reach.total_area_mm2.max(1e-9) * 100.0
    );
    for g in reach.groups.iter().take(4) {
        let head = format!(
            "  ├ {:.0} mm²（{:.1}%）区域 ({:.1},{:.1},{:.1})..({:.1},{:.1},{:.1})",
            g.area_mm2,
            g.area_mm2 / reach.total_area_mm2.max(1e-9) * 100.0,
            g.lo.x,
            g.lo.y,
            g.lo.z,
            g.hi.x,
            g.hi.y,
            g.hi.z
        );
        if g.has_direction() {
            // 整片朝同一个方向：补一道那个方向的定轴工序就能覆盖。
            // 但 A 超行程的方向补不了——那种只能把件翻过来（`--flip`）。
            if g.a_deg() > cncsim_cam::rotary::A_LIMIT_DEG + 1e-9 {
                println!(
                    "{head}  → 需要 A{:.0}°，超出行程 {:.0}°：只能**翻面装夹**（`--flip`）",
                    g.a_deg(),
                    cncsim_cam::rotary::A_LIMIT_DEG
                );
            } else {
                println!(
                    "{head}  → 加一道 --axis={:.2},{:.2},{:.2}（A{:.0}° C{:.0}°）{}",
                    g.dir.x,
                    g.dir.y,
                    g.dir.z,
                    g.a_deg(),
                    g.c_deg(),
                    if g.backfacing_frac > 0.8 {
                        "，当前是纯背面"
                    } else {
                        ""
                    }
                );
            }
        } else {
            println!("{head}  → 封闭/多向：没有单一方向能进去，需翻面或五轴联动");
        }
    }
    if reach.occluded + reach.backfacing == 0 {
        println!("  └ 所有面都可及：剩下的偏差全是刀路问题（刀径/步距/凹角让位）");
    } else {
        println!(
            "  └ {} 点背面（没有工序方向朝向它）+ {} 点被遮挡：**刀路再调也切不到**，只能改工艺",
            reach.backfacing, reach.occluded
        );
        if !dry {
            println!(
                "  └ 剩下能切到的面上若还有残料，那才是刀路问题（实测过切 {:.3} mm³ / 残留 {:.3} mm³）",
                r.gouge_volume_mm3, r.residual_volume_mm3
            );
        }
    }

    // ── 名义曲面逐点偏差：和回转链路同一套口径，直接回答"像不像原版" ──
    let pts = cncsim_voxel::meshvol::surface_points(tris, 0.8);
    let mut sd = cncsim_sim::surface_dev::SurfaceStats::default();
    let mut dev: Vec<f64> = Vec::new();
    if !dry {
        dev = cncsim_sim::surface_dev::signed_deviation(&out.grid, &pts);
        sd = cncsim_sim::surface_dev::stats(&dev, 0.15, 4.0 * h);
        println!(
            "名义曲面：{} 点 / 平均 |d| {:.3} / P95 {:.3} / 合格(≤0.15) {:.1}% / 过切 {:.1}% / 残料 {:.1}% / 最大过切 {:+.2}",
            sd.points,
            sd.mean_abs,
            sd.p95_abs,
            sd.ok_frac() * 100.0,
            sd.gouge_frac() * 100.0,
            sd.residual_frac() * 100.0,
            sd.max_gouge
        );
    }

    // ── 解析口径：设计面点到**已切削面**的距离（与体素 h 无关）──
    //
    // 上面那个"名义曲面"读的是体素场，空间分辨力 ≈ h：薄于一个体素的尖角会被
    // 整格判成空气，过切能报大好几倍。这里直接问"刀具扫掠体的并集离这一点多远"，
    // 只受浮点影响。两个口径并排，差值就是判据本身的分辨力误差。
    let dsw = cncsim_sim::compare::sweep_deviation(&out.cut_segs, &table, &pts, 1.0);
    let ss = cncsim_sim::compare::sweep_stats(&dsw, 0.15, 1.0);
    println!(
        "解析口径（扫掠体，与 h 无关）：{} 点 / 平均 |d| {:.3} / P95 {:.3} / 合格(±0.15) {:.1}% / 过切 {:.1}% / 残料 {:.1}% / 最大过切 {:+.2}",
        ss.points,
        ss.mean_abs,
        ss.p95_abs,
        ss.ok_frac() * 100.0,
        ss.gouge_frac() * 100.0,
        ss.residual_frac() * 100.0,
        ss.max_gouge
    );
    // ── 过切归因（解析口径）：每个过切点是被**哪把刀**扫过的 ──
    //
    // 体素口径那套归因要跑完整仿真；这里只吃 `cut_segs`，所以 `--dry` 也能出。
    // 清根道是否啃到悬伸、哪一道该改，全靠这张表。
    {
        let idx: Vec<usize> = dsw
            .iter()
            .enumerate()
            .filter(|(_, d)| d.is_finite() && **d < -0.15)
            .map(|(i, _)| i)
            .collect();
        if !idx.is_empty() {
            let sub: Vec<cncsim_math::DVec3> = idx.iter().map(|i| pts[*i]).collect();
            let hits = cncsim_sim::compare::attribute_points(&out.cut_segs, &table, &sub, 0.05);
            let mut cnt: std::collections::HashMap<u16, (usize, f64)> =
                std::collections::HashMap::new();
            for h in hits.iter().flatten() {
                let e = cnt.entry(h.tool).or_insert((0, 0.0));
                e.0 += 1;
                e.1 = e.1.max(h.depth);
            }
            let mut rows: Vec<_> = cnt.into_iter().collect();
            rows.sort_by_key(|a| std::cmp::Reverse(a.1.0));
            let top: Vec<String> = rows
                .iter()
                .take(4)
                .map(|(t, (n, d))| {
                    let p = ((*t as usize).saturating_sub(1)) / 2;
                    format!("T{t}(第{}道,{n}点,{d:.2})", p + 1)
                })
                .collect();
            println!(
                "  过切归因：{} 点 >0.15 → 命中扫掠体 {} / 没命中 {}；{}",
                idx.len(),
                hits.iter().flatten().count(),
                idx.len() - hits.iter().flatten().count(),
                top.join(" ")
            );
        }
    }

    // 最深过切点**是谁切的**：只对这一个点做归因（单点，代价可忽略）
    if let Some((i, _)) = dsw
        .iter()
        .enumerate()
        .filter(|(_, d)| d.is_finite())
        .min_by(|a, b| a.1.total_cmp(b.1))
        && dsw[i] < -0.15
    {
        let one = [pts[i]];
        let hit = cncsim_sim::compare::attribute_points(&out.cut_segs, &table, &one, 0.05);
        if let Some(h) = hit[0] {
            println!(
                "  └ 最深过切 ({:.2},{:.2},{:.2})：T{} 第 {} 行 / {:.2} mm",
                pts[i].x,
                pts[i].y,
                pts[i].z,
                h.tool,
                h.line,
                h.depth
            );
        }
        // `CNCSIM_GOUGE=x,y,z`：把它周围所有"负距离"的段全列出来，看是哪一段、
        // 哪一个姿态啃的。查深层过切时比猜快得多。
        if let Ok(v) = std::env::var("CNCSIM_GOUGE") {
            let c: Vec<f64> = v.split(',').filter_map(|s| s.trim().parse().ok()).collect();
            if c.len() == 3 {
                let q = cncsim_math::DVec3::new(c[0], c[1], c[2]);
                let mut hits: Vec<(f64, usize)> = Vec::new();
                for (k, s) in out.cut_segs.iter().enumerate() {
                    let prof = &table.by_number(s.tool).map_or(table.first(), |t| t).cutter;
                    let dd = cncsim_voxel::sweep::sd_sweep(q, &s.from, &s.to, prof);
                    if dd < 0.2 {
                        hits.push((dd, k));
                    }
                }
                hits.sort_by(|a, b| a.0.total_cmp(&b.0));
                for (dd, k) in hits.iter().take(6) {
                    let s = &out.cut_segs[*k];
                    println!(
                        "[gouge] d={dd:.3} T{} D{:.2} 行{} · tip ({:.3},{:.3},{:.3})→({:.3},{:.3},{:.3}) · axis ({:.3},{:.3},{:.3})→({:.3},{:.3},{:.3})",
                        s.tool,
                        table.by_number(s.tool).map_or(0.0, |t| t.diameter),
                        s.line,
                        s.from.tip.x,
                        s.from.tip.y,
                        s.from.tip.z,
                        s.to.tip.x,
                        s.to.tip.y,
                        s.to.tip.z,
                        s.from.axis.x,
                        s.from.axis.y,
                        s.from.axis.z,
                        s.to.axis.x,
                        s.to.axis.y,
                        s.to.axis.z
                    );
                }
            }
        }
    }
    println!(
        "  └ 残料里 {} 点比 1.0 mm 还远（这 {} 道工序从没接近过它们）{}",
        ss.far,
        passes.len(),
        if dry {
            String::new()
        } else {
            format!(" / 体素口径最大过切 {:+.2}", sd.max_gouge)
        }
    );

    // ── 残料分布：厚度分档 + 空间热区 ──
    //
    // "残料 14.4%" 只说明有多少点没切到，不说明是**什么**：0.2 mm 的刀花、
    // 0.4 mm 的凸棱圆角、还是 2 mm 根本没去过的区域，处置完全不同。
    // 按厚度分档 + 按 4 mm 分块，能把这三类分开。
    {
        // 分档上界：(0.15,0.25] / (0.25,0.5] / (0.5,1] / >1
        let bands = [0.25f64, 0.5, 1.0];
        let mut hist = [0usize; 4];
        let mut cell: std::collections::HashMap<[i32; 3], (usize, f64, f64, cncsim_math::DVec3, cncsim_math::DVec3)> =
            std::collections::HashMap::new();
        const SZ: f64 = 4.0;
        for (p, d) in pts.iter().zip(&dsw) {
            let dd = if d.is_finite() { *d } else { 1.0e9 };
            if dd <= 0.15 {
                continue;
            }
            let k = bands.iter().position(|&b| dd <= b).unwrap_or(3);
            hist[k] += 1;
            if dd > 1.0e8 {
                continue; // 刀具从没接近过：不进热区（它们没有空间中心）
            }
            let key = [
                (p.x / SZ).floor() as i32,
                (p.y / SZ).floor() as i32,
                (p.z / SZ).floor() as i32,
            ];
            let e = cell.entry(key).or_insert((0, 0.0, 0.0, *p, *p));
            e.0 += 1;
            e.1 += dd;
            e.2 = e.2.max(dd);
            e.3 = cncsim_math::DVec3::new(e.3.x.min(p.x), e.3.y.min(p.y), e.3.z.min(p.z));
            e.4 = cncsim_math::DVec3::new(e.4.x.max(p.x), e.4.y.max(p.y), e.4.z.max(p.z));
        }
        println!(
            "  残料厚度分档：0.15–0.25 {} / 0.25–0.5 {} / 0.5–1 {} / >1 {}（其中刀具没接近过 {}）",
            hist[0], hist[1], hist[2], hist[3], ss.far
        );
        let mut rows: Vec<_> = cell.into_iter().collect();
        rows.sort_by_key(|a| std::cmp::Reverse(a.1.0));
        for (_, (n, sum, max, lo, hi)) in rows.iter().take(4) {
            println!(
                "  ├ 热区 ({:.1},{:.1},{:.1})..({:.1},{:.1},{:.1})：{n} 点 / 平均 {:.2} / 最深 {max:.2}",
                lo.x,
                lo.y,
                lo.z,
                hi.x,
                hi.y,
                hi.z,
                sum / *n as f64
            );
        }
    }

    // ── 可达 × 偏差 交叉表：把"够不到"和"没切好"彻底分开 ──
    //
    // 不分的话只有一个笼统的"合格 81.6%"，看不出下一步该改刀路还是改工艺。
    // 分开之后两边的结论是定性的：可达面上不合格 → 刀路/刀具问题；
    // 不可达面上残料 → 只能翻面或换工艺，别再调步距。
    {
        let pts: Vec<cncsim_math::DVec3> = vis.points.clone();
        let dsw2 = cncsim_sim::compare::sweep_deviation(&out.cut_segs, &table, &pts, 1.0);
        let dev2 = if dry {
            Vec::new()
        } else {
            cncsim_sim::surface_dev::signed_deviation(&out.grid, &pts)
        };
        // (点数, 合格, 过切, 残料)
        let mut r = (0usize, 0usize, 0usize, 0usize);
        let mut u = (0usize, 0usize, 0usize, 0usize);
        let mut ra = (0usize, 0usize, 0usize, 0usize);
        let mut ua = (0usize, 0usize, 0usize, 0usize);
        for (i, a) in dsw2.iter().enumerate() {
            let reachable = vis.masks[i] & final_mask != 0;
            // 解析口径：+∞ = 刀具从没接近过 → 残料
            let a = *a;
            let slot = if !a.is_finite() || a > 0.15 {
                3
            } else if a < -0.15 {
                2
            } else {
                1
            };
            let t = if reachable { &mut ra } else { &mut ua };
            t.0 += 1;
            match slot {
                1 => t.1 += 1,
                2 => t.2 += 1,
                _ => t.3 += 1,
            }
        }
        for (i, d) in dev2.iter().enumerate() {
            let reachable = vis.masks[i] & final_mask != 0;
            let d = *d;
            let slot = if d.abs() <= 0.15 {
                1
            } else if d > 0.15 {
                2
            } else {
                3
            };
            let t = if reachable { &mut r } else { &mut u };
            t.0 += 1;
            match slot {
                1 => t.1 += 1,
                2 => t.2 += 1,
                _ => t.3 += 1,
            }
        }
        let pct = |n: usize, t: usize| if t == 0 { 0.0 } else { n as f64 / t as f64 * 100.0 };
        if !dry {
            println!(
                "  可达面 {}：体素口径 合格 {:.1}% / 过切 {:.1}% / 残料 {:.1}%",
                r.0,
                pct(r.1, r.0),
                pct(r.2, r.0),
                pct(r.3, r.0)
            );
        }
        println!(
            "  可达面 {}：解析口径 合格 {:.1}% / 过切 {:.1}% / 残料 {:.1}%",
            ra.0,
            pct(ra.1, ra.0),
            pct(ra.2, ra.0),
            pct(ra.3, ra.0)
        );
        // 可达面上的残料再分一次"刀没接近过（>1mm）"和"切了但没切够"：
        // 前者是覆盖率问题（工序没走到），后者是刀径/圆角问题（走也切不到）。
        let (mut far_r, mut near_r) = (0usize, 0usize);
        for (i, a) in dsw2.iter().enumerate() {
            if vis.masks[i] & final_mask == 0 {
                continue;
            }
            if !a.is_finite() || *a > 1.0 {
                far_r += 1;
            } else if *a > 0.15 {
                near_r += 1;
            }
        }
        println!(
            "  └ 其中 >1 mm {far_r} 点（工序没走到）/ 0.15–1 mm {near_r} 点（刀走过了但切不到：刀径/圆角）"
        );
        if !dry {
            println!(
                "  不可达面 {}：体素口径 合格 {:.1}% / 过切 {:.1}% / 残料 {:.1}%（只能翻面或换工艺）",
                u.0,
                pct(u.1, u.0),
                pct(u.2, u.0),
                pct(u.3, u.0)
            );
        }
        println!(
            "  不可达面 {}：解析口径 合格 {:.1}% / 过切 {:.1}% / 残料 {:.1}%",
            ua.0,
            pct(ua.1, ua.0),
            pct(ua.2, ua.0),
            pct(ua.3, ua.0)
        );
    }

    // ── 过切归因：体素口径 × 解析扫掠体 ──
    //
    // 体素口径读的是"材料还剩什么"，空间分辨力 ≈ h：比一个体素还薄的尖角会被
    // 整格判成空气，读出来的过切可以比真实值大好几倍。解析口径只问"刀有没有
    // 真扫过这一点"。两个数一起看才知道下一步该改刀路还是改判据。
    if !dry {
        let hits = cncsim_sim::compare::attribute_points(&out.cut_segs, &table, &pts, 0.05);
        let mut by_tool: std::collections::HashMap<u16, (usize, f64)> =
            std::collections::HashMap::new();
        let (mut n_vox, mut n_hit, mut n_deep) = (0usize, 0usize, 0usize);
        let (mut worst, mut worst_i) = (0.0f64, usize::MAX);
        for (i, d) in dev.iter().enumerate() {
            if *d <= 0.15 {
                continue;
            }
            n_vox += 1;
            if *d > worst {
                worst = *d;
                worst_i = i;
            }
            if let Some(h) = hits[i] {
                n_hit += 1;
                if h.depth > 0.15 {
                    n_deep += 1;
                }
                let e = by_tool.entry(h.tool).or_insert((0, 0.0));
                e.0 += 1;
                e.1 = e.1.max(h.depth);
            }
        }
        let pct = |n: usize| if n_vox == 0 { 0.0 } else { n as f64 / n_vox as f64 * 100.0 };
        println!(
            "过切归因：体素 >0.15 的 {n_vox} 点 → 扫掠体命中 {n_hit}（{:.0}%）/ 未命中 {}（{:.0}%）",
            pct(n_hit),
            n_vox - n_hit,
            pct(n_vox - n_hit)
        );
        let mut rows: Vec<(u16, usize, f64)> =
            by_tool.into_iter().map(|(t, (c, d))| (t, c, d)).collect();
        rows.sort_by_key(|a| std::cmp::Reverse(a.1));
        for (t, n, d) in rows.iter().take(4) {
            let p = ((*t as usize).saturating_sub(1)) / 2;
            let dir = passes.get(p).map(|q| q.dir).unwrap_or(cncsim_math::DVec3::Z);
            println!(
                "  ├ T{t}（第 {} 道 {}({:.2},{:.2},{:.2}) 点 {n} / 最深 {d:.2}）",
                p + 1,
                if t % 2 == 1 { "粗" } else { "精" },
                dir.x,
                dir.y,
                dir.z
            );
        }
        if n_vox > n_hit {
            println!(
                "  ├ 未命中 {} 点：没有任何一段刀具扫过——特征是**薄于 h={h:.2} 的尖角/薄壁**，\
                 体素把整格判成空气。这是判据的分辨力问题，不是刀路问题",
                n_vox - n_hit
            );
        }
        println!(
            "  └ 深度口径：体素最大 {:.2} mm vs 解析最大 {:.2} mm（其中解析也 >0.15 的 {n_deep} 点）",
            worst.max(sd.max_gouge),
            r.gouge_max_depth_mm
        );
        if worst_i < hits.len()
            && let Some(h) = hits[worst_i]
        {
            println!(
                "  └ 最深点 ({:.2},{:.2},{:.2})：T{} 第 {} 行，解析侵入 {:.3} mm（{}）",
                pts[worst_i].x,
                pts[worst_i].y,
                pts[worst_i].z,
                h.tool,
                h.line,
                h.depth,
                if h.depth > 0.15 { "真过切" } else { "体素高估" }
            );
        }
    }

    // ── 体积对账：解析扫掠体 vs 体素去料 ──
    //
    // 两个口径各说各话时（§47.1：解析 97.8%、体素 F1 0.868），先问一句"料到底
    // 去了多少"。解析这边把毛坯按格子采样、逐点问"在不在扫掠体并集里"，
    // 得到的是**与体素 h 无关**的去除体积；体素那边是积分出来的。
    // 两个数对得上 → 切削本身没问题，差在网格提取；对不上 → 仿真或段表有问题。
    if args_flag("--vcheck") {
        let step = (h * 1.5).max(0.6);
        let n = |v: f64| (v / step).ceil().max(1.0) as usize;
        let (nx, ny, nz) = (n(stock.size().x), n(stock.size().y), n(stock.size().z));
        let mut pts = Vec::with_capacity(nx * ny * nz);
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    pts.push(cncsim_math::DVec3::new(
                        stock.min.x + (i as f64 + 0.5) * step,
                        stock.min.y + (j as f64 + 0.5) * step,
                        stock.min.z + (k as f64 + 0.5) * step,
                    ));
                }
            }
        }
        // 先验一件事：**设计场的零点在不在网格上**。设计场是窄带 i8 场，
        // 如果它的零面系统性偏出去 0.1 mm，那么任何"切到位"的刀路都会在体素
        // 口径里显示成"切多了"——这会正好造出 §47.1 那个矛盾。
        let sd: Vec<f64> = samples.iter().map(|(p, _, _)| design.sdf(*p)).collect();
        let mean = sd.iter().sum::<f64>() / sd.len().max(1) as f64;
        let maxabs = sd.iter().fold(0.0f64, |a, v| a.max(v.abs()));
        let out_of_band = sd.iter().filter(|v| v.abs() >= 0.29).count();
        println!(
            "设计场零点：{} 个网格点 → 平均 {:+.4} mm / 最大 |{:.4}| / 超出窄带 {} 点",
            sd.len(),
            mean,
            maxabs,
            out_of_band
        );
        let d = cncsim_sim::compare::sweep_deviation(&out.cut_segs, &table, &pts, 1.0);
        let inside = d.iter().filter(|v| **v < 0.0).count();
        let va = inside as f64 * step * step * step;
        let vb = out.report.removed_volume_mm3;
        println!(
            "体积对账（{:.1} mm 格 / {} 点）：解析扫掠体 {:.1} mm³ / 体素去料 {:.1} mm³ / 差 {:+.1}%",
            step,
            pts.len(),
            va,
            vb,
            (va - vb) / vb.max(1e-9) * 100.0
        );
    }

    // ── 相似度：Hausdorff / Chamfer / F1 / IoU / 法向一致 ──
    //
    // 前面的"名义曲面偏差"是**单向**的（设计面 → 实际面），它看不见"落在设计面
    // 之外很远的残料"；也没有朝向信息。这一节补上双向与体积口径：
    //   Hausdorff = 最坏点（装配/干涉死在这里）
    //   Chamfer   = 双向平均（整体做像了多少）
    //   F1@tol    = 面吻合程度（P 从实际面出发、R 从设计面出发）
    //   bias      = 有符号平均位移（正 = 过切、负 = 残料）
    //   IoU/Dice  = 体积吻合（与采样密度无关）
    //   法向一致  = 位置对但朝向错（斜面、台阶圆角）
    if !dry {
        let mut lod = 1;
        let mut rmesh = out.grid.extract_surface_lod(lod);
        while rmesh.indices.len() / 3 > 600_000 && lod < 3 {
            lod += 1;
            rmesh = out.grid.extract_surface_lod(lod);
        }
        let sim = cncsim_sim::similarity::compare(
            &design,
            &samples,
            &out.grid,
            domain,
            &rmesh,
            0.15,
        );
        println!(
            "相似度（LOD{lod} / {} 顶点）：Hausdorff {:.3}（正向 {:.3} / 反向 {:.3}） / Chamfer {:.3} / F1@0.15 {:.3}（P {:.3} / R {:.3}）",
            sim.actual_points, sim.hausdorff, sim.design_max, sim.actual_max, sim.chamfer,
            sim.f_score, sim.precision, sim.recall
        );
        println!(
            "  ├ IoU {:.4} / Dice {:.4} / 法向一致 {:.3} / 偏置 {:+.3} mm → {}（体素差：切多 {} / 没切到 {}）",
            sim.iou,
            sim.dice,
            sim.normal_consistency,
            sim.bias_mm,
            sim.verdict(),
            sim.only_design,
            sim.only_actual
        );
        if sim.design_saturated + sim.actual_saturated > 0 {
            println!(
                "  └ 距离被窄带钳制：正向 {} 点 / 反向 {} 点 —— Hausdorff 只是**下界**",
                sim.design_saturated, sim.actual_saturated
            );
        }
    }

    // ── 特征级形位公差（ISO 1101 最小区域法）──
    //
    // 上面的相似度是"整件一个数"。验收真正要的是"这个面平不平、这个孔圆不圆"：
    // 平面度 / 圆柱度 = 两张平行平面（两个同轴圆柱）把实际表面夹住的最小间距。
    // 基准本身要优化（最小区域），拿最小二乘当基准会高估。
    //
    // 表面点从设计面采样沿**实际场梯度**投影到实际表面（q = p + d·∇d/|∇d|），
    // 是一阶精确投影，不需要最近邻搜索。
    if !dry {
        let rep = cncsim_sim::form::analyze(
            &samples,
            &out.grid,
            &cncsim_sim::form::FormConfig::default(),
        );
        if rep.features.is_empty() {
            println!("形位：没有面积足够的特征（min_area 20 mm²）");
        } else {
            println!(
                "形位（最小区域法）：{} 个特征 / 自由曲面占 {:.1}% 面积 / 丢弃小分片 {}",
                rep.features.len(),
                rep.freeform_frac * 100.0,
                rep.skipped
            );
            for f in rep.features.iter().take(4) {
                if f.kind == cncsim_sim::form::Kind::Freeform {
                    println!(
                        "  ├ 自由曲面 {:.0} mm²：偏差 平均 {:+.3} / 过切 {:.3} / 残料 {:.3} / 法向一致 {:.3}（不评形位）",
                        f.area_mm2, f.mean_dev, f.max_gouge, f.max_residual, f.normal_tightness
                    );
                } else {
                    println!(
                        "  ├ {} {:.0} mm²：{} {:.3} mm（LSQ {:.3}）/ 偏差 平均 {:+.3} / 法向一致 {:.3}",
                        f.kind.name(),
                        f.area_mm2,
                        f.kind.form_name(),
                        f.form_mz,
                        f.form_lsq,
                        f.mean_dev,
                        f.normal_tightness
                    );
                }
            }
        }
    }

    if dry {
        return;
    }
    if let Some(path) = export_path_from_args() {
        // 自动降 LOD：h 小的时候 LOD 1 也能到几十万三角形，预览面板会卡
        let mut lod = 1;
        let mut mesh = out.grid.extract_surface_lod(lod);
        while mesh.indices.len() / 3 > 400_000 && lod < 4 {
            lod += 1;
            mesh = out.grid.extract_surface_lod(lod);
        }
        let title = format!(
            "多方向 CAM：{}",
            model.file_name().unwrap_or_default().to_string_lossy()
        );
        match cncsim_render::preview::write_html(&path, &mesh, &title) {
            Ok(s) => println!(
                "预览：{}（{} 顶点 / {} 三角形 / {} MB）",
                path.display(),
                s.vertices,
                s.triangles,
                s.bytes / (1 << 20)
            ),
            Err(e) => eprintln!("写预览失败：{e}"),
        }
    }
}

/// `--cam <model.stl> --rotary`：A/C 回转 + RTCP 的外轮廓加工。
///
/// 和 3 轴高度场是两条工艺：这里把工件转起来，刀轴沿外法向，程序用
/// `G43.4 H1` 打开刀尖跟随。对比的是**外轮廓半径偏差**（θ×z），
/// 不是顶视高度差——内腔不在这道工序里。
fn run_cam_rotary(model: &std::path::Path, tris: &[cncsim_cam::Tri], stock: cncsim_cam::Stock) {
    use cncsim_cam::rotary::{RotaryJob, generate_rotary};
    use cncsim_math::DVec2;

    let short = stock.size().x.min(stock.size().y);
    let center = DVec2::new(
        (stock.min.x + stock.max.x) * 0.5,
        (stock.min.y + stock.max.y) * 0.5,
    );

    // ── 刀具按**这个模型自己的曲率**选，不写死 ──
    //
    // 尖球半径决定"能进多窄的谷"：R 大于局部凹曲率半径就会在谷口架桥、把谷底
    // 整段削掉。`suggest_tip_radius` 取面积加权 97 分位的凹曲率半径，任何 STL 通用。
    // 粗加工用平头（每层切出平面，去料率高），精加工用锥度球头（小尖球 + 粗颈）。
    //
    // `--auto` 打开"按模型自选刀/步距"（实验性）；默认用**已验证**的 Ø2 + 1.25°/0.6，
    // 因为自适应路径在花瓶上还没跑通（同样代码换成 Ø1.7 会整件过切，原因未查明）。
    let auto = args_flag("--auto");
    let tip_r = if auto {
        cncsim_cam::rotary::suggest_tip_radius(tris, center)
    } else {
        1.0
    };
    // 实际刀径还要受"件不能太小"约束；后面所有推导都用**实际**半径，不用建议值
    let d_fin = (2.0 * tip_r).min(short / 20.0).max(0.4);
    let tip_r = d_fin * 0.5;
    let d_rough = (d_fin * 8.0).min(short / 4.0).max(2.0);
    // 步距由**弦高预算**反推：球头沿弦走的误差 ≈ p²/(8R)，取 p = √(8R·0.08)。
    let p_mm = (8.0 * tip_r * 0.08).sqrt().max(0.15);
    let r_max = (short * 0.5).max(5.0);
    let step_deg = if auto {
        (p_mm / r_max).to_degrees().clamp(0.3, 6.0)
    } else {
        1.25
    };
    let step_z = if auto { (p_mm * 0.5).clamp(0.15, 2.0) } else { 0.6 };
    println!(
        "选刀：尖球 R{tip_r:.2}（Ø{d_fin:.1}）/ 粗 Ø{d_rough:.1} / 步距 {step_deg:.2}° × {step_z:.2} mm"
    );
    let rough_tool = cncsim_cam::ToolSpec::flat(1, d_rough, 20.0).with_shank(shank_d());
    // 颈长 2.5×D。**不要**轻易缩短：改成 1.2×D 后同一份 G 代码切出来是一根
    // 圆柱（整窗全空气/整窗全料交替），说明"短颈"在这套扫掠里不是简单收益。
    let tool = cncsim_cam::ToolSpec::tapered(2, d_fin, 10.0, d_fin * 2.5).with_shank(shank_d());
    let job = RotaryJob {
        mesh: tris,
        stock,
        center,
        tool: tool.clone(),
        step_deg,
        step_z_mm: step_z,
        feed_mm_min: 1200.0,
        plunge_mm_min: 300.0,
        spindle_rpm: 10_000,
        a_limit_deg: cncsim_cam::rotary::A_LIMIT_DEG,
        // 刀轴朝 +Z 翘 35°：球心仍相切（球心恒在 p+R·n），只让锥颈离开相邻的脊。
        // 12° 实测没用——A≈90° 时锥颈仍是"横着捅"，要翘到能抬离凹槽才行。
        tilt_deg: 35.0,
        // `--no-rough`：只跑精加工，用来隔离"过切是粗加工留下的还是精加工造成的"
        roughing: if args_flag("--no-rough") {
            None
        } else {
            Some(cncsim_cam::rotary::Roughing {
                tool: rough_tool.clone(),
                step_mm: d_rough * 0.6,
                pitch_mm: d_rough * 0.6,
                step_deg: 10.0,
                allowance_mm: 0.5,
            })
        },
    };
    let t0 = std::time::Instant::now();
    let prog = match generate_rotary(&job) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("CAM（回转）生成失败：{e}");
            std::process::exit(1);
        }
    };
    println!(
        "回转刀路：粗 {} 点 + 精 {} 点 / 轮廓 {}×{} / 切 {:.0} mm（{:.2} s）",
        prog.rough_moves,
        prog.moves,
        prog.profile.n_theta,
        prog.profile.n_z,
        prog.cut_mm,
        t0.elapsed().as_secs_f64()
    );
    println!(
        "轮廓 r {:.2}..{:.2} mm / 需要最大 A {:.1}° / 超程限位 {} 点（最大抬刀 {:.2} mm）",
        prog.profile.min_r(),
        prog.profile.max_r(),
        prog.max_a_deg,
        prog.clamped,
        prog.max_lift_mm
    );
    let nc = gcode_path_from_args().unwrap_or_else(|| model.with_extension("rotary.nc"));
    if let Err(e) = std::fs::write(&nc, &prog.gcode) {
        eprintln!("写 G 代码失败：{e}");
    } else {
        println!("G 代码：{}", nc.display());
    }

    let ic = cncsim_gcode::InterpConfig {
        tool_radii: vec![0.0, d_rough * 0.5, tool.radius()],
        // 回转 CAM 用 Head 约定生成 A/C（刀轴绕工件转）。双转台机床装夹等价，
        // 仿真只消费刀尖/刀轴，不在这里做机床逆解。
        rotary: cncsim_gcode::RotaryConvention::Head,
        ..Default::default()
    };
    let program = cncsim_gcode::interpret(&prog.gcode, &ic);
    if program.has_errors() {
        eprintln!("解释器报错：{:?}", program.issues);
        std::process::exit(1);
    }
    let rtcp_blocks = program.blocks.iter().filter(|b| b.rtcp).count();
    println!(
        "解释：{} 块 / RTCP 段 {} / {} 条 issue",
        program.len(),
        rtcp_blocks,
        program.issues.len()
    );

    // 体素边长：跟着**刀的半径**走。h=1 mm 配 Ø2 刀意味着球半径只占 1 个体素，
    // 网格根本表达不出球面（实测偏差 ~0.3 mm，和 h 同量级）。准则 h ≤ R/2；
    // 再用体素总数上限兜住内存/时间。
    let vol = stock.size().x * stock.size().y * stock.size().z;
    let h_budget = (vol / 4.0e7).cbrt().max(0.25);
    let h = h_from_args().unwrap_or(if auto {
        (tip_r * 0.5).max(h_budget).min(1.0)
    } else {
        (h_budget).clamp(0.5, 1.0)
    });
    println!("网格：h = {h:.2} mm（刀尖 R{tip_r:.2}，体素上限 4×10⁷）");
    let size = stock.size();
    let domain = [
        (size.x / h).round().max(1.0) as i32,
        (size.y / h).round().max(1.0) as i32,
        (size.z / h).round().max(1.0) as i32,
    ];
    let grid = cncsim_voxel::GridConfig {
        h,
        band_voxels: 4,
        value_type: cncsim_voxel::ValueType::I8,
    };
    let cfg = cncsim_sim::run::RunConfig {
        grid,
        stock_origin: stock.min,
        stock_size: domain,
        design: None, // 这道工序只做外轮廓，内腔不在验收范围
        collide: false,
        creator_sub: 4,
        colorize: args_flag("--color"),
        // 切下来还挂着的碎块按"掉成切屑"丢掉，否则预览里会浮着一层小方块
        drop_chips: true,
        ..Default::default()
    };
    // 刀表必须与刀路**同一把刀**：粗加工/精加工的母线、刃长稍有出入，
    // 仿真就会按另一把刀去扫，量出来的偏差全是假的。
    let tools = cncsim_tool::ToolTable::new(vec![
        rough_tool.to_tool_def(),
        tool.to_tool_def(),
    ]);
    let t0 = std::time::Instant::now();
    let out = cncsim_sim::run::run_tools(&program, &tools, &cfg);
    println!(
        "仿真：{:.1} s / {} 块 / {} 段 / 去料 {:.1} mm³ / 丢碎屑 {}",
        t0.elapsed().as_secs_f64(),
        out.stats.blocks,
        out.stats.cut_segments,
        out.report.removed_volume_mm3,
        out.report.chip_voxels
    );

    // ── 外轮廓半径偏差 ──
    //
    // 沿半径从外向内找 SDF 由正（空气）变负（材料）的零点，再二分到 0.01 mm。
    // 早先版本直接判"体素值 < 0"，那是把 1 mm 的体素格当尺子：面只能在格子边上，
    // 读数被量化到 ±0.5 mm 以上，刀路本身 0.05 mm 的偏差根本看不出来。
    // 窗口 ±3.5 mm 要小于窄带（4 体素 = 4 mm），否则读到的是钳制值。
    const WIN: f64 = 3.5;
    let sdf_at = |rr: f64, th: f64, z: f64| {
        out.grid.sdf(cncsim_math::DVec3::new(
            center.x + rr * th.cos(),
            center.y + rr * th.sin(),
            z,
        ))
    };
    let prof = &prog.profile;
    let (nth, nz) = (prof.n_theta, prof.n_z);
    let mut dz = vec![f32::NAN; nth * nz];
    let mut vals: Vec<f64> = Vec::new();
    let mut over = 0usize; // 超出窗口：残料 > WIN
    for iz in 0..nz {
        for it in 0..nth {
            let Some(rd) = prof.r_at(it, iz) else {
                continue;
            };
            let th = prof.theta(it);
            let z = prof.z(iz);
            // 粗扫 0.1 mm，找到第一个"空气 → 材料"的区间，再二分
            let steps = (2.0 * WIN / 0.1) as usize;
            let mut prev: Option<f64> = None;
            let mut found = None;
            for s in 0..=steps {
                let rr = rd + WIN - s as f64 * 0.1;
                if sdf_at(rr, th, z) < 0.0 {
                    if let Some(rp) = prev {
                        let (mut lo, mut hi) = (rr, rp); // lo 有料，hi 是空气
                        for _ in 0..12 {
                            let mid = 0.5 * (lo + hi);
                            if sdf_at(mid, th, z) < 0.0 {
                                lo = mid;
                            } else {
                                hi = mid;
                            }
                        }
                        found = Some(0.5 * (lo + hi));
                    }
                    break;
                }
                prev = Some(rr);
            }
            let dv = match found {
                Some(rm) => rm - rd,
                // 整个窗口都是空气 → 这里被掏空了；全是材料 → 根本没切到
                None if prev.is_some() => -WIN,
                None => WIN,
            };
            if dv >= WIN - 1e-9 {
                over += 1;
            }
            dz[iz * nth + it] = dv as f32;
            vals.push(dv);
        }
    }
    if vals.is_empty() {
        eprintln!("外轮廓采样为空——毛坯/网格坐标系可能不匹配");
        return;
    }
    vals.sort_by(f64::total_cmp);
    let n = vals.len() as f64;
    let mean = vals.iter().map(|v| v.abs()).sum::<f64>() / n;
    // P95 取 **|Δr| 的 95 分位**：直接拿带符号序列的分位数，读出来的是
    // "有 5% 比它更负"，跟"偏差多大"不是一回事。
    let mut abs: Vec<f64> = vals.iter().map(|v| v.abs()).collect();
    abs.sort_by(f64::total_cmp);
    let p95 = abs[((n * 0.95) as usize).min(abs.len() - 1)];
    let max_res = vals[vals.len() - 1];
    let max_gouge = vals[0];
    let within = vals.iter().filter(|v| v.abs() <= 0.5).count() as f64 / n;
    println!(
        "外轮廓 Δr：{} 点 / 平均 |Δr| {:.3} mm / P95 {:.3} / 残料 {:+.3} / 过切 {:+.3} / |Δr|≤0.5 {:.1}%",
        vals.len(),
        mean,
        p95,
        max_res,
        max_gouge,
        within * 100.0
    );
    println!(
        "  ├ 合格(<0.5) {:.1}%  余量(0.5~2) {:.1}%  未切到(>2) {:.1}%  过切(<-0.5) {:.1}%",
        within * 100.0,
        vals.iter().filter(|v| **v >= 0.5 && **v < 2.0).count() as f64 / n * 100.0,
        over as f64 / n * 100.0,
        vals.iter().filter(|v| **v <= -0.5).count() as f64 / n * 100.0
    );
    // ── 名义曲面逐点偏差：形状保真度的验收口径 ──
    //
    // 上面那个 Δr 是"沿半径量外轮廓"，对**轴向错位、局部小坑、端面/底面**是盲的：
    // 它只在 (θ,z) 格点上找最外层材料，格子之间的小坑和沿 z 的错位都可能读不出来。
    // 真正判"形状保持得像不像"，要按**设计面**采样、逐点量到实际毛坯面的有符号
    // 距离——这就是三坐标/全尺寸检测的做法。
    //
    // 只取**外轮廓面**上的样本：内腔本工序根本没加工，算进去只会得到一个
    // 恒等于"残料 100%"的假数字。内腔点数如实报出来，不藏。
    // 采样点带**面法向**：能不能加工，取决于面朝哪，不是半径。
    // 法向朝下（n·z < cos(A限位)）的是倒扣面，一次装夹从上面下来根本够不到，
    // 算进"切坏了"就是把工艺限制记成加工误差。
    let surf = cncsim_voxel::meshvol::surface_points_with_normals(tris, 0.8);
    let cos_lim = cncsim_cam::rotary::A_LIMIT_DEG.to_radians().cos();
    let mut outer: Vec<cncsim_math::DVec3> = Vec::with_capacity(surf.len());
    let mut reach: Vec<bool> = Vec::with_capacity(surf.len());
    let mut inner_pts = 0usize;
    let mut under_pts = 0usize;
    for (p, nrm) in &surf {
        let rr = (p.x - center.x).hypot(p.y - center.y);
        let th = (p.y - center.y).atan2(p.x - center.x).rem_euclid(std::f64::consts::TAU);
        let Some(rd) = prog.profile.radius(th, p.z) else {
            continue;
        };
        if rr < rd - 1.5 {
            inner_pts += 1; // 内腔：本工序不加工
            continue;
        }
        if nrm.z < cos_lim {
            under_pts += 1; // 倒扣：A 超行程，一次装夹够不到
            continue;
        }
        outer.push(*p);
        reach.push(true);
    }
    let _ = &reach;
    let dev = cncsim_sim::surface_dev::signed_deviation(&out.grid, &outer);
    let band = 4.0 * h;
    let sd = cncsim_sim::surface_dev::stats(&dev, 0.15, band);
    // ── 溯源：这块面到底是哪把刀切出来的 ──
    //
    // 沿半径向内走，找到第一个"材料"体素，读它的 creator（工具号 = creator/10）。
    // 过切点如果全是 1 号刀（粗加工），说明精加工根本没到过那里；
    // 全是 2 号刀，说明就是精加工自己啃的。
    let mut who_gouge = [0usize; 4];
    let mut who_ok = [0usize; 4];
    for (p, d) in outer.iter().zip(dev.iter()) {
        let th = (p.y - center.y).atan2(p.x - center.x);
        let (sn, cs) = th.sin_cos();
        let mut found = 0usize;
        let mut rr = (p.x - center.x).hypot(p.y - center.y) - 0.3;
        for _ in 0..24 {
            let q = cncsim_math::DVec3::new(
                center.x + rr * cs,
                center.y + rr * sn,
                p.z,
            );
            if q.x < stock.min.x || q.y < stock.min.y || q.z < stock.min.z {
                break;
            }
            let idx = [
                ((q.x - stock.min.x) / h).floor() as i32,
                ((q.y - stock.min.y) / h).floor() as i32,
                ((q.z - stock.min.z) / h).floor() as i32,
            ];
            if idx[0] < domain[0] && idx[1] < domain[1] && idx[2] < domain[2] {
                let v = out.grid.get(idx);
                if v < 0 {
                    let c = out.grid.creator_of_voxel(idx);
                    found = (c / 10) as usize;
                    break;
                }
            }
            rr -= 0.25;
        }
        if d.abs() <= 0.15 {
            who_ok[found.min(3)] += 1;
        } else if *d > 0.15 {
            who_gouge[found.min(3)] += 1;
        }
    }
    println!(
        "  ├ 溯源(过切点)：粗刀 {} / 精刀 {} / 无标记 {}",
        who_gouge[1], who_gouge[2], who_gouge[0]
    );
    println!(
        "  ├ 溯源(合格点)：粗刀 {} / 精刀 {} / 无标记 {}",
        who_ok[1], who_ok[2], who_ok[0]
    );
    // 按「这一点的外法向需要多大 A 角」分组：一次装夹够不到的区域会自己显形，
    // 比"整体合格率"有用得多——平均值会把"够不到的 40%"摊到"够得到的 60%"上。
    let bins = [30.0, 60.0, 90.0, 110.0, 180.0];
    let mut acc = vec![(0usize, 0.0f64, 0usize); bins.len()]; // (点数, Σ|d|, 合格数)
    for (p, d) in outer.iter().zip(dev.iter()) {
        let th = (p.y - center.y).atan2(p.x - center.x).rem_euclid(std::f64::consts::TAU);
        let a = prog
            .profile
            .normal(th, p.z)
            .z
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        let k = bins.iter().position(|&b| a < b).unwrap_or(bins.len() - 1);
        acc[k].0 += 1;
        acc[k].1 += d.abs();
        if d.abs() <= 0.15 {
            acc[k].2 += 1;
        }
    }
    println!(
        "名义曲面偏差：{} 点 / 平均 |d| {:.3} mm / P95 {:.3} / 合格(≤0.15) {:.1}% / 过切 {:.1}% / 残料 {:.1}%",
        sd.points,
        sd.mean_abs,
        sd.p95_abs,
        sd.ok_frac() * 100.0,
        sd.gouge_frac() * 100.0,
        sd.residual_frac() * 100.0
    );
    println!(
        "  ├ 最大过切 {:+.2} / 最大残料 {:.2} / 超窄带(≥{band:.0} mm) {} 点",
        sd.max_gouge, sd.max_residual, sd.saturated
    );
    println!(
        "  ├ 未纳入：内腔面 {inner_pts} 点 / 倒扣面（A>{:.0}°）{under_pts} 点",
        cncsim_cam::rotary::A_LIMIT_DEG
    );
    for (k, (c, sum, ok)) in acc.iter().enumerate() {
        if *c == 0 {
            continue;
        }
        let lo = if k == 0 { 0.0 } else { bins[k - 1] };
        println!(
            "  ├ A {lo:3.0}..{:<3.0}°：{c:7} 点 / 平均 |d| {:.3} / 合格 {:.1}%",
            bins[k],
            sum / *c as f64,
            *ok as f64 / *c as f64 * 100.0
        );
    }

    let stats = HeightDiff {
        columns: vals.len(),
        mean_abs: mean,
        p95,
        max_residual: max_res,
        max_gouge,
        within_0_1: within,
    };
    let diff_path = diff_path_from_args().unwrap_or_else(|| model.with_extension("rotary.diff.html"));
    let title = format!(
        "{} 外轮廓 Δr（θ×z）",
        model.file_name().unwrap_or_default().to_string_lossy()
    );
    match write_diff_html(&diff_path, &dz, nth as i32, nz as i32, &stats, &title) {
        Ok(()) => println!("偏差图：{}", diff_path.display()),
        Err(e) => eprintln!("写偏差图失败：{e}"),
    }

    if let Some(path) = export_path_from_args() {
        // 回转件的曲面是主特征，LOD 3（8 mm 粗格）会把波浪面抽成一块块疙瘩，
        // 所以从 LOD 1 起抽。但 h 变小后 LOD 1 也能到百万三角形（36 MB HTML），
        // 预览面板会卡——超过 40 万三角形自动再降一级。
        let mut lod = 1;
        let mut mesh = out.grid.extract_surface_lod(lod);
        while mesh.indices.len() / 3 > 400_000 && lod < 3 {
            lod += 1;
            mesh = out.grid.extract_surface_lod(lod);
        }
        if lod > 1 {
            println!("预览用 LOD {lod}（LOD 1 三角形过多）");
        }
        match cncsim_render::preview::write_html(&path, &mesh, "CAM rotary RTCP") {
            Ok(s) => println!(
                "预览：{}（{} 顶点 / {} 三角形 / {} MB）",
                path.display(),
                s.vertices,
                s.triangles,
                s.bytes / (1 << 20)
            ),
            Err(e) => eprintln!("写预览失败：{e}"),
        }
    }
}

fn rotary_from_args() -> bool {
    std::env::args().skip(1).any(|a| a == "--rotary")
}

/// `--no-rough`：跳过粗加工，只跑精加工（对照用：隔离过切来源）。
/// `--axis x,y,z`：**定轴 3+2**（把旋转轴锁到该方向，走普通三轴刀路）。
///
/// 这就是"立铣刀 + 定旋转轴三轴加工"：程序里 A/C 固定，只有 XYZ 在动；
/// 侧向加工时刀轴不再是世界 +Z，所以仍走 `G43.4`（刀尖跟随）。
fn axis_from_args() -> Option<cncsim_math::DVec3> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let raw = if let Some(v) = args.iter().position(|a| a == "--axis") {
        args.get(v + 1)?.clone()
    } else {
        // 也接受 `--axis=x,y,z`
        args.iter().find_map(|a| a.strip_prefix("--axis="))?.to_string()
    };
    let raw = raw.as_str();
    let p: Vec<f64> = raw
        .split(',')
        .filter_map(|s| s.trim().parse().ok())
        .collect();
    if p.len() != 3 {
        eprintln!("--axis 需要三个数，例如 --axis -1,0,0（当前给的是 {raw}）");
        std::process::exit(1);
    }
    let d = cncsim_math::DVec3::new(p[0], p[1], p[2]);
    if d.length() < 1e-9 {
        eprintln!("--axis 不能是零向量");
        std::process::exit(1);
    }
    Some(d.normalize())
}

/// `--slope <度>`：精加工允许的最大坡度（默认 45）。
///
/// 放宽它只多走路：drop-cutter 的刀心取刀盘内最高点，坡再陡也不会多切
/// （实测 45→70 残料一点没降、时间 3.4×，所以默认仍是 45）。
fn slope_deg() -> f64 {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--slope")
        .and_then(|(_, v)| v.parse().ok())
        .unwrap_or(45.0)
}

/// `--finish-step <mm>`：精加工行距（默认 0.4×刀径）。
fn surface_tilt_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--surface-tilt")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v >= 0.0)
}

fn surface_step_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--surface-step")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
}

fn surface_d_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--surface-d")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
}

fn surface_neck_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--surface-neck")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v >= 0.0)
}

fn surface_flute_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--surface-flute")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
}

/// 刀柄直径（mm）。默认 **6.0** —— 真实刀具统一 Ø6 h6 夹持，小刃径的刀经
/// 缩颈扩到 Ø6。`--shank-d 0` 恢复"等径柄"的旧行为（用于和历史数字对拍）。
fn shank_d() -> f64 {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--shank-d")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v >= 0.0)
        .unwrap_or(6.0)
}

fn rest_d_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--rest-d")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
}

fn finish_d_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--finish-d")
        .and_then(|(_, v)| v.parse::<f64>().ok())
        .filter(|v| *v > 0.0)
}

fn finish_step_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--finish-step")
        .and_then(|(_, v)| v.parse().ok())
}

/// `--tilt <度> --tilt-n <条>`：斜向精加工的倾角与条数（默认 54.7° × 4）。
///
/// 角度不受"必须 90° 的倍数"约束——只要 A 轴能摆到（≤ 机床行程）就是合法工序。
fn tilt_from_args() -> Option<(f64, usize)> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let take = |key: &str| -> Option<String> {
        let v = args.iter().position(|a| a == key)?;
        args.get(v + 1).cloned().or_else(|| {
            args.iter()
                .find_map(|a| a.strip_prefix(&format!("{key}=")).map(str::to_string))
        })
    };
    let t = take("--tilt")?;
    let deg: f64 = t.parse().ok()?;
    let n: usize = take("--tilt-n")
        .and_then(|v| v.parse().ok())
        .unwrap_or(4);
    Some((deg, n.max(1)))
}

/// `--h <mm>`：强制体素边长（默认按刀半径自动定）。
fn h_from_args() -> Option<f64> {
    std::env::args()
        .skip(1)
        .zip(std::env::args().skip(2))
        .find(|(a, _)| a == "--h")
        .and_then(|(_, v)| v.parse().ok())
}

fn args_flag(flag: &str) -> bool {
    std::env::args().skip(1).any(|a| a == flag)
}

fn cam_path_from_args() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--cam" {
            return args.next().map(Into::into);
        }
        if let Some(v) = a.strip_prefix("--cam=") {
            return Some(v.into());
        }
    }
    None
}

/// 装不装得下：`None` = 装得下；`Some(k)` = 等比缩小到 `k`（留 1% 余量）。
///
/// 只判两条硬约束（台面直径、刀尖高度）。"摆角与尺寸耦合"不在这里——
/// 那是工序选方向的事（`r ≤ L/tanA`），缩放解决不了。
fn shrink_for_envelope(
    r: f64,
    h: f64,
    env: &cncsim_config::WorkEnvelope,
) -> Option<f64> {
    let mut s = 1.0f64;
    if env.table_diameter > 0.0 {
        s = s.min(0.5 * env.table_diameter / r.max(1e-9));
    }
    if env.max_height > 0.0 {
        s = s.min(env.max_height / h.max(1e-9));
    }
    (s < 1.0).then(|| (s * 0.99).max(0.05))
}

/// 解释器配置：刀补半径按刀号、A/C 用 Head（双摆头）约定。
///
/// 必须和 `generate_plan` 用**同一份**刀表——半径差一点，仿真就按另一把刀扫。
fn interp_config_for(tools: &[cncsim_tool::ToolDef]) -> cncsim_gcode::InterpConfig {
    let mut radii = vec![0.0; 64];
    for t in tools {
        let k = t.number as usize;
        if k < radii.len() {
            radii[k] = t.diameter * 0.5;
        }
    }
    cncsim_gcode::InterpConfig {
        tool_radii: radii,
        rotary: cncsim_gcode::RotaryConvention::Head,
        ..Default::default()
    }
}

/// 毛坯装夹包络的两个分量：**(外接圆半径, 高度)**。
///
/// 半径取 XY 对角线的半长——工件绕 C 轴会扫出整个外接圆，台面装的是那个圆。
fn stock_envelope(s: &cncsim_cam::Stock) -> (f64, f64) {
    let size = s.size();
    (0.5 * (size.x * size.x + size.y * size.y).sqrt(), size.z)
}

fn cam_stock_from_args() -> Option<cncsim_math::DVec3> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let raw = if a == "--stock" {
            args.next()?
        } else if let Some(v) = a.strip_prefix("--stock=") {
            v.to_string()
        } else {
            continue;
        };
        let parts: Vec<f64> = raw
            .replace(['x', 'X', '*'], ",")
            .split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        if parts.len() == 3 {
            return Some(cncsim_math::DVec3::new(parts[0], parts[1], parts[2]));
        }
        eprintln!("--stock 需要三个数（x,y,z 或 xXyXz）：{raw}");
        std::process::exit(1);
    }
    None
}

fn diff_path_from_args() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--diff" {
            return args.next().map(Into::into);
        }
        if let Some(v) = a.strip_prefix("--diff=") {
            return Some(v.into());
        }
    }
    None
}

fn gcode_path_from_args() -> Option<std::path::PathBuf> {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--gcode" {
            return args.next().map(Into::into);
        }
        if let Some(v) = a.strip_prefix("--gcode=") {
            return Some(v.into());
        }
    }
    None
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
