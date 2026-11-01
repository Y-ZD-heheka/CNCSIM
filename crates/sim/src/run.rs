//! 仿真主循环：**G 代码 → 位姿 → 切削 + 碰撞 → 报告**。
//!
//! 这是把各个 crate 串起来的那一层。它自己**不含算法**：
//!
//! ```text
//! gcode::interpret  →  MotionProgram（已解析、已吸收偏置的位姿序列）
//!        ↓ 每块
//! motion::arc       →  圆弧离散成折线
//!        ↓ 每段
//! voxel::cut        →  刀具扫掠减材料
//! collide + stock   →  刀杆 vs 毛坯
//!        ↓
//! SimReport + episodes
//! ```
//!
//! ## 为什么"第一块"被跳过
//!
//! `gcode::Block::target` 只有**目标**位姿，没有起点。程序的第一个块通常是
//! `G0` 到起始点，起点是机床当前位姿——`MotionProgram` 不存它（那是机床状态）。
//! 所以主循环把第一块的 `target` 当作起点，第一段是零长度（切不到东西）。
//! 想精确复现第一条快移，得把初始位姿也传进来；等接机床配置时再说。
//!
//! ## 快移也查碰撞
//!
//! 真实撞机大多是 `G0` 撞夹具，不是切削时撞。所以 `Rapid` 不切材料，但**照样查碰撞**。

use crate::compare::{self, CutSegment, DEFAULT_DESIGN_STEP_MM, DEFAULT_GOUGE_TOL_MM, DesignModel};
use crate::stock::{check_shank, StockCollision};
use crate::{Alarm, Severity, SimReport, TimeRef};
use cncsim_collide::CollisionEpisode;
use cncsim_collide::DEFAULT_EPISODE_GAP;
use cncsim_collide::continuous::SurfaceBody;
use cncsim_gcode::MotionProgram;
use cncsim_math::DVec3;
use cncsim_motion::arc::{DEFAULT_ARC_SAGITTA, sample_arc};
use cncsim_motion::{MotionKind, Pose, slerp};
use cncsim_tool::ToolDef;
use cncsim_voxel::{GridConfig, SparseGrid};

/// 跑一次仿真需要的全部输入。
#[derive(Clone, Debug)]
pub struct RunConfig {
    /// 体素网格配置（`h` 与窄带宽度）
    pub grid: GridConfig,
    /// 毛坯最小角（世界坐标）
    pub stock_origin: DVec3,
    /// 毛坯尺寸（**体素数**，不是 mm）
    pub stock_size: [i32; 3],
    /// 是否做碰撞检查
    pub collide: bool,
    /// 刀杆表面采样间隙（mm）。0.5 → 检出下限 42 µm；0.2 → 6.7 µm 但点数是 6 倍
    pub shank_gap: f64,
    /// 碰撞 episode 的合并间隔（秒）
    pub episode_gap: f64,
    /// 圆弧离散的弦高上限（mm）
    pub arc_sagitta: f64,
    /// 快移速度（mm/min），只用于时间估算
    pub rapid_feed: f64,
    /// 设计成品（`Some` 时仿真结束做过切/残留比对）。`None` = 跳过比对。
    pub design: Option<DesignModel>,
    /// 机床拓扑（`Some` 时按"关节空间线性插补"口径算非线性误差）。
    /// `None` = 不算（报告里保持 0）。
    pub machine: Option<cncsim_kin::MachineTopology>,
    /// 设计面采样步距（解析过切测试，mm）
    pub design_step: f64,
    /// 过切判定阈值（mm）。小于它跟扫掠迭代噪声不可区分。
    pub gouge_tol: f64,
    /// 非线性误差每段采样数
    pub nonlinear_samples: usize,
    /// 溯源粒度：每片叶切成 `sub³` 个子块各存一个 creator（1 / 2 / 4 / 8）。
    ///
    /// 1 = 每叶一个（最省），8 = 每个体素一个（最锐）。默认 **4**：
    /// 实测最坏溢出 1→0.5→**0.1**→0.025 mm（sub = 1/2/4/8），
    /// 而内存是 1/8/64/512 B per 叶——4 是拐点，8 花 8 倍内存只换来 0.075 mm。
    /// 见 `crates/sim/tests/creator_granularity.rs`。
    pub creator_sub: u8,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            creator_sub: 4,
            grid: GridConfig::default(),
            stock_origin: DVec3::ZERO,
            // 20×20×20 mm，h=0.1
            stock_size: [200, 200, 200],
            collide: true,
            shank_gap: 0.5,
            episode_gap: DEFAULT_EPISODE_GAP,
            arc_sagitta: DEFAULT_ARC_SAGITTA,
            rapid_feed: 10_000.0,
            design: None,
            machine: None,
            design_step: DEFAULT_DESIGN_STEP_MM,
            gouge_tol: DEFAULT_GOUGE_TOL_MM,
            nonlinear_samples: 32,
        }
    }
}

/// 一次碰撞记录：episode 本体 + 首次触发的位置。
///
/// `CollisionEpisode` 只记时间区间；报告要能"跳到那一行"，所以这里补上块号与行号。
#[derive(Clone, Debug)]
pub struct CollisionRecord {
    pub episode: CollisionEpisode,
    /// 首次触发的块序号（`MotionProgram::blocks` 的下标）
    pub first_block: u32,
    /// 首次触发的源文件行号
    pub first_line: u32,
    /// 首次触发时的段内位置（0..1）——把时间轴拖到这一块之后还能再定位
    pub first_t: f64,
    /// 最近一次触发的块序号。**相邻块视为同一次事故**——一次下扎会横跨
    /// `G0` 和紧随其后的 `G1`，两块之间可能隔着 70 ms（超过 episode 的 50 ms 窗口），
    /// 但那是同一个动作。
    pub last_block: u32,
}

/// 主循环的量化结果（不进 `SimReport` 的那些）。
#[derive(Clone, Debug, Default)]
pub struct RunStats {
    pub blocks: usize,
    /// 实际切削的段数（含圆弧离散出来的段）
    pub cut_segments: usize,
    /// 快移段数
    pub rapid_segments: usize,
    pub cut_ms: f64,
    pub collide_ms: f64,
    pub volume_before: f64,
    pub volume_after: f64,
    /// 有多少块的进给是"每转"，时间只能按 mm/min 估（见模块文档）
    pub per_rev_feed_blocks: usize,
    /// 程序在哪里停下的（M0 / M2 / M30）。`None` = 跑到底。
    pub stopped_at: Option<(u32, cncsim_gcode::StopKind)>,
    /// 带停留的块数（G4 与固定循环的孔底暂停）
    pub dwell_blocks: usize,
    /// 设计模型比对的耗时（ms）。未启用比对时为 0。
    pub compare_ms: f64,
    /// 观察者取消了这次仿真。**结果只覆盖到取消时的块**——体积/报警反映部分状态。
    pub cancelled: bool,
}

/// 一条进度事件（观察者）。**只观察，不参与计算**——回调做什么都不影响结果，
/// 这是"后台算 + 前台播"的契约：同一份程序，带不带观察者得到逐位相同的结果。
#[derive(Clone, Copy, Debug)]
pub struct Progress {
    /// 即将开始的块（0 起；`blocks_total` 表示总量）
    pub block: u32,
    pub blocks_total: u32,
    /// 到目前为止的程序时间（秒）
    pub sim_time: f64,
    /// 当前块的起点刀尖（世界坐标）
    pub tip: DVec3,
    pub cut_segments: usize,
}

/// 一次仿真的全部产物。
pub struct RunOutput {
    pub report: SimReport,
    /// 切完的毛坯（调用方可拿去提取网格 / 显示）
    pub grid: SparseGrid<i8>,
    pub collisions: Vec<CollisionRecord>,
    pub stats: RunStats,
    /// 这一趟真正切削过的段（**已按圆弧弦高离散**），带起始时刻。
    ///
    /// 给"边回放边切"用：调用方拿着它就能从毛坯状态重放到任意时刻，
    /// 而不必自己再离散一次圆弧——那会把 `arc_sagitta` 抄成两份。
    pub cut_segs: Vec<CutSegment>,
}

/// 跑完整个程序。
pub fn run(program: &MotionProgram, tool: &ToolDef, cfg: &RunConfig) -> RunOutput {
    run_observed(program, tool, cfg, |_| true)
}

/// 与 [`run`] 相同，但每块开始前回调一次观察者。
///
/// 回调返回 `false` 取消仿真：循环停下，报告从**部分状态**定稿（体积/报警
/// 反映取消时为止的材料与碰撞），`stats.cancelled = true`，并追加一条
/// `CANCELLED` Info 报警。取消只发生在块边界——半切的块不清理，
/// 这是该块当前的诚实状态。
///
/// **确定性**：观察者只读。同一份输入，带观察者与不带观察者的输出逐位相同
/// （`actual_time_s` / `cut_ms` / `collide_ms` 这类墙钟除外）。
pub fn run_observed(
    program: &MotionProgram,
    tool: &ToolDef,
    cfg: &RunConfig,
    mut observer: impl FnMut(&Progress) -> bool,
) -> RunOutput {
    let t_wall = std::time::Instant::now();
    let mut stats = RunStats {
        blocks: program.len(),
        ..Default::default()
    };

    // ── 毛坯 ──
    let mut grid = SparseGrid::<i8>::new(cfg.grid, cfg.stock_origin);
    // 溯源粒度必须在建场之前定：`creator` 是与 `leaves` 平行的数组
    grid.set_creator_sub(cfg.creator_sub);
    grid.fill_box([0, 0, 0], cfg.stock_size, i8::MIN);
    // 折叠整叶同值：`fill_box` 会把内部写成 tile，但边界叶还是 Dense，
    // 不 prune 的话后面每次 `get` 都要多走一跳
    grid.prune();
    stats.volume_before = grid.material_volume();

    // ── 刀杆采样（碰撞用）──
    let body = cfg
        .collide
        .then(|| SurfaceBody::revolve(&tool.shank, cfg.shank_gap, cfg.shank_gap));

    // ── 逐块推进 ──
    let mut prev = program
        .blocks
        .first()
        .map(|b| b.target)
        .unwrap_or(Pose { tip: DVec3::ZERO, axis: DVec3::Z });

    // 设计比对用的切削段（含块号/行号/起始时刻，报警要能跳回源文件）
    let mut cut_segs: Vec<CutSegment> = Vec::new();

    // 非线性误差追踪。`nl_seed` 给逆解做多解连续：上一块的解当下一块的种子，
    // 解不出来就清空复位（下一块从零起步）。
    let mut nl_max = 0.0f64;
    let mut nl_block = 0u32;
    let mut nl_line = 0u32;
    let mut nl_pose = DVec3::ZERO;
    let mut nl_seed: Vec<f64> = Vec::new();

    let mut sim_time = 0.0f64;
    let mut collisions: Vec<CollisionRecord> = Vec::new();
    let mut cut_ms = 0.0;
    let mut collide_ms = 0.0;

    let mut stopped_at = None;
    let mut cancelled_at: Option<u32> = None;
    // 已经开始的"第几刀"。**只数切削块**（`G1`/`G2`/`G3`），
    // 快移、停延、换刀不占颜色索引——否则一刀一刀之间夹着的 `G0`
    // 会让渐变跳号（见 `cncsim_voxel::creator_of_cut`）。
    let mut cut_ordinal: u32 = 0;

    for (bi, block) in program.blocks.iter().enumerate() {
        // ── 溯源：这一刀切削出来的面盖上"第几刀"的调色板索引（ADR-005）──
        //
        // 盖在**刀**上而不是插补段上：一段圆弧会离散成几十段，按段着色会在同一刀
        // 内部变成噪点。索引函数与渲染层取刀刃颜色用的是同一个 `creator_of_cut`，
        // 所以"刀刃什么颜色、切出来的面就是什么颜色"是构造保证的。
        let is_cut = matches!(block.motion, MotionKind::Linear | MotionKind::Arc { .. });
        if is_cut {
            grid.set_creator(cncsim_voxel::creator_of_cut(cut_ordinal));
            cut_ordinal += 1;
        }

        // ── 进度观察 + 取消检查（块边界）──
        let progress = Progress {
            block: bi as u32,
            blocks_total: program.len() as u32,
            sim_time,
            tip: prev.tip,
            cut_segments: stats.cut_segments,
        };
        if !observer(&progress) {
            cancelled_at = Some(bi as u32);
            stats.cancelled = true;
            break;
        }

        // ── 程序停止 ──
        //
        // M0 / M2 / M30 都让仿真到此为止：M2/M30 之后没有代码；M0 是操作员要
        // 检查的地方，接着跑下去等于替人做主。M1（选择性停）默认不停——那是
        // 机床面板上的开关，仿真里不该猜。
        if let Some(s) = program.stops.iter().find(|s| s.block == bi)
            && !matches!(s.kind, cncsim_gcode::StopKind::OptionalStop)
        {
            stopped_at = Some((s.line_no, s.kind));
            break;
        }

        let to = block.target;
        // 与上面 `is_cut` 同一判据（切削块 = 一刀）
        let cutting = is_cut;

        // ⓪ 非线性误差（`docs/技术方案` §4.6）：这段程序若被控制器按
        // **关节空间线性插补**执行，刀尖会偏离 CAM 意图的直线多少。
        // 快移也算——`G0` 带摆角时刀尖画的弧最危险。旋转轴全程不动的块
        //（三轴、纯平移）在 `nonlinear_error` 里早退为 0。
        if let Some(topo) = &cfg.machine {
            let solved = cncsim_kin::inverse(topo, &prev, &nl_seed).and_then(|qs| {
                cncsim_kin::inverse(topo, &to, &qs).map(|qe| (qs, qe))
            });
            match solved {
                Some((qs, qe)) => {
                    let ne = cncsim_kin::nonlinear_error(
                        topo, &qs, &qe, prev.tip, to.tip, cfg.nonlinear_samples,
                    );
                    if ne.max_deviation > nl_max {
                        nl_max = ne.max_deviation;
                        nl_block = bi as u32;
                        nl_line = block.line_no;
                        // 峰值时刻机床实际在哪里（正解一把就够）
                        let qt: Vec<f64> = qs
                            .iter()
                            .zip(&qe)
                            .map(|(a, b)| a + (b - a) * ne.at_param)
                            .collect();
                        nl_pose = cncsim_kin::forward(topo, &qt).tip;
                    }
                    nl_seed = qe;
                }
                None => nl_seed.clear(), // 机床够不着：复位，下一块从零起步
            }
        }

        // 本块的刀尖折线（不含起点、含终点）
        let pts: Vec<DVec3> = match block.motion {
            MotionKind::Rapid | MotionKind::Linear => vec![to.tip],
            MotionKind::Arc { center, clockwise, plane } => {
                sample_arc(prev.tip, to.tip, center, clockwise, plane, cfg.arc_sagitta)
            }
            MotionKind::Dwell { .. } => Vec::new(),
        };

        let mut cur = prev;
        let n = pts.len().max(1);
        for (k, &p) in pts.iter().enumerate() {
            // 刀轴按块内进度插值（三轴时两端相同，插值退化成常量）
            let u = (k + 1) as f64 / n as f64;
            let next = Pose { tip: p, axis: slerp(prev.axis, to.axis, u) };

            let seg_len = (next.tip - cur.tip).length();

            // ① 切削
            if cutting {
                if cfg.design.is_some() {
                    cut_segs.push(CutSegment {
                        from: cur,
                        to: next,
                        block: bi as u32,
                        line: block.line_no,
                        t_start: sim_time,
                    });
                }
                let t0 = std::time::Instant::now();
                grid.cut_segment(&cur, &next, &tool.cutter);
                cut_ms += t0.elapsed().as_secs_f64() * 1e3;
                stats.cut_segments += 1;
            } else if matches!(block.motion, MotionKind::Rapid) {
                stats.rapid_segments += 1;
            }

            // ② 碰撞（快移也查：真实撞机大多是 G0 撞夹具）
            if let Some(b) = &body {
                let t0 = std::time::Instant::now();
                if let Some(c) = check_shank(&grid, b, &tool.shank, &cur, &next) {
                    record_collision(&mut collisions, &c, sim_time, bi, block.line_no, cfg.episode_gap);
                }
                collide_ms += t0.elapsed().as_secs_f64() * 1e3;
            }

            // ③ 时间
            sim_time += segment_time(seg_len, block, cfg);
            cur = next;
        }

        // ④ 停留：G4 的停延与固定循环的孔底暂停都不动刀，只吃时间。
        //
        // 早版本两者都没算——一条 `G4 P2` 在报告里是 0 秒，一排钻 20 个
        // `G82 P0.5` 的沉孔少算 10 秒。时间不进几何，但它是节拍估算的输入。
        let hold = match block.motion {
            MotionKind::Dwell { seconds } => seconds,
            _ => block.dwell,
        };
        if hold > 0.0 {
            sim_time += hold;
            stats.dwell_blocks += 1;
        }

        if matches!(block.feed.mode, cncsim_motion::FeedMode::UnitsPerRev) {
            stats.per_rev_feed_blocks += 1;
        }
        prev = to;
    }

    stats.stopped_at = stopped_at;
    stats.volume_after = grid.material_volume();
    stats.cut_ms = cut_ms;
    stats.collide_ms = collide_ms;

    // ── 设计模型比对（过切 / 残留）──
    let mut cmp = compare::DesignComparison::default();
    if let Some(design) = &cfg.design {
        let t0 = std::time::Instant::now();
        cmp = compare::compare_with_design(
            &grid,
            cfg.stock_size,
            &cut_segs,
            &tool.cutter,
            design,
            cfg.design_step,
            cfg.gouge_tol,
        );
        stats.compare_ms = t0.elapsed().as_secs_f64() * 1e3;
    }

    // ── 报告 ──
    let mut report = SimReport {
        programmed_time_s: sim_time,
        actual_time_s: t_wall.elapsed().as_secs_f64(),
        removed_volume_mm3: (stats.volume_before - stats.volume_after).max(0.0),
        gouge_volume_mm3: cmp.gouge_volume_mm3,
        gouge_max_depth_mm: cmp.gouge_max_depth,
        gouge_at: cmp.gouge_at,
        gouge_region: cmp.gouge_region,
        residual_volume_mm3: cmp.residual_volume_mm3,
        residual_max_thickness: cmp.residual_max_thickness,
        residual_thickness_capped: cmp.residual_thickness_capped,
        residual_thickness_cap: cmp.residual_thickness_cap,
        residual_at: cmp.residual_at,
        alarms: Vec::new(),
        max_nonlinear_error: nl_max,
        max_nonlinear_block: nl_block,
        max_nonlinear_line: nl_line,
        worst_pose: nl_pose,
    };

    for c in &collisions {
        report.alarms.push(
            Alarm::new(
                "COLLISION",
                Severity::Error,
                format!(
                    "{} 与 {} 碰撞，最深 {:.3} mm，持续 {:.3} s",
                    c.episode.a,
                    c.episode.b,
                    c.episode.max_penetration,
                    c.episode.duration()
                ),
            )
            .with_hint("检查该块的快移路径与装夹高度；把刀杆缩短或抬高安全平面")
            .at(TimeRef {
                sim_time: c.episode.t_start,
                block: c.first_block,
                line: c.first_line,
            }),
        );
    }
    if let Some((line, kind)) = stats.stopped_at {
        report.alarms.push(
            Alarm::new(
                "PROGRAM_STOP",
                Severity::Info,
                format!("程序在第 {line} 行停下（{kind:?}）"),
            )
            .with_hint("M0/M2/M30 之后的代码没有被仿真；要跑完请去掉停机指令"),
        );
    }
    if stats.per_rev_feed_blocks > 0 {
        report.alarms.push(
            Alarm::new(
                "FEED_PER_REV",
                Severity::Info,
                format!(
                    "{} 个块使用每转进给（G95），时间按 mm/min 估算",
                    stats.per_rev_feed_blocks
                ),
            )
            .with_hint("接主轴转速后再重算时间；切削结果不受影响"),
        );
    }
    if let Some(bi) = cancelled_at {
        report.alarms.push(
            Alarm::new(
                "CANCELLED",
                Severity::Info,
                format!("仿真被取消：只算到第 {bi} 块（共 {} 块）", program.len()),
            )
            .with_hint("结果只覆盖已算部分；重新跑一遍才能得到完整报告"),
        );
    }

    // ── 解释器的问题逐条进报告 ──
    //
    // 编号是稳定契约（GCODE-xxx），严重度 1:1 映射——两层的 `Severity` 有意不共享
    // （架构 §2.3：各层自带 finding，由 sim 组合），但四级语义完全一致。
    // 解析期问题没有块号/时刻，行号是唯一能跳回源文件的锚。
    for issue in &program.issues {
        let mut a = Alarm::new(issue.id, map_severity(issue.severity), issue.message.clone());
        a.hint = issue.hint.clone();
        a.at = TimeRef { sim_time: 0.0, block: 0, line: issue.line };
        report.alarms.push(a);
    }
    // 兜底：解释器判了"结果不可信/中止"，上面却没有产出任何 Error+ 报警。
    // 正常路径不会发生（Unreliable 总是伴随 Error 级 issue）——这是给未来路径的保险。
    if !matches!(program.trust, cncsim_gcode::Trust::Reliable)
        && !report.alarms.iter().any(|a| a.severity >= Severity::Error)
    {
        report.alarms.push(
            Alarm::new("GCODE_TRUST", Severity::Error, "解释器判定仿真结果不可信")
                .with_hint("逐条查看 GCODE-xxx 报警；不可信的结果不能作为验收依据"),
        );
    }

    // ── 设计比对的报警 ──
    // 过切 = Error：零件已经报废（"能跑，但结果错"）。残留 = Warning：
    // 没铣到位通常下一刀能救。口径见 compare 模块文档。
    if cmp.gouge_max_depth > cfg.gouge_tol {
        let region = cmp
            .gouge_region
            .map(|(lo, hi)| {
                format!(
                    "区域 x {:.2}..{:.2}, y {:.2}..{:.2}, z {:.2}..{:.2}",
                    lo.x, hi.x, lo.y, hi.y, lo.z, hi.z
                )
            })
            .unwrap_or_default();
        report.alarms.push(
            Alarm::new(
                "GOUGE",
                Severity::Error,
                format!(
                    "过切体积 {:.2} mm³，设计面最大侵入 {:.3} mm @ ({:.2}, {:.2}, {:.2})；{region}",
                    cmp.gouge_volume_mm3, cmp.gouge_max_depth, cmp.gouge_at.x, cmp.gouge_at.y,
                    cmp.gouge_at.z
                ),
            )
            .with_hint("切进了设计模型；检查该块的 Z 高度 / 刀补方向，或确认设计模型本身")
            .at(TimeRef {
                sim_time: cmp.gouge_t,
                block: cmp.gouge_block,
                line: cmp.gouge_line,
            }),
        );
    }
    if cmp.residual_volume_mm3 > 0.0 {
        report.alarms.push(
            Alarm::new(
                "RESIDUAL",
                Severity::Warning,
                format!(
                    "残留 {:.2} mm³，最厚 {:.3} mm @ ({:.2}, {:.2}, {:.2})",
                    cmp.residual_volume_mm3, cmp.residual_max_thickness, cmp.residual_at.x,
                    cmp.residual_at.y, cmp.residual_at.z
                ),
            )
            .with_hint("设计面没有铣到位；检查 Z 对刀、刀路覆盖范围或刀具直径"),
        );
    }
    // 非线性误差超阈值（`docs/技术方案` §4.6：默认口径 0.01 mm）
    const NONLINEAR_WARN_MM: f64 = 0.01;
    if nl_max > NONLINEAR_WARN_MM {
        report.alarms.push(
            Alarm::new(
                "NONLINEAR",
                Severity::Warning,
                format!(
                    "关节线性插补下第 {} 块（第 {} 行）刀尖偏离直线 {:.3} mm",
                    nl_block + 1,
                    nl_line,
                    nl_max
                ),
            )
            .with_hint("五轴联动点位太稀；加密摆角步距，或开后处理的 RTCP（G43.4）"),
        );
    }

    RunOutput { report, grid, collisions, stats, cut_segs }
}

/// `gcode::Severity` → `sim::Severity`。四级一一对应（Info/Warning/Error/Fatal），
/// 顺序一致，见 docs/G代码解释器设计.md 的分级定义。
fn map_severity(s: cncsim_gcode::Severity) -> Severity {
    match s {
        cncsim_gcode::Severity::Info => Severity::Info,
        cncsim_gcode::Severity::Warning => Severity::Warning,
        cncsim_gcode::Severity::Error => Severity::Error,
        cncsim_gcode::Severity::Fatal => Severity::Fatal,
    }
}

/// 一段的仿真耗时（秒）。
///
/// `UnitsPerMin` 直接用；`UnitsPerRev` **按 mm/min 近似**——每转进给要主轴转速，
/// 而 `gcode::Block` 不带 S 值（那是主轴状态，不是运动状态）。统计里记了这类块的数量，
/// 报告会提一句。快移没有进给值时用 `rapid_feed`。
/// 一段走刀要多久。**三种进给模式口径完全不同**：
///
/// | 模式 | `F` 的含义 | 时间 |
/// |---|---|---|
/// | `G94`（默认） | mm/min | `len / (F/60)` |
/// | `G95` | mm/**rev** | `len / (F·S/60)`——`S` 是主轴 rpm |
/// | `G93` | 1/min | `60/F`——**与长度无关**，F=2 表示这块走 0.5 分钟 |
///
/// 以前只按 `G94` 算，`G95` 的块会把 `F0.1`（每转 0.1 mm）当成 `0.1 mm/min`，
/// 时间虚高几个数量级；`G93` 同理。五轴后处理里 `G93` 很常见（它让刀尖速度恒定）。
///
/// **退路**：`G95` 没给 `S`（或 `S=0`）时按快移速度算，`G93` 的 `F=0` 时返回 0——
/// 两种都只在畸形程序里出现，但要有个确定的行为，不能除零。
pub(crate) fn segment_time(len: f64, block: &cncsim_gcode::Block, cfg: &RunConfig) -> f64 {
    use cncsim_motion::FeedMode;
    if len < 1e-12 {
        return 0.0;
    }
    let f = block.feed.value;
    match block.feed.mode {
        FeedMode::InverseTime => {
            if f > 0.0 { 60.0 / f } else { 0.0 }
        }
        FeedMode::UnitsPerRev => {
            if f > 0.0 && block.spindle_speed > 0.0 {
                len / (f * block.spindle_speed / 60.0)
            } else {
                len / (cfg.rapid_feed / 60.0)
            }
        }
        FeedMode::UnitsPerMin => {
            let v = if f > 0.0 { f } else { cfg.rapid_feed };
            len / (v / 60.0)
        }
    }
}

/// 把一次命中并进 episode 列表。
fn record_collision(
    list: &mut Vec<CollisionRecord>,
    c: &StockCollision,
    t: f64,
    block: usize,
    line: u32,
    gap: f64,
) {
    // 保守深度：`clearance_lower` 是下界，取反得到"至少扎进去这么深"。
    // 确切穿透深度要用 EPA（那是凸体专用，毛坯是体素场，只能说下界）。
    let depth = (-c.hit.clearance).max(0.0);
    const A: &str = "刀杆";
    const B: &str = "毛坯";

    if let Some(last) = list.iter_mut().rev().find(|r| r.episode.a == A && r.episode.b == B) {
        // 相邻块（含同一块的下一段）一定并进来，不看时间间隔
        let same_run = block as u32 <= last.last_block + 1;
        if same_run || last.episode.absorb(t, depth, gap) {
            last.episode.absorb(t, depth, f64::INFINITY);
            last.last_block = block as u32;
            return;
        }
    }
    list.push(CollisionRecord {
        episode: CollisionEpisode::new(A, B, t, depth),
        first_block: block as u32,
        first_line: line,
        first_t: c.hit.t,
        last_block: block as u32,
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_gcode::{InterpConfig, interpret};
    use cncsim_math::DVec2;
    use cncsim_tool::{CutterKind, Profile, flat_end_mill};

    fn tool() -> ToolDef {
        let shank = Profile::new(vec![
            DVec2::new(0.0, 20.0),
            DVec2::new(3.0, 20.0),
            DVec2::new(3.0, 60.0),
            DVec2::new(0.0, 60.0),
        ]);
        ToolDef {
            number: 1,
            kind: CutterKind::Flat,
            diameter: 6.0,
            corner_radius: 0.0,
            flute_length: 20.0,
            shank,
            length_offset: 60.0,
            cutter: flat_end_mill(6.0, 20.0),
        }
    }

    /// 20×20×10 mm 毛坯，顶面 z = 10
    fn cfg() -> RunConfig {
        RunConfig {
            stock_origin: DVec3::ZERO,
            stock_size: [200, 200, 100],
            ..Default::default()
        }
    }

    /// 刀尖在 z=9.5（毛坯顶面 z=10 之下 0.5 mm）走一条直线槽 → 切到料，不撞刀杆
    const SAFE: &str = "G0 X2 Y10 Z9.5\nG1 X18 F300\nM30\n";

    /// 刀尖压到 z=−5 → 刀杆（z≥20）还在毛坯上方，仍然安全
    const DEEP_CUT: &str = "G0 X2 Y10 Z-5\nG1 X18 F300\nM30\n";

    /// 刀尖压到 z=−25 → 刀杆底端到 z=−5，穿过毛坯（0..10）
    const CRASH: &str = "G0 X2 Y10 Z-25\nG1 X18 F300\nM30\n";

    fn run_src(src: &str) -> RunOutput {
        let ic = InterpConfig::default();
        let prog = interpret(src, &ic);
        run(&prog, &tool(), &cfg())
    }

    /// 端到端：仿真按块盖的溯源标记，必须原样出现在提取出来的网格里，
    /// 而且**相邻两块不同色**——这正是"每次切割变颜色"的判据。
    ///
    /// 这条把 `run` → `SparseGrid::set_creator` → `extract_surface` 三段串起来；
    /// 任何一段漏了，下面看到的索引就会全 0 或者全相同。
    #[test]
    fn cut_surface_colors_follow_the_block_index() {
        // 三段直线，每段一个块 → 三个不同的切割色
        let out = run_src("G0 X2 Y5 Z9.5\nG1 X18 F300\nG1 Y10\nG1 Y15\nM30\n");
        let m = out.grid.extract_surface_lod(2);
        assert!(!m.positions.is_empty(), "应该有表面");

        let mut seen = std::collections::BTreeSet::new();
        for c in &m.colors {
            seen.insert(c[0]);
            assert!(
                c[0] == 0 || (1..=cncsim_voxel::CUT_COLORS as u8).contains(&c[0]),
                "索引 {} 既不是毛坯面 0 也不是切割色 1..=12",
                c[0]
            );
        }
        let cut: Vec<u8> = seen.iter().copied().filter(|c| *c != 0).collect();
        assert!(
            cut.len() >= 3,
            "三段切削应该至少留下 3 种颜色，实际只有 {cut:?}（说明溯源没按块区分）"
        );
        // 相邻两刀必须不同色——与 `creator_of_cut` 的往返周期一致
        for w in cut.windows(2) {
            assert_ne!(w[0], w[1], "相邻的切割色相同：{cut:?}");
        }
    }

    #[test]
    fn program_parses_and_runs() {
        let out = run_src(SAFE);
        assert!(out.stats.blocks >= 2, "应解析出至少两块，实际 {}", out.stats.blocks);
        assert!(out.stats.cut_segments >= 1);
        assert_eq!(out.collisions.len(), 0, "正常切削不该报碰撞");
    }

    #[test]
    fn cutting_removes_material() {
        let out = run_src(SAFE);
        assert!(
            out.report.removed_volume_mm3 > 0.0,
            "切了一刀应去掉材料，实际 {}",
            out.report.removed_volume_mm3
        );
        // 槽长约 16 mm、宽 6 mm、深 0（刀尖正好在表面）→ 去掉的很薄
        assert!(
            out.report.removed_volume_mm3 < 200.0,
            "不该切掉太多，实际 {}",
            out.report.removed_volume_mm3
        );
    }

    #[test]
    fn deep_cut_with_the_flutes_is_still_safe() {
        // 刀尖在 z=−5，刀刃长 20 → 刀杆在 z≥15，毛坯顶面 z=10 → 不相交
        let out = run_src(DEEP_CUT);
        assert_eq!(out.collisions.len(), 0, "刀刃切深不算刀杆碰撞");
        assert!(out.report.removed_volume_mm3 > 100.0, "这一刀应该切掉不少");
    }

    #[test]
    fn plunging_the_shank_into_the_stock_is_reported() {
        let out = run_src(CRASH);
        assert!(!out.collisions.is_empty(), "刀杆扎进材料必须报");
        let c = &out.collisions[0];
        assert_eq!(c.episode.a, "刀杆");
        assert!(c.episode.max_penetration > 0.0, "应报出穿透深度");
        assert!(c.first_line > 0, "要能跳回源文件行");

        // 报告里必须有对应报警
        assert!(
            out.report.alarms.iter().any(|a| a.id == "COLLISION"),
            "应生成 COLLISION 报警"
        );
    }

    #[test]
    fn episodes_merge_within_the_gap() {
        // 一整条都在撞 → 应该只有一个 episode，不是每段一个
        let out = run_src(CRASH);
        assert_eq!(out.collisions.len(), 1, "连续碰撞应合并成一次事故");
    }

    #[test]
    fn arc_moves_are_discretised_and_cut() {
        // 半径 5 的 3/4 圆（G2 顺时针从 0° 到 90°，走 −270°），圆心 (10,10)
        let src = "G0 X15 Y10 Z9.5\nG2 X10 Y15 I-5 J0 F300\nM30\n";
        let out = run_src(src);
        assert!(out.stats.cut_segments > 3, "圆弧应离散成多段，实际 {}", out.stats.cut_segments);
        assert!(out.report.removed_volume_mm3 > 0.0);
    }

    #[test]
    fn rapid_collisions_are_caught_too() {
        // G0 直接扎下去 —— 撞机大多是这种
        let src = "G0 X10 Y10 Z-25\nM30\n";
        let out = run_src(src);
        assert!(!out.collisions.is_empty(), "G0 也要查碰撞");
    }

    #[test]
    fn m0_stops_the_simulation() {
        // M0 之后还有一段切削，但仿真必须停在 M0
        let src = "G0 X2 Y10 Z9.5
M0
G1 X18 F300
M30
";
        let out = run_src(src);
        assert!(out.stats.stopped_at.is_some(), "应记录停止点");
        assert_eq!(out.stats.stopped_at.unwrap().0, 2, "停在第 2 行");
        assert_eq!(out.stats.cut_segments, 0, "M0 之后不该切削");
        assert!(out.report.alarms.iter().any(|a| a.id == "PROGRAM_STOP"));
    }

    #[test]
    fn m30_at_the_end_is_not_a_truncation() {
        // M30 在最后：前面的块全部跑完，`stopped_at` 保持 None
        let out = run_src(SAFE);
        assert_eq!(out.stats.cut_segments, 1);
        assert!(
            out.stats.stopped_at.is_none(),
            "M30 在末尾属于正常结束，不该记成截断：{:?}",
            out.stats.stopped_at
        );
        // 但 stop 点本身要被解析出来
        let prog = cncsim_gcode::interpret(SAFE, &cncsim_gcode::InterpConfig::default());
        assert_eq!(prog.stops.len(), 1);
        assert_eq!(prog.stops[0].kind, cncsim_gcode::StopKind::ProgramEndReset);
    }

    #[test]
    fn drilling_program_removes_material_and_counts_dwell() {
        // 2×2 孔阵列，每孔 3 mm 深、孔底停 0.5 s
        let src = "G0 X5 Y5 Z5\nG82 Z2 R4 P0.5 F100\nX15 Y5\nX5 Y15\nX15 Y15\nG80\nM30\n";
        let out = run_src(src);
        assert!(out.report.removed_volume_mm3 > 300.0, "四个 φ6×3 的孔约 340 mm³，实际 {}", out.report.removed_volume_mm3);
        assert_eq!(out.stats.dwell_blocks, 4, "每孔一次孔底暂停");
        // 2 s 的纯停留必须进时间
        assert!(out.report.programmed_time_s > 2.0, "实际 {}", out.report.programmed_time_s);
        assert_eq!(out.collisions.len(), 0, "钻进工件不是碰撞（刀刃切料是本职工作）");
    }

    #[test]
    fn cutter_comp_changes_what_gets_cut() {
        // 同一个 10×10 方框，G41 D3（刀半径 3）之后刀心内缩到 3..7：
        // 扫出来的环带外径仍是 0..10，内孔消失 → 去料明显变少。
        let ic = cncsim_gcode::InterpConfig {
            tool_radii: vec![0.0, 3.0],
            ..Default::default()
        };
        let comped = "G0 X5 Y5 Z9.5
G1 G41 D1 X15 Y5 F300
X15 Y15
X5 Y15
X5 Y5
G40
M30
";
        let plain = "G0 X5 Y5 Z9.5
G1 X15 Y5 F300
X15 Y15
X5 Y15
X5 Y5
M30
";
        let a = run(
            &cncsim_gcode::interpret(comped, &ic),
            &tool(),
            &cfg(),
        );
        let b = run(
            &cncsim_gcode::interpret(plain, &ic),
            &tool(),
            &cfg(),
        );
        assert!(a.report.removed_volume_mm3 > 1.0, "补偿后仍应切到料");
        assert!(
            a.report.removed_volume_mm3 < b.report.removed_volume_mm3 * 0.7,
            "补偿后应少切：{:.1} vs {:.1} mm³",
            a.report.removed_volume_mm3,
            b.report.removed_volume_mm3
        );
        assert_eq!(a.collisions.len(), 0);
    }

    #[test]
    fn g4_dwell_counts_toward_programmed_time() {
        let a = run_src("G0 X2 Y10 Z9.5\nG4 P2\nM30\n");
        let b = run_src("G0 X2 Y10 Z9.5\nM30\n");
        assert!(
            a.report.programmed_time_s - b.report.programmed_time_s > 1.9,
            "G4 P2 应让时间多 2 秒：{} vs {}",
            a.report.programmed_time_s,
            b.report.programmed_time_s
        );
    }

    #[test]
    fn observer_sees_progress_and_does_not_change_results() {
        let a = run_src(SAFE);
        let prog = interpret(SAFE, &InterpConfig::default());
        let mut seen = 0usize;
        let b = run_observed(&prog, &tool(), &cfg(), |p| {
            seen += 1;
            assert_eq!(p.blocks_total as usize, prog.len());
            assert!(p.sim_time >= 0.0);
            true
        });
        assert_eq!(seen, prog.len(), "每块一次回调");
        // 逐位一致（墙钟除外）
        assert_eq!(a.report.programmed_time_s, b.report.programmed_time_s);
        assert_eq!(a.report.removed_volume_mm3, b.report.removed_volume_mm3);
        assert_eq!(a.stats.cut_segments, b.stats.cut_segments);
        assert_eq!(a.stats.volume_after, b.stats.volume_after);
        assert!(!b.stats.cancelled);
    }

    #[test]
    fn observer_can_cancel_and_report_reflects_partial_state() {
        // 三刀切削；在第 2 块前取消 → 只切了第一刀
        let src = "G0 X2 Y10 Z9.5
G1 X8 F300
G1 X14
G1 X18
M30
";
        let prog = interpret(src, &InterpConfig::default());
        let whole = run(&prog, &tool(), &cfg());
        let partial = run_observed(&prog, &tool(), &cfg(), |p| p.block < 2);

        assert!(partial.stats.cancelled);
        assert!(partial.stats.cut_segments > 0, "取消前已切的不能丢");
        assert!(
            partial.stats.cut_segments < whole.stats.cut_segments,
            "取消后段数应更少：{} vs {}",
            partial.stats.cut_segments,
            whole.stats.cut_segments
        );
        assert!(
            partial.report.removed_volume_mm3 < whole.report.removed_volume_mm3,
            "部分状态去料应更少"
        );
        assert!(
            partial.report.alarms.iter().any(|a| a.id == "CANCELLED"),
            "取消要有 CANCELLED 报警"
        );
    }

    #[test]
    fn gcode_issues_become_alarms_with_their_own_severity() {
        // 圆弧少了 I/J/K → GCODE-008（Error）。逐条映射后：
        // 报警沿用解释器的稳定编号与严重度，不再笼统报 Fatal。
        let out = run_src("G0 X0 Y0 Z10\nG2 X10 Y10 F300\nM30\n");
        let a = out
            .report
            .alarms
            .iter()
            .find(|a| a.id == "GCODE-008")
            .expect("圆弧参数无效必须成为报警");
        assert_eq!(a.severity, Severity::Error, "GCODE-008 自身是 Error 级");
        assert_eq!(a.at.line, 2, "报警要能跳回源文件第 2 行");
        assert!(a.hint.is_some(), "每条报警必须有 hint");
        // 总体可信度由 overall_severity 给出：Error = 结果不可信
        assert_eq!(out.report.overall_severity(), Some(Severity::Error));
        // 警告级 issue 也进报告（未知 M 代码）
        let out = run_src("G0 X0 Y0 Z10\nM99 F300\nM30\n");
        let a = out
            .report
            .alarms
            .iter()
            .find(|a| a.id == "GCODE-002")
            .expect("未知 M 代码（Warning）也要进报告");
        assert_eq!(a.severity, Severity::Warning);
    }

    #[test]
    fn unreliable_trust_without_error_alarm_gets_a_fallback() {
        // 正常路径里 Unreliable 总是伴随 Error 级 issue；这是保险丝：
        // 解释器说了"不可信"，报告里就必须有一条 Error+。
        let prog = cncsim_gcode::MotionProgram {
            trust: cncsim_gcode::Trust::Aborted,
            ..Default::default()
        };
        let out = run(&prog, &tool(), &cfg());
        assert!(
            out.report.alarms.iter().any(|a| a.id == "GCODE_TRUST" && a.severity >= Severity::Error),
            "Aborted 且无 Error 报警时必须兜底，实际 {:?}",
            out.report.alarms.iter().map(|a| (a.id, a.severity)).collect::<Vec<_>>()
        );
    }
}


