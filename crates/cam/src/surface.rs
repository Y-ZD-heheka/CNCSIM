//! **五轴曲面法向点铣**：刀尖沿设计面走，刀轴沿**该点的外法向**。
//!
//! 前面几条链路（[`crate::generate`] / [`crate::multi`]）都是**高度场**：
//! 每个 XY 列只认最上面那一层。于是叶轮的叶片侧壁在任何方向的列里都不是
//! 最高面，怎么摆方向都切不到——`docs/项目记忆.md` §40 记的就是这件事。
//!
//! 这一条链路不经过高度场：直接在设计面上采样，把球头刀的**球心放在
//! `p + R·n`**。球头是与方向无关的（转刀轴只转刀杆，不转球），所以：
//!
//! * 切出来的形状**只取决于刀尖点**，五轴在这里不改变几何，只改变**能不能进去**；
//! * 球心偏置沿外法向放置时，球在 `p` 点与设计面相切——不会啃进去，
//!   只要没有更近的面（凹坑里 R 塞不下就整点放弃，见 [`SurfaceStats::gouge_rejected`]）。
//!
//! 于是"能不能切"变成了两条可判的几何问题：
//!
//! 1. **球面闸门**：`dist(p + R·n, 网格) ≥ R − tol`（凹处塞不进就跳过）；
//! 2. **刀杆闸门**：刀轴沿 `n` 伸出去的杆会不会撞到别处（叶片之间的通道）。
//!
//! 剩下的是路径：目标点是一堆散点，用**最近邻串链**（带方向偏好）连成折线，
//! 再按弧长抽稀成 G 代码块。链与链之间沿刀轴退 `lift_mm` 再快移。
//!
//! ## 为什么不是"侧刃铣"
//!
//! 严格意义的叶轮侧刃铣要求叶片是**直纹面**，刀轴沿每根直素线走、用圆柱面
//! 侧刃接触。那要先把网格拟合成直纹面，属于另一套几何（且对非直纹叶片无解）。
//! 这里先做**球头法向点铣**——它对任意网格都成立，代价是行距决定刀花，
//! 覆盖面积一样能吃到，只是切长更长。两条路子对"像不像原版"是同一量级，
//! 对"加工时间"侧刃铣更省。选它是因为可以先量出上限，再决定值不值得做侧刃。

use rayon::prelude::*;
use crate::{CamError, Stock, ToolKind, ToolSpec, Tri};
use cncsim_math::DVec3;

/// 五轴曲面法向点铣的参数。
#[derive(Clone, Debug)]
pub struct SurfaceOpts {
    /// 球头刀（只支持球头：平底刀的法向偏置不是一个点，见模块注释）。
    pub tool: ToolSpec,
    /// 刀号（要**全局唯一**，由调用方给）。
    pub tool_number: u16,
    /// 链内最大连接距离（mm）。点距是采样间距，取 2~3 倍不会连到隔壁面上。
    pub link_mm: f64,
    /// 输出抽稀间距（mm）：相邻 G 代码块之间的弧长。
    pub out_step_mm: f64,
    /// 链间退刀（沿刀轴，mm）。
    pub lift_mm: f64,
    /// 刀杆（刃长以上，不切东西）要求的最小间隙（mm）。**必须为正**。
    pub shank_clearance_mm: f64,
    /// 球面闸门容差（mm）。
    pub gate_tol_mm: f64,
    /// A 轴行程上限（度）。
    pub max_a_deg: f64,
    /// 串链时是否沿**局部流向**（曲率最小的切向）走。
    ///
    /// **默认关**：实测它对切除量没有帮助，见 §50——叶轮 1 沿流向 1385 条链 /
    /// 切长 15.2 m / 过切 1.6%，纯最近邻 1427 条 / 14.9 m / 1.5%。链是变长了
    /// （−3%），但块数 +311、切长 +2%，净收益为负。留给 `--flow` 做实验。
    pub follow_flow: bool,
    /// 刀轴可以在法向周围歪多少度（`0` = 只用纯法向）。
    ///
    /// 歪刀轴只为了让开障碍：球在 `p` 的接触点与刀轴无关，形状不变。
    /// 实测叶轮 1：40° → 91.2% / 60° → **98.2%** / 75° → 98.1%（没更多收益）。
    pub max_tilt_deg: f64,
    pub spindle_rpm: u32,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    /// **进给自适应**：按凹度逐块调制 F（见 [`crate::feed`]）。
    ///
    /// 默认开。关掉时全程 `feed_mm_min`（F 值不变，省掉重复的 `F` 字）。
    pub feed: crate::feed::FeedOpts,
    /// 链间快速移动的安全高度（世界 Z，绝对）。
    pub safe_z_mm: f64,
}

impl SurfaceOpts {
    /// 以世界毛坯 + 刀为基准的一套默认参数。
    pub fn new(tool: ToolSpec, tool_number: u16, stock: Stock) -> Self {
        let r = tool.radius();
        let flute = tool.flute_mm.max(r);
        Self {
            tool,
            tool_number,
            link_mm: (2.5 * r).clamp(0.6, 2.5),
            out_step_mm: (1.2 * r).clamp(0.3, 1.2),
            // **退刀量 = 刃长 + 2**：换姿态（A/C 大角度）必须整根刃部先退出工件，
            // 否则刀杆在离面 2 mm 处转 90°，那一段圆柱会把旁边的面扫掉。
            lift_mm: flute + 2.0,
            shank_clearance_mm: 0.3,
            gate_tol_mm: 0.03,
            follow_flow: false,
            max_a_deg: crate::rotary::A_LIMIT_DEG,
            max_tilt_deg: 60.0,
            spindle_rpm: 12_000,
            feed_mm_min: 600.0,
            plunge_mm_min: 200.0,
            feed: crate::feed::FeedOpts::default(),
            safe_z_mm: stock.max.z + 5.0,
        }
    }
}

/// 收/拒的计数。**每一项都是量出来的**，不是估计。
#[derive(Clone, Copy, Debug, Default)]
pub struct SurfaceStats {
    /// 目标点数（调用方给的）。
    pub targets: usize,
    /// 通过全部闸门的点数。
    pub accepted: usize,
    /// A 超行程（刀摆不到那个法向）。
    pub a_rejected: usize,
    /// **歪刀轴才通过**的点数：纯法向会被刀杆挡住，歪一点就让开了。
    pub axis_retried: usize,
    /// 球面会啃进设计面（凹处塞不下这么大的球）。
    pub gouge_rejected: usize,
    /// 刀杆会撞到别处（通道太窄/深腔）。
    pub shank_rejected: usize,
    /// 沿法向切入那一刀本身会啃（相邻特征挡在法向上）。
    pub plunge_rejected: usize,
    /// 链间快移里**会擦到工件**的条数（仿真不计，实机会撞）。
    pub link_unsafe: usize,
    /// 链间转移的**平均退刀量**（mm）。固定 18 mm 时这个数恒等于 `lift_mm`；
    /// 自适应之后能看出"大部分链之间其实只需要几毫米"。
    pub avg_link_lift: f64,
    /// 连成的折线条数。
    pub runs: usize,
    /// 实际输出的 G 代码运动块数。
    pub moves: usize,
    /// 切长（mm）。
    pub cut_mm: f64,
    /// 快移长度（mm）。
    pub rapid_mm: f64,
    /// **进给调度**的结果：最低 F、按长度加权的平均比、切削时间。
    pub feed: crate::feed::FeedStats,
    /// 凹度 `κR ≥ 0.1` 的切削块数（"有多少块真的被减速"的几何侧口径）。
    pub conc_blocks: usize,
    /// 凹度直方图：`[i]` = `κR ∈ [i/10, (i+1)/10)` 的切削块数。
    ///
    /// 调 `k_conc` 全靠它：有了分布就能**算**出任意 `k` 下的平均进给比，
    /// 不用一个 k 跑一遍 14 分钟的整件仿真。
    pub conc_hist: [usize; 10],
}

pub struct SurfaceOut {
    pub gcode: String,
    pub stats: SurfaceStats,
}

/// 生成五轴曲面道。`targets` 是 `(点, 单位外法向)`，世界坐标。
///
/// 返回的 `gcode` 是一段**自包含**的片段（含换刀、`G43.4`、`G49` 收尾），
/// 由调用方插到多工序程序的收尾指令之前——不在这里拼 `M30`。
pub fn generate_surface(
    mesh: &[Tri],
    stock: Stock,
    targets: &[(DVec3, DVec3)],
    opts: &SurfaceOpts,
) -> Result<SurfaceOut, CamError> {
    if opts.tool.kind != ToolKind::Ball {
        return Err(CamError::BadTool("五轴曲面道只支持球头刀"));
    }
    if mesh.is_empty() {
        return Err(CamError::EmptyMesh);
    }
    let mut st = SurfaceStats {
        targets: targets.len(),
        ..Default::default()
    };
    if targets.is_empty() {
        return Ok(SurfaceOut {
            gcode: String::new(),
            stats: st,
        });
    }
    let ix = crate::MeshIndex::build(mesh, stock);
    let r = opts.tool.radius();

    // ── 闸门 ──
    //
    // 这里存四个量：设计面上的点 `p`、它的外法向 `n`、**实际用的刀轴** `a`、
    // 以及刀尖 `tip`。四个不等价，关系是 `tip = p + R(n − a)`：
    //
    // * 球在 `p` 与设计面相切 ⟺ 球心在 `p + R·n` ⟺ `tip + R·a = p + R·n`；
    // * 所以**刀轴不一定要沿法向**：球是各向同性的，只要把刀尖补偿一个
    //   `R(n − a)`，切出来的位置完全一样，只是刀杆歪了。
    //
    // 这一条把"A 超行程"从"切不了"变成"换个姿态还能切"：法向朝下（A=180°）
    // 的面，把刀轴拉到 A=110° 的极限上、刀尖偏出去 `R(n−a)`，球照样在 `p` 相切。
    // 实测叶轮 1 有 45% 的残料点栽在 A 超程上，这一改全部变成候选。
    let mut pts: Vec<DVec3> = Vec::new();
    let mut nrm: Vec<DVec3> = Vec::new();
    let g = Geo::of(&opts.tool, opts.gate_tol_mm, opts.shank_clearance_mm);
    let flute = g.flute;
    let mut axs: Vec<DVec3> = Vec::new();

    // ── 逐点可行性**并行预筛**（曲面道最热的循环）──
    //
    // 一个候选轴可不可行，只由 (p, n, 轴) 决定——**与串链顺序无关**：
    // `last_axis` 只影响主循环"挑哪一个"的顺序（`axis_candidates_indexed`
    // 的基准下标不变量保证两者对得上）。所以每点 17 个候选 × 整根刀检查
    // （~117 个采样点）可以全部提前、按线程摊开（叶轮 ~3400 万次查询，
    // 原来全在单线程）；串行主循环退化为纯查表。
    let pre: Vec<(bool, u32)> = targets
        .par_iter()
        .map(|&(p, n)| {
            let n = n.normalize();
            if !n.length_squared().is_finite()
                || !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())
            {
                return (false, 0);
            }
            // 球心 `p + R·n`：`p` 自己在球面上，所以最近距离天然 = R；只有**别的面**
            // 比 R 更近才说明塞不下（凹坑/窄槽）。**signed**：球心万一在别的
            // 面片材质里，无符号读数 = 深度（可能 ≥ need → 漏判），负值必拒。
            let need = (r - opts.gate_tol_mm).max(1e-6);
            if ix.min_dist_signed(p + n * r, need) < need {
                return (false, 0);
            }
            let mut bits = 0u32;
            for (k, a) in axis_candidates_indexed(n, opts.max_a_deg, opts.max_tilt_deg, None) {
                if a.dot(n) <= 0.0 {
                    continue; // 轴跑到设计面背后：球只能从里面顶
                }
                let tip = p + (n - a) * r;
                if !shank_hits_axis(&ix, tip, a, &g) {
                    bits |= 1 << k;
                }
            }
            (true, bits)
        })
        .collect();

    // 上一个通过点的刀轴：候选按"与它最像"排序，保证轴场连续
    let mut last_axis: Option<DVec3> = None;
    for (ti, &(p, n)) in targets.iter().enumerate() {
        let (ball_ok, bits) = pre[ti];
        let n = n.normalize();
        if !ball_ok {
            // 球心塞不下（或输入非法）——**与刀轴无关**，最便宜的拒绝。
            st.gouge_rejected += 1;
            continue;
        }
        // 刀轴搜索：先试法向本身，不行就在锥面上转。
        //
        // 这是叶轮上最大的一笔：残料点里 73% 是"刀杆蹭到对面叶片"被拒的。
        // 刀轴偏转 `R(n−a)` 那点补偿对形状没有影响（球照样在 `p` 相切），
        // 但**刀杆整根跟着摆**——歪一点就能让开对面那片叶子。
        // 可行性已在上面的并行预筛里算好（位图），这里只按顺序挑。
        let mut pick: Option<(DVec3, bool)> = None;
        for (k, a) in axis_candidates_indexed(n, opts.max_a_deg, opts.max_tilt_deg, last_axis)
        {
            if a.dot(n) <= 0.0 {
                continue; // 轴跑到设计面背后：球只能从里面顶
            }
            if bits & (1 << k) != 0 {
                pick = Some((a, k > 0));
                break;
            }
        }
        match pick {
            Some((a, tilted)) => {
                if tilted {
                    st.axis_retried += 1;
                }
                pts.push(p);
                nrm.push(n);
                axs.push(a);
                last_axis = Some(a);
            }
            None => st.shank_rejected += 1,
        }
    }
    st.accepted = pts.len();
    if pts.is_empty() {
        return Ok(SurfaceOut {
            gcode: String::new(),
            stats: st,
        });
    }
    // 刀尖：`p + R(n − a)`。球心 = tip + R·a = p + R·n，与法向闸门同一处。
    let tips: Vec<DVec3> = pts
        .iter()
        .zip(&nrm)
        .zip(&axs)
        .map(|((p, n), a)| *p + (*n - *a) * r)
        .collect();

    // ── 串链 ──
    //
    // 连之前要问一句"这条直线扫过去会不会啃到别处"。**必须问**：接受的点和被
    // 拒的点是混在一起的，两个相距 1 mm 的点中间可能正好隔着一道凸棱或一堵薄壁，
    // 直连过去球就从中间穿过去了（实测桌腿件 237 块里啃出 6.8% 过切、最深 2.92 mm）。
    // 流向场：邻域半径取 2.5×link（太小噪声大、太大把棱线两侧混在一起）
    let flow = if opts.follow_flow {
        Some(flow_field(&tips, &axs, opts.link_mm() * 2.5))
    } else {
        None
    };
    let runs = chain(&tips, &axs, opts.link_mm(), &ix, &g, flow.as_deref());
    st.runs = runs.len();
    // 把链按"离上一条链的终点最近"重排（可整条反向）。链的顺序本来是种子顺序，
    // 跟空间没关系——1225 条链之间的快移加起来比切长还长。
    let runs = order_runs(runs, &tips, &axs, &ix, &g, g.flute + 2.0);

    // ── 凹度（进给调度的几何输入） ──
    //
    // 一次算全：**只跟点云有关**，与链怎么串无关。邻域半径取 2.5×`link`
    // （≈ 2.5 个采样间距，够撑一个二次拟合，又不会跨到隔壁叶片的背面）。
    // 上限 1.5 mm：`2d/r²` 对半径 ρ 的圆有 `1 − r²/(12ρ²)` 的截断误差，
    // 邻域越大越低估**尖**的凹角（r=1.5、ρ=0.8 时低估 29%），不能放任它涨。
    let conc = if opts.feed.adaptive {
        concavity(&pts, &nrm, (2.5 * opts.link_mm()).clamp(0.35, 1.5), r)
    } else {
        vec![0.0; pts.len()]
    };

    // ── 出 G 代码 ──
    let mut e = Sink::new();
    e.line(&format!(
        "(PASS SURFACE T{} D{:.2} BALL 法向点铣 目标 {} / 通过 {} / A超程 {} / 球面 {} / 刀杆 {})",
        opts.tool_number,
        opts.tool.diameter,
        st.targets,
        st.accepted,
        st.a_rejected,
        st.gouge_rejected,
        st.shank_rejected
    ));
    e.line("G49");
    e.line(&format!("G0 Z{:.3}", opts.safe_z_mm));
    e.line(&format!("T{} M6", opts.tool_number));
    e.line(&format!("S{} M3", opts.spindle_rpm));
    e.line("G43.4 H1");

    let max_lift = opts.lift_mm;
    let mut st_last: Option<usize> = None;
    // 上一条链的**刀尖 + 刀轴**（不是退刀位）：用来算这一条链要退多高
    let mut prev_tip: Option<(DVec3, DVec3)> = None;
    let (mut lift_sum, mut lift_n) = (0.0f64, 0usize);
    // G 代码里 `F` 是**模态**字：只在变化时写。跨链也要记住（G0 不改 F）。
    // 初值 `None` 表示"文件里还没有出现过 F"——第一条下刀一定会写它。
    #[allow(unused_assignments)]
    let mut cur_f: Option<f64> = None;
    // 实际写出的 F 字数（模态：与上一块相同就省）。报告里用它算省了多少字。
    let mut f_words = 0usize;
    // 抽稀**按链并行**：每条链的 decimate 互不依赖（每条几十~几百次段扫掠，
    // 是曲面道最贵的一步），rayon 按线程摊开。链间顺序逻辑（min_link_lift、
    // 出码顺序）保持串行——它们依赖"上一条链"的状态，天然串行。
    let kepts: Vec<Vec<usize>> = {
        let _t = std::time::Instant::now();
        runs.par_iter()
            .map(|run| decimate(run, &tips, &axs, opts.out_step_mm, &ix, &g))
            .collect()
    };
    for (run, kept) in runs.iter().zip(&kepts) {
        // 链间转移的退刀量：按"这一段能不能过"自适应，不再固定 18 mm
        let lift = match prev_tip {
            Some((pt, pa)) => min_link_lift(&ix, pt, pa, tips[run[0]], axs[run[0]], &g, max_lift),
            None => max_lift,
        };
        lift_sum += lift;
        lift_n += 1;
        // 抽稀：沿弧长每 `out_step` 留一个点，首尾必留（见上，已并行算好）
        if kept.is_empty() {
            continue;
        }
        let i0 = kept[0];
        let (t0, a0v) = (tips[i0], axs[i0]);
        // 沿刀轴切入那一刀本身也可能啃（旁边有凸起挡在刀轴上）
        if !sweep_ok(&ix, t0 + a0v * lift, a0v, t0, a0v, &g) {
            st.plunge_rejected += 1;
            continue;
        }
        let (a0, c0) = crate::rotary::ac_of(a0v);
        // 摆姿态 → 快移到起点上方（沿刀轴退 lift）→ **快移**下到 1 mm 余量 →
        // 最后 1 mm 才进给。
        //
        // 两处是纯为省时间：`lift = 刃长 + 2 = 18 mm`，如果这 18 mm 全用
        // G1 F200 下刀，每条链 5.4 s——叶轮 1 有 1225 条链，光下刀就 110 min。
        // 而已验证过整条下刀路径（`sweep_ok`）是空的，所以除了最后贴上去的
        // 那 1 mm，前面都可以快移。
        let up0 = t0 + a0v * lift;
        if let Some((pt, pa)) = prev_tip
            && !link_ok(&ix, pt + pa * lift, pa, up0, a0v, &g)
        {
            st.link_unsafe += 1;
        }
        e.line(&format!("G0 A{a0:.3} C{c0:.3}"));
        e.line(&format!("G0 X{:.4} Y{:.4} Z{:.4}", up0.x, up0.y, up0.z));
        let near = t0 + a0v * 1.0;
        if lift > 1.0 + 1e-9 {
            e.line(&format!("G0 X{:.4} Y{:.4} Z{:.4}", near.x, near.y, near.z));
        }
        e.line(&format!(
            "G1 X{:.4} Y{:.4} Z{:.4} A{a0:.3} C{c0:.3} F{:.0}",
            t0.x, t0.y, t0.z, opts.plunge_mm_min
        ));
        cur_f = Some(opts.plunge_mm_min);
        f_words += 1;
        st.moves += 1;

        // ── 进给调度（一条链一次） ──
        //
        // 负载只取**几何**：块两端里更凹的那一个（从开阔处走进凹角的这一块，
        // 必须以凹角的进给走完，不能"到了才刹车"）。路径转向角默认不参与
        // （`k_turn = 0`）：点铣的链是最近邻串的，锯齿是链的噪声不是真拐角。
        let nb = kept.len() - 1;
        let mut seg = Vec::with_capacity(nb);
        let mut load = Vec::with_capacity(nb);
        for k in 1..kept.len() {
            let (ia, ib) = (kept[k - 1], kept[k]);
            seg.push((tips[ib] - tips[ia]).length());
            let c = conc[ia].max(conc[ib]);
            let mut l = crate::feed::load_of(c, 0.0, &opts.feed);
            if opts.feed.k_turn > 0.0 {
                l += opts.feed.k_turn * turn_at(&tips, kept, k);
            }
            load.push(l);
        }
        let (vf, fs) = crate::feed::schedule(&seg, &load, opts.feed_mm_min, &opts.feed);
        st.feed.merge(&fs);
        if opts.feed.adaptive {
            for k in 1..kept.len() {
                let c = conc[kept[k - 1]].max(conc[kept[k]]);
                st.conc_hist[((c * 10.0) as usize).min(9)] += 1;
            }
        }

        for k in 1..kept.len() {
            let i = kept[k];
            let (t, av) = (tips[i], axs[i]);
            let (a, c) = crate::rotary::ac_of(av);
            let f = vf[k - 1];
            // `F` 是模态字：只在变化时写。一段一段写满是 3 万个 F 字，占 200 KB。
            if cur_f.map(|x| (x - f).abs() > 1e-9).unwrap_or(true) {
                e.line(&format!(
                    "G1 X{:.4} Y{:.4} Z{:.4} A{a:.3} C{c:.3} F{f:.0}",
                    t.x, t.y, t.z
                ));
                cur_f = Some(f);
                f_words += 1;
            } else {
                e.line(&format!(
                    "G1 X{:.4} Y{:.4} Z{:.4} A{a:.3} C{c:.3}",
                    t.x, t.y, t.z
                ));
            }
            st.moves += 1;
        }
        // 沿刀轴退刀（只动 XYZ，姿态不变）。**出刀和进刀一样要查**：
        // 退刀方向是 +a，大多数时候是往空气里退，但凹腔里会撞对面的壁。
        let last = *kept.last().unwrap();
        let up = tips[last] + axs[last] * lift;
        if !sweep_ok(&ix, tips[last], axs[last], up, axs[last], &g) {
            st.plunge_rejected += 1;
            continue;
        }
        // 退刀用 **G0**：沿刀轴离开工件，`sweep_ok` 已经验证这条路上没有料。
        // 用 G1 的话每条链 18 mm × 2（进+退）全算进切长——叶轮 1 的 55 m 里
        // 44 m 是这段空行程，真正的切削只有 11 m。
        e.line(&format!("G0 X{:.4} Y{:.4} Z{:.4}", up.x, up.y, up.z));
        prev_tip = Some((tips[last], axs[last]));
        st_last = Some(last);
    }
    if lift_n > 0 {
        st.avg_link_lift = lift_sum / lift_n as f64;
    }
    // 和直方图同口径（按块），别用"点"当分母
    st.conc_blocks = st.conc_hist.iter().skip(1).sum();
    st.feed.words = f_words;
    // **收尾必须自己把刀退出去**：多工序的收尾是 `G49` + `G53 Z<安全高度>`，
    // 那是从"刀尖当前位置"竖直往上抬。三轴道没问题（每列上方都已切空），
    // 五轴道结束时刀尖在侧壁上、刀杆还歪着——竖直一抬就是从工件里穿出去。
    // 实测这一条直线啃掉 0.67 mm（`sd_sweep` 报在 82 mm 的长扫掠上）。
    if let Some(last) = st_last {
        let park = tips[last] + axs[last] * (flute + opts.lift_mm);
        e.line(&format!("G0 X{:.4} Y{:.4} Z{:.4}", park.x, park.y, park.z));
        e.line("G49");
        e.line("G0 A0.000 C0.000");
        e.line(&format!("G0 Z{:.3}", opts.safe_z_mm));
    }
    e.line("G49");
    st.rapid_mm = e.rapid_mm;
    st.cut_mm = e.cut_mm_extra;
    Ok(SurfaceOut {
        gcode: e.out,
        stats: st,
    })
}

impl SurfaceOpts {
    fn link_mm(&self) -> f64 {
        self.link_mm.max(1e-3)
    }
}

/// 在 A 轴行程内**最接近法向**的刀轴。
///
/// 球头是各向同性的：刀轴取什么方向，球在同一个点上照样相切，只是刀尖要
/// 补偿 `R(n − a)`。所以法向超出行程时不必放弃——把轴往 +Z 拉到限位就好，
/// 水平方位保持不变（否则同一个法向会有无数解）。
fn feasible_axis(n: DVec3, max_a_deg: f64) -> DVec3 {
    let a = n.z.clamp(-1.0, 1.0).acos().to_degrees();
    if a <= max_a_deg {
        return n;
    }
    let t = max_a_deg.to_radians();
    let h = DVec3::new(n.x, n.y, 0.0);
    let dir = if h.length() < 1e-9 {
        // 法向正对 ±Z：方位任意，取 +X 免得抖动
        DVec3::new(1.0, 0.0, 0.0)
    } else {
        h.normalize()
    };
    (dir * t.sin() + DVec3::Z * t.cos()).normalize()
}

/// 候选刀轴：法向本身，再在 ±`tilt_deg` 的锥面上按 8 个方位各取半步/整步。
///
/// 排在最前的是"不歪"的解，所以绝大多数点第一次就命中；只有被刀杆拦住的点
/// 才会往下试。每个候选都过 [`feasible_axis`]，所以永远在 A 行程内。
/// （生产路径走 [`axis_candidates_indexed`]；这个包装只剩测试用。）
#[cfg(test)]
fn axis_candidates(n: DVec3, max_a_deg: f64, tilt_deg: f64, prefer: Option<DVec3>) -> Vec<DVec3> {
    axis_candidates_indexed(n, max_a_deg, tilt_deg, prefer)
        .into_iter()
        .map(|(_, a)| a)
        .collect()
}

/// [`axis_candidates`] 的**带基准下标**版本：下标是候选在 `prefer = None`
/// （基准顺序）里的位置，`prefer` 只重排顺序、**不改变下标→向量的对应**。
///
/// 这个不变量是逐点可行性**并行预筛**的基础：可行性位图按基准下标算好
/// （与串链顺序无关），主循环按 `last_axis` 重排后照下标查位图即可。
fn axis_candidates_indexed(
    n: DVec3,
    max_a_deg: f64,
    tilt_deg: f64,
    prefer: Option<DVec3>,
) -> Vec<(usize, DVec3)> {
    let mut out = vec![(0usize, feasible_axis(n, max_a_deg))];
    if tilt_deg <= 0.0 {
        return out;
    }
    let f = crate::Frame::from_dir(n);
    // `rot` 的列是局部 x/y/z 在世界里的方向
    let x = DVec3::new(f.rot[0][0], f.rot[1][0], f.rot[2][0]);
    let y = DVec3::new(f.rot[0][1], f.rot[1][1], f.rot[2][1]);
    for frac in [0.6, 1.0] {
        let th = (tilt_deg * frac).to_radians();
        let (st, ct) = th.sin_cos();
        let mut ring: Vec<(usize, DVec3)> = (0..8)
            .map(|k| {
                let phi = k as f64 / 8.0 * std::f64::consts::TAU;
                let (sp, cp) = phi.sin_cos();
                let idx = out.len() + k;
                (idx, (n * ct + (x * cp + y * sp) * st).normalize())
            })
            .collect();
        // **同一圈里挑"跟上一个点最像"的那个**：刀轴在面上要连续，否则相邻
        // 两块之间轴摆 50°，机器抖、刃部圆柱还会扫出一片（实测 0.50 mm）。
        if let Some(p) = prefer {
            ring.sort_by(|a, b| b.1.dot(p).total_cmp(&a.1.dot(p)));
        }
        out.extend(ring.into_iter().map(|(i, a)| (i, feasible_axis(a, max_a_deg))));
    }
    out
}

/// 刀杆/刃部圆柱会不会碰到工件——**沿任意刀轴**。
///
/// 两段用两套判据，这是从实测里分出来的：
///
/// * **刃部圆柱**（球顶到刃长）：它是要切东西的，贴着墙走是正常的（凹角里本来就
///   相切）。判据与球面闸门同口径——侵入不超过 `tol`。
/// * **刀杆**（刃长以上）：不切东西，必须留正间隙（默认 0.3 mm）。
///
/// 早先只从刃长以上查，等于假设"刃部圆柱不会啃"——在 3 轴 drop-cutter 里成立
/// （球已经保证刀柱在料外），五轴不成立：刀轴一歪，刃部圆柱就横着压在相邻面上。
/// 实测桌腿件最深过切 0.67 mm 全在这一段。
/// 刃部段（球心 + 刃部圆柱）的碰撞检查。`shank_hits_axis` 的 ① 段，
/// 拆出来是为了**按转角分层**：每个位姿都查刃部（便宜、常拒），
/// 颈/柄只在需要时查（见 `neck_shank_hits`）。
fn ball_flute_hits(ix: &crate::MeshIndex, tip: DVec3, axis: DVec3, g: &Geo) -> bool {
    let r = g.r;
    let need_f = (r - g.tol).max(1e-6);
    let step = (r * 0.8).clamp(0.5, 2.0);
    let mut z = r;
    while z < g.flute {
        if ix.min_dist_signed(tip + axis * z, need_f) < need_f {
            return true;
        }
        z += step;
    }
    false
}

/// 颈/柄段（等径避空颈 + 30° 缩颈 + Ø6 柄）的碰撞检查。
///
/// 每位姿 ~50 个采样点（锥 0.3 步长 + 柄粗步长），是整根刀检查里最贵的部分
/// ——段扫掠按转角分层查它（`sweep_ok`）。
fn neck_shank_hits(ix: &crate::MeshIndex, tip: DVec3, axis: DVec3, g: &Geo) -> bool {
    let r = g.r;
    let clearance = g.clear;
    let flute_end = g.flute;
    let step = (r * 0.8).clamp(0.5, 2.0);
    // ② 等径避空颈 + 30° 缩颈：从刃顶按实际半径查。
    //    拿柄径一刀切会把这几毫米也当成 Ø6——Ø1 刀的避空段本来只有 Ø1→Ø3。
    let taper = g.tool.taper_len();
    let mut z = flute_end;
    while z < g.tool.neck_top() + taper {
        let rr = g.tool.radius_at(z);
        let need = (rr + clearance).max(1e-3);
        if ix.min_dist_signed(tip + axis * z, need) < need {
            return true;
        }
        z += 0.3;
    }
    // ③ Ø6 刀柄：半径恒定，用粗步长
    let shank_r = g.tool.shank_radius().max(r);
    let need_s = (shank_r + clearance).max(1e-3);
    let mut z = (g.tool.neck_top() + taper).max(flute_end);
    while z <= g.tool.neck_top() + 20.0 {
        if ix.min_dist_signed(tip + axis * z, need_s) < need_s {
            return true;
        }
        z += step;
    }
    false
}

/// 整根刀（刃 + 颈/柄）的单一位姿碰撞检查。逐点接受闸门用；
/// 段扫掠（`sweep_ok`）按转角分层调用上面两个分段版。
fn shank_hits_axis(ix: &crate::MeshIndex, tip: DVec3, axis: DVec3, g: &Geo) -> bool {
    ball_flute_hits(ix, tip, axis, g) || neck_shank_hits(ix, tip, axis, g)
}

/// 【诊断】单个刀位姿下，整根刀各高度的净距报告。
///
/// 返回 `(z, radius_at(z), min_dist_signed − need)`——裕量 < 0 的层会判碰撞。
/// 采样网格与 `shank_hits_axis` 完全一致，只是**不短路、摊开读数**，
/// 用于对拍 CAM 闸门口径与仿真扫掠口径（`sd_tool`/`sd_sweep`）：
/// 若仿真报告侵入而这里全部为正，分歧出在刀形（profile vs `radius_at`）
/// 或网格（MeshIndex 的桶/reach），一眼就能定位。
pub fn axis_clearance_report(
    mesh: &[crate::Tri],
    stock: crate::Stock,
    tip: DVec3,
    axis: DVec3,
    tool: &ToolSpec,
    tol: f64,
    clear: f64,
) -> Vec<(f64, f64, f64)> {
    let ix = crate::MeshIndex::build(mesh, stock);
    let g = Geo::of(tool, tol, clear);
    let r = g.r;
    let flute_end = g.flute;
    let step = (r * 0.8).clamp(0.5, 2.0);
    let mut out = Vec::new();
    let need_f = (r - tol).max(1e-6);
    let mut z = r;
    while z < flute_end {
        let d = ix.min_dist_signed(tip + axis * z, need_f) - need_f;
        out.push((z, r, d));
        z += step;
    }
    let taper = tool.taper_len();
    let mut z = flute_end;
    while z < tool.neck_top() + taper {
        let rr = tool.radius_at(z);
        let need = (rr + clear).max(1e-3);
        let d = ix.min_dist_signed(tip + axis * z, need) - need;
        out.push((z, rr, d));
        z += 0.3;
    }
    let shank_r = tool.shank_radius().max(r);
    let need_s = (shank_r + clear).max(1e-3);
    let mut z = (tool.neck_top() + taper).max(flute_end);
    while z <= tool.neck_top() + 20.0 {
        let d = ix.min_dist_signed(tip + axis * z, need_s) - need_s;
        out.push((z, shank_r, d));
        z += step;
    }
    out
}

/// 一串闸门共用的几何常量（刀半径、容差、刃长、**整根刀**）。
///
/// 打成一包只是因为它们出双入对——判据本身各是各的，注释在各函数上。
/// `tool` 是 §56 的教训：段扫掠只查刃部时，避空颈/柄在大转角段里横扫过
/// 叶片棱（叶轮 T53 的 399 点、啃 0.398 mm，"只球头 sd=+14" 证明啃的
/// 是刃部以外）——半径/颈长/柄径都必须从**出码用的同一把刀**取。
#[derive(Clone)]
struct Geo {
    r: f64,
    tol: f64,
    flute: f64,
    tool: ToolSpec,
    /// 颈/柄的避空间隙（`SurfaceOpts.shank_clearance_mm`）。
    clear: f64,
}

impl Geo {
    /// 从刀具与两个容差打包：`tol` = 过切容差（球/刃），`clear` = 颈/柄间隙。
    fn of(tool: &ToolSpec, tol: f64, clear: f64) -> Self {
        let r = tool.radius();
        Self {
            r,
            tol,
            flute: tool.flute_mm.max(r),
            tool: tool.clone(),
            clear,
        }
    }
}

/// 球刀从 `a`（姿态 `na`）扫到 `b`（姿态 `nb`）时，**会不会啃到设计面**。
///
/// 两头都要查：
///
/// * **球**：球心轨迹上每一点的最近距离 ≥ `R − tol`（两端是已过闸门的接受点，
///   只查中间）。
/// * **刃部圆柱**：这一条是实测逼出来的。刀轴沿法向时，刃长那一段圆柱是**横着**
///   躺在曲面旁边的：球在 `p` 相切，圆柱却顺着法向伸出去。两个相邻点的法向差
///   几十度时，这根圆柱在块与块之间扫出一个扇面，能把旁边的面啃掉 `R` 量级
///   （实测 0.67 mm < R 0.75）。3 轴 drop-cutter 里不存在这个问题，因为刀轴恒定。
fn sweep_ok(ix: &crate::MeshIndex, a: DVec3, na: DVec3, b: DVec3, nb: DVec3, g: &Geo) -> bool {
    // 刃部圆柱沿轴**密采样**（步长 ≤ 0.5R）。
    //
    // 早先在 1/4/8/12/16 mm 五个高度上查，漏了 0.49 mm：设计面点在 z=6，
    // 而采样在 4 和 8——两个采样点离它 2 mm，读数 2 mm ≫ R，判"安全"，
    // 可那个点在 z=6 处离轴只有 0.01 mm。**圆柱的判据必须沿轴连续**，
    // 不能靠稀疏的"高度"抽样。
    //
    // **位姿数随转角加密**：刀轴在段内 slerp，转角大时刀杆（长 `flute`）
    // 横向摆动 = flute·sin(θ/2·2)——固定 2 个刀杆采样位姿会漏掉中间的摆动
    // （实测叶轮 T53 段内转 33~55°，刀杆扫过叶片 0.40 mm）。
    //
    // 步长取 **2°** 是被"薄叶片"卡出来的口径：位姿之间的弦长 = z·Δθ，
    // 要保证**必有一个位姿落在叶片内部**（signed 闸门才拦得住），弦长必须
    // 小于叶片厚度——叶轮叶片 ~0.8 mm、刃顶 z≈16 mm → Δθ < 0.8/16 rad ≈ 2.9°，
    // 取 2°（弦 0.56 mm）留 40% 余量。4° 时弦 1.1 mm 可以**一步跨过** 0.8 mm
    // 的叶片——位姿两侧都在空气里、signed 全部读正。跨棱的段（~55°）要
    // 29 个位姿，只有它们付这个成本。
    let theta = na.dot(nb).clamp(-1.0, 1.0).acos();
    let n_t = (((theta / 0.035).ceil() as usize) + 1).clamp(4, 40);
    // 颈/柄的分层阈值：θ ≥ 10° 的段（大转角、颈/柄横扫风险——§56 的教训段
    // 都是 33~55°）**全位姿查**；小转角段只查**中点一次**。依据：段两端点的
    // 逐点接受闸门已保证端位姿干净，小转角（θ<10°）+ 短段（≤ link 0.8 mm）
    // 的中间读数近似端点插值，单谷情形中点位姿就能兜住。
    const TH_NECK: f64 = 10.0f64.to_radians();
    let full_neck = theta >= TH_NECK;
    for k in 0..n_t {
        let t = (k + 1) as f64 / (n_t as f64 + 1.0);
        // **刀轴按 slerp 插值**（与 `Pose::lerp_slerp` 同口径），球心再按
        // `tip + R·a` 现算——不能拿两端球心做 lerp：两端轴差 60° 时
        // lerp 出来的球心比真实位置短 `R(1−cos30°)`。
        let ax = slerp_axis(na, nb, t);
        let tip = a + (b - a) * t;
        // **整根刀**都查（球心 + 刃 + 等径避空颈 + 30° 锥 + Ø6 柄）——§56 的
        // 教训：只查到刃长时，避空颈锥面在 33° 转角段里横扫叶片棱、啃
        // 0.398 mm（叶轮 T53 的 399 点；"只球头 sd=+14" 证明啃的是刃部以外）。
        // 逐点闸门（接受目标点时）只保证**位姿处**干净，段中间的颈/柄扫掠
        // 只有这里管。
        if ball_flute_hits(ix, tip, ax, g) {
            return false;
        }
        if (full_neck || k * 2 + 1 == n_t) && neck_shank_hits(ix, tip, ax, g) {
            return false;
        }
    }
    true
}

/// 串链候选的**快速预检**：只查球心 + 刃部两个位姿的四个点。
///
/// 串链每个候选都要过一遍扫掠闸门，而被拒的候选（球面塞不下/刃部撞）多数
/// 在这些便宜点上就能否掉——通过的才付整根刀 + 全部位姿的完整检查
/// （`sweep_ok`，每位姿 ~117 个采样点）。省的是"反正要拒"的候选的全价。
fn sweep_ok_quick(ix: &crate::MeshIndex, a: DVec3, na: DVec3, b: DVec3, nb: DVec3, g: &Geo) -> bool {
    let need = (g.r - g.tol).max(1e-6);
    for t in [1.0 / 3.0, 2.0 / 3.0] {
        let ax = slerp_axis(na, nb, t);
        let tip = a + (b - a) * t;
        if ix.min_dist_signed(tip + ax * g.r, need) < need {
            return false;
        }
        for z in [(2.0 * g.r).min(g.flute), g.flute] {
            if ix.min_dist_signed(tip + ax * z, need) < need {
                return false;
            }
        }
    }
    true
}

/// 两个刀轴之间的球面插值（与 `Pose::lerp_slerp` 同一口径）。
fn slerp_axis(a: DVec3, b: DVec3, t: f64) -> DVec3 {
    let d = a.dot(b).clamp(-1.0, 1.0);
    let th = d.acos();
    if th < 1e-6 || th > std::f64::consts::PI - 1e-6 {
        return a;
    }
    let s = th.sin();
    (a * (((1.0 - t) * th).sin() / s) + b * ((t * th).sin() / s)).normalize()
}

/// 路径在 `kept[k]` 处的**转向角**，除以 60° 归一（`0` = 直行，`1` = 折返）。
///
/// 跨度取 ±2 块：单块方向在点铣的链上是噪声（相邻点间距 0.4 mm，抖动 0.1 mm
/// 就是 14° 的假转角），跨两块才看得出真的走向变化。
fn turn_at(tips: &[DVec3], kept: &[usize], k: usize) -> f64 {
    let last = kept.len() - 1;
    let back = k.min(2);
    let fwd = (k + 2).min(last);
    if back == 0 || fwd <= k {
        return 0.0;
    }
    let a = tips[kept[k]] - tips[kept[k - back]];
    let b = tips[kept[fwd]] - tips[kept[k]];
    if a.length_squared() < 1e-18 || b.length_squared() < 1e-18 {
        return 0.0;
    }
    let c = a.normalize().dot(b.normalize()).clamp(-1.0, 1.0);
    (c.acos() / 60f64.to_radians()).clamp(0.0, 1.0)
}

/// 切平面上的两个正交基。
fn tangent_basis(n: DVec3) -> (DVec3, DVec3) {
    let a = if n.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    let u = n.cross(a).normalize();
    let v = n.cross(u).normalize();
    (u, v)
}

/// **离散凹度** `κ·R`：0 = 平的或凸的，1 = 凹到与刀球同半径（球刚好塞满）。
///
/// 为什么需要它：进给自适应要一个"负载"的量，而"刀心到设计面的距离"在切点
/// 处恒等于 `R`（球与它相切），给不出啮合。凹度是能直接算、且**方向明确**的
/// 代理——凹处刀球被材料包住的角度更大，同样行距下去除量更大。
///
/// 算法：对每个邻居 `j` 算切平面偏距 `d_j = n_i·(p_j − p_i)`。曲面在 `p_i`
/// 处与切平面相切，所以 `d` 没有一阶项，`d_j = κ_n(方向)·|p_j−p_i|²/2 + O(r³)`，
/// 其中 `κ_n` 是那个方向的**法曲率**。于是 `2d_j/w_j` 就是该方向法曲率的估计，
/// **取所有邻居的最大值** = 最大主曲率 `κ₁`。
///
/// 为什么取 **max** 而不是平均：① 刀球塞不塞得下由最弯的那个方向决定
/// （凹到半径 = R 就进不去了，和球面闸门同一个判据）；② 叶轮的凹角是**槽**
/// （一个方向弯、另一个方向直），平均曲率会把它算小一半。
///
/// 邻域用 3×3×3 网格，半径夹在 `cell` 内（更远的点 `O(r³)` 截断误差大）；
/// 法向差 >45° 的邻居不要（相邻叶片的正反面会被混进来）。
fn concavity(pts: &[DVec3], nrm: &[DVec3], cell: f64, r: f64) -> Vec<f64> {
    let n = pts.len();
    let mut out = vec![0.0f64; n];
    if n == 0 {
        return out;
    }
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let c = cell.max(1e-6);
    let dim = |a: f64, b: f64| (((b - a) / c).floor() as i64 + 1).max(1) as usize;
    let (nx, ny, nz) = (dim(lo.x, hi.x), dim(lo.y, hi.y), dim(lo.z, hi.z));
    let idx = |p: DVec3| -> (i64, i64, i64) {
        (
            (((p.x - lo.x) / c).floor() as i64).clamp(0, nx as i64 - 1),
            (((p.y - lo.y) / c).floor() as i64).clamp(0, ny as i64 - 1),
            (((p.z - lo.z) / c).floor() as i64).clamp(0, nz as i64 - 1),
        )
    };
    let key = |i: i64, j: i64, k: i64| (k as usize * ny + j as usize) * nx + i as usize;
    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx * ny * nz];
    for (i, p) in pts.iter().enumerate() {
        let (a, b, d) = idx(*p);
        buckets[key(a, b, d)].push(i as u32);
    }
    let cos45 = 45f64.to_radians().cos();
    let h2 = c * c;
    // 太近的邻居（重合点 / 共边重复采样）`w→0`，比值会被放大成一个假尖角
    let near2 = (0.15 * c) * (0.15 * c);
    // per-i 完全独立（buckets 只读）→ 并行。叶轮 ~19 万点 × 27 邻域桶。
    out.par_iter_mut().enumerate().for_each(|(i, o)| {
        let (ni, pi) = (nrm[i], pts[i]);
        let (ci, cj, ck) = idx(pi);
        let mut best: f64 = 0.0;
        for dk in -1..=1 {
            let k = ck + dk;
            if k < 0 || k >= nz as i64 {
                continue;
            }
            for dj in -1..=1 {
                let j = cj + dj;
                if j < 0 || j >= ny as i64 {
                    continue;
                }
                for di in -1..=1 {
                    let ii = ci + di;
                    if ii < 0 || ii >= nx as i64 {
                        continue;
                    }
                    for &jj in &buckets[key(ii, j, k)] {
                        let jj = jj as usize;
                        if jj == i || nrm[jj].dot(ni) < cos45 {
                            continue;
                        }
                        let dp = pts[jj] - pi;
                        let w = dp.length_squared();
                        if w < near2 || w > h2 {
                            continue;
                        }
                        let kn = 2.0 * ni.dot(dp) / w;
                        if kn > best {
                            best = kn;
                        }
                    }
                }
            }
        }
        // 乘 R：无量纲的"凹到几分之一个刀球"
        *o = (best * r).clamp(0.0, 1.0);
    });
    out
}

/// **局部流向场**：每点沿"曲率最小"的切向（等参线 / 等高线的方向）。
///
/// 为什么要这个：串链原本只按"最近"连，走出来的是一条条随机折线——方向随时
/// 掉头，行距由残料点密度决定。沿曲率小的方向走有两件好处：
///
/// * 刀路**顺**，块内刀轴转动小（刃部圆柱扫出来的误差也小）；
/// * 相邻两条链之间的行距近似恒定 = 刀花均匀，而不是有的地方叠切、有的地方漏。
///
/// 做法：邻域内法向差在切平面上的分布做 2×2 结构张量
/// `M = Σ (δn·u, δn·v)ᵀ(δn·u, δn·v)`。它的**大**特征值方向是法向变化最快的
/// 方向（横跨棱线），**小**特征值方向就是流线方向。
///
/// 平坦处张量退化（法向几乎不变），返回任意切向——那种地方走哪个方向都一样，
/// 由 `chain` 的连续性项决定。
fn flow_field(pts: &[DVec3], nrm: &[DVec3], cell: f64) -> Vec<DVec3> {
    let n = pts.len();
    let mut out = vec![DVec3::ZERO; n];
    if n == 0 {
        return out;
    }
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let c = cell.max(1e-6);
    let dim = |a: f64, b: f64| (((b - a) / c).floor() as i64 + 1).max(1) as usize;
    let (nx, ny, nz) = (dim(lo.x, hi.x), dim(lo.y, hi.y), dim(lo.z, hi.z));
    let idx = |p: DVec3| -> (i64, i64, i64) {
        (
            (((p.x - lo.x) / c).floor() as i64).clamp(0, nx as i64 - 1),
            (((p.y - lo.y) / c).floor() as i64).clamp(0, ny as i64 - 1),
            (((p.z - lo.z) / c).floor() as i64).clamp(0, nz as i64 - 1),
        )
    };
    let key = |i: i64, j: i64, k: i64| (k as usize * ny + j as usize) * nx + i as usize;
    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx * ny * nz];
    for (i, p) in pts.iter().enumerate() {
        let (a, b, d) = idx(*p);
        buckets[key(a, b, d)].push(i as u32);
    }

    for i in 0..n {
        let n0 = nrm[i];
        let (u, v) = tangent_basis(n0);
        let (mut aa, mut bb, mut cc) = (0.0f64, 0.0f64, 0.0f64);
        let (ci, cj, ck) = idx(pts[i]);
        for dk in -1..=1 {
            let k = ck + dk;
            if k < 0 || k >= nz as i64 {
                continue;
            }
            for dj in -1..=1 {
                let j = cj + dj;
                if j < 0 || j >= ny as i64 {
                    continue;
                }
                for di in -1..=1 {
                    let ii = ci + di;
                    if ii < 0 || ii >= nx as i64 {
                        continue;
                    }
                    for &jj in &buckets[key(ii, j, k)] {
                        let jj = jj as usize;
                        if jj == i {
                            continue;
                        }
                        // **只取同面的邻居**：叶片厚度 2~3 mm，1.5 mm 的邻域很容易
                        // 跨到背面去，那里的法向差接近 180°，会把结构张量搅成一团
                        // 噪声（实测叶轮上沿流向反而切长 +3%、过切 +0.2 点）。
                        if nrm[jj].dot(n0) < 45f64.to_radians().cos() {
                            continue;
                        }
                        let dn = nrm[jj] - n0;
                        let x = dn.dot(u);
                        let y = dn.dot(v);
                        aa += x * x;
                        bb += x * y;
                        cc += y * y;
                    }
                }
            }
        }
        let tr = aa + cc;
        if tr < 1e-12 {
            out[i] = u;
            continue;
        }
        // 小特征值 + 对应特征向量（2×2 对称，闭式解）
        let det = aa * cc - bb * bb;
        let disc = (tr * tr * 0.25 - det).max(0.0).sqrt();
        let l_min = tr * 0.5 - disc;
        let (mut ex, mut ey) = (bb, -(aa - l_min));
        if ex * ex + ey * ey < 1e-18 {
            ex = 1.0;
            ey = 0.0;
        }
        let m = (ex * ex + ey * ey).sqrt();
        out[i] = (u * (ex / m) + v * (ey / m)).normalize();
    }

    // 平滑两轮：单点的结构张量在叶片这种扭面上噪声大，取邻居平均（投影回切平面）。
    // 不投影会慢慢长出法向分量，链就会"飘"离曲面。
    for _ in 0..2 {
        let mut next = out.clone();
        for i in 0..n {
            let (ci, cj, ck) = idx(pts[i]);
            let (u, v) = tangent_basis(nrm[i]);
            let (mut sx, mut sy, mut cn) = (0.0f64, 0.0f64, 0.0f64);
            for dk in -1..=1 {
                let k = ck + dk;
                if k < 0 || k >= nz as i64 {
                    continue;
                }
                for dj in -1..=1 {
                    let j = cj + dj;
                    if j < 0 || j >= ny as i64 {
                        continue;
                    }
                    for di in -1..=1 {
                        let ii = ci + di;
                        if ii < 0 || ii >= nx as i64 {
                            continue;
                        }
                        for &jj in &buckets[key(ii, j, k)] {
                            let jj = jj as usize;
                            if nrm[jj].dot(nrm[i]) < 45f64.to_radians().cos() {
                                continue;
                            }
                            let f = out[jj];
                            // 流线方向无正负：统一到与当前方向同侧再平均
                            let sgn = if f.dot(out[i]) < 0.0 { -1.0 } else { 1.0 };
                            sx += f.dot(u) * sgn;
                            sy += f.dot(v) * sgn;
                            cn += 1.0;
                        }
                    }
                }
            }
            if cn > 0.0 && (sx * sx + sy * sy) > 1e-12 {
                next[i] = (u * sx + v * sy).normalize();
            }
        }
        out = next;
    }
    out
}

/// 最近邻串链，带**方向偏好**（避免来回锯齿）和**扫掠闸门**（不许穿墙连）。
///
/// 返回的是 `pts` 的下标序列。网格哈希的格边长 = `link`，所以邻居必在
/// 3×3×3 的邻格内——不需要多环扩张。
fn chain(
    pts: &[DVec3],
    nrm: &[DVec3],
    link: f64,
    ix: &crate::MeshIndex,
    g: &Geo,
    flow: Option<&[DVec3]>,
) -> Vec<Vec<usize>> {
    let n = pts.len();
    if n == 0 {
        return Vec::new();
    }
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        lo = DVec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = DVec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let cell = link.max(1e-6);
    let dim = |a: f64, b: f64| (((b - a) / cell).floor() as i64 + 1).max(1) as usize;
    let (nx, ny, nz) = (
        dim(lo.x, hi.x),
        dim(lo.y, hi.y),
        dim(lo.z, hi.z),
    );
    let idx = |p: DVec3| -> (i64, i64, i64) {
        (
            (((p.x - lo.x) / cell).floor() as i64).clamp(0, nx as i64 - 1),
            (((p.y - lo.y) / cell).floor() as i64).clamp(0, ny as i64 - 1),
            (((p.z - lo.z) / cell).floor() as i64).clamp(0, nz as i64 - 1),
        )
    };
    let key = |i: i64, j: i64, k: i64| (k as usize * ny + j as usize) * nx + i as usize;
    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx * ny * nz];
    for (i, p) in pts.iter().enumerate() {
        let (a, b, c) = idx(*p);
        buckets[key(a, b, c)].push(i as u32);
    }

    let mut used = vec![false; n];
    let mut runs: Vec<Vec<usize>> = Vec::new();
    let mut cand: Vec<(f64, usize)> = Vec::new();
    for seed in 0..n {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        let mut run = vec![seed];
        let mut cur = seed;
        let mut dir: Option<DVec3> = None;
        loop {
            let (ci, cj, ck) = idx(pts[cur]);
            cand.clear();
            for dk in -1..=1 {
                let k = ck + dk;
                if k < 0 || k >= nz as i64 {
                    continue;
                }
                for dj in -1..=1 {
                    let j = cj + dj;
                    if j < 0 || j >= ny as i64 {
                        continue;
                    }
                    for di in -1..=1 {
                        let i = ci + di;
                        if i < 0 || i >= nx as i64 {
                            continue;
                        }
                        for &c in &buckets[key(i, j, k)] {
                            let c = c as usize;
                            if used[c] {
                                continue;
                            }
                            let v = pts[c] - pts[cur];
                            let d = v.length();
                            if d > link || d < 1e-9 {
                                continue;
                            }
                            // **刀轴差 > 45° 不连**：那是另一个面（棱的对面），或者姿态急转。
                            // 连过去意味着一个块里刀轴转 60° 以上，刃部圆柱会扫平
                            // 中间那条棱——实测最深 0.67 mm 就是这么来的。
                            if nrm[cur].dot(nrm[c]) < 45f64.to_radians().cos() {
                                continue;
                            }
                            // 打分 = 距离 × (1 + 偏离流线的罚 + 偏离上一步的罚)。
                            //
                            //   * **流线项**（权 1.5）：沿曲率最小的方向走，链才长、
                            //     行距才均匀。方向无正负，取 `abs`。
                            //   * **连续项**（权 0.8）：压住"回头"，免得在一条流线上
                            //     来回锯齿。两项一起才定得住 180° 的歧义。
                            let vn = v * (1.0 / d);
                            let align = match flow {
                                Some(f) => vn.dot(f[cur]).abs().clamp(0.0, 1.0),
                                None => 1.0,
                            };
                            let cont = match dir {
                                Some(u) => vn.dot(u).max(0.0),
                                None => 1.0,
                            };
                            let score = d * (1.0 + 1.5 * (1.0 - align) + 0.8 * (1.0 - cont));
                            cand.push((score, c));
                        }
                    }
                }
            }
            // 由近到远试：**第一条几何上走得通的才算数**。穿墙的近邻再近也不能要——
            // 那正是上一版啃出 2.92 mm 的原因。
            cand.sort_by(|a, b| a.0.total_cmp(&b.0));
            let next = cand
                .iter()
                .find(|(_, c)| {
                    sweep_ok_quick(ix, pts[cur], nrm[cur], pts[*c], nrm[*c], g)
                        && sweep_ok(ix, pts[cur], nrm[cur], pts[*c], nrm[*c], g)
                })
                .map(|(_, c)| *c);
            match next {
                Some(c) => {
                    let v = pts[c] - pts[cur];
                    if v.length() > 1e-9 {
                        dir = Some(v.normalize());
                    }
                    used[c] = true;
                    run.push(c);
                    cur = c;
                }
                None => break,
            }
        }
        runs.push(run);
    }
    runs
}

/// 链与链之间的**快移**会不会从工件里穿过去。
///
/// 这一段是 5 轴联动快移（`G0` 带 A/C），仿真里不计切削，所以过切表上看不见它；
/// 但实机会真的撞上去。这里按 1 mm 步距扫一遍刀尖轨迹 + 刃部圆柱，只用于**计数
/// 报警**，不改刀路。
fn link_ok(ix: &crate::MeshIndex, a: DVec3, na: DVec3, b: DVec3, nb: DVec3, g: &Geo) -> bool {
    let len = (b - a).length();
    // 步距 0.5 mm 且至少 2 段：1 mm 的链按 1 mm 步距只会取到两个端点，
    // 中间那堵墙正好从采样缝里漏过去（测试里抓到的）。
    let n = ((len / 0.5).ceil() as usize).clamp(2, 128);
    for k in 0..=n {
        let t = k as f64 / n as f64;
        let ax = slerp_axis(na, nb, t);
        let tip = a + (b - a) * t;
        // 整根刀（同 `sweep_ok`）——快移虽然不切削，擦到工件照样是事故。
        // 颈/柄只查中点：link 只用于计数报警，不值得每位姿付 ~50 查询。
        if ball_flute_hits(ix, tip, ax, g) {
            return false;
        }
        if k * 2 == n && neck_shank_hits(ix, tip, ax, g) {
            return false;
        }
    }
    true
}

/// 链间转移：**能安全通过的最小沿轴退刀量**。
///
/// 固定 `lift = 刃长 + 2 = 18 mm` 是"刀杆整根退出"的安全值，但绝大多数链间
/// 转移 3~6 mm 就够——叶轮 1 有 1427 条链，每条省 10 mm 就是 14 m 空移。
/// 从 3 mm 往上试，第一个通过 `link_ok` 的就是答案；全都不行返回上限
/// （那种情况仍会擦料，`st.link_unsafe` 照旧计数）。
fn min_link_lift(
    ix: &crate::MeshIndex,
    a: DVec3,
    na: DVec3,
    b: DVec3,
    nb: DVec3,
    g: &Geo,
    max_lift: f64,
) -> f64 {
    let mut d = 3.0f64.min(max_lift);
    while d <= max_lift {
        if link_ok(ix, a + na * d, na, b + nb * d, nb, g) {
            return d;
        }
        d += 2.0;
    }
    max_lift
}

/// 贪心排序：从位置最高的一条链出发，每次挑"起点或终点离当前位置最近"的链，
/// 必要时整条反向（折线反着走是合法的）。
///
/// 链的顺序原本是种子顺序（`for seed in 0..n`），跟空间分布毫无关系。
/// 叶轮 1 有 1225 条链，按种子顺序的平均跳距比切长还长。
fn order_runs(
    runs: Vec<Vec<usize>>,
    tips: &[DVec3],
    axs: &[DVec3],
    ix: &crate::MeshIndex,
    g: &Geo,
    lift: f64,
) -> Vec<Vec<usize>> {
    let n = runs.len();
    if n <= 1 {
        return runs;
    }
    let mut used = vec![false; n];
    let mut out: Vec<Vec<usize>> = Vec::with_capacity(n);
    // 起点：z 最高的链（从上面往下做，落差小）
    let cur = (0..n)
        .max_by(|a, b| {
            let za = tips[runs[*a][0]].z;
            let zb = tips[runs[*b][0]].z;
            za.total_cmp(&zb)
        })
        .unwrap_or(0);
    used[cur] = true;
    out.push(runs[cur].clone());
    let mut at = *runs[cur].last().unwrap();
    for _ in 1..n {
        // 先按距离排出前三名，再**优先取快移不擦料的那条**。只排最近的会撞：
        // 实测叶轮 1 有 46/1225 条链间快移会从叶片的另一侧穿过去。
        let mut cand: Vec<(f64, usize, bool)> = Vec::new();
        for (k, r) in runs.iter().enumerate() {
            if used[k] {
                continue;
            }
            let d0 = (tips[r[0]] - tips[at]).length();
            let d1 = (tips[*r.last().unwrap()] - tips[at]).length();
            cand.push(if d0 <= d1 {
                (d0, k, false)
            } else {
                (d1, k, true)
            });
        }
        cand.sort_by(|a, b| a.0.total_cmp(&b.0));
        let pick = cand
            .iter()
            .take(3)
            .find(|(_, k, rev)| {
                let r = &runs[*k];
                let (s0, s1) = if *rev {
                    (*r.last().unwrap(), r[0])
                } else {
                    (r[0], *r.last().unwrap())
                };
                link_ok(
                    ix,
                    tips[at] + axs[at] * lift,
                    axs[at],
                    tips[s0] + axs[s0] * lift,
                    axs[s0],
                    g,
                ) && {
                    let _ = s1;
                    true
                }
            })
            .or_else(|| cand.first());
        let Some((_, k, rev)) = pick else { break };
        let (k, rev) = (*k, *rev);
        used[k] = true;
        let mut r = runs[k].clone();
        if rev {
            r.reverse();
        }
        at = *r.last().unwrap();
        out.push(r);
    }
    out
}

/// 沿弧长抽稀：每 `step` 留一个点，**首尾必留**。
///
/// 跳过中间点之前要确认"直连这一段仍然安全"——抽稀把 0.6 mm 的步子拉成 1.2 mm，
/// 在凹凸处正是啃刀的高发点。
fn decimate(
    run: &[usize],
    pts: &[DVec3],
    nrm: &[DVec3],
    step: f64,
    ix: &crate::MeshIndex,
    g: &Geo,
) -> Vec<usize> {
    if run.len() <= 2 || step <= 0.0 {
        return run.to_vec();
    }
    let mut out = vec![run[0]];
    let mut last = run[0];
    let mut acc = 0.0;
    for w in run.windows(2) {
        acc += (pts[w[1]] - pts[w[0]]).length();
        if acc < step {
            continue;
        }
        // 抽稀不能把"两段各转 30°"合成"一块转 60°"：轴摆太大时机器抖，
        // 刃部圆柱还会扫出一片（实测块内轴摆 60° 啃 0.50 mm）。
        let ok_axis = nrm[last].dot(nrm[w[1]]) >= 30f64.to_radians().cos();
        if ok_axis && sweep_ok(ix, pts[last], nrm[last], pts[w[1]], nrm[w[1]], g) {
            out.push(w[1]);
            last = w[1];
            acc = 0.0;
        } else {
            // 直连不安全：把中间那点补回来（宁可多一块，也不啃）
            out.push(w[0]);
            last = w[0];
            acc = (pts[w[1]] - pts[w[0]]).length();
        }
    }
    if *out.last().unwrap() != *run.last().unwrap() {
        out.push(*run.last().unwrap());
    }
    out
}

/// 极简 G 代码出口：只写 G0/G1 和 A/C，**记住上一个点**好算切长。
struct Sink {
    out: String,
    last: Option<DVec3>,
    rapid_mm: f64,
    cut_mm_extra: f64,
}

impl Sink {
    fn new() -> Self {
        Self {
            out: String::with_capacity(1 << 16),
            last: None,
            rapid_mm: 0.0,
            cut_mm_extra: 0.0,
        }
    }

    fn line(&mut self, s: &str) {
        self.out.push_str(s);
        self.out.push('\n');
        // 从 X/Y/Z 字里取回目标点，用于下一次算距离
        let mut p = self.last;
        let mut q = p.unwrap_or(DVec3::ZERO);
        for w in s.split_whitespace() {
            let mut ch = w.chars();
            let Some(c) = ch.next() else { continue };
            let rest: String = ch.collect();
            let Ok(v) = rest.parse::<f64>() else {
                continue;
            };
            match c {
                'X' => q.x = v,
                'Y' => q.y = v,
                'Z' => q.z = v,
                _ => continue,
            }
            p = Some(q);
        }
        if let (Some(a), Some(b)) = (self.last, p) {
            let d = (b - a).length();
            if s.starts_with("G0") {
                self.rapid_mm += d;
            } else {
                self.cut_mm_extra += d;
            }
        }
        self.last = p;
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    fn plate() -> Vec<Tri> {
        let z = 5.0;
        let p = |x: f64, y: f64| DVec3::new(x, y, z);
        vec![
            [p(0.0, 0.0), p(20.0, 0.0), p(20.0, 20.0)],
            [p(0.0, 0.0), p(20.0, 20.0), p(0.0, 20.0)],
        ]
    }

    fn stock() -> Stock {
        Stock {
            min: DVec3::ZERO,
            max: DVec3::new(20.0, 20.0, 10.0),
        }
    }

    /// 凹度的方向和量纲：槽内壁 `κR = R/ρ`，翻过来（凸）必须是 0。
    ///
    /// 取 max 主曲率而不是平均：槽是一个方向弯、另一个方向直，平均会小一半
    /// （ρ=2 时平均给 0.125、max 给 0.25，而刀球只沿弯的那个方向被包住）。
    #[test]
    fn concavity_reads_the_groove_radius() {
        let (rho, r) = (2.0f64, 0.5f64);
        let mut pts = Vec::new();
        let mut nrm = Vec::new();
        for iy in 0..=8 {
            let y = iy as f64 * 0.2;
            // 槽底 ±40°：再往外法向转得太快，已经不是同一条"面"了
            for it in 0..=40 {
                let th = -0.7 + it as f64 * 0.035;
                let (s, c) = (th.sin(), th.cos());
                pts.push(DVec3::new(rho * s, y, -rho * c));
                // 外法向指向槽心（凹）
                nrm.push(DVec3::new(-s, 0.0, c));
            }
        }
        let k = concavity(&pts, &nrm, 0.6, r);
        // 槽底正中那一行
        let mid = 4 * 41 + 20;
        assert!(
            (k[mid] - r / rho).abs() < 0.02,
            "槽底凹度应为 R/ρ = {}，实测 {}",
            r / rho,
            k[mid]
        );
        // 法向翻过来 → 凸面 → 凹度归零
        let out: Vec<DVec3> = nrm.iter().map(|n| -*n).collect();
        let k2 = concavity(&pts, &out, 0.6, r);
        assert!(k2.iter().all(|v| *v == 0.0), "凸面不该有凹度");
    }

    /// 平板上法向 +Z 的点：刀轴就是 +Z，刀尖应恰好落在**设计面**上。
    #[test]
    fn flat_plate_program_puts_the_tip_on_the_surface() {
        let targets: Vec<(DVec3, DVec3)> = (0..3)
            .flat_map(|i| {
                (0..3).map(move |j| {
                    (
                        DVec3::new(5.0 + i as f64, 5.0 + j as f64, 5.0),
                        DVec3::new(0.0, 0.0, 1.0),
                    )
                })
            })
            .collect();
        let mut o = SurfaceOpts::new(ToolSpec::ball(0, 2.0, 10.0), 7, stock());
        o.link_mm = 2.0;
        o.out_step_mm = 0.0; // 不抽稀，逐点验证
        let out = generate_surface(&plate(), stock(), &targets, &o).expect("应能生成");
        assert_eq!(out.stats.accepted, 9, "平板法向点上不该有拒绝");
        assert!(out.gcode.contains("G43.4 H1"));
        assert_eq!(out.stats.moves, 9);
        // 每个 G1 的 Z 必须是 5.0000（刀尖在面上，不是 R 之上）
        let g1: Vec<&str> = out
            .gcode
            .lines()
            .filter(|l| l.starts_with("G1") && l.contains("Z5.0000"))
            .collect();
        assert_eq!(g1.len(), 9, "9 个点都该有 Z5.0000：\n{}", out.gcode);
    }

    /// 朝下的面（法向 −Z，A = 180°）**不放弃，改成把刀轴倾斜到限位**。
    ///
    /// 球是各向同性的：只要刀尖补偿 `R(n − a)`，球在同一个点上照样相切。
    /// 这一条把"A 超行程"从"切不了"变成"换个姿态还能切"——叶轮 1 的 45% 残料
    /// 就栽在 A 超程上，改之前直接丢掉，改之后全部变成候选。
    #[test]
    fn downward_normals_are_tilted_into_the_a_stroke() {
        // 一小片天花板，目标点取在它下面（法向朝下）
        let p = DVec3::new(0.05, 0.05, 5.0);
        // 绕序外向（法向 −Z）：signed 距离按外法向定符号，材质隐含在天花板上方
        let mesh = vec![[
            DVec3::new(0.0, 0.0, 5.0),
            DVec3::new(0.0, 0.1, 5.0),
            DVec3::new(0.1, 0.0, 5.0),
        ]];
        let targets = vec![(p, DVec3::new(0.0, 0.0, -1.0))];
        let o = SurfaceOpts::new(ToolSpec::ball(0, 1.0, 10.0), 7, stock());
        let out = generate_surface(&mesh, stock(), &targets, &o).expect("生成不报错");
        assert_eq!(out.stats.a_rejected, 0, "不该因为 A 超程丢掉这个点");
        assert_eq!(out.stats.accepted, 1);
        // 刀轴拉到 A = 110°（机床极限），而不是 180°
        assert!(
            out.gcode.contains("A110.000"),
            "刀轴该拉到 A110；实际：\n{}",
            out.gcode
        );
        // 刀尖偏到 `p + R(n − a)`：R = 0.5、a = (sin110°,0,cos110°)
        let a = DVec3::new(110f64.to_radians().sin(), 0.0, 110f64.to_radians().cos());
        let tip = p + (DVec3::new(0.0, 0.0, -1.0) - a) * 0.5;
        let up = tip + a * o.lift_mm;
        let want = format!("G0 X{:.4} Y{:.4} Z{:.4}", up.x, up.y, up.z);
        assert!(
            out.gcode.contains(&want),
            "刀尖该偏到 ({:.3},{:.3},{:.3}) 上方；实际：\n{}",
            tip.x,
            tip.y,
            tip.z,
            out.gcode
        );
    }

    /// 窄 V 槽：**同一个点，Ø4 塞不进、Ø0.2 塞得进**。
    ///
    /// 这是"可达不等于能切"的几何本体：槽的张开角固定，刀径越大，球在贴到
    /// 一侧时越早碰到另一侧（`R·|n₁·n₂| = 0.866R`）。闸门必须按刀径判。
    #[test]
    fn gouge_gate_is_tool_dependent() {
        // 两条 15° 斜面拼成 30° 的 V 槽，交线在原点、沿 x
        let u = DVec3::new(0.0, 15f64.to_radians().sin(), 15f64.to_radians().cos());
        let p0 = DVec3::ZERO;
        let p1 = DVec3::new(2.0, 0.0, 0.0);
        let p2 = u * 2.0;
        let q2 = DVec3::new(0.0, -p2.y, p2.z);
        let mesh = vec![[p0, p1, p2], [p0, q2, p1]];
        let n1 = (p1 - p0).cross(p2 - p0).normalize();
        assert!((n1 - DVec3::new(0.0, -0.9659, 0.2588)).length() < 1e-3, "{n1:?}");
        // 目标点取在斜面 1 上（离槽底 0.5 mm），法向就是 n1
        let pt = (p0 + u * 0.5, n1);

        let big = SurfaceOpts::new(ToolSpec::ball(0, 4.0, 10.0), 7, stock());
        let small = SurfaceOpts::new(ToolSpec::ball(0, 0.2, 10.0), 7, stock());
        let ob = generate_surface(&mesh, stock(), &[pt], &big).expect("生成不报错");
        let os = generate_surface(&mesh, stock(), &[pt], &small).expect("生成不报错");
        assert_eq!(ob.stats.gouge_rejected, 1, "Ø4 球贴住一侧就会啃到另一侧");
        assert_eq!(os.stats.accepted, 1, "Ø0.2 球塞得进 30° 槽");
    }

    /// 串链：3×3 点阵必须连成**一条**折线（不是 9 条单点），且每点只出现一次。
    #[test]
    fn chaining_links_a_patch_into_one_run() {
        let pts: Vec<DVec3> = (0..3)
            .flat_map(|i| (0..3).map(move |j| DVec3::new(i as f64, j as f64, 0.0)))
            .collect();
        let nrm = vec![DVec3::Z; pts.len()];
        // 空网格 → `min_dist` 恒为 ∞ → 扫掠闸门全放行（这里只测拓扑）
        let ix = crate::MeshIndex::build(&[], stock());
        let g = Geo::of(&ToolSpec::ball(0, 1.0, 10.0), 0.03, 0.3);
        let runs = chain(&pts, &nrm, 1.5, &ix, &g, None);
        assert_eq!(runs.len(), 1, "3×3 点阵在 link=1.5 下应连成一条：{runs:?}");
        assert_eq!(runs[0].len(), 9);
        let mut seen = runs[0].clone();
        seen.sort_unstable();
        assert_eq!(seen, (0..9).collect::<Vec<_>>());
    }

    /// 扫掠闸门：**两点再近，中间隔着料就不能连**。
    ///
    /// 桌腿件的实测教训：接受点和被拒点是混在一起的，直连穿过凸棱时球从中间
    /// 钻过去，237 块里啃出 6.8% 过切、最深 2.92 mm。
    #[test]
    fn chaining_refuses_to_cross_a_ridge() {
        // 一堵薄墙在 x = 0.5，两侧各一个点（相距 1 mm，link=1.5 够近）
        let wall = vec![
            [
                DVec3::new(0.5, -5.0, -5.0),
                DVec3::new(0.5, 5.0, -5.0),
                DVec3::new(0.5, 5.0, 5.0),
            ],
            [
                DVec3::new(0.5, -5.0, -5.0),
                DVec3::new(0.5, 5.0, 5.0),
                DVec3::new(0.5, -5.0, 5.0),
            ],
        ];
        let pts = vec![
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
        ];
        let nrm = vec![DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0)];
        let ix = crate::MeshIndex::build(&wall, stock());
        let g = Geo::of(&ToolSpec::ball(0, 1.0, 10.0), 0.03, 0.3);
        let runs = chain(&pts, &nrm, 1.5, &ix, &g, None);
        assert_eq!(runs.len(), 2, "隔墙的两点必须分成两条链：{runs:?}");
    }

    /// 候选刀轴：**第一个是法向本身**，全都在 A 行程内、都在设计面外侧。
    #[test]
    fn axis_candidates_start_at_the_normal_and_stay_feasible() {
        let n = DVec3::new(0.3, -0.4, 0.866).normalize();
        let c = axis_candidates(n, 110.0, 40.0, None);
        assert_eq!(c[0], n, "第一个候选必须是不歪的法向");
        assert_eq!(c.len(), 17, "1 + 8（40°）+ 8（24°）= 17");
        for a in &c {
            assert!((a.length() - 1.0).abs() < 1e-9, "必须是单位向量：{a:?}");
            assert!(a.dot(n) > 0.0, "刀轴不能跑到设计面背后：{a:?}");
            assert!(crate::rotary::ac_of(*a).0 <= 110.0 + 1e-9, "不能超 A 行程：{a:?}");
        }
        // 关掉倾斜就只剩一个
        assert_eq!(axis_candidates(n, 110.0, 0.0, None).len(), 1);
        // 给一个偏好轴，同圈里离它最近的排最前
        let prefer = DVec3::new(-1.0, 0.0, 0.0);
        let c2 = axis_candidates(DVec3::Z, 110.0, 40.0, Some(prefer));
        let ring1 = &c2[1..9];
        let best = ring1
            .iter()
            .map(|a| a.dot(prefer))
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (c2[1].dot(prefer) - best).abs() < 1e-12,
            "同圈里该把最贴偏好轴的那个排最前：{:?}",
            c2[1]
        );
    }

    /// **刀杆挡住时歪刀轴能救**：高墙旁边的地面点，竖直下去刀杆贴墙太近，
    /// 往反方向歪 40° 就让开了。这是叶轮上 69% 的残料点靠的那一招。
    #[test]
    fn tilted_axis_clears_a_tall_wall_the_normal_axis_hits() {
        // 地面 z=0（x ∈ [-2, 1.2]），一堵高墙在 x = 1.2（0..10 高）
        let f = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
        let mesh = vec![
            [f(-2.0, -2.0, 0.0), f(1.2, -2.0, 0.0), f(1.2, 2.0, 0.0)],
            [f(-2.0, -2.0, 0.0), f(1.2, 2.0, 0.0), f(-2.0, 2.0, 0.0)],
            [f(1.2, -2.0, 0.0), f(1.2, 2.0, 0.0), f(1.2, 2.0, 10.0)],
            [f(1.2, -2.0, 0.0), f(1.2, 2.0, 10.0), f(1.2, -2.0, 10.0)],
        ];
        let st = Stock {
            min: DVec3::new(-2.0, -2.0, 0.0),
            max: DVec3::new(1.2, 2.0, 10.0),
        };
        let targets = vec![(DVec3::new(0.5, 0.0, 0.0), DVec3::Z)];
        // 刃长 3：3 mm 以下是刃部（离墙 0.7 > 0.5−0.03，没问题），
        // 3 mm 以上是刀杆（要 0.5+0.3=0.8，0.7 差一点 → 竖直必撞）
        let tool = ToolSpec::ball(1, 1.0, 3.0);

        let mut o = SurfaceOpts::new(tool.clone(), 1, st);
        o.max_tilt_deg = 0.0;
        let flat = generate_surface(&mesh, st, &targets, &o).expect("不报错");
        assert_eq!(flat.stats.shank_rejected, 1, "竖直下去刀杆该撞墙");

        o.max_tilt_deg = 40.0;
        let tilted = generate_surface(&mesh, st, &targets, &o).expect("不报错");
        assert_eq!(tilted.stats.accepted, 1, "歪 40° 该能进");
        assert_eq!(tilted.stats.axis_retried, 1, "而且必须是靠歪刀轴进来的");
    }

    /// 链排序：**按空间就近**，必要时整条反向。
    ///
    /// 种子顺序是 `for seed in 0..n`，跟空间无关；1225 条链按种子顺序跳，
    /// 空移比切长还长（§46）。
    #[test]
    fn runs_are_reordered_by_proximity_and_may_be_reversed() {
        // 三条链：x≈0（z 最高，作起点）、x≈10、x≈20
        let tips: Vec<DVec3> = vec![
            DVec3::new(0.0, 0.0, 2.0),
            DVec3::new(0.0, 1.0, 2.0),
            DVec3::new(0.0, 2.0, 2.0),
            DVec3::new(20.0, 0.0, 0.0),
            DVec3::new(20.0, 1.0, 0.0),
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::new(10.0, 1.0, 0.0),
        ];
        // 链 A: 0,1,2（x=0）；链 B: 3,4（x=20）；链 C: 5,6（x=10，起点离 A 的终点远）
        let runs = vec![vec![0, 1, 2], vec![3, 4], vec![5, 6]];
        let axs = vec![DVec3::Z; tips.len()];
        let ix = crate::MeshIndex::build(&[], stock());
        let g = Geo::of(&ToolSpec::ball(0, 1.0, 10.0), 0.03, 0.3);
        let out = order_runs(runs, &tips, &axs, &ix, &g, 12.0);
        assert_eq!(out.len(), 3);
        // 从 z 最高的一条开始 → 链 A（z=0/1/2 里最高 2）……
        // 但更关键的是：A 之后必须接离它最近的 C（x=10，可反向），最后 B
        assert_eq!(out[0], vec![0, 1, 2], "起点链：{:?}", out[0]);
        assert_eq!(out[1], vec![6, 5], "应接最近的 C 且反向：{:?}", out[1]);
        assert_eq!(out[2], vec![3, 4], "最后才是 B：{:?}", out[2]);
    }

    /// 链间快移**不许穿墙**：两个点分别在薄墙两侧时判"擦料"。
    #[test]
    fn link_check_refuses_to_cross_a_wall() {
        let wall = vec![
            [
                DVec3::new(0.5, -5.0, -5.0),
                DVec3::new(0.5, 5.0, -5.0),
                DVec3::new(0.5, 5.0, 5.0),
            ],
            [
                DVec3::new(0.5, -5.0, -5.0),
                DVec3::new(0.5, 5.0, 5.0),
                DVec3::new(0.5, -5.0, 5.0),
            ],
        ];
        let ix = crate::MeshIndex::build(&wall, stock());
        let g = Geo::of(&ToolSpec::ball(0, 1.0, 10.0), 0.03, 0.3);
        let a = DVec3::new(0.0, 0.0, 0.0);
        let same = DVec3::new(0.0, 0.0, 2.0);
        let across = DVec3::new(1.0, 0.0, 0.0);
        assert!(link_ok(&ix, a, DVec3::Z, same, DVec3::Z, &g), "同侧该放行");
        assert!(
            !link_ok(&ix, a, DVec3::Z, across, DVec3::Z, &g),
            "跨墙该判擦料"
        );
    }

    /// `min_link_lift`：平地上一小段转移只需要**最小退刀量**，不是固定 18 mm。
    /// §55 余党的几何复刻：刃部圆柱**埋进材质 0.5 mm**——无符号距离 0.5 ≥
    /// R − 0.03 = 0.37 判"安全"（实测叶轮 T53 的 407 点就是这么漏的），
    /// signed 读 −0.5 必须拦下。
    #[test]
    fn sweep_gate_catches_a_cylinder_buried_in_a_wall() {
        // 地板 z=2（法向 +Z）+ 一堵 x∈[3,4]、z∈[4,10] 的墙（侧面外向缠绕）
        let mut mesh: Vec<[DVec3; 3]> = Vec::new();
        mesh.push([
            DVec3::new(0.0, 0.0, 2.0),
            DVec3::new(20.0, 0.0, 2.0),
            DVec3::new(20.0, 20.0, 2.0),
        ]);
        mesh.push([
            DVec3::new(0.0, 0.0, 2.0),
            DVec3::new(20.0, 20.0, 2.0),
            DVec3::new(0.0, 20.0, 2.0),
        ]);
        // 墙 x=3 面（法向 −X）
        mesh.push([
            DVec3::new(3.0, 0.0, 4.0),
            DVec3::new(3.0, 0.0, 10.0),
            DVec3::new(3.0, 20.0, 10.0),
        ]);
        mesh.push([
            DVec3::new(3.0, 0.0, 4.0),
            DVec3::new(3.0, 20.0, 10.0),
            DVec3::new(3.0, 20.0, 4.0),
        ]);
        // 墙 x=4 面（法向 +X）
        mesh.push([
            DVec3::new(4.0, 0.0, 4.0),
            DVec3::new(4.0, 20.0, 10.0),
            DVec3::new(4.0, 0.0, 10.0),
        ]);
        mesh.push([
            DVec3::new(4.0, 0.0, 4.0),
            DVec3::new(4.0, 20.0, 4.0),
            DVec3::new(4.0, 20.0, 10.0),
        ]);
        // 墙顶 z=10（法向 +Z）
        mesh.push([
            DVec3::new(3.0, 0.0, 10.0),
            DVec3::new(4.0, 0.0, 10.0),
            DVec3::new(4.0, 20.0, 10.0),
        ]);
        mesh.push([
            DVec3::new(3.0, 0.0, 10.0),
            DVec3::new(4.0, 20.0, 10.0),
            DVec3::new(3.0, 20.0, 10.0),
        ]);
        let st = Stock {
            min: DVec3::new(-1.0, -1.0, 0.0),
            max: DVec3::new(21.0, 21.0, 12.0),
        };
        let ix = crate::MeshIndex::build(&mesh, st);

        // 刀尖 (3.5,10,3)、轴 +Z：球心 z=3.4 在地板上方（干净），
        // 刃部圆柱在 z=6 处位于墙内（材质内 0.5 mm）。
        let a = DVec3::new(3.5, 10.0, 3.0);
        let b = DVec3::new(3.5, 10.2, 3.0);
        let z = DVec3::Z;
        let g = Geo::of(&ToolSpec::ball(0, 0.8, 25.0), 0.03, 0.3);
        // 无符号口径会放行（0.5 ≥ 0.37），signed 必须拒
        assert!(
            ix.min_dist_signed(DVec3::new(3.5, 10.0, 6.0), 0.37) + 0.5 < 1e-9,
            "墙内点必须读负值"
        );
        assert!(
            !sweep_ok(&ix, a, z, b, z, &g),
            "埋进墙里的刃部圆柱必须被拦下"
        );
        // 同样的位姿放到开放空气里（x=10，离墙 6 mm）不该误报
        let a2 = DVec3::new(10.0, 10.0, 3.0);
        let b2 = DVec3::new(10.0, 10.2, 3.0);
        assert!(
            sweep_ok(&ix, a2, z, b2, z, &g),
            "开放空气不该误报"
        );
    }

    #[test]
    fn min_link_lift_is_small_over_flat_ground() {
        let mesh = vec![
            [
                DVec3::new(-50.0, -50.0, 0.0),
                DVec3::new(50.0, -50.0, 0.0),
                DVec3::new(50.0, 50.0, 0.0),
            ],
            [
                DVec3::new(-50.0, -50.0, 0.0),
                DVec3::new(50.0, 50.0, 0.0),
                DVec3::new(-50.0, 50.0, 0.0),
            ],
        ];
        let st = Stock {
            min: DVec3::new(-50.0, -50.0, 0.0),
            max: DVec3::new(50.0, 50.0, 1.0),
        };
        let ix = crate::MeshIndex::build(&mesh, st);
        let g = Geo::of(&ToolSpec::ball(0, 0.8, 16.0), 0.03, 0.3);
        let a = DVec3::new(-10.0, 0.0, 0.0);
        let b = DVec3::new(10.0, 0.0, 0.0);
        let d = min_link_lift(&ix, a, DVec3::Z, b, DVec3::Z, &g, 18.0);
        assert!(d < 6.0, "平地上只需小退刀量，实得 {d} mm");
        assert!(d >= 2.9, "不该退到 0（球会贴地），实得 {d}");
        let empty = crate::MeshIndex::build(&[], st);
        let d0 = min_link_lift(&empty, a, DVec3::Z, b, DVec3::Z, &g, 18.0);
        assert!((d0 - 3.0).abs() < 1e-9, "无料时取下限 3 mm，实得 {d0}");
    }

    /// 流向场：圆柱面上必须沿**母线**（曲率 0），不是绕圆周（曲率 1/R）。
    ///
    /// 这是"行距均匀"的几何前提：绕圆周走，每圈的曲率半径都是 R，刀轴一直在转；
    /// 沿母线走，曲率为 0，刀轴只在跨圈时转。
    #[test]
    fn flow_field_follows_the_cylinder_axis() {
        let r = 5.0;
        let mut pts = Vec::new();
        let mut nrm = Vec::new();
        for i in 0..48 {
            let th = i as f64 / 48.0 * std::f64::consts::TAU;
            for j in 0..6 {
                pts.push(DVec3::new(r * th.cos(), r * th.sin(), j as f64 * 0.5));
                nrm.push(DVec3::new(th.cos(), th.sin(), 0.0));
            }
        }
        let f = flow_field(&pts, &nrm, 1.2);
        let along = f.iter().filter(|d| d.z.abs() > 0.9).count();
        assert!(
            along * 10 >= f.len() * 9,
            "圆柱面上流向该沿 z 轴：{along}/{}",
            f.len()
        );
    }

    /// 流向不能把覆盖弄丢：同 3×3 点阵，带流向也必须连成一条、每点出现一次。
    #[test]
    fn flow_guided_chain_still_covers_every_point() {
        let pts: Vec<DVec3> = (0..3)
            .flat_map(|i| (0..3).map(move |j| DVec3::new(i as f64, j as f64, 0.0)))
            .collect();
        let nrm = vec![DVec3::Z; pts.len()];
        let ix = crate::MeshIndex::build(&[], stock());
        let g = Geo::of(&ToolSpec::ball(0, 1.0, 10.0), 0.03, 0.3);
        let f = flow_field(&pts, &nrm, 2.0);
        let runs = chain(&pts, &nrm, 1.5, &ix, &g, Some(&f));
        let mut seen: Vec<usize> = runs.iter().flatten().copied().collect();
        let total = seen.len();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(total, 9, "每个点只该出现一次：{total}");
        assert_eq!(seen.len(), 9, "9 个点都要被覆盖");
    }

    /// 抽稀：0..10 共 11 个点、间距 1、抽稀 3 → 0,3,6,9 + 末点 10。
    #[test]
    fn decimation_keeps_both_ends() {
        let pts: Vec<DVec3> = (0..11).map(|i| DVec3::new(i as f64, 0.0, 0.0)).collect();
        let nrm = vec![DVec3::Z; pts.len()];
        let ix = crate::MeshIndex::build(&[], stock());
        let run: Vec<usize> = (0..11).collect();
        let g = Geo::of(&ToolSpec::ball(0, 1.0, 10.0), 0.03, 0.3);
        let k = decimate(&run, &pts, &nrm, 3.0, &ix, &g);
        assert_eq!(k.first(), Some(&0));
        assert_eq!(k.last(), Some(&10), "末点必须留：{k:?}");
        assert!(k.len() <= 6, "抽稀没生效：{k:?}");
    }
}
