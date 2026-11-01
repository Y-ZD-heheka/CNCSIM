//! 逐块 fold：`state' = step(state, line)`。
//!
//! ## 执行顺序是**结构性的**，不是约定
//!
//! 一行里的所有操作先收进 `Vec<Op>`，再按 [`order::Phase`] **排序后执行**。
//! 这样"G43 在运动之前"不是靠代码摆放顺序保证的，而是靠排序——
//! 谁往 `ops` 里塞东西都不会打乱它。
//!
//! 见 [`crate::order`] 顶部的权威原文引用。

use crate::lex::{LexedLine, lex};
use crate::modal::{
    ArcDistanceMode, CutterComp, DistanceMode, GCode, MCode, ModalState, PathControl, RetractMode,
    SpindleDir, SpindleMode, Units, is_axis_letter,
};
use crate::order::{self, Phase};
use crate::{Block, MotionProgram, ParseIssue, Severity, Trust};
use cncsim_math::{DMat3, DMat4, DVec3};
use cncsim_motion::{Feed, MotionKind, Plane, Pose};
use std::f64::consts::TAU;

/// 解释器配置。
#[derive(Clone, Debug)]
pub struct InterpConfig {
    /// G54–G59.3 偏置表。解释器把所有坐标解析到**单一**工件坐标系。
    pub work_offsets: crate::WorkOffsets,
    /// 刀长表（H 寄存器）。索引 0 保留不用。
    pub tool_lengths: Vec<f64>,
    /// 刀径表（D 寄存器），**半径**不是直径。索引 0 保留不用。
    ///
    /// 为什么解释器需要它：`G41/G42` 的 `D` 选的是寄存器号，不是半径。
    /// 表里没有值就不知道偏多少——那会**整整差一个刀半径**，所以缺值时报错
    /// 而不是当 0 处理。
    pub tool_radii: Vec<f64>,
    /// 机床原点在统一工件坐标系中的位置（G53 用）
    pub machine_origin: DVec3,
    /// 初始刀尖位置（统一工件坐标系）
    pub initial_position: DVec3,
    /// 初始刀轴
    pub initial_axis: DVec3,
    /// A/B/C 是转工件还是转刀具（见 [`crate::RotaryConvention`]）
    pub rotary: crate::RotaryConvention,
}

impl Default for InterpConfig {
    fn default() -> Self {
        Self {
            work_offsets: crate::WorkOffsets::default(),
            tool_radii: Vec::new(),
            tool_lengths: vec![0.0; 100],
            machine_origin: DVec3::ZERO,
            initial_position: DVec3::ZERO,
            initial_axis: Pose::UP,
            rotary: crate::RotaryConvention::Table,
        }
    }
}

impl InterpConfig {
    /// H 寄存器查表
    pub fn tool_length(&self, h: u16) -> f64 {
        self.tool_lengths.get(h as usize).copied().unwrap_or(0.0)
    }

    /// D 寄存器对应的**刀具半径**（mm）。没登记就是 0。
    pub fn tool_radius(&self, d: u16) -> f64 {
        self.tool_radii.get(d as usize).copied().unwrap_or(0.0)
    }
}

// ─────────────────────────── 一行内的操作 ───────────────────────────

#[derive(Clone, Copy, Debug)]
enum Op {
    G(GCode),
    M(MCode),
    F(f64),
    S(f64),
    T(f64),
    /// A/B/C 轴字：下标 0/1/2，值按 `st.units` 已换算（角度不换算，但保留统一入口）
    Rotary(usize, f64),
}

impl Op {
    fn phase(self) -> Phase {
        match self {
            Op::G(g) => order::phase_of_g(g).unwrap_or(Phase::Motion),
            Op::M(m) => order::phase_of_m(m).unwrap_or(Phase::UserCommand),
            Op::F(_) => Phase::FeedRate,
            Op::S(_) => Phase::SpindleSpeed,
            Op::T(_) => Phase::SelectTool,
            Op::Rotary(..) => Phase::RotaryAxis,
        }
    }
}

/// 一行解析过程中的临时标志。
#[derive(Default)]
struct LineFlags {
    /// 本行有 G53：运动按机床坐标解释
    machine_coords: bool,
    /// 组 1 被组 0 挂起：本行不产出运动块
    motion_suspended: bool,
    /// 本行有影响几何但不支持的代码
    geometry_unreliable: bool,
}

// ─────────────────────────── 入口 ───────────────────────────

/// 把整份程序解析成运动程序。
pub fn interpret(src: &str, cfg: &InterpConfig) -> MotionProgram {
    let (lines, lex_issues) = lex(src);
    let mut out = MotionProgram {
        blocks: Vec::new(),
        issues: lex_issues,
        trust: Trust::Reliable,
        stops: Vec::new(),
    };
    let mut st = ModalState {
        position: cfg.initial_position,
        axis: cfg.initial_axis,
        // 坐标系表是**副本**：G10 会改它，cfg 必须保持不变（见 ModalState 的注释）
        work_offsets: cfg.work_offsets.clone(),
        ..ModalState::default()
    };

    for line in &lines {
        step(line, cfg, &mut st, &mut out);
    }

    // 刀径补偿是**后处理**：它需要看连续几块的方向才能算出拐角。
    // 放在这里而不是 `step` 里，是因为拐角求交要同时知道"上一段"和"下一段"。
    crate::comp::apply(&mut out.blocks, cfg.initial_position, &mut out.issues);

    if out.trust == Trust::Reliable && out.has_errors() {
        out.trust = Trust::Unreliable { first_bad_line: 0 };
    }
    out
}

fn step(line: &LexedLine, cfg: &InterpConfig, st: &mut ModalState, out: &mut MotionProgram) {
    if line.blank {
        return;
    }

    // ── ① 收集并校验 G / M 代码 ──
    let mut ops: Vec<Op> = Vec::new();
    let mut seen: Vec<(crate::modal::GGroup, GCode)> = Vec::new();

    for w in line.words_of(b'G') {
        let Some(g) = GCode::from_f64(w.value) else {
            out.issues.push(ParseIssue::new(
                "GCODE-001",
                Severity::Warning,
                line.line_no,
                format!("G{} 不是合法 G 代码（最多一位小数）", w.value),
                "检查是否写成了 G43.45 这类两位小数；LinuxCNC 的 G 代码只有一位小数",
            ));
            continue;
        };
        let Some(group) = crate::modal::g_group_of(g) else {
            out.issues.push(ParseIssue::new(
                "GCODE-001",
                Severity::Warning,
                line.line_no,
                format!("{} 不在 Fanuc 子集内", g.label()),
                "已跳过；若是自定义循环或机床厂商扩展，请检查后处理配置",
            ));
            continue;
        };
        // 同一模态组两个代码同行 → LinuxCNC 判为错误，取最后一个
        if let Some(pos) = seen.iter().position(|(gr, _)| *gr == group) {
            let prev = seen[pos].1;
            out.issues.push(ParseIssue::new(
                "GCODE-003",
                Severity::Error,
                line.line_no,
                format!(
                    "同一模态组（{}）里出现了两个代码：{} 与 {}",
                    group as i32,
                    prev.label(),
                    g.label()
                ),
                "同一行同一模态组只能有一个；已取最后一个，请删掉多余的那个",
            ));
            seen[pos].1 = g;
        } else {
            seen.push((group, g));
        }
        ops.push(Op::G(g));
    }

    for w in line.words_of(b'M') {
        let Some(m) = MCode::from_f64(w.value) else {
            out.issues.push(ParseIssue::new(
                "GCODE-002",
                Severity::Warning,
                line.line_no,
                format!("M{} 不是合法 M 代码", w.value),
                "已跳过；检查数值格式",
            ));
            continue;
        };
        if crate::modal::m_group_of(m).is_none() {
            out.issues.push(ParseIssue::new(
                "GCODE-002",
                Severity::Warning,
                line.line_no,
                format!("{} 不在 Fanuc 子集内", m.label()),
                "已跳过；若它影响几何（如厂商自定义循环），请检查后处理配置",
            ));
            continue;
        }
        ops.push(Op::M(m));
    }

    for w in line.words_of(b'F') {
        ops.push(Op::F(w.value));
    }
    for w in line.words_of(b'S') {
        ops.push(Op::S(w.value));
    }
    for w in line.words_of(b'T') {
        ops.push(Op::T(w.value));
    }
    // 旋转轴：A/B/C。**必须在运动之前**，所以走独立的 Phase（见 order::Phase::RotaryAxis）
    for (idx, letter) in b"ABC".iter().enumerate() {
        for w in line.words_of(*letter) {
            ops.push(Op::Rotary(idx, w.value));
        }
    }

    // ── ② 组 0 / 组 1 的轴字冲突 ──
    let has_axis_word = line.words.iter().any(|w| is_axis_letter(w.letter));
    let group1_here = seen.iter().find(|(g, _)| *g == crate::modal::GGroup::Motion).map(|(_, c)| *c);
    let group0_axis_here = seen
        .iter()
        .filter(|(g, _)| *g == crate::modal::GGroup::NonModal)
        .map(|(_, c)| *c)
        .find(|c| crate::modal::group_zero_uses_axis_words(*c));

    let mut flags = LineFlags::default();
    if let Some(g0) = group0_axis_here
        && has_axis_word
    {
        if let Some(g1) = group1_here {
            // 两者同行都用轴字 —— 错误
            out.issues.push(ParseIssue::new(
                "GCODE-004",
                Severity::Error,
                line.line_no,
                format!("{} 与 {} 同行且都使用轴字", g1.label(), g0.label()),
                "把组 0 代码（G10/G28/G30/G52/G92）单独放一行；本行未产出运动",
            ));
        } else {
            // 组 1 隐式生效，被组 0 挂起
            out.issues.push(ParseIssue::new(
                "GCODE-010",
                Severity::Info,
                line.line_no,
                format!(
                    "{} 使用轴字，本行的组 1 运动（{}）被挂起",
                    g0.label(),
                    st.motion.label()
                ),
                "这是 LinuxCNC 的既定语义；若你期望本行也走刀，请把运动拆到另一行",
            ));
        }
        flags.motion_suspended = true;
    }

    // ── ③ 按 Phase 排序后执行 ──
    //
    // 这一步是"字序无关"的**结构保证**：无论字在行内怎么排，执行顺序都由
    // Phase 决定。已用「去掉排序」验证过：`G1 X10 G91` 会退化成绝对坐标 X=10，
    // 被 `word_order_within_a_block_does_not_matter` 抓到。
    // ── ②.5 隐式运动：只有轴字、没有组 1 的 G 代码 ──
    //
    // 真实程序绝大多数行是这样的：
    //     G1 X10 F100
    //     X20            ← 隐式 G1，必须走
    //     Y5             ← 必须走
    // 不认这一条，整份程序只有"带 G 字的那些行"会动，其余全丢——
    // 而且**不会报任何错**，只是切出来的东西不对。
    let has_motion_word = line
        .words
        .iter()
        .any(|w| matches!(w.letter, b'X' | b'Y' | b'Z' | b'A' | b'B' | b'C'));
    let has_group1_g = seen.iter().any(|(g, _)| *g == crate::modal::GGroup::Motion);
    // **G68.2 的 X/Y/Z 是倾斜平面的原点，不是运动轴字。**
    // 漏掉这一条会在这行额外生成一段隐式运动，把机床拽到平面原点去。
    let twp_here = seen
        .iter()
        .any(|&(_, c)| matches!(c, GCode::G68_2 | GCode::G68_4));

    // ── ②.4 固定循环的模态调用 ──
    //
    // 孔阵列的后处理长这样：
    //     G81 Z-10 R2 F100
    //     X20 Y30      ← 在 (20,30) 再钻一个，不带任何 G 字
    //     X40 Y30
    // 只有 X/Y、没有组 1 的 G 代码，就是"下一个孔"。少了这一条，
    // 一个 20 孔的阵列只会钻第一个。
    let has_xy = line
        .words
        .iter()
        .any(|w| matches!(w.letter, b'X' | b'Y'));
    let cycle_repeat = !flags.motion_suspended
        && !twp_here
        && has_xy
        && !has_group1_g
        && st.cycle.is_some();
    if cycle_repeat {
        repeat_cycle(out, line, cfg, st);
    }

    if !flags.motion_suspended
        && !twp_here
        && !cycle_repeat
        && has_motion_word
        && !has_group1_g
        && matches!(st.motion, GCode::G0 | GCode::G1 | GCode::G2 | GCode::G3)
    {
        ops.push(Op::G(st.motion));
    }

    ops.sort_by_key(|o| o.phase());
    for op in ops {
        match op {
            Op::F(v) => st.feed = v,
            Op::S(v) => st.spindle_speed = v,
            Op::T(v) => st.tool = v.max(0.0) as u16,
            Op::Rotary(idx, v) => {
                // 角度单位是**度**，与 G20/G21 无关。受 G90/G91 影响。
                let rad = v.to_radians();
                st.angles[idx] = match st.distance {
                    DistanceMode::Absolute => rad,
                    DistanceMode::Incremental => st.angles[idx] + rad,
                };
            }
            Op::M(m) => exec_m(m, line, st, out),
            Op::G(g) => exec_g(g, line, cfg, st, out, &mut flags),
        }
    }

    // ── ④ 影响几何但不支持的代码 → 整份结果不可信 ──
    if flags.geometry_unreliable && out.trust == Trust::Reliable {
        out.trust = Trust::Unreliable { first_bad_line: line.line_no };
    }
}

// ─────────────────────────── M 代码 ───────────────────────────

fn exec_m(m: MCode, line: &LexedLine, st: &mut ModalState, out: &mut MotionProgram) {
    match m {
        MCode::M3 => st.spindle_on = Some(SpindleDir::Cw),
        MCode::M4 => st.spindle_on = Some(SpindleDir::Ccw),
        MCode::M5 => st.spindle_on = None,
        MCode::M7 | MCode::M8 => st.coolant = 1,
        MCode::M9 => st.coolant = 0,
        MCode::M61 => st.tool = line.value_of(b'Q').unwrap_or(st.tool as f64).max(0.0) as u16,
        // M6 换刀：刀具号已由 T 字设定；v1 不模拟换刀动作（换刀点本身是运动）
        MCode::M6 => {}
        MCode::M0 => push_stop(out, line, crate::StopKind::ProgramStop),
        MCode::M1 => push_stop(out, line, crate::StopKind::OptionalStop),
        MCode::M2 => push_stop(out, line, crate::StopKind::ProgramEnd),
        MCode::M30 => push_stop(out, line, crate::StopKind::ProgramEndReset),
        // M60 托盘交换：属于机床辅助动作，与几何无关
        _ => {}
    }
    let _ = out;
}

// ─────────────────────────── G 代码 ───────────────────────────

fn exec_g(
    g: GCode,
    line: &LexedLine,
    cfg: &InterpConfig,
    st: &mut ModalState,
    out: &mut MotionProgram,
    flags: &mut LineFlags,
) {
    match g {
        // ── 组 5：进给模式 ──
        GCode::G93 => st.feed_mode = cncsim_motion::FeedMode::InverseTime,
        GCode::G94 => st.feed_mode = cncsim_motion::FeedMode::UnitsPerMin,
        GCode::G95 => st.feed_mode = cncsim_motion::FeedMode::UnitsPerRev,

        // ── 组 0：停延（注意：在平面/单位/补偿之前执行）──
        GCode::G4 => {
            let Some(p) = line.value_of(b'P') else {
                out.issues.push(ParseIssue::new(
                    "GCODE-007",
                    Severity::Error,
                    line.line_no,
                    "G4 缺少 P（停延秒数）",
                    "写成 G4 P1.5；Fanuc 下 P 的单位是秒",
                ));
                return;
            };
            push_block(
                out,
                line,
                st,
                cfg,
                MotionKind::Dwell { seconds: p },
                st.position,
            );
        }

        // ── 组 2：平面 ──
        GCode::G17 => st.plane = Plane::Xy,
        GCode::G18 => st.plane = Plane::Zx,
        GCode::G19 => st.plane = Plane::Yz,
        GCode::G17_1 | GCode::G18_1 | GCode::G19_1 => unsupported(
            out,
            flags,
            line,
            g,
            "倾斜平面坐标系（G17.1/G18.1/G19.1）v1 未实现，会改变所有后续坐标",
        ),

        // ── 组 6：单位 ──
        GCode::G20 => st.units = Units::Inch,
        GCode::G21 => st.units = Units::Mm,

        // ── 组 7：刀径补偿 ──
        GCode::G40 => {
            st.cutter_comp = CutterComp::Off;
            st.comp_radius = 0.0;
        }
        GCode::G41 | GCode::G42 => {
            let left = g == GCode::G41;
            st.cutter_comp = if left {
                CutterComp::Left
            } else {
                CutterComp::Right
            };
            // D 给的是**寄存器号**，半径要去表里查。不给 D 就按 1 号刀算。
            let d = line.value_of(b'D').map(|v| v.max(0.0) as u16).unwrap_or(1);
            st.comp_radius = cfg.tool_radius(d);
            if st.comp_radius <= 1e-9 {
                out.issues.push(ParseIssue::new(
                    "GCODE-011",
                    Severity::Error,
                    line.line_no,
                    format!(
                        "{} 的补偿半径未知（D{d} 在 tool_radii 表里没有值），路径会整整差一个刀半径",
                        g.label()
                    ),
                    "在 InterpConfig::tool_radii 里登记 D 寄存器对应的半径；只想关掉补偿就用 G40",
                ));
                flags.geometry_unreliable = true;
            }
        }
        GCode::G41_1 | GCode::G42_1 => unsupported(
            out,
            flags,
            line,
            g,
            "带参数的刀径补偿 v1 未实现",
        ),

        // ── 组 8：刀长补偿 ──
        GCode::G43 => match line.value_of(b'H') {
            Some(h) => {
                st.tool_length_h = Some(h.max(0.0) as u16);
                st.rtcp = false;
            }
            None => out.issues.push(ParseIssue::new(
                "GCODE-007",
                Severity::Error,
                line.line_no,
                "G43 缺少 H（刀长寄存器号）",
                "写成 G43 H7；H 指向刀具表里的长度",
            )),
        },
        GCode::G43_4 => {
            // RTCP：标志交给 kin，gcode 只记录
            if let Some(h) = line.value_of(b'H') {
                st.tool_length_h = Some(h.max(0.0) as u16);
            }
            st.rtcp = true;
        }
        GCode::G49 => {
            st.tool_length_h = None;
            st.rtcp = false;
        }
        GCode::G43_1 | GCode::G43_2 => unsupported(out, flags, line, g, "动态刀长（G43.1/G43.2）v1 未实现"),

        // ── 组 9：倾斜工作平面 ──
        GCode::G68_2 => {
            // Fanuc G68.2：X/Y/Z 给倾斜平面原点，I/J/K 给欧拉角（度），
            // 分别是绕 Z / X / Y，复合顺序 **Z → X → Y**。
            let s = st.units.scale();
            let ang = |l: u8| line.value_of(l).unwrap_or(0.0).to_radians();
            let origin = DVec3::new(
                line.value_of(b'X').unwrap_or(0.0) * s,
                line.value_of(b'Y').unwrap_or(0.0) * s,
                line.value_of(b'Z').unwrap_or(0.0) * s,
            );
            st.twp = Some(crate::modal::TwpFrame {
                origin,
                rot: euler_zxy(ang(b'I'), ang(b'J'), ang(b'K')),
            });
            st.axis = tool_axis(st, cfg);
        }
        GCode::G68_4 => unsupported(
            out,
            flags,
            line,
            g,
            "G68.4（按刀具方向定义倾斜平面）v1 未实现；改用 G68.2 的 I/J/K 欧拉角",
        ),
        GCode::G69 => {
            st.twp = None;
            st.axis = tool_axis(st, cfg);
        }

        // ── 组 12：坐标系 ──
        GCode::G54 => st.wcs = 1,
        GCode::G55 => st.wcs = 2,
        GCode::G56 => st.wcs = 3,
        GCode::G57 => st.wcs = 4,
        GCode::G58 => st.wcs = 5,
        GCode::G59 => st.wcs = 6,
        GCode::G59_1 => st.wcs = 7,
        GCode::G59_2 => st.wcs = 8,
        GCode::G59_3 => st.wcs = 9,

        // ── 组 13：路径控制 ──
        GCode::G61 => st.path_control = PathControl::ExactStop,
        GCode::G61_1 => st.path_control = PathControl::ExactPath,
        GCode::G64 => st.path_control = PathControl::Continuous,

        // ── 组 3 / 组 4：距离模式 ──
        GCode::G90 => st.distance = DistanceMode::Absolute,
        GCode::G91 => st.distance = DistanceMode::Incremental,
        GCode::G90_1 => st.arc_distance = ArcDistanceMode::Absolute,
        GCode::G91_1 => st.arc_distance = ArcDistanceMode::Incremental,

        // ── 组 10：退刀模式 ──
        GCode::G98 => st.retract = RetractMode::OldZ,
        GCode::G99 => st.retract = RetractMode::RPlane,

        // ── 组 14 / 15：车床相关，v1 不做 ──
        GCode::G96 => st.spindle_mode = SpindleMode::ConstantSurface,
        GCode::G97 => st.spindle_mode = SpindleMode::Rpm,
        GCode::G7 | GCode::G8 => {
            st.lathe_diameter = g == GCode::G7;
            unsupported(out, flags, line, g, "车床直径/半径模式 v1 未实现");
        }

        // ── G53：只修饰本行运动 ──
        GCode::G53 => flags.machine_coords = true,

        // ── 组 0 里其余影响坐标的：v1 不实现，但必须标不可信 ──
        // ── 坐标数据：G10 / G52 / G92 ──
        //
        // 这三条**不动刀**，只改"程序坐标怎么映射到机床位置"。它们一旦漏掉，
        // 后续**所有**绝对坐标都会静默偏移——比少钻一个孔更难发现，因为
        // 每一块的形状都对，只是整体挪了位置。
        GCode::G10 => set_coordinates(out, flags, line, st),
        GCode::G52 => {
            // 局部坐标系：叠在当前 WCS 上。G52 X0 Y0 Z0 取消。
            let scale = st.units.scale();
            let mut off = st.local_offset;
            for (idx, letter) in b"XYZ".iter().enumerate() {
                if let Some(v) = line.value_of(*letter) {
                    let v = v * scale;
                    let new = match st.distance {
                        DistanceMode::Absolute => v,
                        DistanceMode::Incremental => off.component(idx) + v,
                    };
                    set_comp(&mut off, idx, new);
                }
            }
            st.local_offset = off;
        }
        GCode::G92 => {
            // 让"当前位置"读作给定坐标：`位置 = 新基点 + 给定值`
            //
            // 与 G10 L20 的区别：G10 改**坐标系表**（写进机床、断电保存），
            // G92 只叠一个临时平移。程序里看效果一样，取消方式不同：
            // G10 要再写一次，G92 用 G92.1。
            let scale = st.units.scale();
            let base = st.base();
            let prog = st.position - base;
            let mut v = prog;
            for (idx, letter) in b"XYZ".iter().enumerate() {
                if let Some(w) = line.value_of(*letter) {
                    let w = w * scale;
                    let new = match st.distance {
                        DistanceMode::Absolute => w,
                        DistanceMode::Incremental => prog.component(idx) + w,
                    };
                    set_comp(&mut v, idx, new);
                }
            }
            // base_new = position − v；而 base = wcs + G52 + G92，所以
            // G92 = position − v − wcs − G52
            st.g92_offset = st.position - v - st.work_offsets.get(st.wcs) - st.local_offset;
            st.g92_suspended = false;
        }
        GCode::G92_1 => {
            // 清零。Fanuc 的 G92.1 是"取消并复位"，不留恢复的可能
            st.g92_offset = DVec3::ZERO;
            st.g92_suspended = false;
        }
        GCode::G92_2 => st.g92_suspended = true,
        GCode::G92_3 => st.g92_suspended = false,
        GCode::G28 | GCode::G28_1 | GCode::G28_2 | GCode::G30 | GCode::G30_1 => {
            unsupported(out, flags, line, g, "回参考点（G28/G30）v1 未实现，机床位置未知")
        }

        // ── 组 1：运动 ──
        GCode::G0 | GCode::G1 => {
            st.motion = g;
            st.cycle = None;
            if flags.motion_suspended {
                return;
            }
            let target = linear_target(line, cfg, st, flags.machine_coords);
            let kind = if g == GCode::G0 {
                MotionKind::Rapid
            } else {
                MotionKind::Linear
            };
            push_block(out, line, st, cfg, kind, target);
            st.position = target;
        }
        GCode::G2 | GCode::G3 => {
            st.motion = g;
            st.cycle = None;
            if flags.motion_suspended {
                return;
            }
            let target = linear_target(line, cfg, st, flags.machine_coords);
            let clockwise = g == GCode::G2;
            match arc_center(line, st, target, clockwise) {
                Ok(center) => {
                    push_block(
                        out,
                        line,
                        st,
                        cfg,
                        MotionKind::Arc {
                            center,
                            clockwise,
                            plane: st.plane,
                        },
                        target,
                    );
                    st.position = target;
                }
                Err(msg) => out.issues.push(ParseIssue::new(
                    "GCODE-008",
                    Severity::Error,
                    line.line_no,
                    format!("{} 圆弧参数无效：{msg}", g.label()),
                    "圆心用 I/J/K（相对起点）或 R 给半径；检查平面选择（G17/G18/G19）与起终点距离",
                )),
            }
        }
        GCode::G80 => {
            // 取消固定循环：本行不产生运动
            st.motion = GCode::G80;
            st.cycle = None;
        }
        // ── 固定循环 ──
        //
        // **必须展开成真实运动**，不能"跳过"：一个 G81 少钻一个孔，切出来的件
        // 就少一个孔，而报告上看不出任何异常。这是"静默错误"那一类。
        GCode::G73 | GCode::G76 => {
            st.motion = g;
            start_cycle(out, line, cfg, st, flags, g);
        }
        GCode::G5 | GCode::G5_1 | GCode::G5_2 | GCode::G5_3 => {
            unsupported(out, flags, line, g, "样条插补（G5.x）v1 未实现，路径无法离散化")
        }
        GCode::G33 | GCode::G33_1 => unsupported(out, flags, line, g, "螺纹/刚性攻丝 v1 未实现"),
        _ => {
            if g.0 == 870 {
                unsupported(
                    out,
                    flags,
                    line,
                    g,
                    "反镗循环（G87）从工件下方向上加工，本版运动模型不支持",
                );
            } else if crate::modal::CycleKind::from_gcode(g).is_some() {
                st.motion = g;
                start_cycle(out, line, cfg, st, flags, g);
            }
        }
    }
}

fn unsupported(
    out: &mut MotionProgram,
    flags: &mut LineFlags,
    line: &LexedLine,
    g: GCode,
    why: &str,
) {
    flags.geometry_unreliable = true;
    out.issues.push(ParseIssue::new(
        "GCODE-005",
        Severity::Error,
        line.line_no,
        format!("{} 不支持：{why}", g.label()),
        "结果已标为不可信；若该程序依赖此功能，请先在 CAM 里展开或换用支持的指令",
    ));
}

// ─────────────────────────── 目标点 ───────────────────────────

/// 计算本行的目标点（统一工件坐标系）。
fn linear_target(
    line: &LexedLine,
    cfg: &InterpConfig,
    st: &ModalState,
    machine_coords: bool,
) -> DVec3 {
    let scale = st.units.scale();
    let base = if machine_coords {
        cfg.machine_origin
    } else {
        // WCS 表 + G52 + G92。**不能直接用 cfg.work_offsets**——G10 改过之后
        // 那张表已经过时了
        st.base()
    };
    let mut words = [None; 3];
    for (idx, letter) in b"XYZ".iter().enumerate() {
        words[idx] = line.value_of(*letter).map(|v| v * scale);
    }

    // G53 走机床坐标，是绝对物理位置 —— TWP 对它不生效。
    let twp = if machine_coords { None } else { st.twp };

    let Some(f) = twp else {
        // 没有倾斜平面：逐分量算，和以前一样
        let mut p = st.position;
        for (idx, w) in words.iter().enumerate() {
            if let Some(v) = *w {
                let new = match st.distance {
                    DistanceMode::Absolute => base.component(idx) + v,
                    DistanceMode::Incremental => p.component(idx) + v,
                };
                set_comp(&mut p, idx, new);
            }
        }
        return p;
    };

    match st.distance {
        // 增量向量是**在程序坐标里**给的，要转成工件坐标
        DistanceMode::Incremental => {
            let mut d = DVec3::ZERO;
            for (idx, w) in words.iter().enumerate() {
                if let Some(v) = *w {
                    set_comp(&mut d, idx, v);
                }
            }
            st.position + f.rot.mul_vec3(d)
        }
        // 绝对值是程序坐标下的目标点。没给的轴保持**当前的程序坐标**，
        // 不是当前的工件坐标——否则倾斜平面里一个只给 X 的块会把 Y/Z 拽回平面外。
        DistanceMode::Absolute => {
            let mut q = f.to_prog(st.position - base);
            for (idx, w) in words.iter().enumerate() {
                if let Some(v) = *w {
                    set_comp(&mut q, idx, v);
                }
            }
            base + f.to_wcs(q)
        }
    }
}

// ─────────────────────── 旋转轴与刀轴方向 ───────────────────────

/// 绕某轴转 `angle`（弧度）的 4×4，用来做矩阵连乘。
fn rot(axis: DVec3, angle: f64) -> DMat4 {
    DMat4::from_axis_angle(axis, angle)
}

/// A/B/C 的复合旋转 `Rz(C)·Ry(B)·Rx(A)`（ZYX 欧拉序）。
///
/// 顺序不是随意的：Fanuc 的 A/B/C 定义就是"绕 X/Y/Z 转"，而双转台的物理堆叠
/// （C 在 A 上面）正好对应 `Rz·Rx`。B 轴在中间，所以整体是 ZYX。
fn euler_zyx(a: f64, b: f64, c: f64) -> DMat3 {
    rot(DVec3::Z, c)
        .mul_mat4(&rot(DVec3::Y, b))
        .mul_mat4(&rot(DVec3::X, a))
        .rotation()
}

/// G68.2 的 I/J/K：绕 Z / X / Y，复合顺序 **Z → X → Y**（Fanuc 原文）。
fn euler_zxy(i: f64, j: f64, k: f64) -> DMat3 {
    rot(DVec3::Y, k)
        .mul_mat4(&rot(DVec3::X, j))
        .mul_mat4(&rot(DVec3::Z, i))
        .rotation()
}

/// 当前刀轴方向（工件坐标系，单位向量）。
///
/// 三步复合，顺序不能换：
///
/// 1. **旋转轴**：按 [`InterpConfig::rotary`] 决定是转工件还是转刀具。
///    转工件时刀轴在工件坐标系里是**反向**转的（`Rᵀ`）。
/// 2. **倾斜平面**：G68.2 定义的程序坐标系再转一次。工件坐标系里的刀轴 = `R_twp · axis_prog`。
/// 3. 归一化（矩阵正交，长度本该是 1，纯粹防浮点）。
fn tool_axis(st: &ModalState, cfg: &InterpConfig) -> DVec3 {
    let [a, b, c] = st.angles;
    let mut axis = if a.abs() < 1e-12 && b.abs() < 1e-12 && c.abs() < 1e-12 {
        DVec3::Z
    } else {
        let r = euler_zyx(a, b, c);
        match cfg.rotary {
            crate::RotaryConvention::Table => r.transpose().mul_vec3(DVec3::Z),
            crate::RotaryConvention::Head => r.mul_vec3(DVec3::Z),
        }
    };
    if let Some(f) = st.twp {
        axis = f.rot.mul_vec3(axis);
    }
    axis.normalize()
}

/// 记一个程序停止点。
fn push_stop(out: &mut MotionProgram, line: &LexedLine, kind: crate::StopKind) {
    out.stops.push(crate::StopPoint {
        block: out.blocks.len(),
        line_no: line.line_no,
        kind,
    });
}

/// 按索引改分量。`DVec3` 的字段是公开的，这里只是避免三处重复 match。
fn set_comp(v: &mut DVec3, i: usize, x: f64) {
    match i {
        0 => v.x = x,
        1 => v.y = x,
        _ => v.z = x,
    }
}

// ─────────────────────────── 圆弧圆心 ───────────────────────────

fn arc_center(
    line: &LexedLine,
    st: &ModalState,
    end: DVec3,
    clockwise: bool,
) -> Result<DVec3, String> {
    let start = st.position;
    let scale = st.units.scale();
    let (iu, iv) = plane_indices(st.plane);

    // R 格式
    if let Some(r_raw) = line.value_of(b'R') {
        let r = r_raw * scale;
        return center_from_radius(start, end, r, iu, iv, clockwise);
    }

    // I/J/K 格式
    let i = line.value_of(b'I').map(|v| v * scale);
    let j = line.value_of(b'J').map(|v| v * scale);
    let k = line.value_of(b'K').map(|v| v * scale);
    if i.is_none() && j.is_none() && k.is_none() {
        return Err("既没有 I/J/K，也没有 R".into());
    }
    // 按平面挑出两个分量
    let pick = |axis: usize| match axis {
        0 => i.unwrap_or(0.0),
        1 => j.unwrap_or(0.0),
        _ => k.unwrap_or(0.0),
    };
    let mut c = start;
    for idx in 0..3usize {
        if idx == iu || idx == iv {
            let off = pick(idx);
            let new = match st.arc_distance {
                // Fanuc 默认：I/J/K 相对起点
                ArcDistanceMode::Incremental => start.component(idx) + off,
                // G90.1：I/J/K 是绝对坐标
                ArcDistanceMode::Absolute => off,
            };
            set_comp(&mut c, idx, new);
        }
    }
    if (c - start).length() < 1e-9 {
        return Err("圆心与起点重合".into());
    }
    Ok(c)
}

/// 由半径与起终点求圆心（R 格式）。
///
/// 判据：`R > 0` 走**次弧**（扫角 ≤ 180°），`R < 0` 走**优弧**。
/// 两个候选圆心（弦的两侧）里挑扫角与这个判据相符的那个。
fn center_from_radius(
    start: DVec3,
    end: DVec3,
    r: f64,
    iu: usize,
    iv: usize,
    clockwise: bool,
) -> Result<DVec3, String> {
    let (su, sv) = (start.component(iu), start.component(iv));
    let (eu, ev) = (end.component(iu), end.component(iv));
    let (du, dv) = (eu - su, ev - sv);
    let chord = (du * du + dv * dv).sqrt();
    if chord < 1e-12 {
        return Err("R 格式圆弧的起终点重合".into());
    }
    let r_abs = r.abs();
    if chord > 2.0 * r_abs + 1e-9 {
        return Err(format!("半径 {r_abs} 太小，起终点距离 {chord} 超过直径"));
    }
    let half = chord / 2.0;
    let h = (r_abs * r_abs - half * half).max(0.0).sqrt();
    let (mu, mv) = ((su + eu) / 2.0, (sv + ev) / 2.0);
    // 弦的垂直单位向量
    let (nu, nv) = (-dv / chord, du / chord);
    let want_minor = r > 0.0;

    let mut fallback = (mu + h * nu, mv + h * nv);
    for sign in [1.0f64, -1.0] {
        let (cu, cv) = (mu + sign * h * nu, mv + sign * h * nv);
        let a0 = (sv - cv).atan2(su - cu);
        let a1 = (ev - cv).atan2(eu - cu);
        let mut sweep = if clockwise { a0 - a1 } else { a1 - a0 };
        while sweep < 0.0 {
            sweep += TAU;
        }
        if (sweep <= std::f64::consts::PI) == want_minor {
            let mut c = start;
            set_comp(&mut c, iu, cu);
            set_comp(&mut c, iv, cv);
            return Ok(c);
        }
        fallback = (cu, cv);
    }
    // 退化：起终点恰为直径两端时两个候选等价
    let mut c = start;
    set_comp(&mut c, iu, fallback.0);
    set_comp(&mut c, iv, fallback.1);
    Ok(c)
}

fn plane_indices(plane: Plane) -> (usize, usize) {
    match plane {
        Plane::Xy => (0, 1),
        Plane::Zx => (2, 0),
        Plane::Yz => (1, 2),
    }
}

// ─────────────────────────── 产出块 ───────────────────────────

fn push_block(
    out: &mut MotionProgram,
    line: &LexedLine,
    st: &ModalState,
    cfg: &InterpConfig,
    motion: MotionKind,
    target: DVec3,
) {
    let feed_value = match st.feed_mode {
        // G93 的 F 是 1/min，不是长度，不随单位缩放
        cncsim_motion::FeedMode::InverseTime => st.feed,
        _ => st.feed * st.units.scale(),
    };
    let tool_length = st
        .tool_length_h
        .map(|h| cfg.tool_length(h))
        .unwrap_or(0.0);
    out.blocks.push(Block {
        line_no: line.line_no,
        motion,
        target: Pose {
            tip: target,
            axis: tool_axis(st, cfg),
        },
        feed: Feed {
            value: feed_value,
            mode: st.feed_mode,
        },
        spindle_speed: st.spindle_speed,
        rtcp: st.rtcp,
        tool: st.tool,
        tool_length,
        wcs: st.wcs,
        angles: st.angles,
        dwell: 0.0,
        cutter_comp: st.cutter_comp,
        comp_radius: st.comp_radius,
    });
}

// ─────────────────────────── 坐标数据 ───────────────────────────

/// `G10 L2 / L20`：改工件坐标系表。
///
/// | L | 语义 |
/// |---|---|
/// | `L2` | `X/Y/Z` **就是**该坐标系原点在机床坐标下的位置 |
/// | `L20` | 让**当前位置**在新坐标系里读作 `X/Y/Z`（对刀常用） |
/// | `L1/L10/L11` | 刀表，本版不支持（会报 `GCODE-005`） |
///
/// `P0` 表示当前坐标系，`P1`–`P9` 是 G54–G59.3。`G91` 下是**增量**：在原有
/// 偏移上累加。`G10` 本身不产生任何运动。
fn set_coordinates(
    out: &mut MotionProgram,
    flags: &mut LineFlags,
    line: &LexedLine,
    st: &mut ModalState,
) {
    let Some(l) = line.value_of(b'L') else {
        out.issues.push(ParseIssue::new(
            "GCODE-009",
            Severity::Error,
            line.line_no,
            "G10 缺少 L（要改什么表）",
            "对刀用 G10 L2/L20；L1/L10/L11 是刀表，本版不支持",
        ));
        flags.geometry_unreliable = true;
        return;
    };
    let l = l.round() as i32;
    if l != 2 && l != 20 {
        unsupported(
            out,
            flags,
            line,
            GCode::G10,
            &format!("G10 L{l}（刀表）v1 未实现；本版只支持 L2 / L20 改坐标系"),
        );
        return;
    }
    let p = line.value_of(b'P').unwrap_or(0.0).round();
    // P0 = 当前坐标系；P1..P9 = G54..G59.3
    let target = if p == 0.0 { st.wcs } else { p.clamp(1.0, 9.0) as u8 };

    let scale = st.units.scale();
    let mut want = [None; 3];
    for (idx, letter) in b"XYZ".iter().enumerate() {
        want[idx] = line.value_of(*letter).map(|v| v * scale);
    }

    let old = st.work_offsets.get(target);
    let mut next = old;
    match (l, st.distance) {
        // L2：给的就是机床坐标下的原点
        (2, DistanceMode::Absolute) => {
            for (idx, w) in want.iter().enumerate() {
                if let Some(v) = *w {
                    set_comp(&mut next, idx, v);
                }
            }
        }
        (2, DistanceMode::Incremental) => {
            for (idx, w) in want.iter().enumerate() {
                if let Some(v) = *w {
                    set_comp(&mut next, idx, old.component(idx) + v);
                }
            }
        }
        // L20：让当前位置读作给定值 → 原点 = 位置 − 给定值 − (G52 + G92)
        //
        // G52/G92 必须减掉：它们也参与"位置 = 基点 + 程序坐标"，不减的话
        // 一次 G52 之后对刀，原点会差一个 G52 的量。
        (20, mode) => {
            let trim = st.trim();
            let cur_prog = st.position - old - trim;
            let mut v = cur_prog;
            for (idx, w) in want.iter().enumerate() {
                if let Some(val) = *w {
                    let new = match mode {
                        DistanceMode::Absolute => val,
                        DistanceMode::Incremental => cur_prog.component(idx) + val,
                    };
                    set_comp(&mut v, idx, new);
                }
            }
            next = st.position - v - trim;
        }
        _ => unreachable!(),
    }
    st.work_offsets.set(target, next);
}

// ─────────────────────────── 固定循环 ───────────────────────────

/// 啄钻（G73）每次断屑抬起的量（mm）。Fanuc 用机床参数给，这里取一个常见值。
const CHIP_BREAK_MM: f64 = 0.5;
/// 啄钻（G83）下一刀之前，快速下到"上一刀孔底 + 这个量"再开始进给。
const PECK_CLEARANCE_MM: f64 = 0.5;

/// 开始（或重新声明）一个固定循环：读参数、落到 `st.cycle`，然后钻第一个孔。
fn start_cycle(
    out: &mut MotionProgram,
    line: &LexedLine,
    cfg: &InterpConfig,
    st: &mut ModalState,
    flags: &mut LineFlags,
    g: GCode,
) {
    let Some(kind) = crate::modal::CycleKind::from_gcode(g) else {
        return;
    };
    // `G53 G81 ...` 这类组合：组 0 用了轴字 → 本行运动被挂起（与 G0/G1 同一语义）
    if flags.motion_suspended {
        return;
    }
    if !cycle_plane_ok(out, line, st) {
        return;
    }
    let scale = st.units.scale();
    let base_z = st.base().z;
    let initial_z = if st.cycle.is_some() {
        // 循环中途重新声明：起始平面沿用第一次的（Fanuc 的"初始平面"是循环级别的）
        st.cycle.expect("刚判过").initial_z
    } else {
        st.position.z
    };

    // R 与 Z 的参照点：绝对模式是工件坐标；增量模式下 R 相对起始平面、Z 相对 R 平面。
    let r_word = line.value_of(b'R').map(|v| v * scale);
    let z_word = line.value_of(b'Z').map(|v| v * scale);
    let (r_plane, depth) = match st.distance {
        DistanceMode::Absolute => (
            r_word.map(|r| base_z + r),
            z_word.map(|z| base_z + z),
        ),
        DistanceMode::Incremental => {
            let r = r_word.map(|r| initial_z + r);
            (r, z_word.map(|z| r.unwrap_or(initial_z) + z))
        }
    };

    let prev = st.cycle;
    if r_plane.is_none() && prev.is_none() {
        // 没有 R 就只能拿当前高度当 R 平面 —— 语义上等于"从这里直接进给"，
        // 与 Fanuc 要求显式 R 不一致，但也不会算错位置。报一声，别静默。
        out.issues.push(ParseIssue::new(
            "GCODE-009",
            Severity::Warning,
            line.line_no,
            format!("{} 没有 R（R 平面），按当前高度 {} 处理", g.label(), st.position.z),
            "Fanuc 要求显式给 R；写成 G81 Z-10 R2 F100",
        ));
    }
    let r_plane = r_plane
        .or(prev.map(|c| c.r_plane))
        .unwrap_or(st.position.z);
    let Some(depth) = depth.or(prev.map(|c| c.depth)) else {
        out.issues.push(ParseIssue::new(
            "GCODE-009",
            Severity::Error,
            line.line_no,
            format!("{} 缺少 Z（孔底深度）", g.label()),
            "固定循环的 Z 是模态的，但第一次必须给；写成 G81 Z-10 R2 F100",
        ));
        flags.geometry_unreliable = true;
        return;
    };
    // P 的单位由小数点决定：`P500` 是 500 毫秒，`P0.5` 是 0.5 秒（Fanuc 约定）
    let dwell = line.word(b'P').map(|w| {
        if w.has_dot {
            w.value.max(0.0)
        } else {
            (w.value / 1000.0).max(0.0)
        }
    });
    let peck = line.value_of(b'Q').map(|v| (v * scale).abs());

    st.cycle = Some(crate::modal::Cycle {
        kind,
        depth,
        r_plane,
        initial_z,
        dwell: dwell.or(prev.map(|c| c.dwell)).unwrap_or(0.0),
        peck: peck.or(prev.map(|c| c.peck)).unwrap_or(0.0),
    });
    if kind.pecks() && st.cycle.expect("刚写").peck <= 1e-9 {
        out.issues.push(ParseIssue::new(
            "GCODE-009",
            Severity::Error,
            line.line_no,
            format!("{} 是啄钻，但缺少 Q（每次深度）", g.label()),
            "写成 G83 Z-30 R2 Q5 F100；缺 Q 会退化成一次扎到底（本行已按此处理）",
        ));
    }

    let xy = cycle_xy(line, st);
    emit_hole(out, line, st, cfg, xy);
}

/// 模态复用的固定循环：本行只给 XY。
fn repeat_cycle(out: &mut MotionProgram, line: &LexedLine, cfg: &InterpConfig, st: &mut ModalState) {
    if !cycle_plane_ok(out, line, st) {
        return;
    }
    let xy = cycle_xy(line, st);
    emit_hole(out, line, st, cfg, xy);
}

/// 循环只能沿所选平面的法向钻。我们的运动模型只有 Z 向刀轴，所以只支持 G17。
fn cycle_plane_ok(out: &mut MotionProgram, line: &LexedLine, st: &ModalState) -> bool {
    if st.twp.is_some() {
        out.issues.push(ParseIssue::new(
            "GCODE-005",
            Severity::Error,
            line.line_no,
            "固定循环 + 倾斜工作平面（G68.2）：钻孔方向不是 Z 向，本版不支持",
            "先在 CAM 里展开成 G0/G1，或去掉 G68.2",
        ));
        return false;
    }
    if st.plane != cncsim_motion::Plane::Xy {
        out.issues.push(ParseIssue::new(
            "GCODE-005",
            Severity::Error,
            line.line_no,
            format!("固定循环只在 G17（XY 平面）下支持，当前是 {:?}", st.plane),
            "钻孔方向由平面选择决定；本版运动模型的刀轴固定沿 Z 向下",
        ));
        return false;
    }
    true
}

/// 本行的孔位（只算 X/Y，没给的保持当前值）。
fn cycle_xy(line: &LexedLine, st: &ModalState) -> DVec3 {
    let scale = st.units.scale();
    let base = st.base();
    let mut p = st.position;
    for (idx, letter) in b"XY".iter().enumerate() {
        if let Some(v) = line.value_of(*letter) {
            let v = v * scale;
            let new = match st.distance {
                DistanceMode::Absolute => base.component(idx) + v,
                DistanceMode::Incremental => p.component(idx) + v,
            };
            set_comp(&mut p, idx, new);
        }
    }
    p
}

/// 把一个孔展开成真实的刀路块，并把 `st.position` 停到退刀位置。
///
/// 顺序（Fanuc 的标准动作）：
///
/// ```text
/// ① 在起始 Z 上快移到孔位        ← 不在 R 平面上横移，避免撞夹具
/// ② 快移下到 R 平面
/// ③ 进给到孔底（G73/G83 分次，每次退刀排屑）
/// ④ 退刀：G98 回起始 Z，G99 回 R 平面
/// ```
fn emit_hole(
    out: &mut MotionProgram,
    line: &LexedLine,
    st: &mut ModalState,
    cfg: &InterpConfig,
    xy: DVec3,
) {
    let cycle = st.cycle.expect("调用方已保证");
    let mut pos = st.position;

    // ① XY 定位：保持在起始高度上平移
    if (pos.x - xy.x).abs() > 1e-12 || (pos.y - xy.y).abs() > 1e-12 {
        set_comp(&mut pos, 0, xy.x);
        set_comp(&mut pos, 1, xy.y);
        push_block(out, line, st, cfg, MotionKind::Rapid, pos);
    }
    // ② 快移到 R 平面
    if pos.z > cycle.r_plane + 1e-12 {
        pos.z = cycle.r_plane;
        push_block(out, line, st, cfg, MotionKind::Rapid, pos);
    }

    // ③ 下扎
    let pecking = cycle.kind.pecks() && cycle.peck > 1e-9;
    if pecking && cycle.r_plane - cycle.depth > cycle.peck + 1e-9 {
        let mut z = cycle.r_plane;
        loop {
            let next = (z - cycle.peck).max(cycle.depth);
            pos.z = next;
            push_block(out, line, st, cfg, MotionKind::Linear, pos);
            if next <= cycle.depth + 1e-12 {
                break;
            }
            // 退刀排屑：G83 完全退出到 R 平面，G73 只抬一点断屑
            let back = if cycle.kind == crate::modal::CycleKind::Peck {
                cycle.r_plane
            } else {
                (next + CHIP_BREAK_MM).min(cycle.r_plane)
            };
            if back > next + 1e-12 {
                pos.z = back;
                push_block(out, line, st, cfg, MotionKind::Rapid, pos);
                // 快速回到上一刀孔底上方一点，剩下的留给进给
                let approach = (next + PECK_CLEARANCE_MM).min(back);
                if approach > next + 1e-12 {
                    pos.z = approach;
                    push_block(out, line, st, cfg, MotionKind::Rapid, pos);
                }
            }
            z = next;
        }
    } else {
        pos.z = cycle.depth;
        push_block(out, line, st, cfg, MotionKind::Linear, pos);
    }

    // 孔底暂停：只有带 P 的循环才有。**几何不动，只影响时间**
    if cycle.kind.dwells() && cycle.dwell > 1e-9 {
        if let Some(b) = out.blocks.last_mut() {
            b.dwell = cycle.dwell;
        }
    }

    // ④ 退刀
    let retract_z = match st.retract {
        RetractMode::OldZ => cycle.initial_z,
        RetractMode::RPlane => cycle.r_plane,
    };
    if (pos.z - retract_z).abs() > 1e-12 {
        pos.z = retract_z;
        let kind = if cycle.kind.retract_is_rapid() {
            MotionKind::Rapid
        } else {
            MotionKind::Linear
        };
        push_block(out, line, st, cfg, kind, pos);
    }

    st.position = pos;
}
