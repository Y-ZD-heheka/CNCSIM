//! # cncsim-gcode
//!
//! G 代码解释器：文本 → 运动程序。
//!
//! **只做"文本 → 位姿"，不做运动学。** 一旦这里调 `kin::inverse`，
//! 解释器就绑死在某台机床上——换机床要改解析器。
//!
//! **TWP（G68.2）归这里，RTCP（G43.4）归 `kin`。** 前者是坐标变换（不依赖机床），
//! 后者是机床运动学（强依赖）。见 docs/G代码解释器设计.md §1.3。
//!
//! ## 两条必须钉死的语义（都有权威原文支撑）
//!
//! **① 块内字序无关。** LinuxCNC `overview.adoc` §Item order：
//! > The first group (the words) may be reordered in any way without changing
//! > the meaning of the line.
//!
//! 所以 `G91 G1 X10` 与 `G1 X10 G91` **完全等价**，都是增量。
//! 执行顺序由 [`order::Phase`] 的固定序列决定，**不由字在行内的位置决定**
//! （`overview.adoc` §G-code Order of Execution 开篇原话）。
//!
//! **② 组 0 轴字代码会挂起组 1。** `overview.adoc` §Modal Groups：
//! > If an axis word-using G-code from group 1 is implicitly in effect on a line
//! > ... and a group 0 G-code that uses axis words appears on the line,
//! > the activity of the group 1 G-code is suspended for that line.
//!
//! 涉及 G10、G28、G30、G52、G92。不实现则 `G28 X0 Y0` 之后会多走一段。
//!
//! ## 模块
//!
//! | 模块 | 职责 |
//! |---|---|
//! | [`lex`] | 一行文本 → 字序列（`letter, value`），处理注释与程序分隔符 |
//! | [`modal`] | G/M 代码编号、15 个 G 模态组、8 个 M 模态组、[`modal::ModalState`] |
//! | [`order`] | 块内执行顺序（[`order::Phase`]），照搬 LinuxCNC 的 25 项清单 |
//! | [`interp`] | 逐块 fold：按执行顺序应用模态字，产出 [`Block`] |

pub mod interp;
pub mod lex;
pub mod modal;
pub mod order;

pub use interp::{InterpConfig, interpret};
pub use modal::{Cycle, CycleKind, TwpFrame};

mod comp;
pub use lex::{LexedLine, Word, lex};
pub use modal::{
    ArcDistanceMode, CutterComp, DistanceMode, GCode, GGroup, MCode, MGroup, ModalState, PathControl,
    RetractMode, SpindleMode, Units,
};
pub use order::{Phase, phase_of_g, phase_of_m};

use cncsim_math::DVec3;
use cncsim_motion::{Feed, MotionKind, Pose};

/// 解析结果的总体可信度。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Trust {
    #[default]
    Reliable,
    /// 有跳过的块且含增量运动 → 之后所有位置静默偏移，**结果不可信**
    Unreliable { first_bad_line: u32 },
    Aborted,
}

/// 问题等级。四级里 `Error` 专门用来避免把"能跑但结果错"伪装成成功。
///
/// 注意：这是 `gcode` 层自己的枚举，与 `sim::Severity` **故意不共享**——
/// 各层自带 finding，由 `sim` 组合（见 docs/架构设计.md §2.3）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// 只是提示，结果不受影响
    Info,
    /// 可能有问题，结果仍可信
    Warning,
    /// **能继续，但结果不可信**
    Error,
    /// 无法继续
    Fatal,
}

/// 一条解析问题。**不中断解析**，只记录。
#[derive(Clone, Debug)]
pub struct ParseIssue {
    /// 稳定编号，如 `GCODE-012`。测试断言编号，不断言消息文本。
    pub id: &'static str,
    pub severity: Severity,
    pub line: u32,
    pub message: String,
    /// 怎么办——报错的价值在于下一步做什么
    pub hint: Option<String>,
}

impl ParseIssue {
    pub fn new(
        id: &'static str,
        severity: Severity,
        line: u32,
        message: impl Into<String>,
        hint: impl Into<String>,
    ) -> Self {
        Self {
            id,
            severity,
            line,
            message: message.into(),
            hint: Some(hint.into()),
        }
    }
}

/// 一个已解析的块。下游不需要再做任何模态判断。
#[derive(Clone, Debug, PartialEq)]
pub struct Block {
    pub line_no: u32,
    pub motion: MotionKind,
    /// **统一工件坐标系**下的目标位姿（WCS 偏置已在解析时吸收）
    pub target: Pose,
    pub feed: Feed,
    /// 主轴转速（rpm，模态）。**只有 `G95`（每转进给）算时间时用得上**——
    /// `mm/rev × rev/min = mm/min`。其它模式忽略它。
    ///
    /// 为什么必须带下来：`G95 F0.1` 单独看只说"每转进 0.1 mm"，没有转速就算不出
    /// 时间；而 `S` 是模态的，可能写在几十行之前。
    pub spindle_speed: f64,
    /// 是否处于 RTCP（G43.4）模式——决定 `sim` 用哪种插补空间
    pub rtcp: bool,
    pub tool: u16,
    /// 本块生效的刀长偏置（mm）。**本行 G43 必须在本行运动之前生效**，
    /// 所以这个值可能与本行之前的状态不同。
    pub tool_length: f64,
    /// 当前工件坐标系编号（1 = G54 … 9 = G59.3），**仅供显示**
    pub wcs: u8,
    /// A/B/C 轴的**绝对角**（弧度，模态保持）。
    ///
    /// 为什么不把刀轴方向直接算进 `target.axis` 就完事：同一个 `A30.` 在双转台和
    /// 双摆头上，刀轴方向是**相反的**（工件转 vs 刀具转）。这个选择由机床构型决定，
    /// 不属于 G 代码。所以这里存"轴角"这个**无歧义**的量，`target.axis` 只是
    /// 按 [`InterpConfig::rotary`] 给的约定算出来的方便值。
    pub angles: [f64; 3],
    /// 孔底暂停（秒）。只有固定循环的孔底块会带值；`0` = 不停。
    ///
    /// 为什么挂在块上而不是单独一类块：暂停是**在这一块的位置上**发生的，
    /// 拆出去反而要维护"块 → 暂停"的映射。几何上它不动，只影响时间。
    pub dwell: f64,
    /// 本块生效的刀径补偿（G41/G42/G40）。
    ///
    /// 记在块上是为了让**后处理**（`comp`）能把一整段补偿路径当成一条折线
    /// 来偏移——拐角求交要同时知道上一段和下一段的方向。
    pub cutter_comp: CutterComp,
    /// 补偿半径（mm）。`G40` / 未补偿时为 0。
    pub comp_radius: f64,
}

/// 旋转轴的语义约定：A/B/C 是在转工件还是在转刀具。
///
/// 同一个 `A30.`：
/// - [`RotaryConvention::Table`]（双转台）：工件绕 X 转 +30°，刀轴在**工件坐标系**里
///   变成 `Rx(−30)·Z` —— 看上去是往反方向偏的。
/// - [`RotaryConvention::Head`]（双摆头）：刀具绕 X 转 +30°，刀轴就是 `Rx(+30)·Z`。
///
/// 这不是"参数"，是**机床构型**。默认取 `Table`，因为本项目的参考机床是 AC 双转台。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RotaryConvention {
    /// 工件侧旋转（双转台 / 摇篮）
    #[default]
    Table,
    /// 刀具侧旋转（双摆头）
    Head,
}

/// 程序停止点。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopKind {
    /// M0 程序停
    ProgramStop,
    /// M1 计划停（机床面板上的"选择性停止"开关决定是否真停）
    OptionalStop,
    /// M2 程序结束
    ProgramEnd,
    /// M30 程序结束并复位
    ProgramEndReset,
}

/// 一个停止点：`blocks[..block]` 执行完之后停。
///
/// 为什么不塞进 `Block`：`M0` / `M30` 常常独占一行，那种行**不产生运动块**。
/// 挂在块上的话这些停顿会凭空消失。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StopPoint {
    /// 执行完 `blocks[..block]` 后停
    pub block: usize,
    pub line_no: u32,
    pub kind: StopKind,
}

/// 整份程序。**不可变**，支持 O(1) 按块访问（时间轴拖动需要）。
#[derive(Clone, Debug, Default)]
pub struct MotionProgram {
    pub blocks: Vec<Block>,
    pub issues: Vec<ParseIssue>,
    pub trust: Trust,
    /// 程序停止点，按 `block` 升序。`sim` 跑到这里就该停。
    pub stops: Vec<StopPoint>,
}

impl MotionProgram {
    pub fn block_at(&self, i: usize) -> Option<&Block> {
        self.blocks.get(i)
    }
    pub fn len(&self) -> usize {
        self.blocks.len()
    }
    pub fn is_empty(&self) -> bool {
        self.blocks.is_empty()
    }
    /// 是否有关键问题（Error 及以上）
    pub fn has_errors(&self) -> bool {
        self.issues.iter().any(|i| i.severity >= Severity::Error)
    }
    /// 按编号查问题——测试用
    pub fn issues_with(&self, id: &str) -> Vec<&ParseIssue> {
        self.issues.iter().filter(|i| i.id == id).collect()
    }
}

/// 工件坐标系偏置表。解释器用它把所有坐标解析到**单一**工件坐标系。
///
/// 索引 0 对应 G54，1 对应 G55，……，8 对应 G59.3。
#[derive(Clone, Debug)]
pub struct WorkOffsets {
    pub wcs: Vec<DVec3>,
}

impl Default for WorkOffsets {
    fn default() -> Self {
        Self { wcs: vec![DVec3::ZERO; 9] }
    }
}

impl WorkOffsets {
    /// G54 = 1 … G59.3 = 9
    pub fn get(&self, index_1_based: u8) -> DVec3 {
        let i = index_1_based.saturating_sub(1) as usize;
        self.wcs.get(i).copied().unwrap_or(DVec3::ZERO)
    }

    /// 改一个坐标系的原点（G10 L2/L20 用）。越界索引被忽略。
    pub fn set(&mut self, index_1_based: u8, offset: DVec3) {
        let i = index_1_based.saturating_sub(1) as usize;
        if i < self.wcs.len() {
            self.wcs[i] = offset;
        }
    }

    /// 有几个坐标系（9 = G54 … G59.3）
    pub fn len(&self) -> usize {
        self.wcs.len()
    }
}

/// 方言接缝。v1 只有 Fanuc 实现；它是**测试接缝**，不是插件系统。
pub trait Dialect {
    fn default_feed_mode(&self) -> cncsim_motion::FeedMode;
    /// 圆弧 I/J/K 是否相对起点（Fanuc 是）
    fn arc_ijk_relative(&self) -> bool;
}

/// Fanuc 子集方言。
#[derive(Clone, Copy, Debug, Default)]
pub struct Fanuc;

impl Dialect for Fanuc {
    fn default_feed_mode(&self) -> cncsim_motion::FeedMode {
        cncsim_motion::FeedMode::UnitsPerMin
    }
    fn arc_ijk_relative(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn program_is_randomly_accessible() {
        let p = MotionProgram::default();
        assert!(p.is_empty());
        assert!(p.block_at(0).is_none());
    }

    #[test]
    fn work_offsets_are_one_based() {
        let mut w = WorkOffsets::default();
        w.wcs[0] = DVec3::new(10.0, 0.0, 0.0);
        w.wcs[8] = DVec3::new(90.0, 0.0, 0.0);
        assert_eq!(w.get(1), DVec3::new(10.0, 0.0, 0.0), "G54 是索引 1");
        assert_eq!(w.get(9), DVec3::new(90.0, 0.0, 0.0), "G59.3 是索引 9");
        assert_eq!(w.get(5), DVec3::ZERO);
    }

    #[test]
    fn every_issue_has_a_hint() {
        // 质量门：报错的价值在于"下一步做什么"
        let p = interpret(
            "G123 X1\nG1 X10 F100\n",
            &InterpConfig::default(),
        );
        assert!(!p.issues.is_empty());
        for i in &p.issues {
            assert!(i.hint.is_some(), "{} 缺 hint", i.id);
            assert!(!i.hint.as_ref().unwrap().is_empty());
        }
    }
}
