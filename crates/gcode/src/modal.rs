//! G/M 代码编号、模态组、模态状态。
//!
//! ## 编号用「十分之一」定点整数
//!
//! LinuxCNC 的 G 代码最多一位小数（`G43.4`、`G59.3`、`G61.1`、`G92.1`、`G28.1`），
//! 所以 [`GCode`] 内部存 `值 × 10` 的 `i32`。**不用 f64 比较**——`43.4` 与 `43.399999`
//! 在浮点下不相等，而这种 bug 只在特定代码上偶发。
//!
//! ## 模态组编号照搬 LinuxCNC（15 个）
//!
//! 权威表在 `refs/linuxcnc/overview.adoc` §Modal Groups。**编号 0–15 中 11 未使用**
//! （历史上被移除），所以是 **15 个**组而不是 16 个——早期文档写 16 是错的。
//!
//! 照搬编号的好处：遇到歧义直接查原文档，`ModalState` 的字段可以一一对应。

use cncsim_math::{DMat3, DVec3};
use cncsim_motion::{FeedMode, Plane};

// ─────────────────────────── G 代码 ───────────────────────────

/// G 代码，内部为 `值 × 10` 的定点整数。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GCode(pub i32);

impl GCode {
    /// 从 `值 × 10` 构造
    pub const fn tenths(t: i32) -> Self {
        Self(t)
    }
    /// 从浮点构造。多于一位小数返回 `None`——那是非法 G 代码，不该被静默接受。
    pub fn from_f64(v: f64) -> Option<Self> {
        let scaled = v * 10.0;
        let rounded = scaled.round();
        if (scaled - rounded).abs() > 1e-6 || !v.is_finite() {
            return None;
        }
        Some(Self(rounded as i32))
    }
    pub fn value(self) -> f64 {
        self.0 as f64 / 10.0
    }
    /// 显示用，如 `G43.4`
    pub fn label(self) -> String {
        let v = self.value();
        if (v - v.trunc()).abs() < 1e-9 {
            format!("G{}", v as i64)
        } else {
            format!("G{v}")
        }
    }
}

macro_rules! gcodes {
    ($($name:ident = $t:expr),* $(,)?) => {
        impl GCode { $(pub const $name: GCode = GCode($t);)* }
    };
}

gcodes! {
    G0 = 0, G1 = 10, G2 = 20, G3 = 30, G4 = 40,
    G5 = 50, G5_1 = 51, G5_2 = 52, G5_3 = 53,
    G7 = 70, G8 = 80,
    G10 = 100,
    G17 = 170, G18 = 180, G19 = 190,
    G17_1 = 171, G18_1 = 181, G19_1 = 191,
    G20 = 200, G21 = 210,
    G28 = 280, G28_1 = 281, G28_2 = 282,
    G30 = 300, G30_1 = 301,
    G33 = 330, G33_1 = 331,
    G38_2 = 382, G38_3 = 383, G38_4 = 384, G38_5 = 385,
    G40 = 400, G41 = 410, G42 = 420, G41_1 = 411, G42_1 = 421,
    G43 = 430, G43_1 = 431, G43_2 = 432, G43_4 = 434, G49 = 490,
    G52 = 520, G53 = 530,
    G54 = 540, G55 = 550, G56 = 560, G57 = 570, G58 = 580, G59 = 590,
    G59_1 = 591, G59_2 = 592, G59_3 = 593,
    G61 = 610, G61_1 = 611, G64 = 640,
    G68_2 = 682, G68_4 = 684, G69 = 690,
    G73 = 730, G76 = 760,
    G80 = 800,
    G90 = 900, G91 = 910, G90_1 = 901, G91_1 = 911,
    G92 = 920, G92_1 = 921, G92_2 = 922, G92_3 = 923,
    G93 = 930, G94 = 940, G95 = 950,
    G96 = 960, G97 = 970,
    G98 = 980, G99 = 990,
}

/// G 模态组。**编号即 LinuxCNC 编号，11 未使用。**
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum GGroup {
    NonModal = 0,
    Motion = 1,
    Plane = 2,
    Distance = 3,
    ArcDistance = 4,
    FeedMode = 5,
    Units = 6,
    CutterRadiusComp = 7,
    ToolLengthOffset = 8,
    TiltedWorkPlane = 9,
    CannedCycleReturn = 10,
    CoordinateSystem = 12,
    PathControl = 13,
    SpindleSpeedMode = 14,
    LatheDiameter = 15,
}

/// 查一个 G 代码属于哪个模态组。返回 `None` 表示不在权威表里（未知代码）。
pub fn g_group_of(g: GCode) -> Option<GGroup> {
    use GGroup::*;
    Some(match g.0 {
        // 组 0：非模态。G28.1/G30.1 在 LinuxCNC 表里没列，但同样是非模态。
        40 | 100 | 280 | 281 | 282 | 300 | 301 | 520 | 530 | 920 | 921 | 922 | 923 => NonModal,
        // 组 1：运动。G5.x/G33.x 在权威表里未列，但它们产生运动，归组 1 便于统一处理。
        0 | 10 | 20 | 30 | 50 | 51 | 52 | 53 | 330 | 331 | 382..=385 | 730 | 760 | 800..=890 => Motion,
        // 组 2：平面
        170 | 180 | 190 | 171 | 181 | 191 => Plane,
        // 组 3：距离
        900 | 910 => Distance,
        // 组 4：圆弧 IJK 距离
        901 | 911 => ArcDistance,
        // 组 5：进给模式
        930 | 940 | 950 => FeedMode,
        // 组 6：单位
        200 | 210 => Units,
        // 组 7：刀径补偿
        400 | 410 | 420 | 411 | 421 => CutterRadiusComp,
        // 组 8：刀长补偿
        430 | 431 | 432 | 434 | 490 => ToolLengthOffset,
        // 组 9：倾斜工作平面
        682 | 684 | 690 => TiltedWorkPlane,
        // 组 10：固定循环退刀
        980 | 990 => CannedCycleReturn,
        // 组 12：坐标系
        540..=593 => CoordinateSystem,
        // 组 13：路径控制
        610 | 611 | 640 => PathControl,
        // 组 14：主轴速度模式
        960 | 970 => SpindleSpeedMode,
        // 组 15：车床直径
        70 | 80 => LatheDiameter,
        _ => return None,
    })
}

/// 使用轴字的组 0 代码——它们会**挂起**同行的组 1 运动。
///
/// `overview.adoc` §Modal Groups 原文列举：G10, G28, G30, G52, G92。
pub fn group_zero_uses_axis_words(g: GCode) -> bool {
    matches!(g.0, 100 | 280 | 281 | 300 | 301 | 520 | 920)
}

// ─────────────────────────── M 代码 ───────────────────────────

/// M 代码，内部为 `值 × 10`。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MCode(pub i32);

impl MCode {
    pub const fn tenths(t: i32) -> Self {
        Self(t)
    }
    pub fn from_f64(v: f64) -> Option<Self> {
        let scaled = v * 10.0;
        let rounded = scaled.round();
        if (scaled - rounded).abs() > 1e-6 || !v.is_finite() {
            return None;
        }
        Some(Self(rounded as i32))
    }
    pub fn value(self) -> f64 {
        self.0 as f64 / 10.0
    }
    pub fn label(self) -> String {
        format!("M{}", self.value() as i64)
    }
}

macro_rules! mcodes {
    ($($name:ident = $t:expr),* $(,)?) => {
        impl MCode { $(pub const $name: MCode = MCode($t);)* }
    };
}

mcodes! {
    M0 = 0, M1 = 10, M2 = 20, M3 = 30, M4 = 40, M5 = 50,
    M6 = 60, M7 = 70, M8 = 80, M9 = 90,
    M26 = 260, M27 = 270, M30 = 300,
    M48 = 480, M49 = 490, M60 = 600, M61 = 610,
    M62 = 620, M63 = 630, M64 = 640, M65 = 650, M66 = 660, M67 = 670, M68 = 680,
    M70 = 700, M71 = 710, M72 = 720, M73 = 730,
}

/// M 模态组（8 个）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MGroup {
    RotaryModulo = 3,
    Stopping = 4,
    IoPins = 5,
    ToolChange = 6,
    Spindle = 7,
    Coolant = 8,
    OverrideSwitches = 9,
    UserDefined = 10,
}

pub fn m_group_of(m: MCode) -> Option<MGroup> {
    use MGroup::*;
    Some(match m.0 {
        260 | 270 => RotaryModulo,
        0 | 10 | 20 | 300 | 600 => Stopping,
        620..=680 => IoPins,
        60 => ToolChange,
        30 | 40 | 50 => Spindle,
        70 | 80 | 90 => Coolant,
        480 | 490 => OverrideSwitches,
        1000..=1990 => UserDefined,
        _ => return None,
    })
}

// ─────────────────────────── 模态状态 ───────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistanceMode {
    /// G90
    Absolute,
    /// G91
    Incremental,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArcDistanceMode {
    /// G90.1：I/J/K 是绝对坐标
    Absolute,
    /// G91.1：I/J/K 相对起点（Fanuc 默认）
    Incremental,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Units {
    Mm,
    Inch,
}

impl Units {
    /// 到毫米的比例
    pub fn scale(self) -> f64 {
        match self {
            Units::Mm => 1.0,
            Units::Inch => 25.4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutterComp {
    Off,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetractMode {
    /// G98：退回起始 Z
    OldZ,
    /// G99：退回 R 平面
    RPlane,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathControl {
    /// G61：精确停
    ExactStop,
    /// G61.1：精确路径
    ExactPath,
    /// G64：连续（v1 按理想路径模拟，见 docs/G代码解释器设计.md §12.4）
    Continuous,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpindleMode {
    /// G97：恒定转速
    Rpm,
    /// G96：恒定线速度
    ConstantSurface,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpindleDir {
    Cw,
    Ccw,
}

/// 全部模态状态 + 运行状态。
///
/// 默认值取 LinuxCNC 的初始状态（`overview.adoc` §Modal Groups 末段）。
#[derive(Clone, Debug)]
pub struct ModalState {
    /// 组 1。**初值取 G1**——程序应在预置段显式设置（`G17 G21 G40 G49 G54 G80 G90 G94`）
    pub motion: GCode,
    /// 组 2
    pub plane: Plane,
    /// 组 3
    pub distance: DistanceMode,
    /// 组 4。Fanuc 默认 I/J/K 相对起点
    pub arc_distance: ArcDistanceMode,
    /// 组 5
    pub feed_mode: FeedMode,
    /// 组 6
    pub units: Units,
    /// 组 7
    pub cutter_comp: CutterComp,
    /// 当前补偿半径（mm）：G41/G42 时从 D 寄存器查出来，G40 清 0
    pub comp_radius: f64,
    /// 组 8。`None` = G49 取消
    pub tool_length_h: Option<u16>,
    /// 组 8 的特例：G43.4 打开 RTCP。**由 `kin` 消费**，`gcode` 只记录
    pub rtcp: bool,
    /// 组 9。`None` = 没开（G69 或从未开过）
    pub twp: Option<TwpFrame>,
    /// 组 10
    pub retract: RetractMode,
    /// 组 12。1 = G54 … 9 = G59.3
    pub wcs: u8,
    /// 组 13
    pub path_control: PathControl,
    /// 组 14
    pub spindle_mode: SpindleMode,
    /// 组 15
    pub lathe_diameter: bool,

    // ── 非模态运行状态 ──
    pub feed: f64,
    pub spindle_speed: f64,
    pub spindle_on: Option<SpindleDir>,
    pub coolant: u8,
    pub tool: u16,
    /// **统一工件坐标系**下的当前位置（WCS 偏置已吸收）
    pub position: DVec3,
    /// 刀轴方向（工件坐标系）。由 `angles` + `twp` 推出来，见 `interp::tool_axis`
    pub axis: DVec3,
    /// A/B/C 轴的**绝对角**（弧度）。模态保持，受 G90/G91 影响
    pub angles: [f64; 3],
    /// 组 1 的固定循环状态。`None` = 没在循环里（G80 或从未开过）
    ///
    /// **循环是模态的**：`G81 Z-10 R2` 之后只写 `X20 Y30` 就是"在那里再钻一个"。
    /// 这是真实程序的常态——一个孔阵列每行只有坐标。
    pub cycle: Option<Cycle>,

    /// 工件坐标系表（G54…G59.3）。
    ///
    /// **是 `InterpConfig` 的副本，不是引用**：`G10 L2/L20` 会改它，而
    /// `InterpConfig` 必须保持"程序的初始条件"不变——同一份配置要能跑第二遍、
    /// 也要能拿去做单元测试的基线。改配置等于让 interpret 有副作用。
    pub work_offsets: crate::WorkOffsets,
    /// G52 局部坐标系偏移（**叠在**当前 WCS 上，G52 X0 Y0 Z0 取消）
    pub local_offset: DVec3,
    /// G92 临时偏移（叠在 WCS + G52 上，G92.1 清零）
    ///
    /// 与 G10 的区别：G10 改的是**坐标系表**（永久，写进机床），G92 只是一次
    /// 临时平移（不进表）。两者的效果在程序里看起来一样，但取消方式不同。
    pub g92_offset: DVec3,
    /// G92.2 挂起（值还在，但暂时不生效；G92.3 恢复）
    pub g92_suspended: bool,
}

impl ModalState {
    /// 叠在 WCS 表之上的临时平移：`G52 + G92`。
    pub fn trim(&self) -> DVec3 {
        self.local_offset
            + if self.g92_suspended {
                DVec3::ZERO
            } else {
                self.g92_offset
            }
    }

    /// 当前生效的「程序坐标 → 统一工件坐标」平移。
    ///
    /// `位置 = base + 程序坐标`。三处叠加：WCS 表（G54–G59.3）、G52 局部偏移、
    /// G92 临时偏移。
    pub fn base(&self) -> DVec3 {
        self.work_offsets.get(self.wcs) + self.trim()
    }
}

/// 固定循环的**类型**。几何上都是"一条圆柱孔"，差别在分次下扎与退刀方式。
///
/// 这里只保留**影响路径**的差别；主轴正反转、孔底让刀这类不影响几何的动作
/// 记在文档里，不做假动作。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CycleKind {
    /// G73 高速啄钻：每 `Q` 抬一点断屑，**不退出孔外**
    ChipBreak,
    /// G81 钻孔：一次扎到底
    Drill,
    /// G82 沉孔 / 钻孔：孔底暂停
    DrillDwell,
    /// G83 啄钻：每 `Q` 完全退出到 R 平面（深孔排屑）
    Peck,
    /// G84 攻丝：进给进、**进给退**（主轴反转）
    Tap,
    /// G85 镗孔：进给进、进给退
    Bore,
    /// G86 镗孔：进给进、主轴停、**快速退**
    BoreStop,
    /// G88 镗孔：孔底暂停、主轴停、快速退
    BoreDwell,
    /// G89 镗孔：孔底暂停、进给退
    BoreDwellFeed,
    /// G76 精镗：孔底让刀后快速退（让刀动作不建模）
    FineBore,
}

impl CycleKind {
    /// 从 G 代码取循环类型。
    pub fn from_gcode(g: GCode) -> Option<Self> {
        Some(match g.0 {
            730 => Self::ChipBreak,
            760 => Self::FineBore,
            810 => Self::Drill,
            820 => Self::DrillDwell,
            830 => Self::Peck,
            840 => Self::Tap,
            850 => Self::Bore,
            860 => Self::BoreStop,
            880 => Self::BoreDwell,
            890 => Self::BoreDwellFeed,
            _ => return None,
        })
    }

    /// 退刀用快速还是进给。
    ///
    /// 攻丝（G84）和镗孔（G85/G89）是**进给退**——主轴还在转、还在切削，
    /// 拿快速退会在孔壁上拉出螺旋痕。其余都是快速退。
    pub fn retract_is_rapid(self) -> bool {
        !matches!(self, Self::Tap | Self::Bore | Self::BoreDwellFeed)
    }

    /// 孔底是否暂停（`P`）。
    pub fn dwells(self) -> bool {
        matches!(
            self,
            Self::DrillDwell | Self::BoreDwell | Self::BoreDwellFeed | Self::FineBore
        )
    }

    /// 是否分次下扎（需要 `Q`）。
    pub fn pecks(self) -> bool {
        matches!(self, Self::ChipBreak | Self::Peck)
    }
}

/// 一个固定循环的**模态参数**。
///
/// 深度和 R 平面都是**工件坐标下的绝对 Z**：解析时就把 G90/G91 折算掉，
/// 后面每个孔复用同一组数——这正合 Fanuc 的语义（增量模式下 `R` 相对起始平面、
/// `Z` 相对 `R` 平面，逐孔重算得到的是同一组绝对值）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cycle {
    pub kind: CycleKind,
    /// 孔底 Z（绝对，mm）
    pub depth: f64,
    /// R 平面 Z（绝对，mm）
    pub r_plane: f64,
    /// 循环开始时的 Z（绝对，mm）——`G98` 退回到这里
    pub initial_z: f64,
    /// 孔底暂停（秒）
    pub dwell: f64,
    /// 每次啄钻深度（mm，正数）；`0` = 不分次
    pub peck: f64,
}

/// G68.2 倾斜工作平面。
///
/// 程序坐标 → 工件坐标：`p_wcs = origin + rot · p_prog`。
///
/// **归解释器管**（`docs/G代码解释器设计.md` §4.3）：它只是坐标变换，
/// 不需要知道机床构型。RTCP（G43.4）才归 `kin`。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwpFrame {
    /// 倾斜平面原点（工件坐标系）
    pub origin: DVec3,
    /// 程序坐标 → 工件坐标的旋转
    pub rot: DMat3,
}

impl TwpFrame {
    /// 程序坐标 → 工件坐标
    pub fn to_wcs(&self, p: DVec3) -> DVec3 {
        self.origin + self.rot.mul_vec3(p)
    }
    /// 工件坐标 → 程序坐标
    pub fn to_prog(&self, p: DVec3) -> DVec3 {
        self.rot.transpose().mul_vec3(p - self.origin)
    }
}

impl Default for ModalState {
    fn default() -> Self {
        Self {
            motion: GCode::G1,
            plane: Plane::Xy,
            distance: DistanceMode::Absolute,
            arc_distance: ArcDistanceMode::Incremental,
            feed_mode: FeedMode::UnitsPerMin,
            units: Units::Mm,
            cutter_comp: CutterComp::Off,
            comp_radius: 0.0,
            tool_length_h: None,
            rtcp: false,
            twp: None,
            retract: RetractMode::OldZ,
            wcs: 1,
            path_control: PathControl::Continuous,
            spindle_mode: SpindleMode::Rpm,
            lathe_diameter: false,
            feed: 0.0,
            spindle_speed: 0.0,
            spindle_on: None,
            coolant: 0,
            tool: 0,
            position: DVec3::ZERO,
            axis: cncsim_motion::Pose::UP,
            angles: [0.0; 3],
            cycle: None,
            work_offsets: crate::WorkOffsets::default(),
            local_offset: DVec3::ZERO,
            g92_offset: DVec3::ZERO,
            g92_suspended: false,
        }
    }
}

/// 轴字字母（X Y Z A B C U V W）。
pub fn is_axis_letter(letter: u8) -> bool {
    matches!(letter, b'X' | b'Y' | b'Z' | b'A' | b'B' | b'C' | b'U' | b'V' | b'W')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gcode_roundtrips_through_tenths() {
        assert_eq!(GCode::from_f64(1.0), Some(GCode::G1));
        assert_eq!(GCode::from_f64(43.4), Some(GCode::G43_4));
        assert_eq!(GCode::from_f64(59.3), Some(GCode::G59_3));
        assert_eq!(GCode::G43_4.value(), 43.4);
        assert_eq!(GCode::G43_4.label(), "G43.4");
        assert_eq!(GCode::G1.label(), "G1");
    }

    #[test]
    fn gcode_rejects_two_decimals() {
        // 多于一位小数是非法 G 代码，不能静默四舍五入
        assert_eq!(GCode::from_f64(1.25), None);
    }

    #[test]
    fn modal_state_defaults_match_linuxcnc() {
        let s = ModalState::default();
        assert_eq!(s.plane, Plane::Xy, "默认 G17");
        assert_eq!(s.distance, DistanceMode::Absolute, "默认 G90");
        assert_eq!(s.units, Units::Mm);
        assert_eq!(s.feed_mode, FeedMode::UnitsPerMin, "默认 G94");
        assert_eq!(s.wcs, 1, "默认 G54");
        assert_eq!(s.arc_distance, ArcDistanceMode::Incremental, "Fanuc 默认 IJK 相对");
        assert_eq!(s.cutter_comp, CutterComp::Off);
    }

    #[test]
    fn group_11_is_not_used() {
        // 权威表里组 11 空缺——所以是 15 个组，不是 16 个
        let groups: Vec<i32> = [
            GGroup::NonModal,
            GGroup::Motion,
            GGroup::Plane,
            GGroup::Distance,
            GGroup::ArcDistance,
            GGroup::FeedMode,
            GGroup::Units,
            GGroup::CutterRadiusComp,
            GGroup::ToolLengthOffset,
            GGroup::TiltedWorkPlane,
            GGroup::CannedCycleReturn,
            GGroup::CoordinateSystem,
            GGroup::PathControl,
            GGroup::SpindleSpeedMode,
            GGroup::LatheDiameter,
        ]
        .iter()
        .map(|g| *g as i32)
        .collect();
        assert_eq!(groups.len(), 15);
        assert!(!groups.contains(&11), "组 11 在 LinuxCNC 里未使用");
    }

    #[test]
    fn units_scale() {
        assert_eq!(Units::Mm.scale(), 1.0);
        assert_eq!(Units::Inch.scale(), 25.4);
    }
}
