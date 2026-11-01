//! # cncsim-sim
//!
//! 编排：仿真调度、回放缓冲、后台线程、报告聚合。
//!
//! **不含算法细节**——去除在 `voxel`，碰撞在 `collide`，运动学在 `kin`。
//! 这里只负责"按时间顺序把它们串起来"。

pub mod compare;
pub mod demo;
pub mod job;
pub mod playback;
pub mod run;
pub mod stock;

use cncsim_math::DVec3;

/// 仿真调度状态（`cncsim-sim`）。
///
/// **与 `hmi::ControlState` 是两回事**：这里说"后台在算什么"，
/// 那里说"用户在看什么"。暂停只停回放，计算继续。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SimState {
    #[default]
    Idle,
    Parsing,
    Ready,
    Computing,
    Done,
    Failed,
}

/// 报警严重度。**`Error` 的定义是"能继续，但结果不可信"**——
/// 这一级专门避免把"能跑但结果错"伪装成成功。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Info,
    Warning,
    Error,
    Fatal,
}

/// 位置三元组：仿真时间 + 块号 + 源文件行号。缺一不可。
#[derive(Clone, Copy, Debug, Default)]
pub struct TimeRef {
    /// 时间轴定位
    pub sim_time: f64,
    /// 跳转
    pub block: u32,
    /// 回编辑器改代码
    pub line: u32,
}

/// 一条报警。编号是稳定契约，消息可以改。
#[derive(Clone, Debug)]
pub struct Alarm {
    pub id: &'static str,
    pub severity: Severity,
    pub at: TimeRef,
    pub message: String,
    /// 怎么办。**每条必须有**（有质量门测试守着）。
    pub hint: Option<String>,
    /// 聚合计数
    pub count: u32,
}

impl Alarm {
    pub fn new(id: &'static str, severity: Severity, message: impl Into<String>) -> Self {
        Self {
            id,
            severity,
            at: TimeRef::default(),
            message: message.into(),
            hint: None,
            count: 1,
        }
    }
    pub fn with_hint(mut self, h: impl Into<String>) -> Self {
        self.hint = Some(h.into());
        self
    }
    pub fn at(mut self, r: TimeRef) -> Self {
        self.at = r;
        self
    }
}

/// 仿真报告。
#[derive(Clone, Debug, Default)]
pub struct SimReport {
    pub programmed_time_s: f64,
    pub actual_time_s: f64,
    pub removed_volume_mm3: f64,
    pub gouge_volume_mm3: f64,
    /// 过切最大深度（解析扫掠点测试，mm）。0 = 没有超阈值的过切。
    ///
    /// 语义是"设计面被刀具表面侵入的深度"——掠切（刀面擦过设计面）时就是
    /// 真实过切量；孔/槽这类整体切除时它给出侵入的局部尺度，全貌看区域。
    pub gouge_max_depth_mm: f64,
    /// 最深过切的位置（世界坐标，在设计面上）
    pub gouge_at: DVec3,
    /// 过切区域的包围盒（世界坐标，体素中心范围）。`None` = 没有过切。
    pub gouge_region: Option<(DVec3, DVec3)>,
    /// 残留体积（mm³）：设计气区里还剩的材料
    pub residual_volume_mm3: f64,
    pub residual_max_thickness: f64,
    /// 残留最厚那一点是否**已饱和**（设计气区距离被钳到 `band`）。
    ///
    /// `true` 时 `residual_max_thickness` 只是**下限**，真实值更大。
    /// 只有 `--design x.stl`（`DesignModel::Field`）会饱和；解析盒没有上限。
    pub residual_thickness_capped: bool,
    /// 设计气区距离的钳制上限（`None` = 解析盒无上限）。显示"上限"时用它。
    pub residual_thickness_cap: Option<f64>,
    pub residual_at: DVec3,
    pub alarms: Vec<Alarm>,
    pub max_nonlinear_error: f64,
    /// 非线性误差最大的块（`MotionProgram::blocks` 下标）与源文件行号
    pub max_nonlinear_block: u32,
    pub max_nonlinear_line: u32,
    pub worst_pose: DVec3,
}

impl SimReport {
    /// 总体可信度：任何 Fatal → 中止；任何 Error → 不可信。
    pub fn overall_severity(&self) -> Option<Severity> {
        self.alarms.iter().map(|a| a.severity).max()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn severity_orders_correctly() {
        assert!(Severity::Fatal > Severity::Error);
        assert!(Severity::Error > Severity::Warning);
        assert!(Severity::Warning > Severity::Info);
    }

    #[test]
    fn error_alarm_marks_report_untrustworthy() {
        let mut r = SimReport::default();
        r.alarms.push(Alarm::new("GCODE-012", Severity::Warning, "未知 M 代码").with_hint("已跳过"));
        assert_eq!(r.overall_severity(), Some(Severity::Warning));
        r.alarms.push(Alarm::new("KIN-001", Severity::Error, "奇异点").with_hint("降低摆角"));
        assert_eq!(r.overall_severity(), Some(Severity::Error));
    }
}
