//! # cncsim-hmi
//!
//! 人机控制：状态机、急停、倍率、报警队列。
//!
//! **HMI 不是 UI 的一部分，是独立状态机。** `ui` 只依赖它，不直连 `sim`。
//! 见 docs/HMI与控制语义.md。

use cncsim_sim::Alarm;

/// **UI 的唯一入海口**。
///
/// `ui` 只允许依赖 `hmi`（架构铁律），但面板要显示报警的编号/严重度/位置，
/// 就必须能看到 `Alarm` 和 `Severity`。与其让 `ui` 绕过白名单去依赖 `sim`
/// （那正是这层隔离要防的事），不如由 HMI 把「UI 需要的那一小片 sim 词汇」
/// 显式转出来 —— 范围就这么大，加宽要改这里。
pub use cncsim_sim::{Alarm as AlarmView, Severity};


/// 操作控制状态（`cncsim-hmi`）。与 `sim::SimState` 正交。
#[derive(Clone, Debug, PartialEq, Default)]
pub enum ControlState {
    #[default]
    Idle,
    Ready {
        program_len: usize,
    },
    Running {
        since: f64,
    },
    Paused {
        at: f64,
        reason: PauseReason,
    },
    /// 急停：**必须复位才能离开**
    EStopped {
        at: f64,
        block: u32,
    },
    /// 致命错误：必须复位
    Faulted {
        alarm: &'static str,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PauseReason {
    Operator,
    ProgramM0,
    BufferStarved,
}

/// 计算模式：交互限速保 p99，批处理全速保吞吐。
///
/// 关键前提：仿真全速是 14–36 倍实时，而用户回放最多 8 倍——
/// **交互模式下"全速算"是浪费**，多出的算力只在抢 UI 的核。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ComputeMode {
    Interactive { speed_cap: f64 },
    Batch,
}

impl Default for ComputeMode {
    fn default() -> Self {
        Self::Interactive { speed_cap: 4.0 }
    }
}

/// 急停事件。**可撤销的带时间戳事件，不是不可逆破坏**——
/// 因为可以从头确定性重算，用户可以反复演练"第 30 秒急停会撞吗"。
#[derive(Clone, Copy, Debug)]
pub struct EStopEvent {
    pub at: f64,
    pub block: u32,
    pub tip: cncsim_math::DVec3,
}

/// HMI 状态容器。
#[derive(Default)]
pub struct Hmi {
    state: ControlState,
    mode: ComputeMode,
    alarms: Vec<Alarm>,
    estop: Option<EStopEvent>,
}

impl Hmi {
    pub fn state(&self) -> &ControlState {
        &self.state
    }

    /// 直接设定状态，**仅供测试与「从快照恢复」**。
    ///
    /// 为什么不给 `ui` 用：UI 若能改状态，就会有人用它绕过 `estop()`/`reset()`
    /// 的副作用（报警入队、必须复位）。这条通道刻意做成「显式、丑、有文档」，
    /// 而不是顺手给个 `set_state`。
    pub fn force_state(&mut self, s: ControlState) {
        self.state = s;
    }

    pub fn mode(&self) -> ComputeMode {
        self.mode
    }

    pub fn set_mode(&mut self, m: ComputeMode) {
        self.mode = m;
    }

    pub fn alarms(&self) -> &[Alarm] {
        &self.alarms
    }

    /// 急停。急停后**不能直接 play**，必须 `reset()`。
    pub fn estop(&mut self, at: f64, block: u32, tip: cncsim_math::DVec3) {
        self.estop = Some(EStopEvent { at, block, tip });
        self.state = ControlState::EStopped { at, block };
        self.alarms.push(
            Alarm::new("HMI-001", cncsim_sim::Severity::Fatal, "急停")
                .with_hint("程序已作废；复位后可重新仿真"),
        );
    }

    /// 复位：清空仿真与报警，回到 Idle。
    pub fn reset(&mut self) {
        self.state = ControlState::Idle;
        self.alarms.clear();
        self.estop = None;
    }

    pub fn can_play(&self) -> bool {
        !matches!(self.state, ControlState::EStopped { .. } | ControlState::Faulted { .. })
    }

    /// 把一次仿真的报告灌进来：报警入队；有 Error+ 就进 `Faulted`。
    ///
    /// **不清空旧报警**——"第一版过切 3 mm，改完 0.2 mm"的对比是常见操作，
    /// 清空走 `reset()`。只升不降：`Faulted` 不会被一次"干净"的报告救回来，
    /// 那要操作员确认（`reset()`）。
    pub fn ingest(&mut self, report: &cncsim_sim::SimReport) {
        self.alarms.extend(report.alarms.iter().cloned());
        let Some(sev) = report.overall_severity() else {
            return;
        };
        if sev < cncsim_sim::Severity::Error {
            return; // 警告/提示只入队，不改状态
        }
        if !matches!(self.state, ControlState::Faulted { .. }) {
            let id = report
                .alarms
                .iter()
                .find(|a| a.severity >= cncsim_sim::Severity::Error)
                .map(|a| a.id)
                .unwrap_or("REPORT");
            self.state = ControlState::Faulted { alarm: id };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_math::DVec3;

    #[test]
    fn estop_requires_reset() {
        let mut h = Hmi::default();
        assert!(h.can_play());
        h.estop(12.34, 1240, DVec3::new(1.0, 2.0, 3.0));
        assert!(!h.can_play(), "急停后必须复位才能继续");
        assert_eq!(h.alarms().len(), 1);
        assert_eq!(h.alarms()[0].id, "HMI-001");
        h.reset();
        assert!(h.can_play());
        assert!(h.alarms().is_empty());
    }

    #[test]
    fn interactive_mode_caps_speed() {
        let m = ComputeMode::default();
        assert!(matches!(m, ComputeMode::Interactive { speed_cap } if speed_cap > 1.0));
    }

    #[test]
    fn ingest_queues_alarms_and_faults_on_error() {
        use cncsim_sim::{Alarm, Severity, SimReport};
        let mut h = Hmi {
            state: ControlState::Ready { program_len: 10 },
            ..Default::default()
        };

        // 只有 Warning：报警入队，状态不动
        let warn = SimReport {
            alarms: vec![Alarm::new("RESIDUAL", Severity::Warning, "残留 1 mm³")],
            ..Default::default()
        };
        h.ingest(&warn);
        assert_eq!(h.alarms().len(), 1);
        assert_eq!(*h.state(), ControlState::Ready { program_len: 10 });
        assert!(h.can_play());

        // 有 Error：进 Faulted，必须复位才能再播
        let err = SimReport {
            alarms: vec![Alarm::new("GOUGE", Severity::Error, "过切 0.3 mm")],
            ..Default::default()
        };
        h.ingest(&err);
        assert_eq!(h.alarms().len(), 2, "旧报警不清，累积对比");
        assert_eq!(*h.state(), ControlState::Faulted { alarm: "GOUGE" });
        assert!(!h.can_play());

        // 再来一份干净的报告也救不回来——要操作员 reset
        h.ingest(&SimReport::default());
        assert!(!h.can_play());
        h.reset();
        assert!(h.can_play());
        assert!(h.alarms().is_empty());
    }
}
