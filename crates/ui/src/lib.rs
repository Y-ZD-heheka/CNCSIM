//! # cncsim-ui
//!
//! bevy_egui 面板的**数据层**：把 HMI 状态翻译成可直接画的东西（文案 / 色调 / 行）。
//!
//! **只依赖 `cncsim-hmi`，不直连 `sim`**——HMI 是 UI 的唯一后端。
//! 这样 UI 换实现（egui → 内置 Feathers）不影响仿真层。
//! 这条由 `tools/check_deps.py` 守卫。
//!
//! ## 为什么这里是纯函数，不是 bevy 系统
//!
//! 「面板上该显示什么」是**可判定的**，「怎么画」不是。所以这里的函数只返回数据
//! （`&'static str`、`Vec<AlarmRow>`、`bool`），一个 `egui::Ui` 都不碰：
//!
//! - 能在**没有窗口**的环境 `cargo test` 锁定（CI 不需要显示器）；
//! - 改排布、改配色不会让测试变红；改判据、改文案才会；
//! - `render::shell` 只负责把结果塞进 `egui::Window`。
//!
//! 真正该被测试盯住的是**判据**：哪种状态算「卡死」、四级严重度能不能一眼分开、
//! 最严重的报警在不在最上面。这些判错了，界面照样能画出来。

use cncsim_hmi::{AlarmView, ControlState, Hmi, Severity};

/// 横幅色调。**只表达「能不能继续操作」，不表达「好不好看」**。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tone {
    /// 中性：空闲 / 已加载
    Neutral,
    /// 正常进行
    Active,
    /// 需要用户注意（暂停）
    Attention,
    /// 坏了，必须复位
    Fatal,
}

/// 一条报警的显示行。
///
/// **编号是契约，消息会改**，所以拆成两个字段：测试断言 `id`，不碰 `text`。
#[derive(Clone, Debug, PartialEq)]
pub struct AlarmRow {
    pub id: &'static str,
    /// 严重度徽标（`[!!]` / `[E]` / `[W]` / `[i]`）
    pub badge: &'static str,
    pub text: String,
    /// 位置三元组拼成的可读串：`t=3.742s · 块 4 · 行 5`
    pub location: String,
    /// 怎么办。没有 hint 的报警不该出现在 UI 上（质量门在 `hmi` 里守着）。
    pub hint: Option<String>,
    /// 聚合计数，`>1` 才显示
    pub count: u32,
}

/// UI 只读视图：从 HMI 取状态渲染，不反向修改。
pub struct UiView<'a> {
    pub hmi: &'a Hmi,
}

impl UiView<'_> {
    /// 顶部横幅文案：直接反映总体可信度，不用用户翻报警列表。
    pub fn status_banner(&self) -> &'static str {
        match self.hmi.state() {
            ControlState::Idle => "就绪",
            ControlState::Ready { .. } => "已加载",
            ControlState::Running { .. } => "运行中",
            ControlState::Paused { .. } => "已暂停",
            ControlState::EStopped { .. } => "急停——需复位",
            ControlState::Faulted { .. } => "故障——需复位",
        }
    }

    /// 横幅色调。**急停与故障都是 `Fatal`**：对用户都是「停下、复位、重来」，
    /// 没必要在颜色上区分「人按的」和「机器判的」。
    pub fn status_tone(&self) -> Tone {
        match self.hmi.state() {
            ControlState::Idle | ControlState::Ready { .. } => Tone::Neutral,
            ControlState::Running { .. } => Tone::Active,
            ControlState::Paused { .. } => Tone::Attention,
            ControlState::EStopped { .. } | ControlState::Faulted { .. } => Tone::Fatal,
        }
    }

    /// 播放按钮该不该亮。
    pub fn play_enabled(&self) -> bool {
        self.hmi.can_play()
    }

    /// 复位按钮该不该亮：**只有卡死的状态才需要复位**。
    /// 正常运行中摆一个亮着的复位键，就是给用户一个误触的机会。
    pub fn reset_enabled(&self) -> bool {
        matches!(
            self.hmi.state(),
            ControlState::EStopped { .. } | ControlState::Faulted { .. }
        )
    }

    /// 报警列表，已按严重度降序排。
    ///
    /// 排序放在这里而不是 egui 里：一屏 30 条时，「最严重的那条在哪」是判据问题，
    /// 不是排版问题。同严重度保持入队顺序（稳定排序）——后到的排后面，
    /// 与「反复仿真的报告逐次追加」的语义一致。
    pub fn alarm_rows(&self) -> Vec<AlarmRow> {
        let mut rows: Vec<AlarmRow> = self.hmi.alarms().iter().map(row_of).collect();
        rows.sort_by_key(|r| std::cmp::Reverse(r.severity_rank()));
        rows
    }
}

impl AlarmRow {
    /// 排序用的严重度序（越大越严重）。`badge` 已经映射过一遍，这里不重复 match。
    fn severity_rank(&self) -> u8 {
        match self.badge {
            "[!!]" => 3,
            "[E]" => 2,
            "[W]" => 1,
            _ => 0,
        }
    }
}

fn badge_of(sev: Severity) -> &'static str {
    use Severity::*;
    match sev {
        Fatal => "[!!]",
        Error => "[E]",
        Warning => "[W]",
        Info => "[i]",
    }
}

fn row_of(a: &AlarmView) -> AlarmRow {
    AlarmRow {
        id: a.id,
        badge: badge_of(a.severity),
        text: a.message.clone(),
        location: format!(
            "t={:.3}s · 块 {} · 行 {}",
            a.at.sim_time, a.at.block, a.at.line
        ),
        hint: a.hint.clone(),
        count: a.count,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_hmi::{ControlState, Hmi};
    use cncsim_math::DVec3;
    use cncsim_sim::{Alarm, Severity, SimReport, TimeRef};

    fn view(h: &Hmi) -> UiView<'_> {
        UiView { hmi: h }
    }

    /// 每个状态都要有文案、且**不能互相撞车**——两个状态显示同一句话，
    /// 用户就分不出"在跑"和"卡住了"。这是批量版本，比逐条断言更难糊弄。
    #[test]
    fn every_state_has_a_distinct_banner() {
        let states = [
            ControlState::Idle,
            ControlState::Ready { program_len: 10 },
            ControlState::Running { since: 0.0 },
            ControlState::Paused {
                at: 1.0,
                reason: cncsim_hmi::PauseReason::Operator,
            },
            ControlState::EStopped { at: 0.0, block: 3 },
            ControlState::Faulted { alarm: "GOUGE" },
        ];
        let mut seen: Vec<&'static str> = Vec::new();
        for s in states {
            let mut h = Hmi::default();
            h.force_state(s.clone()); // 见下方 force_state 说明
            let b = view(&h).status_banner();
            assert!(!b.is_empty(), "{s:?} 没有横幅文案");
            assert!(!seen.contains(&b), "{s:?} 与之前的状态文案相同：{b}");
            seen.push(b);
        }
        assert_eq!(seen.len(), 6);
    }

    /// 色调必须反映"能不能继续"，不是"好不好看"。
    #[test]
    fn tone_tracks_operability() {
        let mut h = Hmi::default();
        assert_eq!(view(&h).status_tone(), Tone::Neutral);

        h.ingest(&{
            let mut r = SimReport::default();
            r.alarms
                .push(Alarm::new("GOUGE", Severity::Error, "过切"));
            r
        });
        assert_eq!(view(&h).status_tone(), Tone::Fatal);
        assert!(!view(&h).play_enabled());

        h.reset();
        h.estop(1.0, 2, DVec3::ZERO);
        assert_eq!(view(&h).status_tone(), Tone::Fatal);
    }

    /// 复位按钮只在"卡死"时亮。正常运行中给一个亮着的复位键 = 误触机会。
    #[test]
    fn reset_only_enabled_when_stuck() {
        let mut h = Hmi::default();
        assert!(!view(&h).reset_enabled(), "Idle 不该亮复位");

        h.ingest(&{
            let mut r = SimReport::default();
            r.alarms
                .push(Alarm::new("GOUGE", Severity::Error, "过切 0.3 mm"));
            r
        });
        assert!(view(&h).reset_enabled(), "故障态必须能复位");

        h.reset();
        assert!(!view(&h).reset_enabled(), "复位后按键要灭");
    }

    /// 报警行必须带**位置三元组**与**提示**——只有编号和消息的报警，
    /// 用户知道错了但不知道去哪改。
    #[test]
    fn alarm_row_carries_location_and_hint() {
        let mut h = Hmi::default();
        let mut r = SimReport::default();
        r.alarms.push(
            Alarm::new("GOUGE", Severity::Error, "过切 0.32 mm")
                .with_hint("检查第 3 序的 Z 向偏置")
                .at(TimeRef {
                    sim_time: 3.742,
                    block: 4,
                    line: 5,
                }),
        );
        h.ingest(&r);

        let rows = view(&h).alarm_rows();
        assert_eq!(rows.len(), 1);
        let row = &rows[0];
        assert_eq!(row.id, "GOUGE");
        assert_eq!(row.badge, "[E]");
        assert_eq!(row.text, "过切 0.32 mm");
        // 位置串整串断言，不拆成 contains——第一次写的是 `contains('4')`，
        // 结果 `t=3.742` 里的 4 让「块号写成 0」也能通过（已实测漏掉）。
        assert_eq!(row.location, "t=3.742s · 块 4 · 行 5");
        assert_eq!(row.hint.as_deref(), Some("检查第 3 序的 Z 向偏置"));
        assert_eq!(row.count, 1);
    }

    /// 四级严重度必须有**四种不同徽标**——两级都显示 `[!]`，
    /// 用户就分不出"跑完看看就行"和"别上机床"。
    #[test]
    fn severity_badges_are_distinct() {
        let sev = [
            Severity::Info,
            Severity::Warning,
            Severity::Error,
            Severity::Fatal,
        ];
        let mut h = Hmi::default();
        let mut r = SimReport::default();
        for s in sev {
            r.alarms.push(Alarm::new("TEST", s, format!("{s:?}")));
        }
        h.ingest(&r);

        let rows = view(&h).alarm_rows();
        let badges: Vec<&str> = rows.iter().map(|r| r.badge).collect();
        assert_eq!(badges.len(), 4);
        let uniq: std::collections::BTreeSet<&str> = badges.iter().copied().collect();
        assert_eq!(uniq.len(), 4, "四级严重度徽标应互不相同，实际 {badges:?}");
    }

    /// 报警按严重度**降序**排：最严重的必须在最上面，
    /// 不然一屏 30 条时用户要滚到哪儿都不知道。
    #[test]
    fn alarms_sorted_by_severity_desc() {
        let mut h = Hmi::default();
        let mut r = SimReport::default();
        r.alarms.push(Alarm::new("A", Severity::Info, "提示"));
        r.alarms.push(Alarm::new("B", Severity::Fatal, "致命"));
        r.alarms.push(Alarm::new("C", Severity::Warning, "警告"));
        r.alarms.push(Alarm::new("D", Severity::Error, "错误"));
        h.ingest(&r);

        let ids: Vec<&str> = view(&h).alarm_rows().iter().map(|r| r.id).collect();
        assert_eq!(ids, vec!["B", "D", "C", "A"]);
    }
}
