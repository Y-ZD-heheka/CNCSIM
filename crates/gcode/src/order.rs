//! 块内执行顺序。
//!
//! ## 权威定义
//!
//! `refs/linuxcnc/overview.adoc` §G-code Order of Execution 开篇原文：
//!
//! > The order of execution of items on a line is defined **not by the position**
//! > of each item on the line, but by the following list
//!
//! 所以一行里 `G43 H7 G1 Z-5` 与 `G1 Z-5 G43 H7` **完全等价**。
//! 顺序由本模块的 [`Phase`] 固定，与字在行内的位置无关。
//!
//! ## 为什么顺序错了很危险
//!
//! | 顺序 | 错了会怎样 |
//! |---|---|
//! | `Dwell` 在 `PlaneSelect` 之前 | 这是 LinuxCNC 的实际顺序；把它放到最后（早期文档的错误）会让 `G4 P1` 与后续字互相影响 |
//! | `CutterLengthComp` 在 `Motion` 之前 | `G43 H7 G1 Z-5` 若先动后补，**第一刀会偏一个刀长** |
//! | `DistanceMode` 在 `Motion` 之前 | `G91 G1 X10` 若不生效，会走成绝对坐标 |
//! | `FeedMode` 在 `FeedRate` 之前 | `G93 F0.5` 里 F 是倒数时间，不是 mm/min |
//!
//! 顺序写错，单字测试全过，**只有组合块才会错**。

/// 块内执行阶段。**判别值即执行顺序**，直接比较大小即可。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Phase {
    /// O-word 流程控制
    FlowControl = 0,
    /// 注释 / 消息
    Comment = 1,
    /// G93 / G94 / G95
    FeedMode = 2,
    /// F
    FeedRate = 3,
    /// S
    SpindleSpeed = 4,
    /// T
    SelectTool = 5,
    /// M62–M68
    HalIo = 6,
    /// M6 / M61
    ChangeTool = 7,
    /// M3 / M4 / M5
    Spindle = 8,
    /// M70 / M71 / M72 / M73
    SaveState = 9,
    /// M7 / M8 / M9
    Coolant = 10,
    /// M48 / M49
    Overrides = 11,
    /// M100–M199
    UserCommand = 12,
    /// **G4 —— 在平面/单位/补偿/距离模式之前**，不是最后
    Dwell = 13,
    /// G17 / G18 / G19
    PlaneSelect = 14,
    /// G20 / G21
    Units = 15,
    /// G40 / G41 / G42
    CutterRadiusComp = 16,
    /// **G43 / G49 —— 必须在运动之前**
    CutterLengthComp = 17,
    /// G54–G59.3
    CoordSystem = 18,
    /// G61 / G61.1 / G64
    PathControl = 19,
    /// **G90 / G91 —— 必须在运动之前**
    DistanceMode = 20,
    /// G98 / G99
    RetractMode = 21,
    /// **A / B / C 轴字——必须在运动之前**
    ///
    /// 与 G90/G91 的先后不能颠倒：`G91 A30` 是增量角。
    /// 上面 20 号（DistanceMode）已经跑完了，所以这里读到的是本行最新的距离模式。
    RotaryAxis = 22,
    /// G28 / G30 / G10 / G52 / G92——**会挂起组 1**
    GroupZero = 23,
    /// G0–G3 等，受 G53 修饰
    Motion = 24,
    /// M0 / M1 / M2 / M30 / M60
    Stop = 25,
}

/// G 代码的执行阶段。`None` = 未知代码。
pub fn phase_of_g(g: crate::modal::GCode) -> Option<Phase> {
    use crate::modal::GCode as G;
    Some(match g {
        G::G93 | G::G94 | G::G95 => Phase::FeedMode,
        G::G4 => Phase::Dwell,
        G::G17 | G::G18 | G::G19 | G::G17_1 | G::G18_1 | G::G19_1 => Phase::PlaneSelect,
        G::G20 | G::G21 => Phase::Units,
        G::G40 | G::G41 | G::G42 | G::G41_1 | G::G42_1 => Phase::CutterRadiusComp,
        G::G43 | G::G43_1 | G::G43_2 | G::G43_4 | G::G49 => Phase::CutterLengthComp,
        G::G54 | G::G55 | G::G56 | G::G57 | G::G58 | G::G59 | G::G59_1 | G::G59_2 | G::G59_3 => {
            Phase::CoordSystem
        }
        G::G61 | G::G61_1 | G::G64 => Phase::PathControl,
        // 组 3 与组 4 都在运动之前定死距离语义
        G::G90 | G::G91 | G::G90_1 | G::G91_1 => Phase::DistanceMode,
        G::G98 | G::G99 => Phase::RetractMode,
        // 组 0 里影响坐标的那些
        G::G10 | G::G28 | G::G28_1 | G::G28_2 | G::G30 | G::G30_1 | G::G52 | G::G92 | G::G92_1
        | G::G92_2 | G::G92_3 => Phase::GroupZero,
        // 运动。G53 不产生运动，它只修饰同行的运动，所以归到 Motion 之前最近的位置
        G::G53 => Phase::GroupZero,
        G::G0 | G::G1 | G::G2 | G::G3 | G::G5 | G::G5_1 | G::G5_2 | G::G5_3 | G::G33 | G::G33_1
        | G::G38_2 | G::G38_3 | G::G38_4 | G::G38_5 | G::G73 | G::G76 => Phase::Motion,
        // G80–G89 固定循环：v1 只把 G80 当"取消"
        G::G80 => Phase::Motion,
        // 组 9 / 14 / 15：v1 未实现，但要知道它们在哪个阶段
        G::G68_2 | G::G68_4 | G::G69 => Phase::CoordSystem,
        G::G96 | G::G97 => Phase::FeedMode,
        G::G7 | G::G8 => Phase::Units,
        _ => {
            // G81–G89 等：用范围判断
            if (810..=890).contains(&g.0) {
                Phase::Motion
            } else {
                return None;
            }
        }
    })
}

/// M 代码的执行阶段。`None` = 未知代码。
pub fn phase_of_m(m: crate::modal::MCode) -> Option<Phase> {
    use crate::modal::MCode as M;
    Some(match m {
        M::M62 | M::M63 | M::M64 | M::M65 | M::M66 | M::M67 | M::M68 => Phase::HalIo,
        M::M6 | M::M61 => Phase::ChangeTool,
        M::M3 | M::M4 | M::M5 => Phase::Spindle,
        M::M70 | M::M71 | M::M72 | M::M73 => Phase::SaveState,
        M::M7 | M::M8 | M::M9 => Phase::Coolant,
        M::M48 | M::M49 => Phase::Overrides,
        M::M0 | M::M1 | M::M2 | M::M30 | M::M60 => Phase::Stop,
        _ => {
            if (1000..=1990).contains(&m.0) {
                Phase::UserCommand
            } else {
                return None;
            }
        }
    })
}

/// 普通字的执行阶段（F / S / T）。
pub fn phase_of_letter(letter: u8) -> Option<Phase> {
    Some(match letter.to_ascii_uppercase() {
        b'F' => Phase::FeedRate,
        b'S' => Phase::SpindleSpeed,
        b'T' => Phase::SelectTool,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::modal::{GCode, MCode};

    /// **这是本模块最重要的一条测试**：把 LinuxCNC 权威清单的相对顺序钉死。
    ///
    /// 早期设计文档把 Dwell 排在最后（第 12 位），与原文矛盾——原文里
    /// Dwell 在 PlaneSelect / Units / 补偿 / 距离模式**之前**。这个测试就是为抓它。
    #[test]
    fn order_matches_linuxcnc_list() {
        let seq = [
            GCode::G93,  // 进给模式
            GCode::G17,  // 平面
            GCode::G20,  // 单位
            GCode::G40,  // 刀径补偿
            GCode::G43,  // 刀长补偿
            GCode::G54,  // 坐标系
            GCode::G61,  // 路径控制
            GCode::G90,  // 距离模式
            GCode::G98,  // 退刀模式
            GCode::G28,  // 组 0
            GCode::G1,   // 运动
        ];
        for w in seq.windows(2) {
            let a = phase_of_g(w[0]).unwrap();
            let b = phase_of_g(w[1]).unwrap();
            assert!(
                a < b,
                "{} 应在 {} 之前执行，实际 Phase {a:?} vs {b:?}",
                w[0].label(),
                w[1].label()
            );
        }
    }

    #[test]
    fn dwell_runs_before_plane_and_units() {
        let dwell = phase_of_g(GCode::G4).unwrap();
        assert!(dwell < phase_of_g(GCode::G17).unwrap(), "G4 在平面选择之前");
        assert!(dwell < phase_of_g(GCode::G20).unwrap(), "G4 在单位之前");
        assert!(dwell < phase_of_g(GCode::G43).unwrap(), "G4 在刀长补偿之前");
        assert!(dwell < phase_of_g(GCode::G90).unwrap(), "G4 在距离模式之前");
    }

    #[test]
    fn compensation_and_distance_precede_motion() {
        let motion = phase_of_g(GCode::G1).unwrap();
        assert!(phase_of_g(GCode::G43).unwrap() < motion, "G43 必须在运动之前");
        assert!(phase_of_g(GCode::G90).unwrap() < motion, "G90/G91 必须在运动之前");
        assert!(phase_of_g(GCode::G91).unwrap() < motion);
        assert!(phase_of_g(GCode::G54).unwrap() < motion);
    }

    #[test]
    fn feed_mode_precedes_feed_rate() {
        let fm = phase_of_g(GCode::G93).unwrap();
        let f = phase_of_letter(b'F').unwrap();
        assert!(fm < f, "G93 必须在 F 之前——否则 F 会被当成 mm/min");
    }

    #[test]
    fn group_zero_runs_between_distance_mode_and_motion() {
        let gz = phase_of_g(GCode::G28).unwrap();
        assert!(phase_of_g(GCode::G90).unwrap() < gz);
        assert!(gz < phase_of_g(GCode::G1).unwrap());
    }

    #[test]
    fn stop_runs_last() {
        let stop = phase_of_m(MCode::M30).unwrap();
        assert_eq!(stop, Phase::Stop);
        assert!(phase_of_g(GCode::G1).unwrap() < stop, "运动在停机之前");
    }

    #[test]
    fn spindle_and_coolant_before_compensation() {
        let spindle = phase_of_m(MCode::M3).unwrap();
        let coolant = phase_of_m(MCode::M8).unwrap();
        assert!(spindle < phase_of_g(GCode::G43).unwrap());
        assert!(coolant < phase_of_g(GCode::G43).unwrap());
        assert!(spindle < coolant, "主轴在冷却之前");
    }

    #[test]
    fn unknown_codes_have_no_phase() {
        assert_eq!(phase_of_g(GCode::tenths(1230)), None);
        assert_eq!(phase_of_m(MCode::tenths(9990)), None);
    }
}
