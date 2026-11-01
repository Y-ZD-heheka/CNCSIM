//! 回放时间轴：块 → 时间区间的预计算，供 UI 拖时间轴。
//!
//! **与 [`crate::run`] 同一套时间口径**：同样的圆弧离散（`arc_sagitta`）、
//! 同样的 `segment_time`、同样的 M0/M2/M30 截断。所以
//! `total_time() == SimReport::programmed_time_s` **逐位相等**——这不是巧合，
//! 是测试锁住的契约。UI 时间轴的总长与报告里的节拍必须对得上，否则
//! "报告说 8.6 秒，时间轴却拖到 8.61"这种不一致会毁掉信任。
//!
//! 回放只做**位置插值**（tip 线性 + 刀轴 slerp），不是物理仿真——
//! 材料状态已经由 `run` 算完，回放只是把"某一时刻刀具在哪"画出来。

use crate::run::{segment_time, RunConfig};
use cncsim_gcode::{MotionProgram, StopKind};
use cncsim_math::DVec3;
use cncsim_motion::arc::sample_arc;
use cncsim_motion::{slerp, MotionKind, Pose};

/// 一个块的时间区间与端点姿态。
#[derive(Clone, Copy, Debug)]
pub struct BlockSpan {
    /// `MotionProgram::blocks` 下标
    pub block: u32,
    /// 源文件行号
    pub line: u32,
    pub t0: f64,
    pub t1: f64,
    pub from: Pose,
    pub to: Pose,
    pub motion: MotionKind,
    /// 这是**第几刀**（只数切削块，快移/停延不计）。非切削块填"即将切的那一刀"。
    ///
    /// 与 `cncsim_sim::run` 里盖到体素上的序号是同一个数：
    /// 回放用它取刀刃颜色，仿真用它盖切面颜色，两边因此必然一致。
    /// 判定必须与 `run` 逐字相同（`Linear | Arc`），否则刀刃和刚切出的面
    /// 会差一刀——那种错看起来像"颜色对不上"，很难往回追。
    pub cut: u32,
}

impl BlockSpan {
    pub fn duration(&self) -> f64 {
        self.t1 - self.t0
    }
}

/// 某一时刻的回放帧。
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub block: u32,
    pub line: u32,
    /// 第几刀（非切削块为"即将切的那一刀"）。刀刃颜色由它决定。
    pub cut: u32,
    pub pose: Pose,
}

/// 回放时间轴。
#[derive(Clone, Debug, Default)]
pub struct Playback {
    spans: Vec<BlockSpan>,
    total: f64,
}

impl Playback {
    /// 从程序预计算时间轴。**不碰体素、不碰碰撞**——纯运动学 + 时间。
    pub fn from_program(program: &MotionProgram, cfg: &RunConfig) -> Self {
        let mut spans = Vec::with_capacity(program.len());
        let mut prev = program
            .blocks
            .first()
            .map(|b| b.target)
            .unwrap_or(Pose { tip: DVec3::ZERO, axis: DVec3::Z });
        let mut t = 0.0f64;
        let mut cut_ordinal: u32 = 0;

        for (bi, block) in program.blocks.iter().enumerate() {
            // 程序停止：M0/M2/M30 之后不播（M1 是面板开关，不猜）
            if let Some(s) = program.stops.iter().find(|s| s.block == bi)
                && !matches!(s.kind, StopKind::OptionalStop)
            {
                break;
            }

            let to = block.target;
            // 与 run 相同的离散：圆弧按弦高采样，其余单折线
            let pts: Vec<DVec3> = match block.motion {
                MotionKind::Arc { center, clockwise, plane } => {
                    sample_arc(prev.tip, to.tip, center, clockwise, plane, cfg.arc_sagitta)
                }
                MotionKind::Dwell { .. } => Vec::new(),
                MotionKind::Rapid | MotionKind::Linear => vec![to.tip],
            };

            // 切削块 = 一刀（判据与 `run` 相同）。
        //
        // **非切削块也用当前的 `cut_ordinal`**：那是"即将要切的那一刀"的序号，
        // 于是抬刀横移的过程中刀刃已经换成了下一刀的颜色，不会在快移里
        // 闪回第 1 刀的颜色（`cut_ordinal` 只在切完之后才自增）。
        let is_cut = matches!(block.motion, MotionKind::Linear | MotionKind::Arc { .. });
        let cut_here = cut_ordinal;

        let mut dur = 0.0f64;
            let mut cur = prev.tip;
            for &p in &pts {
                dur += segment_time((p - cur).length(), block, cfg);
                cur = p;
            }
            // 停留：G4 的停延与固定循环的孔底暂停都不动刀，只吃时间
            let hold = match block.motion {
                MotionKind::Dwell { seconds } => seconds,
                _ => block.dwell,
            };
            dur += hold;

            spans.push(BlockSpan {
                block: bi as u32,
                line: block.line_no,
                t0: t,
                t1: t + dur,
                from: prev,
                to,
                motion: block.motion,
                cut: cut_here,
            });
            if is_cut {
                cut_ordinal += 1;
            }
            t += dur;
            prev = to;
        }

        Self { spans, total: t }
    }

    /// 总时长（与 `SimReport::programmed_time_s` 逐位相等）。
    pub fn total_time(&self) -> f64 {
        self.total
    }

    pub fn spans(&self) -> &[BlockSpan] {
        &self.spans
    }

    pub fn is_empty(&self) -> bool {
        self.spans.is_empty()
    }

    /// `t` 落在哪个块区间。零时长块被跳过（`t1 <= t` 的二分条件天然滤掉）。
    pub fn span_index_at(&self, t: f64) -> Option<usize> {
        if self.spans.is_empty() {
            return None;
        }
        let i = self.spans.partition_point(|s| s.t1 <= t);
        Some(i.min(self.spans.len() - 1))
    }

    /// 某一时刻的刀位。`t` 会被钳制到 `[0, total_time]`；
    /// 空程序返回 `None`。
    pub fn at(&self, t: f64) -> Option<Frame> {
        let i = self.span_index_at(t)?;
        let s = &self.spans[i];
        let u = if s.duration() > 0.0 {
            ((t - s.t0) / s.duration()).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let pose = match s.motion {
            // 停留：刀不动
            MotionKind::Dwell { .. } => s.to,
            _ => Pose {
                tip: s.from.tip + (s.to.tip - s.from.tip) * u,
                axis: slerp(s.from.axis, s.to.axis, u),
            },
        };
        Some(Frame { block: s.block, line: s.line, cut: s.cut, pose })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::run::run;
    use cncsim_gcode::{interpret, InterpConfig};
    use cncsim_math::DVec2;
    use cncsim_tool::{flat_end_mill, CutterKind, Profile, ToolDef};

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

    fn cfg() -> RunConfig {
        RunConfig {
            stock_origin: DVec3::ZERO,
            stock_size: [200, 200, 100],
            ..Default::default()
        }
    }

    fn prog(src: &str) -> MotionProgram {
        interpret(src, &InterpConfig::default())
    }

    #[test]
    fn total_time_is_bit_identical_to_the_report() {
        // 直线 + 圆弧 + G4 停延 + 固定循环孔底暂停：四条时间路径全走一遍
        let src = "G0 X2 Y10 Z9.5
G1 X8 F300
G2 X8 Y14 I0 J2
G4 P0.3
G82 X12 Y10 Z6 R9 P0.2 F100
G80
M30
";
        let p = prog(src);
        let play = Playback::from_program(&p, &cfg());
        let out = run(&p, &tool(), &cfg());
        assert_eq!(
            play.total_time(),
            out.report.programmed_time_s,
            "回放总长必须与报告节拍逐位一致"
        );
    }

    #[test]
    fn stops_truncate_the_timeline() {
        let src = "G0 X2 Y10 Z9.5
G1 X18 F300
M0
G1 X2 F300
M30
";
        let p = prog(src);
        let play = Playback::from_program(&p, &cfg());
        // M0 之后不播：时间轴止于第一刀
        let last = play.spans().last().unwrap();
        assert_eq!(last.line, 2, "最后一块应是 M0 之前那行");
        let out = run(&p, &tool(), &cfg());
        assert_eq!(play.total_time(), out.report.programmed_time_s);
    }

    #[test]
    fn endpoints_and_midpoint_interpolate() {
        let p = prog("G0 X0 Y0 Z10\nG1 X10 Y0 Z5 F600\nM30\n");
        let play = Playback::from_program(&p, &cfg());
        let total = play.total_time();
        let a = play.at(0.0).unwrap();
        let b = play.at(total).unwrap();
        assert!((a.pose.tip - DVec3::new(0.0, 0.0, 10.0)).length() < 1e-12);
        assert!((b.pose.tip - DVec3::new(10.0, 0.0, 5.0)).length() < 1e-12);

        // 匀速段中点：tip 恰在两端中点
        let i = play.span_index_at(0.0).unwrap();
        let s = play.spans()[i];
        let mid = play.at((s.t0 + s.t1) / 2.0).unwrap();
        let want = s.from.tip + (s.to.tip - s.from.tip) * 0.5;
        assert!((mid.pose.tip - want).length() < 1e-12);
    }

    #[test]
    fn seek_is_monotonic_across_the_whole_timeline() {
        let src = "G0 X2 Y10 Z9.5
G1 X18 F300
G2 X18 Y14 I0 J2
G1 X2 F300
M30
";
        let p = prog(src);
        let play = Playback::from_program(&p, &cfg());
        let total = play.total_time();
        assert!(total > 0.0);
        let mut last_block = 0u32;
        for k in 0..=200 {
            let t = total * k as f64 / 200.0;
            let f = play.at(t).unwrap();
            assert!(f.block >= last_block, "块号必须单调不减：{} < {}", f.block, last_block);
            last_block = f.block;
        }
        // 越界钳制
        assert!(play.at(-1.0).is_some());
        assert!(play.at(total * 10.0).is_some());
    }

    #[test]
    fn dwell_holds_the_pose() {
        let p = prog("G0 X5 Y5 Z10\nG4 P1.0\nM30\n");
        let play = Playback::from_program(&p, &cfg());
        let dwell = play
            .spans()
            .iter()
            .find(|s| matches!(s.motion, MotionKind::Dwell { .. }))
            .expect("应有 Dwell 块");
        let mid = play.at((dwell.t0 + dwell.t1) / 2.0).unwrap();
        assert!((mid.pose.tip - dwell.to.tip).length() < 1e-12, "停留期间刀不动");
    }

    #[test]
    fn empty_program_has_no_frames() {
        let p = prog("");
        let play = Playback::from_program(&p, &cfg());
        assert!(play.is_empty());
        assert_eq!(play.at(0.0).map(|f| f.block), None);
    }
}
