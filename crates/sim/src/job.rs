//! 后台仿真任务：把 [`crate::run::run_observed`] 挪到工作线程，用 channel 回报事件。
//!
//! 三条契约（测试锁住）：
//!
//! 1. **结果一致**：异步跑完与同步 `run` 的产物逐位相同（墙钟字段除外）——
//!    同一代码路径，工作线程里跑的正是 `run_observed`。
//! 2. **取消在块边界**：`AtomicBool` 由观察者检查，最长一个块的延迟。
//!    `Drop` 会取消并 join，绝不泄漏线程。
//! 3. **事件顺序**：`Started → Progress* → Done`。Progress 按墙钟节流
//!    （50 ms，块 0 必发）——UI 要的是流畅而不是逐块日志。
//!
//! 为什么不用 rayon：那是数据并行（同一算法切给多个核）。这里是**任务并行**
//! ——整段仿真挪走，主线程继续跑 UI/回放。两件事，两个工具。

use crate::run::{run_observed, Progress, RunConfig, RunOutput};
use cncsim_gcode::MotionProgram;
use cncsim_tool::ToolDef;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// 后台任务事件。
pub enum JobEvent {
    Started {
        blocks: u32,
    },
    /// 进度（墙钟节流：50 ms 一条，块 0 必发）
    Progress(Progress),
    /// 跑完或取消——看 `output.stats.cancelled`
    Done(Box<RunOutput>),
    /// 工作线程 panic（正常路径不会出现；出现就是 bug，不能静默）
    Failed(String),
}

/// 手工 Debug：`RunOutput` 里躺着整个毛坯网格，不该为打日志去 derive。
impl std::fmt::Debug for JobEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            JobEvent::Started { blocks } => write!(f, "Started {{ blocks: {blocks} }}"),
            JobEvent::Progress(p) => write!(
                f,
                "Progress(block {}/{}, t={:.3}s)",
                p.block, p.blocks_total, p.sim_time
            ),
            JobEvent::Done(o) => write!(
                f,
                "Done(cancelled={}, blocks={}, cut_segments={})",
                o.stats.cancelled, o.stats.blocks, o.stats.cut_segments
            ),
            JobEvent::Failed(e) => write!(f, "Failed({e:?})"),
        }
    }
}

/// 后台仿真任务的句柄。**Drop 会取消并 join**——丢掉句柄等于取消。
pub struct SimJob {
    rx: Receiver<JobEvent>,
    cancel: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

/// 进度节流间隔。50 ms 对 UI 足够（20 fps 的进度条），对 100k 块的程序
/// 也就是几千条消息。
const PROGRESS_THROTTLE: Duration = Duration::from_millis(50);

impl SimJob {
    /// 起一个工作线程跑整段程序。`program`/`tool`/`cfg` 按值移动进去——
    /// 调用方之后随便改自己的副本，不会影响正在跑的任务。
    pub fn spawn(program: MotionProgram, tool: ToolDef, cfg: RunConfig) -> Self {
        let (tx, rx) = mpsc::channel();
        let cancel = Arc::new(AtomicBool::new(false));
        let cancel_worker = Arc::clone(&cancel);

        let worker = move || {
            let blocks = program.len() as u32;
            if tx.send(JobEvent::Started { blocks }).is_err() {
                return; // 接收端已丢，没人要结果了
            }
            let mut last_emit = Instant::now()
                .checked_sub(PROGRESS_THROTTLE) // 保证块 0 必发
                .unwrap_or_else(Instant::now);

            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_observed(&program, &tool, &cfg, |p| {
                    if cancel_worker.load(Ordering::Relaxed) {
                        return false;
                    }
                    let now = Instant::now();
                    if now.duration_since(last_emit) >= PROGRESS_THROTTLE {
                        last_emit = now;
                        // 发送失败说明接收端没了——继续算也没人看，早早收场
                        if tx.send(JobEvent::Progress(*p)).is_err() {
                            return false;
                        }
                    }
                    true
                })
            }));

            match result {
                Ok(out) => {
                    let _ = tx.send(JobEvent::Done(Box::new(out)));
                }
                Err(payload) => {
                    let msg = panic_message(&payload);
                    let _ = tx.send(JobEvent::Failed(msg));
                }
            }
        };

        match std::thread::Builder::new()
            .name("cncsim-sim-job".into())
            .spawn(worker)
        {
            Ok(handle) => Self { rx, cancel, handle: Some(handle) },
            Err(e) => {
                // 起线程失败（资源耗尽）：交出一个只会报 Failed 的任务
                let (tx2, rx2) = mpsc::channel();
                let _ = tx2.send(JobEvent::Failed(format!("无法启动工作线程：{e}")));
                Self { rx: rx2, cancel, handle: None }
            }
        }
    }

    /// 取一条事件；没有则立即返回 `None`（不阻塞）。
    pub fn try_recv(&self) -> Option<JobEvent> {
        match self.rx.try_recv() {
            Ok(e) => Some(e),
            Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => None,
        }
    }

    /// 等到一条事件（最长 `timeout`）。
    pub fn recv_timeout(&self, timeout: Duration) -> Option<JobEvent> {
        self.rx.recv_timeout(timeout).ok()
    }

    /// 请求取消。**块边界生效**——半切的块不清洗，那是该块当前的诚实状态。
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    pub fn is_cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }
}

impl Drop for SimJob {
    fn drop(&mut self) {
        // 丢掉句柄 = 取消 + 等线程收尾。绝不让后台线程悬空。
        self.cancel.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn panic_message(payload: &Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        format!("工作线程 panic：{s}")
    } else if let Some(s) = payload.downcast_ref::<String>() {
        format!("工作线程 panic：{s}")
    } else {
        "工作线程 panic（消息不可读）".into()
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
            stock_size: [200, 200, 100],
            ..Default::default()
        }
    }

    fn prog(src: &str) -> MotionProgram {
        interpret(src, &InterpConfig::default())
    }

    /// 十刀横穿毛坯——块够多，取消能落在块边界上
    fn many_blocks() -> MotionProgram {
        let mut src = String::from("G0 X2 Y5 Z9.5\n");
        for i in 0..10 {
            let y = 5.0 + i as f64 * 1.5;
            src.push_str(&format!("G1 X190 Y{y:.1} F600\nG1 X2\n"));
        }
        src.push_str("M30\n");
        prog(&src)
    }

    fn drain_to_done(job: &SimJob) -> Box<RunOutput> {
        loop {
            match job.recv_timeout(Duration::from_secs(30)).expect("任务应在 30 s 内结束") {
                JobEvent::Done(out) => return out,
                JobEvent::Failed(e) => panic!("工作线程失败：{e}"),
                JobEvent::Started { .. } | JobEvent::Progress(_) => {}
            }
        }
    }

    #[test]
    fn async_result_is_identical_to_sync_run() {
        let p = prog("G0 X2 Y10 Z9.5\nG1 X8 F300\nG2 X8 Y14 I0 J2\nG82 X12 Y10 Z6 R9 P0.2 F100\nG80\nM30\n");
        let sync = run(&p, &tool(), &cfg());
        let job = SimJob::spawn(p.clone(), tool(), cfg());
        let asy = drain_to_done(&job);

        // 墙钟字段之外逐位一致
        assert_eq!(asy.report.programmed_time_s, sync.report.programmed_time_s);
        assert_eq!(asy.report.removed_volume_mm3, sync.report.removed_volume_mm3);
        assert_eq!(asy.report.gouge_volume_mm3, sync.report.gouge_volume_mm3);
        assert_eq!(asy.stats.cut_segments, sync.stats.cut_segments);
        assert_eq!(asy.stats.volume_after, sync.stats.volume_after);
        assert_eq!(asy.collisions.len(), sync.collisions.len());
        let ids = |r: &crate::SimReport| r.alarms.iter().map(|a| a.id).collect::<Vec<_>>();
        assert_eq!(ids(&asy.report), ids(&sync.report));
        assert!(!asy.stats.cancelled);
    }

    #[test]
    fn events_start_with_started_and_end_with_done() {
        let mut events = Vec::new();
        let job = SimJob::spawn(prog("G0 X2 Y10 Z9.5\nG1 X18 F300\nM30\n"), tool(), cfg());
        loop {
            let e = job.recv_timeout(Duration::from_secs(30)).expect("应有事件");
            let tag = match &e {
                JobEvent::Started { .. } => "started",
                JobEvent::Progress(_) => "progress",
                JobEvent::Done(_) => "done",
                JobEvent::Failed(_) => "failed",
            };
            events.push(tag.to_string());
            if matches!(e, JobEvent::Done(_) | JobEvent::Failed(_)) {
                break;
            }
        }
        assert_eq!(events.first().unwrap(), "started");
        assert_eq!(events.last().unwrap(), "done");
        assert!(events.len() >= 3, "至少 Started + 一条 Progress + Done：{events:?}");
    }

    #[test]
    fn cancel_mid_way_returns_partial_output() {
        let job = SimJob::spawn(many_blocks(), tool(), cfg());
        // 等到它真正开跑再取消，确保不是"取消太快什么都没干"
        job.recv_timeout(Duration::from_secs(10)).expect("应有 Started");
        job.cancel();
        let out = drain_to_done(&job);
        assert!(out.stats.cancelled, "应报告为已取消");
        assert!(
            out.report.alarms.iter().any(|a| a.id == "CANCELLED"),
            "取消要有 CANCELLED 报警"
        );
        // 部分结果：块的子集
        let full = run(&many_blocks(), &tool(), &cfg());
        assert!(out.stats.volume_after >= full.stats.volume_after, "取消时材料更多");
    }

    #[test]
    fn drop_cancels_and_joins_without_hanging() {
        let job = SimJob::spawn(many_blocks(), tool(), cfg());
        job.recv_timeout(Duration::from_secs(10)).expect("应有 Started");
        drop(job); // Drop 内部 cancel + join；挂住就说明取消失效
    }
}
