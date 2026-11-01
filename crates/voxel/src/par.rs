//! 分块并行：**常驻 rayon 池 + 按块号归位**。
//!
//! ## 为什么要一个模块而不是到处 `par_iter`
//!
//! 三件事必须由一处说了算，散在调用点上会各自漂移：
//!
//! 1. **线程数**：`CNCSIM_THREADS` 覆盖，缺省用 `available_parallelism`。
//!    池是进程级单例，第一次用到才建。
//! 2. **确定性**：结果按**块号**摊平，与谁先算完无关。项目硬门槛（ADR-006 §7.1 坑三）。
//! 3. **门槛**：小活别进池。抢一次任务才几 µs，但块太碎时每块的固定开销
//!    （分配结果 `Vec`、合并拷贝）会超过省下的时间。
//!
//! ## 为什么最终接了 rayon（ADR-009 §1.1）
//!
//! 第一版用 `std::thread::scope` 手写 fork-join：每次 `spawn + join` 24 个线程实测 **491 µs**。
//! 真实段长的一次切削才 7.4 ms，光线程开销就占 7%——10 万段就是 50 秒。
//! rayon 的常驻池把这个开销压到 **~5 µs**，这是引它**唯一**的实质理由；
//! `par_iter` 的语法糖是顺带的。
//!
//! 代价是 6 个传递包（rayon / rayon-core / crossbeam-deque / crossbeam-epoch /
//! crossbeam-utils / either），已在 `tools/check_deps.py` 的 `EXTERNAL` 白名单登记。
//!
//! ## 仍然不共享
//!
//! 池只负责"分配 CPU"，数据面的规矩没变（《并发与性能》§2、ADR-009 §2）：
//! **只读并行、按块号归位**，`SparseGrid` 始终单一所有者。rayon 的 work-stealing
//! 只在块之间搬任务，搬不动任何共享状态。

use rayon::prelude::*;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::sync::OnceLock;

/// 低于这个工作量就别进池。一次池往返约 5 µs（实测），但块太碎时
/// 每块的固定开销（结果 `Vec` 的分配与合并）会吃掉收益。
pub(crate) const MIN_PAR_WORK: usize = 20_000;

static POOL: OnceLock<ThreadPool> = OnceLock::new();

fn thread_count() -> usize {
    if let Ok(s) = std::env::var("CNCSIM_THREADS")
        && let Ok(n) = s.trim().parse::<usize>()
    {
        return n.clamp(1, 256);
    }
    std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1).clamp(1, 64)
}

/// 进程级线程池。`CNCSIM_THREADS=1` 建的是单线程池（等价串行，用于对拍与定位）。
pub fn pool() -> &'static ThreadPool {
    POOL.get_or_init(|| {
        ThreadPoolBuilder::new()
            .num_threads(thread_count())
            .thread_name(|i| format!("cncsim-{i}"))
            .build()
            .expect("构建 rayon 线程池失败")
    })
}

/// 工作线程数（= 池的线程数，不是机器核数）。
pub fn worker_count() -> usize {
    pool().current_num_threads()
}

/// 分块并行：`f(i)` 处理第 `i` 块（`0..n`），返回的 `Vec` **按块号排序**。
///
/// `(0..n).into_par_iter()` 是 `IndexedParallelIterator`，rayon 保证 `collect`
/// 按索引顺序落位——所以**块完成的先后不影响输出**：换线程数、换调度、换机器，
/// 结果逐位相同（`tests/parallel.rs` 拿分块数当旋钮对拍这条）。
pub fn map_indexed<T, F>(n: usize, f: F) -> Vec<T>
where
    T: Send,
    F: Fn(usize) -> T + Sync,
{
    if n == 0 {
        return Vec::new();
    }
    // 单块没有并行的意义；单线程池直接串行，省一次池往返
    if n == 1 || worker_count() <= 1 {
        return (0..n).map(&f).collect();
    }
    pool().install(|| (0..n).into_par_iter().map(&f).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn results_come_back_in_chunk_order() {
        // 故意让靠前的块慢：work-stealing 会乱序完成，但结果必须按块号排
        let out = map_indexed(64, |i| {
            if i % 8 == 0 {
                std::thread::yield_now();
            }
            i * i
        });
        assert_eq!(out, (0..64).map(|i| i * i).collect::<Vec<_>>());
    }

    #[test]
    fn single_chunk_needs_no_pool_dispatch() {
        let out = map_indexed(1, |i| i + 7);
        assert_eq!(out, vec![7]);
        assert!(map_indexed(0, |_| 0u8).is_empty());
    }

    #[test]
    fn pool_is_a_singleton_with_a_stable_size() {
        let n = worker_count();
        assert!(n >= 1);
        assert_eq!(n, pool().current_num_threads());
    }
}
