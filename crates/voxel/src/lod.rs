//! # 显示 LOD：粗网格重采样
//!
//! 两条路都能把 3200 万面降到 200 万面，选的是**重新采样**，不是网格抽稀：
//!
//! | | 网格抽稀（QEM 等） | 粗格重采样（本模块） |
//! |---|---|---|
//! | 输入 | 3200 万三角形、834 MB，先得全部造出来 | 稀疏树（2 MB） |
//! | 水密性 | 抽稀会开洞，要靠额外约束守 | **Surface Nets 结构保证** |
//! | 可复现 | 依赖误差度量与堆序 | **纯函数，逐位可复现** |
//! | 代码量 | 上千行 + 一堆启发式 | 复用 `extract` 的顶点/四边形逻辑 |
//!
//! ## 一条硬约束：`k ≤ band_voxels`
//!
//! 粗格边长 `h_c = k·h`。**跨符号的棱，两个端点各在表面两侧**，
//! 所以各自到表面的距离 `≤ h_c`；而窄带只有 `band_voxels·h` 宽。
//! 于是：
//!
//! - `k ≤ band_voxels` → 两个端点都落在窄带内 → 存储值是**真实距离**，
//!   棱上零点插值准确（误差只剩 `O(h_c²/R)` 的曲率项）。
//! - `k > band_voxels` → 远端被钳制到 `±band`，插值**不再等于真实距离**，
//!   偏差随 `(h_c − band)²` 增长。实测 R=6 球：k=4 **9.8 µm**、k=5 **15.5 µm**、
//!   k=6 **34.2 µm**——不是断崖式崩，但**没有上界**，而且悄悄发生。
//!
//! 所以 `max_lod_factor()` 直接返回 `band_voxels`，`extract_surface_lod` 超限即 panic。
//! 默认 band=4 体素、h=0.1 → `h_c ≤ 0.4 mm`，正好是 200 万面的预算点，不是巧合。
//! **要更少的面只能加宽 band（内存换），不能加大 k。**
//!
//! ## 顶点的落在哪儿
//!
//! 粗格的格点**与细体素中心重合**（格点 `p` 对应细体素 `k·p`），所以
//! `sdf()` 在格点上退化为直接取值（三线性权重退化成一个 1），无插值误差。
//! 顶点位置 = 12 条棱零点的平均，与细格同一套 `vertex_of`。
//!
//! ## 只处理表面附近的粗格
//!
//! 200 mm 毛坯 @ `h_c=0.4` 有 1.25 亿个粗格，全扫一遍要十几秒。
//! 但**表面穿过某格 ⇔ 该格 8 个角点里出现混号**，而混号一定牵连到
//! 某片"可能被表面穿过"的叶。所以候选格从叶反推：一片叶最多命中
//! `(8/k + 2)³` 个粗格（k=4 时 27 个）。200 mm 毛坯实测：候选格 **297 万**、
//! 真正出顶点的 100 万（3 倍余量），比 1.25 亿少 **42 倍**。
//!
//! **判 hot 不能只看 `|v|`**：`fill_box` 填出的毛坯是二值场（±MAX），
//! 表面两侧全是饱和值、窄带空空如也。第一版就栽在这里——60 mm 方坯
//! k=1 提取出 **0 个顶点**。现在三档判据（叶内混号 / 叶内有窄带体素 /
//! 26 邻里有反号叶）都写在 [`SparseGrid::lod_candidates`] 上。

use crate::cellmap::{CellMap, VertexRecord};
use crate::extract::{CORNER, PAR_MIN_LEAVES, split_quads, split_quads_in, vertex_of};
use crate::fxhash::{FxHashMap, FxHashSet};
use crate::tree::{LEAF_DIM, Leaf, Scalar, SparseGrid};
use crate::MeshData;
use cncsim_math::DVec3;

/// 按三角形预算选 LOD 因子。
///
/// 三角形数 ≈ `A/h_c² × 2`，与 `k²` 成反比，所以 `k = ⌈√(T_fine / budget)⌉`。
/// 结果被夹到 `[1, max_factor]`——上限由窄带宽度决定（见模块文档）。
/// 夹到上限仍超预算时**不静默接受**：调用方应提示"这台机床的毛坯太大，
/// 得减小 band 或接受更多面"，而不是给出一个错的网格。
pub fn choose_lod_factor(fine_triangles: usize, budget: usize, max_factor: u32) -> u32 {
    let max_factor = max_factor.max(1);
    if budget == 0 || fine_triangles <= budget {
        return 1;
    }
    let k = (fine_triangles as f64 / budget as f64).sqrt().ceil() as u32;
    k.clamp(1, max_factor)
}

/// 叶内符号统计的位标志。
const F_NEG: u8 = 1; // 有材料内的体素（v < 0）
const F_POS: u8 = 2; // 有材料外的体素（v >= 0）
const F_NEAR: u8 = 4; // 有窄带体素（|v| < MAX_MAG）

/// 一片叶的符号统计。
///
/// **不能只看"有没有窄带体素"**：`fill_box` 填出来的毛坯是二值场（±MAX），
/// 表面两侧全是饱和值，窄带是空的。只按 `|v|` 判会漏掉整张毛坯的外表面
/// （实测：60 mm 方坯 k=1 提取出 0 个顶点）。
fn leaf_flags<S: Scalar>(leaf: &Leaf<S>) -> u8 {
    // **一次 min/max 归约，再由极值推三个标志**，不要逐值跑带分支的闭包。
    //
    // 原来每个值判两次（`v < 0` 和 `|v| < MAX_MAG`），512 个值就是 1024 条分支；
    // 一片 Dense 叶 0.47 µs，10305 片叶合计 **4.8 ms**——占 LOD 提取的 21%。
    // min/max 归约是两条无分支的比较链，编译器能向量化成 64 字节一组。
    //
    // 由极值反推是精确的（不是近似）：
    //   · 有负数 ⟺ min < 0
    //   · 有非负数 ⟺ max ≥ 0
    //   · 存在 |v| < MAX_MAG ⟺ 并非全部饱和在 −MAX_MAG 或 +MAX_MAG
    //     ⟺ min < MAX_MAG 且 max > −MAX_MAG
    let (mn, mx) = match leaf {
        Leaf::Constant(v) => (*v, *v),
        Leaf::Dense(a) => a.iter().fold((S::MAX, S::MIN), |(lo, hi), &v| {
            (if v < lo { v } else { lo }, if v > hi { v } else { hi })
        }),
    };
    let mut f = 0u8;
    if mn < S::default() {
        f |= F_NEG;
    }
    if mx >= S::default() {
        f |= F_POS;
    }
    let mm = S::from_f64_round(S::MAX_MAG);
    let nmm = S::from_f64_round(-S::MAX_MAG);
    if mn < mm && mx > nmm {
        f |= F_NEAR;
    }
    f
}

/// 全同号的叶，其 26 邻里是否有反号叶。
///
/// 未分配的邻居按背景处理（`+MAX`，材料外）——毛坯外表面正是这么发现的。
/// 整块 tile 覆盖的邻居不在这张表里，但那种区域是饱和的内部或外部，
/// 反号边界一定落在叶上（`fill_box` 的边界层刻意不做 tile），所以不漏。
fn neighbor_has_opposite_sign(
    lc: [i32; 3],
    f: u8,
    map: &FxHashMap<[i32; 3], u8>,
) -> bool {
    let self_neg = f & F_NEG != 0;
    for dz in -1..=1 {
        for dy in -1..=1 {
            for dx in -1..=1 {
                if dx == 0 && dy == 0 && dz == 0 {
                    continue;
                }
                let nf = map
                    .get(&[lc[0] + dx, lc[1] + dy, lc[2] + dz])
                    .copied()
                    .unwrap_or(F_POS);
                let differs = if self_neg { nf & F_POS != 0 } else { nf & F_NEG != 0 };
                if differs {
                    return true;
                }
            }
        }
    }
    false
}

impl<S: Scalar> SparseGrid<S> {
    /// 可用 LOD 因子的上限 = 窄带宽度（体素数）。
    ///
    /// 不是调参，是几何约束：粗格边长超过窄带宽度后，跨符号棱的远端会被钳制到
    /// `±band`，插值不再等于真实距离，偏差随 `(h_c − band)²` 增长。见模块文档。
    pub fn max_lod_factor(&self) -> u32 {
        self.config.band_voxels.max(1) as u32
    }

    /// 粗格 `cell` 最小角的世界坐标（与细体素 `k·cell` 的中心重合）
    #[inline]
    fn lod_corner(&self, cell: [i32; 3], k: i32) -> DVec3 {
        self.voxel_center([cell[0] * k, cell[1] * k, cell[2] * k])
    }

    /// 候选粗格：**只包含可能有表面穿过的格**。
    ///
    /// 判据分三档（任何一条成立就算"表面可能穿过这片叶"）：
    ///
    /// | 判据 | 抓什么 | 漏了会怎样 |
    /// |---|---|---|
    /// | 叶内混号 `NEG∧POS` | 窄带正常、表面就在叶内 | 网格中部开洞 |
    /// | 叶内有窄带体素 | 弱表面：一侧全饱和，另一侧才进带内 | 缓坡处丢面 |
    /// | 26 邻里有反号叶 | **二值场**（`fill_box` 的毛坯、切空的区域） | 毛坯外表面整张丢失 |
    ///
    /// 未分配的邻居按背景处理（`+MAX`，即材料外），所以毛坯边界叶天然被判为 hot。
    ///
    /// 候选格取"8 个角点里有任意一个落在这片叶内"的格：
    /// `p ∈ [⌊(lo−k)/k⌋, ⌊hi/k⌋]`（`div_euclid` 的 floor 语义，负坐标才和树索引一致）。
    /// 只生成落在 `region`（粗格坐标，闭区间）里的候选格。
    ///
    /// **叶表还是要走一遍**：邻接判断（`neighbor_has_opposite_sign`）需要整棵树，
    /// 只取区域内那片叶会让交界处的候选格算错，表现是缝。所以省下的是
    /// "每片热叶展开 27 个格"那部分，不是走树那部分。
    fn lod_candidates_in(&self, k: i32, region: Option<([i32; 3], [i32; 3])>) -> Vec<[i32; 3]> {
        let prof = crate::cut::prof_enabled();
        let t0 = std::time::Instant::now();
        let mut leaves: Vec<([i32; 3], u8)> = Vec::with_capacity(self.leaf_count());
        match region {
            // 分块重建：只走与区域相交的叶（三层剪枝）。不剪的话每刀都要付
            // 全树走查 + 全叶 `leaf_flags`，实测比全量重提还慢。
            Some((rlo, rhi)) => {
                // 区域是**格坐标**，叶是**体素坐标**：乘 k 再各外扩一格
                let vlo = [rlo[0] * k - k, rlo[1] * k - k, rlo[2] * k - k];
                let vhi = [rhi[0] * k + k, rhi[1] * k + k, rhi[2] * k + k];
                self.for_each_leaf_in(vlo, vhi, |lc, leaf| leaves.push((lc, leaf_flags(leaf))));
            }
            None => self.for_each_leaf(|lc, leaf| leaves.push((lc, leaf_flags(leaf)))),
        }
        let t_walk = t0.elapsed().as_secs_f64() * 1e3;
        let t1 = std::time::Instant::now();
        // for_each_leaf 走 HashMap，顺序随机 → 排序保证候选集与顶点顺序可复现
        leaves.sort_unstable_by_key(|e| e.0);
        let t_sortleaf = t1.elapsed().as_secs_f64() * 1e3;
        let t2 = std::time::Instant::now();
        let map: FxHashMap<[i32; 3], u8> = leaves.iter().copied().collect();
        let t_map = t2.elapsed().as_secs_f64() * 1e3;

        // 按叶分块并行生成（每块一个本地集合），再整体 sort+dedup。
        //
        // 为什么必须并行：k=4 时要插入 675 万次、留下 297 万个候选格，
        // 串行做这一项就占掉 LOD 总时间的一半（k=1 更极端：每片叶 729 个格）。
        // 最后统一排序既去掉块内/块间的重复，也保证**顺序与线程调度无关**。
        let threads = crate::par::worker_count();
        let chunks = if leaves.len() < PAR_MIN_LEAVES || threads == 1 {
            1
        } else {
            (threads * 4).min(leaves.len().max(1))
        };
        let per = leaves.len().div_ceil(chunks).max(1);
        let parts = crate::par::map_indexed(chunks, |i| {
            let s = i * per;
            let e = (s + per).min(leaves.len());
            // `chunks * per` 可能超过叶数（向上取整），末尾的块是空的——必须先挡掉，
            // 否则 `&leaves[s..e]` 会越界 panic（s 可以大于 len）
            if s >= e {
                return Vec::new();
            }
            // **块内必须先去重**：相邻叶的格块互相重叠，不去重的话原始格会从
            // 41.9 万涨到 81.9 万（实测），排序随之从 5.7 ms 涨到 18.4 ms。
            // 花 2.7 ms 的哈希插入省下 12 ms 的排序，很划算。
            let mut set: FxHashSet<[i32; 3]> = FxHashSet::default();
            for &(lc, f) in &leaves[s..e] {
                if let Some((rlo, rhi)) = region {
                    // 这片叶可能覆盖的粗格范围与区域不相交 → 整片跳过
                    let lo = [lc[0] * LEAF_DIM, lc[1] * LEAF_DIM, lc[2] * LEAF_DIM];
                    let hi = [lo[0] + LEAF_DIM, lo[1] + LEAF_DIM, lo[2] + LEAF_DIM];
                    let outside = (0..3).any(|a| {
                        (hi[a] - k).div_euclid(k) < rlo[a] || lo[a].div_euclid(k) > rhi[a]
                    });
                    if outside {
                        continue;
                    }
                }
                let mixed = (f & F_NEG != 0) && (f & F_POS != 0);
                let near = f & F_NEAR != 0;
                let facing = !mixed && !near && neighbor_has_opposite_sign(lc, f, &map);
                if !(mixed || near || facing) {
                    continue;
                }
                let lo = [lc[0] * LEAF_DIM, lc[1] * LEAF_DIM, lc[2] * LEAF_DIM];
                let hi = [lo[0] + LEAF_DIM - 1, lo[1] + LEAF_DIM - 1, lo[2] + LEAF_DIM - 1];
                for cz in (lo[2] - k).div_euclid(k)..=hi[2].div_euclid(k) {
                    for cy in (lo[1] - k).div_euclid(k)..=hi[1].div_euclid(k) {
                        for cx in (lo[0] - k).div_euclid(k)..=hi[0].div_euclid(k) {
                            set.insert([cx, cy, cz]);
                        }
                    }
                }
            }
            set.into_iter().collect::<Vec<[i32; 3]>>()
        });
        let t_par = t2.elapsed().as_secs_f64() * 1e3 - t_map;
        let t3 = std::time::Instant::now();
        let raw: usize = parts.iter().map(|p| p.len()).sum();

        // ── 排序去重：把 `[i32; 3]` 打包成**单个 u64 键**再排 ──
        //
        // 原来直接 `sort_unstable()` 排 `[i32; 3]`，比较走 `core::slice::cmp::
        // chaining_impl`（3 次 i32 比较 + 短路）。实测 41.9 万个格要 **14 ms**，
        // 占整个 LOD 提取的 40%。
        //
        // 打包之后每次比较只有一条 u64 指令。**顺序必须与原字典序逐位相同**
        // （顶点顺序决定网格逐位可复现）：坐标先减下界、各占 `bits` 位、
        // 按 (x,y,z) 从高位到低位拼，u64 的无符号序就等价于 (x,y,z) 字典序。
        // 位宽放不下（`3·bits > 64`）时退回原来的路径——那种网格现实里不存在，
        // 但不能让它静默出错。
        // **基必须用格坐标的真实最小值，不能用网格下界**：`(lo − k).div_euclid(k)`
        // 在 lo = 0 时会给出 −1，减一个非负的 `blo` 之后仍为负，`as u64` 一溢出
        // 就会和大的正坐标撞在一起——实测顶点数从 34802 掉到 17164，是这么发现的。
        let mut mn = [i32::MAX; 3];
        let mut mx = [i32::MIN; 3];
        for part in &parts {
            for c in part {
                for a in 0..3 {
                    mn[a] = mn[a].min(c[a]);
                    mx[a] = mx[a].max(c[a]);
                }
            }
        }
        let span = (0..3)
            .map(|a| (mx[a] as i64 - mn[a] as i64 + 1).max(1) as u64)
            .max()
            .unwrap_or(1);
        let bits = 64 - (span - 1).leading_zeros();
        // 键宽按实际坐标跨度选；放不下再退原字典序路径。
        let cells: Vec<[i32; 3]> = if bits * 3 <= 64 {
            let mut keys: Vec<u64> = Vec::with_capacity(raw);
            for part in &parts {
                for c in part {
                    keys.push(
                        (((c[0] - mn[0]) as u64) << (2 * bits))
                            | (((c[1] - mn[1]) as u64) << bits)
                            | ((c[2] - mn[2]) as u64),
                    );
                }
            }
            // **rayon 自带的并行排序**，跑在项目自己的池里——不能用全局池，
            // 否则 `CNCSIM_THREADS=1` 的串行对拍语义会在这里被悄悄绕过。
            //
            // 结果是确定的：排序的输出只由"键的多重集"决定，`par_sort_unstable`
            // 与 `sort_unstable` 给出同一串有序键（相同的键本来就不可区分），
            // 所以去重之后的格序列逐位一致。
            //
            // 试过但更慢/更啰嗦的：LSD 基数排序（3 趟全量散射 5.9 ms）、
            // 手写并行归并（最后一轮归并是 O(n) 串行，3.7–5.1 ms，多 50 行）。
            crate::par::pool().install(|| {
                use rayon::slice::ParallelSliceMut;
                keys.par_sort_unstable();
            });
            keys.dedup();
            let m = (1u64 << bits) - 1;
            keys.into_iter()
                .map(|k| {
                    [
                        ((k >> (2 * bits)) & m) as i32 + mn[0],
                        ((k >> bits) & m) as i32 + mn[1],
                        (k & m) as i32 + mn[2],
                    ]
                })
                .collect()
        } else {
            let mut c: Vec<[i32; 3]> = parts.into_iter().flatten().collect();
            c.sort_unstable();
            c.dedup();
            c
        };
        let t_flat = t3.elapsed().as_secs_f64() * 1e3;
        if prof {
            println!(
                "  [cand] 走树 {t_walk:.1} | 排叶 {t_sortleaf:.1} | 建map {t_map:.1} | 并行插 {t_par:.1} | 打包+排序+去重 {t_flat:.1} ms；叶 {} / 原始格 {raw} / 去重后 {} / 键宽 {bits}×3 位",
                leaves.len(),
                cells.len()
            );
        }
        cells
    }

    /// 在粗网格（边长 `k·h`）上重采样并提取等值面，返回世界坐标网格。
    ///
    /// `k = 1` 的结果与 [`Self::extract_surface`] 逐位一致（有测试锁）。
    /// `k > max_lod_factor()` 会 panic——那种网格的顶点位置是错的，不该产出。
    pub fn extract_surface_lod(&self, k: u32) -> MeshData {
        self.extract_surface_lod_chunks(k, 0)
    }

    /// `chunks = 0` 表示按线程数自动分块；大于 0 时强制分块（对拍用）。
    /// 只提取粗格区间 `[lo, hi]` 内的表面。
    ///
    /// 分块重建用的：切削只影响它扫过的那几块，其余块的网格该原样留着。
    ///
    /// **三角形的归属按"发射它的格"划**，不按质心：每个四边形由最小角所在的格
    /// 发射一次，所以只要让每块只发射 `[lo, hi]` 内的格，各块的三角形天然不重不漏。
    ///
    /// 顶点要多取一圈（`lo − 1`）：四边形会连到相邻格上的顶点，少一圈就在块的
    /// 交界处撕出缝来。
    pub fn extract_surface_lod_in(&self, k: u32, lo: [i32; 3], hi: [i32; 3]) -> MeshData {
        self.extract_lod_impl(k, 0, Some((lo, hi)))
    }

    pub fn extract_surface_lod_chunks(&self, k: u32, chunks: usize) -> MeshData {
        self.extract_lod_impl(k, chunks, None)
    }

    fn extract_lod_impl(
        &self,
        k: u32,
        chunks: usize,
        region: Option<([i32; 3], [i32; 3])>,
    ) -> MeshData {
        assert!(k >= 1, "LOD 因子至少为 1");
        assert!(
            k <= self.max_lod_factor(),
            "LOD 因子 {k} 超过上限 {}（= 窄带宽度 band_voxels）：粗格边长会超过窄带宽度，\
             跨符号棱的远端被钳制到 ±band，插值不再等于真实距离（实测 R=6 球 k=6 偏 34 µm）",
            self.max_lod_factor()
        );
        let k = k as i32;
        let hc = self.config.h * k as f64;

        // 顶点多取一圈：四边形会连到相邻格上的顶点
        let cell_region = region.map(|(a, b)| {
            (
                [a[0] - 1, a[1] - 1, a[2] - 1],
                b,
            )
        });
        let prof = crate::cut::prof_enabled();
        let t_all = std::time::Instant::now();
        let t0 = std::time::Instant::now();
        let mut cells = self.lod_candidates_in(k, cell_region);
        if let Some((rlo, rhi)) = region {
            let vlo = [rlo[0] - 1, rlo[1] - 1, rlo[2] - 1];
            cells.retain(|p| (0..3).all(|a| p[a] >= vlo[a] && p[a] <= rhi[a]));
        }

        let t_cand = t0.elapsed().as_secs_f64() * 1e3;
        let t0 = std::time::Instant::now();

        // ── ① 顶点：候选格分块并行，块内按格点缓存角点值 ──
        //
        // 角点缓存是必需的：一个格点最多被 8 个粗格共用，不缓存等于把 `sdf()`
        // 重复算 8 次（实测 k=2 慢 20%）。缓存**按块**：跨块重复只发生在块边界，
        // 代价是几十个点，换来块间零共享。
        let threads = crate::par::worker_count();
        let chunks = if chunks > 0 {
            chunks.clamp(1, cells.len().max(1))
        } else if cells.len() < PAR_MIN_LEAVES || threads == 1 {
            1
        } else {
            (threads * 4).min(cells.len().max(1))
        };
        let per = cells.len().div_ceil(chunks).max(1);
        let grid: &Self = self;
        let parts = crate::par::map_indexed(chunks, |i| {
            let s = i * per;
            let e = (s + per).min(cells.len());
            if s >= e {
                return Vec::new();
            }
            grid.lod_vertex_batch(&cells[s..e], k, hc)
        });

        let t_vert = t0.elapsed().as_secs_f64() * 1e3;
        let t0 = std::time::Instant::now();

        // 与细格共用同一个索引构建器：**k=1 与细格逐位一致**靠的就是这一条
        let (mut mesh, cellmap) = CellMap::build(&parts);
        let t_cellmap = t0.elapsed().as_secs_f64() * 1e3;
        let t0 = std::time::Instant::now();

        // ── ② 四边形 ──
        //
        // 符号来自格点的 `sdf()`：窄带外虽然被钳制到 ±band，但**符号是对的**，
        // 而这一步只需要符号。这里不再共享 ① 的缓存（跨块就散了），
        // 每格最多问 4 次 `sdf()`——比维护一张全局表便宜。
        let inside = |p: [i32; 3]| self.sdf(self.lod_corner(p, k)) < 0.0;
        let quads = match region {
            Some((rlo, rhi)) => split_quads_in(&cellmap, &inside, threads, |p| {
                (0..3).all(|a| p[a] >= rlo[a] && p[a] <= rhi[a])
            }),
            None => split_quads(&cellmap, &inside, threads),
        };
        mesh.indices = quads.indices;
        mesh.skipped_quads = quads.skipped;

        if prof {
            let t_quad = t0.elapsed().as_secs_f64() * 1e3;
            println!(
                "  [lod k={k}] 候选 {t_cand:.1} | 顶点 {t_vert:.1} | CellMap {t_cellmap:.1} | 四边形 {t_quad:.1} | 合计 {:.1} ms（{} 格 / {} 顶点 / {} 块）",
                t_all.elapsed().as_secs_f64() * 1e3,
                cells.len(),
                mesh.positions.len(),
                chunks
            );
        }
        mesh
    }

    /// 处理 `cells[..]` 这一批粗格，产出顶点（按格序）。
    fn lod_vertex_batch(&self, cells: &[[i32; 3]], k: i32, hc: f64) -> Vec<VertexRecord> {
        let mut out: Vec<VertexRecord> = Vec::new();
        let mut corner_value: FxHashMap<[i32; 3], f64> = FxHashMap::default();
        // 根游标：一片叶 8³ 体素、root 是 64³，块内几乎不换 root
        let mut rc = crate::tree::RootCursor::default();
        // 取色游标：相邻粗格几乎总在同一片叶里，省掉每顶点的叶查找
        let mut cur = crate::tree::CreatorCursor::default();
        for &p in cells {
            let mut corners = [0.0f64; 8];
            for (i, o) in CORNER.iter().enumerate() {
                let g = [p[0] + o[0], p[1] + o[1], p[2] + o[2]];
                corners[i] = *corner_value
                    .entry(g)
                    .or_insert_with(|| self.sdf_cached(self.lod_corner(g, k), &mut rc));
            }
            if let Some((pos, normal)) = vertex_of(&corners, self.lod_corner(p, k), hc) {
                // **别改成"用细步长重算梯度"。**
                //
                // 试过：`gradient_at(pos)` 用 0.8h 的中心差分取 `sdf_cached` 的
                // 梯度，本意是让法线比粗格的 0.4 mm 差分更准。结果是**顶点数从
                // 14.5 万涨到 86.8 万、HTML 从 10 MB 涨到 37 MB**：`sdf` 是
                // 三线性插值的体素场，在格面处梯度不连续，细差分会把每个格面
                // 的折痕都放大成 >45° 的法线跳变，`split_crease_normals(0.707)`
                // 于是把它们全当成硬边拆开。
                //
                // 也就是说：粗步长差分在这里**恰好起了一个低通滤波的作用**，
                // 它把体素场的格面噪声抹平了。要更光滑只能提高网格精度
                // （k 更小）或换解析曲面，不能靠缩小差分步长。
                out.push(VertexRecord {
                    cell: p,
                    pos: [pos.x as f32, pos.y as f32, pos.z as f32],
                    normal: [normal.x as f32, normal.y as f32, normal.z as f32],
                    // **按顶点位置取色**，不是按格的最小角。
                    //
                    // 用最小角时，落在格子"远端"的面会取错：格边长 `k·h`（k=3 是
                    // 0.3 mm），而最小角到那个面的距离最远 0.3 mm；溯源子块只有
                    // `sub·h`（0.2 mm），盖不到 → 查到 0（毛坯灰）。表现出来就是
                    // **台阶的竖直面中间是灰的**（地平面恰好凑巧，所以只有竖面暴露）。
                    //
                    // 顶点本身就在面上，离它最近的体素必然 `|v| ≤ h/2 < h`，
                    // 一定被这一刀盖过章——按位置查是精确的，不是近似。
                    // 取色要**两侧都试**，不能只往"材料侧"挪。
                    //
                    // 面上顶点的两侧，只有被这一刀改动过的那一侧带得到颜色：
                    // · 毛坯外棱：外侧是空气（从没被写过）、内侧是料 → 要取内侧；
                    // · 已铣区与未铣料之间的**台阶**：法线朝已铣的空气侧，
                    //   "内侧"是没动过的料（creator 0）→ 要取外侧。
                    //
                    // 所以先按顶点位置取（含 `floor` 边界，取到空气侧也无妨），
                    // 取到 0 再沿 `-normal` 挪一个体素试另一侧。
                    creator: {
                        let at = |p: cncsim_math::DVec3, cur: &mut crate::tree::CreatorCursor| {
                            let v = self.to_voxel_space(p);
                            self.creator_of_voxel_cached(
                                [v.x.floor() as i32, v.y.floor() as i32, v.z.floor() as i32],
                                cur,
                            )
                        };
                        // **取色：从顶点位置出发，先往外、再往里探。**
                        //
                        // 早先的兜底是"取不到就往 `-normal` 探一格"，理由是
                        // "毛坯外棱：外侧是空气、内侧是料"。这一条对**台阶**成立，
                        // 对**凸面**正好相反：球的刀在外侧，而 `-normal` 是**从没被
                        // 切过的原始料**——那里 creator 恒 0，越探越 0。
                        //
                        // 实测：R15 半球凸台 12935 个球面顶点里 411 个取到 0（灰），
                        // 全是孤立点，撒在球面上就是一层麻点。
                        //
                        // 偏移取 `h` 的**奇数倍**（0.5h / 1.5h / 2.5h）：顶点落在
                        // 格面上时 `floor` 掉到哪一侧是任意的，取半格能避开那条边界；
                        // 2.5h = 0.25 mm 仍在窄带（4h = 0.4 mm）内，一定被最近的
                        // 那一刀盖过章。
                        let a = at(pos, &mut cur);
                        if a != 0 {
                            a
                        } else {
                            let h = self.config.h;
                            let mut c = 0;
                            for o in [0.5f64, -0.5, 1.5, -1.5, 2.5, -2.5] {
                                c = at(pos + normal * (o * h), &mut cur);
                                if c != 0 {
                                    break;
                                }
                            }
                            c
                        }
                    },
                });
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn budget_picks_sqrt_scaling() {
        // 3200 万面 → 200 万预算：k = ⌈√16⌉ = 4
        assert_eq!(choose_lod_factor(32_000_000, 2_000_000, 8), 4);
        // 已在预算内 → 不降级
        assert_eq!(choose_lod_factor(1_500_000, 2_000_000, 8), 1);
        // 4 倍 → k=2
        assert_eq!(choose_lod_factor(8_000_000, 2_000_000, 8), 2);
        // 上限夹紧：需要 k=10，但窄带只允许 4
        assert_eq!(choose_lod_factor(400_000_000, 2_000_000, 4), 4);
        // budget = 0 不当成"无穷小"，退回不降级
        assert_eq!(choose_lod_factor(10, 0, 4), 1);
    }
}
