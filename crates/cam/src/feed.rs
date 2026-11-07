//! **进给调度**：把"名义 F 一刀到底"换成按**负载**逐块调制的 F。
//!
//! 原来整条曲面道只有一个 `F800`。可真实切削的负载沿路径变化很大：凹角里
//! 包角大、排屑差，同一进给下让刀和振纹都更明显。这一层按几何给每块算一个
//! 负载 `load ≥ 1`，再折成进给 `vf = vf_nom / load`。
//!
//! 三遍（`docs/rust-cnc-cam-research.md` §5.11.1 的形状）：
//!
//! 1. **逐点限载**：`vf(i) = clamp(vf_nom / load(i), vf_nom·min_ratio, vf_nom)`；
//! 2. **前视窗**：对 `vf` 做一次**向前看**的窗口最小值——把未来窗口里的低值
//!    提到现在，于是**进凹角之前**就开始降速（等走到凹角才降已经晚了）；
//! 3. **斜率限幅**：相邻块 `|ΔF| ≤ slope·vf_nom`，正反交替扫到稳定。
//!    它同时负责"出凹角别立刻拉满"（400 → 800 走 7 块 ≈ 3.4 mm）——所以
//!    **不需要**再做一次反向窗口，那等于把降速区左右各拉宽一次。
//!
//! ## 为什么不是字面的"恒 MRR"
//!
//! 文档的公式 `vf = vf_ref·(ae_ref·ap_ref)/(ae·ap)` 要的是**真实啮合**。我们
//! 在 CAM 阶段拿不到切削中的材料（没有 Z-map 回写，材料只在仿真里存在），
//! 而"刀心到设计面的距离"在切点处恒等于 `R`——球是和它相切的，这个量给不出
//! `ae`。所以 `ae` 的代理换成几何能直接算的**凹度** `κ·R`：凹处刀球被材料
//! 包住的角度更大，同样行距下去除量更大，这正是"内角负载骤增"的几何来源。
//!
//! **这是工艺参数，不是几何承诺。** 仿真能验证的只有"F 变了、切出来的形状
//! 一位没变"（F 不参与切除计算）；表面质量/让刀我们**没有力学模型，测不到**，
//! 所以不许声称"提高了表面质量"。

/// 进给调度的开关与旋钮。
#[derive(Clone, Debug)]
pub struct FeedOpts {
    /// 是否启用。关掉时全程 `vf_nom`——**F 值**与改造前完全一致，只是不再
    /// 每块都写一遍 `F`（模态字，值不变就不写）。
    pub adaptive: bool,
    /// 下限**占比**（0.35 = 最低降到名义值的 35%）。再紧的凹角也不会低于它
    /// ——进给太低会烧刀。用比例而不是绝对值，换名义进给时下限自动跟着走。
    pub min_ratio: f64,
    /// 凹度罚系数：`load = 1 + k·(κR)^p`。`κR = 1` 表示凹到与刀球同半径
    /// （球刚好塞满），此时进给降到 `1/(1+k)`。
    pub k_conc: f64,
    /// 凹度罚的**指数** `p`（默认 2）。
    ///
    /// 线性（`p=1`）会把"中等弯"也一起减速：叶轮 1 的残料面里 24% 的路程
    /// `κR ≥ 0.8`、51% 的路程 `κR ≥ 0.1`，线性 k=1 换来 **+31%** 切削时间，
    /// 而真正需要减速的只有最塞的那一小撮。二次方让 `κR < 0.4` 几乎不受影响
    /// （罚只有 0.16），`κR ≈ 1` 仍然减半——同样的最深处，一半的代价。
    pub power: f64,
    /// 路径转向罚系数。**默认 0**：点铣的链是最近邻串出来的，锯齿很常见，
    /// 转向角大部分是链的噪声而不是真的拐角（见 `--feed-turn`）。
    pub k_turn: f64,
    /// 前视窗宽度（mm）。窗口要盖住"从名义速度刹到下限"所需的距离。
    pub window_mm: f64,
    /// 每块允许的相对变化（0.10 = 10%）。
    pub slope: f64,
    /// 量化步长（mm/min）：F 是模态字，量化后相邻块的相同值才能合并。
    pub quant: f64,
}

impl Default for FeedOpts {
    fn default() -> Self {
        Self {
            adaptive: true,
            min_ratio: 0.35,
            k_conc: 1.0,
            power: 2.0,
            k_turn: 0.0,
            window_mm: 2.0,
            slope: 0.10,
            quant: 10.0,
        }
    }
}

/// 一段刀路（一条链）的进给统计。
///
/// `min_vf` 的初值是 `INFINITY`（"还没有块"），不是 0——0 会被当成"最小值"，
/// 一条零长度链就能把整段的最低进给冲成 0，再被下一条链重置成名义值。
/// 实测叶轮 1 就这样报过"最低 F800 / 平均 73%"这种自相矛盾的行。
#[derive(Clone, Copy, Debug)]
pub struct FeedStats {
    /// 切削块数（不含下刀那一块）。
    pub blocks: usize,
    /// 实际用到的最低进给（mm/min）。
    pub min_vf: f64,
    /// **按弧长加权**的平均进给比 `vf/vf_nom`——这就是"时间涨了多少"。
    pub mean_ratio: f64,
    /// 低于 0.8×名义值的块数（"降过速"的块）。
    pub low_blocks: usize,
    /// 实际写进 G 代码的 `F` 字个数。**由出码方填**（它才知道模态状态：
    /// 跨链若与上一条末值相同，首块可以省掉一个 `F`），`schedule` 不填。
    pub words: usize,
    /// 按 F 估的切削时间（s）。
    pub time_s: f64,
    /// 全程 `vf_nom` 时的切削时间（s）——对照组。
    pub flat_time_s: f64,
}

impl Default for FeedStats {
    fn default() -> Self {
        Self {
            blocks: 0,
            min_vf: f64::INFINITY,
            mean_ratio: 1.0,
            low_blocks: 0,
            words: 0,
            time_s: 0.0,
            flat_time_s: 0.0,
        }
    }
}

impl FeedStats {
    pub fn merge(&mut self, o: &FeedStats) {
        self.blocks += o.blocks;
        self.low_blocks += o.low_blocks;
        self.words += o.words;
        self.time_s += o.time_s;
        self.flat_time_s += o.flat_time_s;
        self.min_vf = self.min_vf.min(o.min_vf);
    }

    /// 平均进给比：`flat_time / time`（按时间加权，和 `mean_ratio` 同义）。
    pub fn ratio(&self) -> f64 {
        if self.time_s > 0.0 {
            self.flat_time_s / self.time_s
        } else {
            1.0
        }
    }
}

/// 排进给表。返回每块的 `F`（mm/min）与统计。
///
/// `seg[i]` 是第 `i` 块的长度（mm），`load[i]` 是它的负载（≥ 1，1 = 开阔处）。
/// 两者等长；`seg[0]` 是从"下刀点"到第一个切削点的距离。
pub fn schedule(seg: &[f64], load: &[f64], vf_nom: f64, o: &FeedOpts) -> (Vec<f64>, FeedStats) {
    let n = seg.len().min(load.len());
    let mut st = FeedStats::default();
    if n == 0 || !vf_nom.is_finite() || vf_nom <= 0.0 {
        return (vec![vf_nom; n], st);
    }
    let flat: f64 = seg.iter().take(n).sum::<f64>() / vf_nom * 60.0;
    st.flat_time_s = flat;
    if !o.adaptive {
        // 关掉时：全程名义值，时间 = 匀速（F 字仍由出码方按模态去重）
        st.blocks = n;
        st.min_vf = vf_nom;
        st.mean_ratio = 1.0;
        st.time_s = flat;
        return (vec![vf_nom; n], st);
    }

    // ── ① 逐点限载 ──
    let lo = (vf_nom * o.min_ratio.clamp(0.0, 1.0)).clamp(1.0, vf_nom);
    let mut vf: Vec<f64> = (0..n)
        .map(|i| (vf_nom / load[i].max(1.0)).clamp(lo, vf_nom))
        .collect();

    // ── ② 前视窗 ──
    //
    // **只向前看**，不做反向的对称窗：出凹角后"别立刻拉满"由第 ③ 步的斜率
    // 限幅负责（400→800 走 7 块 ≈ 3.4 mm）。两样都做等于把降速区左右各拉宽
    // 一次，低速块比例和切削时间都白涨一轮（叶轮 1 的实测见 §53）。
    let w = window_blocks(seg, o.window_mm);
    vf = win_min_fwd(&vf, w);

    // ── ③ 斜率限幅 ──
    let step = (o.slope.max(0.0) * vf_nom).max(1.0);
    for _ in 0..32 {
        let mut moved = false;
        for i in 1..n {
            let cap = vf[i - 1] + step;
            if vf[i] > cap {
                vf[i] = cap;
                moved = true;
            }
        }
        for i in (0..n - 1).rev() {
            let cap = vf[i + 1] + step;
            if vf[i] > cap {
                vf[i] = cap;
                moved = true;
            }
        }
        if !moved {
            break;
        }
    }

    // ── ④ 量化 + 统计 ──
    let q = o.quant.max(0.0);
    if q > 0.0 {
        for v in vf.iter_mut() {
            *v = ((*v / q).round() * q).clamp(lo, vf_nom);
        }
    }
    let mut sum_w = 0.0;
    let mut sum_t = 0.0;
    st.blocks = n;
    st.min_vf = vf_nom;
    for i in 0..n {
        let v = vf[i];
        st.min_vf = st.min_vf.min(v);
        if v < 0.8 * vf_nom - 1e-9 {
            st.low_blocks += 1;
        }
        let t = seg[i] / v; // min
        sum_w += seg[i];
        sum_t += t;
        st.time_s += t * 60.0;
    }
    // 平均比按**弧长**加权（= 匀速时间 / 实际时间），与 `ratio()` 一致
    st.mean_ratio = if sum_w > 0.0 {
        (sum_w / vf_nom) / sum_t.max(1e-12)
    } else {
        1.0
    };
    (vf, st)
}

/// 窗口宽度折成块数：按**中位块长**算（块长沿链会变，均值被长块带偏）。
fn window_blocks(seg: &[f64], win_mm: f64) -> usize {
    if seg.is_empty() || !win_mm.is_finite() || win_mm <= 0.0 {
        return 0;
    }
    let mut v: Vec<f64> = seg.iter().copied().filter(|d| *d > 1e-9).collect();
    if v.is_empty() {
        return 0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let med = v[v.len() / 2];
    ((win_mm / med).round() as usize).clamp(1, 64)
}

/// 宽度 `w` 的**向前看**窗口最小值：`out[i] = min(vf[i..=i+w])`。
///
/// 低值因此**提前**出现——刹车必须早于凹角。窗口是常数（≤64 块），直接扫，
/// 复杂度 `O(n·w)`；n 是块数（叶轮 3 万块），最坏 200 万次比较，可以忽略。
/// 这里不用单调队列：反向滑动窗口的进出端相反，写错了不会报错、只会悄悄偏移，
/// 不值得为这点常数冒险。
fn win_min_fwd(v: &[f64], w: usize) -> Vec<f64> {
    let n = v.len();
    if n == 0 {
        return Vec::new();
    }
    (0..n)
        .map(|i| {
            let hi = (i + w).min(n - 1);
            v[i..=hi].iter().copied().fold(f64::INFINITY, f64::min)
        })
        .collect()
}

/// 负载：`1 + k·(κR)^p + k_turn·turn`（每项都先夹到 `[0,1]`）。
pub fn load_of(conc_r: f64, turn: f64, o: &FeedOpts) -> f64 {
    let c = conc_r.clamp(0.0, 1.0);
    1.0 + o.k_conc * c.powf(o.power.max(1.0)) + o.k_turn * turn.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn o() -> FeedOpts {
        FeedOpts {
            adaptive: true,
            min_ratio: 0.25,
            k_conc: 1.0,
            power: 2.0,
            k_turn: 0.0,
            window_mm: 1.0,
            slope: 0.10,
            quant: 10.0,
        }
    }

    #[test]
    fn flat_is_untouched() {
        // 全程负载 1：不该有任何降速，也不该出现多余 F 字
        let seg = vec![0.5; 40];
        let load = vec![1.0; 40];
        let (vf, st) = schedule(&seg, &load, 800.0, &o());
        assert!(vf.iter().all(|v| (*v - 800.0).abs() < 1e-9));
        assert_eq!(st.words, 0, "F 字由出码方数，schedule 不填");
        assert_eq!(st.low_blocks, 0);
        assert!((st.ratio() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn corner_slows_and_looks_ahead() {
        // 第 20 块是凹角（load 2 → 400），前后 2 mm 内就该开始降速
        let n = 60;
        let seg = vec![0.5; n];
        let mut load = vec![1.0; n];
        load[20] = 2.0;
        let (vf, st) = schedule(&seg, &load, 800.0, &o());
        assert!((vf[20] - 400.0).abs() < 1e-9, "凹角处应减半：{}", vf[20]);
        // 窗口 1 mm = 2 块：第 18 块已经在降（**提前**刹），第 17 块还在斜坡上
        assert!(vf[18] < 800.0, "必须提前降速：{}", vf[18]);
        assert!(vf[10] > 790.0, "远处的块不受影响：{}", vf[10]);
        // 降速区间是有限的：窗口 + 斜率斜坡，不会拖到链尾
        let low = vf.iter().filter(|v| **v < 640.0).count();
        assert!((1..=12).contains(&low), "降速块数应在窗口邻域内：{low}");
        assert!(st.ratio() < 1.0 && st.ratio() > 0.85);
        assert!((st.min_vf - 400.0).abs() < 1e-9, "min_vf = {}", st.min_vf);
        assert!(st.low_blocks >= 1);
    }

    #[test]
    fn slope_is_limited() {
        // load 从 1 跳到 2（800 → 400）：相邻块的变化不能超过 slope·vf_nom = 80
        let n = 60;
        let seg = vec![0.5; n];
        let mut load = vec![1.0; n];
        for x in load.iter_mut().skip(20) {
            *x = 2.0;
        }
        let (vf, _) = schedule(&seg, &load, 800.0, &o());
        for i in 1..n {
            assert!(
                (vf[i] - vf[i - 1]).abs() <= 80.0 + 10.0 + 1e-9,
                "块 {i} 变化太大：{} → {}",
                vf[i - 1],
                vf[i]
            );
        }
        assert!((vf[40] - 400.0).abs() < 1e-9, "稳态仍是 800/2");
    }

    #[test]
    fn off_means_constant() {
        let mut opt = o();
        opt.adaptive = false;
        let seg = vec![0.5; 10];
        let load = vec![4.0; 10];
        let (vf, st) = schedule(&seg, &load, 800.0, &opt);
        assert!(vf.iter().all(|v| (*v - 800.0).abs() < 1e-9));
        assert_eq!(st.blocks, 10);
    }
}
