//! CSG 去除：`d_new = max(d_old, -sd_sweep)`。
//!
//! ## 为什么这个公式是对的
//!
//! 毛坯 `A` 减去刀具扫掠体 `B`：`sd(A \ B) = max(sd_A, -sd_B)`。
//! 在窄带内（`|d| < band`）这是精确的，因为结果的边界只由 `∂A` 与 `∂B` 的片段组成。
//! 远离边界处 `max` 只是下界，但那时值已经被钳制，不影响判定。
//!
//! ## 为什么要裁剪 AABB
//!
//! 一把 φ6×20 的平底刀，扫掠 AABB 是 6×6×20 mm。若不裁剪，刀在毛坯上方空走时
//! 会去遍历整片空气。裁剪到"曾经写过的范围"（[`SparseGrid::bounds`]）之后，
//! 只剩真正可能碰到材料的部分。
//!
//! ## 为什么裁剪之后还要外扩 `band`
//!
//! 裁剪是裁剪到"扫掠体 + 窄带"，不是裁剪到扫掠体本身。新表面落在扫掠体边界上，
//! 所以到新表面距离 `< band` 的体素一定落在"扫掠体 ∪ band 邻域"里。
//!
//! 早先没扩这一圈，刀尖平面以下的材料侧只有 1 个体素拿到真实距离，再往下
//! 全是切前的饱和值 `−band`。**窄带单边完整等于没有窄带**——任何跨过切面的
//! 插值都会被饱和值拽偏。实测：`k=4` 的显示 LOD 在槽底偏 97 µm，细格网格
//! 靠那 1 个体素侥幸正确。碰撞用的是同一套插值，所以这不是显示问题。
//!
//! ## 两条入口，一套内核
//!
//! | 入口 | 语义 | 用途 |
//! |---|---|---|
//! | [`SparseGrid::cut_segment`] | 切**一刀**，切完立即可读 | 仿真主循环（每段之后要查碰撞）|
//! | [`SparseGrid::cut_batch`] | 切**一批**，等价于按序逐刀切 | 粗加工预览 / 只关心最终形状的场合 |
//!
//! 批量不是另一套算法：它把同一批的段按叶归拢——一个叶只从树里取一次、
//! 只写回一次，叶内的 k 段顺序套用同一个 `max`。因为 `max(a, max(b, c))`
//! 与顺序无关、且量化是幂等的，批量的**体素值与逐刀调用逐位相同**
//! （`tests/cut.rs::batch_matches_one_cut_at_a_time` 逐体素对拍）。
//!
//! ## 叶内为什么按"行"分类
//!
//! 平底刀沿单轴走刀时，叶内一整行的 `sd` 是同一个数。一行只有三种命运：
//!
//! 1. `sd ≥ band`：刀够不着，**整行不动**（一行 8 个字节一次都不碰）；
//! 2. `sd ≤ −band`：整行埋在刀体深处，**整行填成空气**（一条 8 字节存储，
//!    不必逐点 `round`——填 `MAX` 对已经是 `MAX` 的体素是幂等的）；
//! 3. 其余：真正的表面行，逐点算 `round(max(d, −sd))`。
//!
//! 粗加工里 2 占绝大多数（铣光实测改动/求值 75%），所以这条把最热的循环
//! 从"每体素一次浮点换算"降成"每 8 体素一次存储"。

use crate::sweep::sd_sweep_with;
use crate::tree::{LEAF_DIM, LEAF_LOG2, LEAF_VOXELS, Leaf, LeafView, Scalar, SparseGrid};
use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::Profile;

/// 一次切削的统计。**这是性能观测点**，也是"到底改了哪些砖"的依据。
/// `CNCSIM_PROF=1` 时打印每刀的 求值/落盘 拆分（临时诊断，只查一次环境变量）。
/// 累计的 eval / write 纳秒（`CNCSIM_PROF=1` 时才有意义）。
///
/// 原来每刀 `eprintln!` 一次，光是格式化 + stderr 写就有几十 µs——**比它要测的
/// 东西还贵**，测出来的拆分不可信。改成原子累加（~1 ns），跑完一次性读。
pub static PROF_EVAL_NS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub static PROF_WRITE_NS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// `collect` 阶段的新建叶数与总块数（判断"是不是都在造叶"）。
pub static PROF_NEW_LEAF: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub static PROF_N_BLOCK: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// 落盘内部五个阶段的累加器（whole / collect / sort / blit / prune）。
pub static PROF_W: [std::sync::atomic::AtomicU64; 5] = [
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
];

/// 取走落盘五阶段的累计毫秒（whole, collect, sort, blit, prune）。
pub fn prof_take_write() -> Option<[f64; 5]> {
    if !prof_enabled() {
        return None;
    }
    use std::sync::atomic::Ordering::Relaxed;
    let mut out = [0.0; 5];
    for (i, a) in PROF_W.iter().enumerate() {
        out[i] = a.swap(0, Relaxed) as f64 / 1e6;
    }
    Some(out)
}

/// 取走累计值并清零，返回 `(eval_ms, write_ms)`。未开 `CNCSIM_PROF` 时返回 `None`。
pub fn prof_take() -> Option<(f64, f64)> {
    if !prof_enabled() {
        return None;
    }
    use std::sync::atomic::Ordering::Relaxed;
    let e = PROF_EVAL_NS.swap(0, Relaxed) as f64 / 1e6;
    let w = PROF_WRITE_NS.swap(0, Relaxed) as f64 / 1e6;
    Some((e, w))
}

pub(crate) fn prof_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("CNCSIM_PROF").is_some())
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CutStats {
    /// 参与求值的体素数（已裁剪）
    pub visited: usize,
    /// 实际改动的体素数
    pub changed: usize,
    /// 裁剪后的 AABB 尺寸（体素）
    pub box_size: [i32; 3],
}

/// 检测"沿单轴走刀"：返回走刀轴（0=x, 1=y）。
///
/// **为什么这个判断值钱**：立铣刀绝大多数切削是沿一个轴走直刀（x 或 y），
/// 刀轴竖直不动。这时扫掠体是"圆柱沿该轴拉伸"，而 `sd_flat_prism` 里
/// `dz` 只依赖 z、`dr` 只依赖到走刀线段的距离——**叶完全落在走刀段内时，
/// `dr` 退化成 |py|，SDF 与走刀方向的那个坐标完全无关**。
/// 于是叶的 512 次求值能降到 64 次（8×8 截面），沿走刀方向原样复用。
///
/// 条件刻意保守：刀轴竖直且两端一致、位移只沿一个轴、且不是沿 z（钻孔的
/// 扫掠体沿刀轴延伸，不是横向拉伸，没有这个性质）。
fn sweep_axis(from: &Pose, to: &Pose) -> Option<usize> {
    if (from.axis - to.axis).length_squared() > 1e-30 {
        return None;
    }
    // 刀轴必须竖直（±Z）——倾斜刀轴下径向平面不是 xz/yz
    if from.axis.x.abs() > 1e-12 || from.axis.y.abs() > 1e-12 {
        return None;
    }
    let d = to.tip - from.tip;
    let tol = 1e-12;
    let (nx, ny, nz) = (d.x.abs() > tol, d.y.abs() > tol, d.z.abs() > tol);
    if (nx as u8 + ny as u8 + nz as u8) != 1 {
        return None; // 斜向或斜插补：两个方向共享同一个 t，没有这个性质
    }
    if nz {
        return None; // 沿 z 下刀：扫掠体沿刀轴，不是横向拉伸
    }
    Some(if nx { 0 } else { 1 })
}

/// 单段扫掠的世界坐标 AABB。
///
/// 平移扫掠的扫掠体包含在两个端点刀具的凸包内，所以取两端 AABB 的并即可（保守但紧）。
pub fn sweep_aabb(from: &Pose, to: &Pose, profile: &Profile) -> (DVec3, DVec3) {
    let r = profile.max_radius();
    // **用母线自己的 z 范围，不是 `0..height()`**：刀杆 profile 从刃长开始
    // （`ToolDef::shank`），从刀尖算起会白扫 20 mm 的空气，多遍历几百层体素。
    let pts = profile.points();
    let (z0, z1) = {
        let a = pts[0].y;
        let b = pts[pts.len() - 1].y;
        (a.min(b), a.max(b))
    };
    let mut lo = DVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = DVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for pose in [from, to] {
        for a in 0..3 {
            let t0 = pose.tip.component(a) + pose.axis.component(a) * z0;
            let t1 = pose.tip.component(a) + pose.axis.component(a) * z1;
            // 径向偏移在轴 a 上的投影上界：垂直于刀轴的向量与 e_a 的夹角余弦 ≤ √(1−axis_a²)。
            // 刀轴沿 Z 时 Z 向投影为 0——这正是"刀尖在 z=3，下界就是 3"。
            let perp = r * (1.0 - pose.axis.component(a) * pose.axis.component(a)).max(0.0).sqrt();
            lo = set(lo, a, lo.component(a).min(t0.min(t1) - perp));
            hi = set(hi, a, hi.component(a).max(t0.max(t1) + perp));
        }
    }
    (lo, hi)
}

#[inline]
fn set(mut v: DVec3, i: usize, x: f64) -> DVec3 {
    match i {
        0 => v.x = x,
        1 => v.y = x,
        _ => v.z = x,
    }
    v
}

/// 平底刀**纯平移**（水平走刀或竖直下刀）的内联特化参数。
///
/// 这条为什么值钱：单轴共享（`Seg::sweep`）只对"刀心路径正上方"的叶成立——
/// 叶必须整个落在 `[tip_a, tip_b]` 区间里。立铣刀走短段（精加工 0.05 mm、圆弧
/// 离散 0.3 mm）时，刀径 6 mm 扫过的范围远大于刀心走过的几微米，**90% 以上的
/// 叶落在端头圆帽区**，共享路径够不着，全部掉进逐体素 `sd_sweep_with`。
/// VTune 实测小段场景 `sd_sweep_with` 占 50%（每次都重算 step、axis_delta、
/// 两级 Option 判断和 `origin_to_segment` 的向量运算）。
///
/// 这里把段级常量折叠成标量，每体素只剩 ~15 次浮点运算 + 1 次 sqrt。
/// 公式与 `sd_flat_prism` 逐项相同（除法不换乘倒数），浮点结果逐位一致。
#[derive(Clone, Copy)]
struct FlatSweep {
    tip: DVec3,
    /// 步进的径向分量（刀轴竖直时就是 step.x/step.y）
    wdx: f64,
    wdy: f64,
    wd_l2: f64,
    /// 步进的轴向分量；扫掠后的轴向区间 `[zlo, zhi]`
    zlo: f64,
    zhi: f64,
    r: f64,
    /// 竖直下刀（径向不动）：径向距离直接是 |w0|
    vertical: bool,
}

impl FlatSweep {
    /// `None` = 不满足特化条件（非平底刀 / 刀轴不竖直 / 斜向进给 / 零步长）。
    fn new(from: &Pose, to: &Pose, flat: Option<(f64, f64)>) -> Option<Self> {
        let (r, len) = flat?;
        // 刀轴必须**恰好**是 +Z：z0/w0 的分解才能退化成减法
        if from.axis.x != 0.0 || from.axis.y != 0.0 || from.axis.z != 1.0 {
            return None;
        }
        if (from.axis - to.axis).length_squared() > 1e-30 {
            return None;
        }
        let step = to.tip - from.tip;
        // 与 `sd_sweep_with` 的快路径条件一致；零步长回退 `sd_tool`
        if step.length_squared() < 1e-30 {
            return None;
        }
        let sz = step.z;
        let (wdx, wdy) = (step.x, step.y);
        let wd_l2 = wdx * wdx + wdy * wdy;
        // 解耦条件：径向或轴向必有一个不动，否则要黄金分割
        let vertical = wd_l2 < 1e-28;
        if !(sz.abs() < 1e-12 || vertical) {
            return None;
        }
        Some(Self {
            tip: from.tip,
            wdx,
            wdy,
            wd_l2,
            zlo: sz.min(0.0),
            zhi: len.max(len + sz),
            r,
            vertical,
        })
    }

    /// 点到这段扫掠体的距离。与 `sd_sweep_with` 的快路径逐位一致。
    #[inline]
    fn sd(&self, px: f64, py: f64, pz: f64) -> f64 {
        let wx = px - self.tip.x;
        let wy = py - self.tip.y;
        let dr = if self.vertical {
            (wx * wx + wy * wy).sqrt() - self.r
        } else {
            // 原点到线段 [w0, w0−wd] 的距离 = 端头圆帽 / 侧壁
            let t = ((wx * self.wdx + wy * self.wdy) / self.wd_l2).clamp(0.0, 1.0);
            let qx = wx - self.wdx * t;
            let qy = wy - self.wdy * t;
            (qx * qx + qy * qy).sqrt() - self.r
        };
        let zc = pz - self.tip.z;
        let dz = (self.zlo - zc).max(zc - self.zhi);
        let (a, b) = (dr.max(0.0), dz.max(0.0));
        (a * a + b * b).sqrt() + dr.max(dz).min(0.0)
    }
}

/// 预处理过的一段切削：批量路径把每段只算一次的东西全提到这里。
///
/// 单刀路径也用同一个结构（批量长度为 1），所以两条路径不会各自漂移——
/// "逐刀与批量结果逐位相同"由**同一份代码**保证，不是靠测试碰运气。
#[derive(Clone, Copy)]
struct Seg {
    from: Pose,
    to: Pose,
    /// 平底刀 `(半径, 刃长)`；`None` = 非平底刀，走通用扫掠
    flat: Option<(f64, f64)>,
    /// 单轴走刀 `(轴, 起点, 终点)`；`None` = 斜向 / 沿 z / 倾斜刀轴
    sweep: Option<(usize, f64, f64)>,
    /// 平底刀内联特化；`None` = 走 `sd_sweep_with`
    fast: Option<FlatSweep>,
    /// 段在体素空间的 AABB（已按窄带外扩并裁剪到网格），半开区间
    lo: [i32; 3],
    hi: [i32; 3],
}

impl Seg {
    /// 这一段在点 `p` 处的扫掠距离（能走特化就走特化）。
    #[inline]
    fn sd(&self, p: DVec3, profile: &Profile) -> f64 {
        match &self.fast {
            Some(f) => f.sd(p.x, p.y, p.z),
            None => sd_sweep_with(p, &self.from, &self.to, profile, self.flat),
        }
    }

    /// 叶的体素范围与这段的 AABB 是否相交——6 次整数比较，比任何 SDF 都便宜。
    #[inline]
    fn touches(&self, bl: [i32; 3], bh: [i32; 3]) -> bool {
        self.lo[0] < bh[0]
            && self.hi[0] > bl[0]
            && self.lo[1] < bh[1]
            && self.hi[1] > bl[1]
            && self.lo[2] < bh[2]
            && self.hi[2] > bl[2]
    }
}

/// 一个切削子块的结果：**只收集真正变了的叶**。
///
/// 不再有"散点改动列表"：整块写（[`Self::blocks`]）覆盖了全部场景——被 AABB
/// 切开的残叶也整块写回（叶内 AABB 之外保持原值，幂等），于是"每体素算下标再
/// 散写"这条路一次都走不到。
struct CutPart<S> {
    visited: usize,
    /// 整叶短路：叶坐标 + 填充值。整叶 512 个体素都在扫掠体深处时，
    /// 一次填充代替 512 次求值+写入。落盘时走 `fill_leaf`。
    whole: Vec<([i32; 3], S)>,
    /// 整块改动：完整叶逐点求值的结果，512 个值一起带回。落盘时走 `blit_leaf`。
    blocks: Vec<LeafBlock>,
    /// 整块值的 **arena**：所有叶的 512 个值平铺在这里，`LeafBlock::offset` 索引。
    ///
    /// 为什么不给每个叶一个 `Box<[S; 512]>`：那是每叶一次 `malloc`，一刀 3920 次。
    /// 24 个线程并发打到同一把分配器锁上——实测**求值加速比卡在 4.8×**，
    /// 8 线程之后再也不涨（机器有 24 线程）。arena 让每片只 realloc 几次，
    /// 追加就是 `extend_from_slice`。
    pool: Vec<S>,
    /// 子块位图的 **arena**：`LeafBlock::mask_off` 索引到这里的 u64。
    ///
    /// 位图**不放在 `LeafBlock` 里**：`sub = 8` 要 8 个 u64，塞进去会把
    /// `LeafBlock` 从 32 B 撑到 96 B——一次批量切削有几十万个块，光指针数组
    /// 就多出几 MB 的无效流量，而 `sub ≤ 4` 只需要 1 个 u64、剩下 7 个纯属浪费。
    /// 平铺进 arena 之后，位图是连续的，块表回到 40 B。
    masks: Vec<u64>,
}

impl<S> CutPart<S> {
    fn empty() -> Self {
        Self {
            visited: 0,
            whole: Vec::new(),
            blocks: Vec::new(),
            pool: Vec::new(),
            masks: Vec::new(),
        }
    }
}

/// 一个叶的整块改动：512 个新值整体替换。
///
/// **为什么不沿用"每体素一条记录"**：那条路每项 16 字节，铣光一刀 21 万条改动
/// 就是 3.4 MB 的中转缓冲——L2 装不下，于是求值时读过的叶到写回时已经被挤出去，
/// 整个工作集被扫了两遍。改成整块之后：
///
/// | | 散点（旧） | 整块（新） |
/// |---|---|---|
/// | 每条记录 | 16 字节 | 512 字节 / 叶（`n` 条改动共用） |
/// | 21 万改动 | 3.4 MB | 758 叶 × 0.5 KB = **388 KB** |
/// | 写回方式 | 21 万次「算下标 + 写 1 字节」 | 758 次 `copy_from_slice` |
///
/// 而且叶内布局是 `x | y<<3 | z<<6`（x 最内层），所以按 x-y-z 遍历时
/// `values` 的写入是**严格顺序**的——本地缓冲常驻 L1。
struct LeafBlock {
    /// 叶坐标（`ijk >> 3`）
    leaf: [i32; 3],
    /// `(叶下标, 所属 level1 下标)`。`None` = 叶还不存在（落盘时要串行创建）。
    /// 有值时落盘按叶下标做**无锁并行分区**，l1 下标用来更新 blind 元数据。
    slot: Option<(u32, u32)>,
    /// 512 个新值在 `CutPart::pool` 里的起点
    offset: usize,
    /// 真正改动的体素数
    n: u16,
    /// 子块位图在 `CutPart::masks` 里的起始字下标（第 i 位 = 叶内第 i 个子块变了）。
    ///
    /// 字数按粒度裁剪：`sub ≤ 4` 是 1 个字，`sub = 8` 是 8 个。
    mask_off: u32,
}

impl<S: Scalar> SparseGrid<S> {
    /// 用一段扫掠刀具切除材料，返回统计。**切完立即可读**——仿真主循环靠它
    /// 逐段推进（每段之后要拿当前网格查碰撞，所以不能攒批）。
    ///
    /// 切削之后**必须**让 `prune` 折叠整叶同值的区域——这里已经自动调用，
    /// 否则切空的叶仍是 `Dense`，碰撞会认为那里还有材料（ADR-004 §5.4）。
    pub fn cut_segment(&mut self, from: &Pose, to: &Pose, profile: &Profile) -> CutStats {
        self.cut_segment_with(from, to, profile, 0)
    }

    /// `slices = 0` 表示自动（按体积与线程数决定）；`> 0` 强制切这么多片。
    ///
    /// 强制切片**只为对拍**：结果（改动数 / 每个体素的值）必须与片数无关。
    pub fn cut_segment_with(
        &mut self,
        from: &Pose,
        to: &Pose,
        profile: &Profile,
        slices: usize,
    ) -> CutStats {
        self.cut_sweeps(&[(*from, *to)], profile, slices)
    }

    /// **一批切削**：与按顺序逐段调用 [`Self::cut_segment`] **逐位等价**。
    ///
    /// 差别只在遍历次数：一批 N 段只把叶从树里取一次、只写回一次、只 prune 一次。
    /// 因为 CSG 是 `max`（交换律 + 结合律）且量化幂等，批内的段顺序不影响结果。
    ///
    /// 适用：只关心最终形状的场合（粗加工预览、材料去除率、过切检查）。
    /// **不适用**：需要在每段之后立刻查碰撞/采样的仿真主循环——那里仍要逐刀切。
    pub fn cut_batch(&mut self, cuts: &[(Pose, Pose)], profile: &Profile) -> CutStats {
        self.cut_sweeps(cuts, profile, 0)
    }

    /// [`Self::cut_batch`] 的强制分片版（对拍用）。
    pub fn cut_batch_with(
        &mut self,
        cuts: &[(Pose, Pose)],
        profile: &Profile,
        slices: usize,
    ) -> CutStats {
        self.cut_sweeps(cuts, profile, slices)
    }

    /// 两条入口共同的内核。
    ///
    /// ## 并行方式：算值并行，落盘串行
    ///
    /// 体素之间**没有数据依赖**——每个点的 `max(d_cur, -sd_sweep)` 只读自己那一个值。
    /// 所以按 AABB 切片并行求值，再串行写回。`SparseGrid` 始终单一所有者
    /// （《并发与性能》§2：并行写同一个网格结果不确定，因为叶的插入顺序会变）。
    ///
    /// 片是按**叶边界**切的，所以一个叶只属于一片：批量路径里"一个叶只写一次"
    /// 在并行下也成立，两个线程不会各写一半。
    fn cut_sweeps(&mut self, cuts: &[(Pose, Pose)], profile: &Profile, slices: usize) -> CutStats {
        let mut stats = CutStats::default();
        let Some((glo, ghi)) = self.bounds() else {
            return stats; // 空网格
        };
        if cuts.is_empty() {
            return stats;
        }

        let h = self.config.h;
        let o = self.origin;
        // 扫掠 AABB **必须再向外扩一个窄带宽度**。
        //
        // 新表面落在扫掠体边界上，所以"到新表面距离 < band"的体素一定落在
        // "扫掠体外扩 band"之内。不扩的话，刀尖平面以下那侧只有 1 个体素
        // （AABB 的 −0.5 取整余量）拿到了真实距离，再往下全是切前的饱和值 −band——
        // **窄带单边完整，等于没有窄带**：跨界插值会把表面往下拽。
        // 实测：k=4 的显示 LOD 在槽底偏了 **97 µm**（真值 0 差），正是这个洞造成的。
        let margin = self.config.band_voxels as i32;

        let mut segs: Vec<Seg> = Vec::with_capacity(cuts.len());
        let mut lo = [i32::MAX; 3];
        let mut hi = [i32::MIN; 3];
        for &(from, to) in cuts {
            let (wlo, whi) = sweep_aabb(&from, &to, profile);
            // 世界 → 体素索引（体素中心在 origin + (i+0.5)h）
            let mut slo = [0i32; 3];
            let mut shi = [0i32; 3];
            let mut outside = false;
            for a in 0..3 {
                let i0 = ((wlo.component(a) - o.component(a)) / h - 0.5).floor() as i32;
                let i1 = ((whi.component(a) - o.component(a)) / h - 0.5).floor() as i32 + 1;
                // 裁剪到已写范围
                slo[a] = (i0 - margin).max(glo[a]);
                shi[a] = (i1 + margin).min(ghi[a]);
                outside |= slo[a] >= shi[a];
            }
            if outside {
                continue; // 这一段完全在网格外
            }
            for a in 0..3 {
                lo[a] = lo[a].min(slo[a]);
                hi[a] = hi[a].max(shi[a]);
            }
            let flat = profile.as_flat_end_mill();
            segs.push(Seg {
                from,
                to,
                flat,
                sweep: sweep_axis(&from, &to).map(|ax| {
                    let (u, v) = (from.tip.component(ax), to.tip.component(ax));
                    (ax, u.min(v), u.max(v))
                }),
                fast: FlatSweep::new(&from, &to, flat),
                lo: slo,
                hi: shi,
            });
        }
        if segs.is_empty() {
            return stats; // 整批都在网格外
        }
        stats.box_size = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];

        let band = self.config.band_mm();
        let volume =
            stats.box_size[0] as usize * stats.box_size[1] as usize * stats.box_size[2] as usize;

        // 小活不开线程：一次 24 线程 fork-join 实测 ~0.5 ms，体积不到 2 万体素时
        // 光分摊开销就吃掉收益。
        // 注：试过在这里**预创建** AABB 内所有叶，好让落盘全走并行路径。
        // 实测是负优化（铣光 439 → 444 ms）：落盘在 arena 改造之后已经不是瓶颈，
        // 而预创建要付每刀约 750 次三级查找。叶不存在的那些块留给串行的
        // `blit_leaf` 就地创建，反而更快。

        // ── 分片策略 ──
        //
        // 两个约束互相拉扯：
        //
        // 1. **边界要落在叶边界上**：片内的叶才是完整的，整块写回才生效
        //    （按体素层切会把每片的叶都切成残块，实测整块路径一次都走不到）。
        // 2. **片数要够喂满线程**：片数受限于"分片轴上有多少个叶层"。
        //
        // 所以选**叶层最多的那个轴**来切。铣光的 AABB 是 284×34×38 体素：
        // x 方向 36 个叶层、y/z 各只有 5 个——死按 z 切，片数被 clamp 到 5，
        // 24 线程只有 5 个在跑（实测求值只加速 4.9×）。按 x 切就能切满。
        let nleaf = [
            ((hi[0] + LEAF_DIM - 1) >> LEAF_LOG2) - (lo[0] >> LEAF_LOG2),
            ((hi[1] + LEAF_DIM - 1) >> LEAF_LOG2) - (lo[1] >> LEAF_LOG2),
            ((hi[2] + LEAF_DIM - 1) >> LEAF_LOG2) - (lo[2] >> LEAF_LOG2),
        ];
        let axis = (0..3).max_by_key(|&a| nleaf[a]).unwrap_or(0);
        let nleaf_axis = nleaf[axis] as usize;
        let l0_axis = lo[axis] >> LEAF_LOG2;

        let slices = if slices > 0 {
            slices.min(nleaf_axis).max(1)
        } else if volume >= crate::par::MIN_PAR_WORK && crate::par::worker_count() > 1 {
            // 片数取 2× 线程数：工作量并不均匀（大量体素因 `d_cur >= band` 跳过），
            // 片比线程多，动态领号才能把尾巴吃掉。
            (crate::par::worker_count() * 2).min(nleaf_axis).max(1)
        } else {
            1
        };

        if slices > 1 {
            let per = nleaf_axis.div_ceil(slices);
            let grid: &Self = self;
            let t_eval = std::time::Instant::now();
            let parts = crate::par::map_indexed(slices, |i| {
                // 叶层范围 → 体素范围，再与 AABB 求交（对齐后首尾片会越界）
                let w0 = ((l0_axis + (i * per) as i32) << LEAF_LOG2).max(lo[axis]);
                let w1 = ((l0_axis + ((i + 1) * per) as i32) << LEAF_LOG2)
                    .min(hi[axis])
                    .max(w0);
                if w0 >= w1 {
                    return CutPart::empty();
                }
                let mut a = lo;
                let mut b = hi;
                a[axis] = w0;
                b[axis] = w1;
                // 只把**真的碰到这一片**的段带进去：铣光一片 2 个叶层（3.2 mm），
                // 而一段的 z 跨度是刃长 6 mm——200 段里只有二十来段落在这一片。
                let sub: Vec<Seg> = segs
                    .iter()
                    .copied()
                    .filter(|s| s.lo[axis] < w1 && s.hi[axis] > w0)
                    .collect();
                if sub.is_empty() {
                    return CutPart::empty();
                }
                grid.cut_box(a, b, &sub, profile, band)
            });
            let eval_ns = t_eval.elapsed().as_nanos() as u64;
            let t_write = std::time::Instant::now();
            self.write_parts(parts, &mut stats);
            if prof_enabled() {
                use std::sync::atomic::Ordering::Relaxed;
                PROF_EVAL_NS.fetch_add(eval_ns, Relaxed);
                PROF_WRITE_NS.fetch_add(t_write.elapsed().as_nanos() as u64, Relaxed);
            }
        } else {
            let t_eval = std::time::Instant::now();
            let part = self.cut_box(lo, hi, &segs, profile, band);
            let eval_ns = t_eval.elapsed().as_nanos() as u64;
            let t_write = std::time::Instant::now();
            self.write_parts(vec![part], &mut stats);
            if prof_enabled() {
                use std::sync::atomic::Ordering::Relaxed;
                PROF_EVAL_NS.fetch_add(eval_ns, Relaxed);
                PROF_WRITE_NS.fetch_add(t_write.elapsed().as_nanos() as u64, Relaxed);
            }
        }

        // prune 已经在 `write_parts` 里按"写过的叶"做了增量版本
        stats
    }

    /// 串行落盘：① 整叶短路里"槽位还是 tile"的少数叶走 `fill_leaf`（要创建叶），
    /// ② 其余整块——叶已存在的攒起来按叶下标**无锁并行**写，要新建的当场串行。
    ///
    /// 顺序不影响结果（`whole` 与 `blocks` 的叶集合天然不相交）。
    /// `CNCSIM_PROF=1` 时打印各阶段耗时——**这轮优化就是它揪出来的**：
    /// 落盘占每刀 90%，其中 blind 重算占 79%（现已标脏化）。
    fn write_parts(&mut self, parts: Vec<CutPart<S>>, stats: &mut CutStats) {
        let prof = prof_enabled();
        let t0 = std::time::Instant::now();
        let mut wc = crate::tree::WriteCache::default();
        // **三个缓冲都按块数上限一次要够**。
        //
        // 这里栽过一次：原来写 `Vec::new()`，铣光一刀 4668 块 × 24 字节从 0 翻倍
        // 增长到 112 KB，中途要搬几百 KB；`touched` 5248 条同理。实测
        // `collect` 阶段 505 µs，占整个落盘的 48%——**比真正写叶还贵**。
        // 这与"中转缓冲不预留导致 realloc 占 61 ms/200 刀"是同一类错误，
        // 上一轮修了 `pool` 却漏了这里。
        let n_blocks: usize = parts.iter().map(|p| p.blocks.len() + p.whole.len()).sum();
        // (叶索引, 512 个新值)：攒齐后按索引分段**并行**写
        let mut blocks_par: Vec<(u32, &[S])> = Vec::with_capacity(n_blocks);
        // 写过的叶（**按下标**，不再按坐标回查）/ 它们所属的一级节点：
        // 落盘后只 prune 这些（全量扫在 50×50×100 上是 10 ms/刀 量级）
        let mut touched: Vec<u32> = Vec::with_capacity(n_blocks);
        // 一级节点**不去重地攒**，末尾统一 `sort_unstable + dedup`。
        //
        // 原来每个块都 `if !l1s.contains(&l1)` —— O(n) 线性扫描套在外层循环里，
        // 而末尾本来就要排一次序去一次重，等于同一个判据做了两遍。
        // 实测铣光一刀 4668 块里 l1 只有 2 个，重复率 99.96%，扫描纯属白费。
        let mut l1s: Vec<usize> = Vec::with_capacity(n_blocks);

        // 借用 `parts` 而不是 move：`blocks_par` 里存着 `pool` 的切片，
        // 必须活到并行写完成之后。
        for part in &parts {
            stats.visited += part.visited;
        }

        // ① 整叶短路（串行：一次填 512 个体素，且可能新建叶）
        for part in &parts {
            for &(leaf, v) in &part.whole {
                let n = self.fill_leaf(leaf, v, &mut wc);
                stats.changed += n;
                if n == 0 {
                    continue; // 已经是这个值：没有脏叶可 prune
                }
                if let Some(li) = wc.leaf_index() {
                    touched.push(li);
                    // 整叶填成 +band（空气）不造面 → 不盖章
                }
                l1s.push(wc.l1());
            }
        }
        let t_whole = t0.elapsed();

        // ② 整块：叶已存在的攒起来并行；要新建的当场串行（不能并发 push）
        let t1 = std::time::Instant::now();
        for part in &parts {
            for block in &part.blocks {
                let vals = &part.pool[block.offset..block.offset + LEAF_VOXELS];
                if prof_enabled() {
                    use std::sync::atomic::Ordering::Relaxed;
                    PROF_N_BLOCK.fetch_add(1, Relaxed);
                }
                stats.changed += block.n as usize;
                // **n == 0 的块直接丢掉**：一个体素都没变，说明这一刀没有
                // 去除材料，不能改任何 creator，也不该白写 512 个相同的字节。
                //
                // 深腔短路路径（`cut_box` 里"整叶埋进扫掠体 → 整叶填 +band"）
                // 不检查 `n` 就把块推了进来，这里是唯一的兜底。
                if block.n == 0 && block.mask_off == u32::MAX {
                    continue;
                }
                match block.slot {
                    Some((li, l1i)) => {
                        touched.push(li);
                        if block.mask_off != u32::MAX {
                            let words = self.creator_words();
                            let mo = block.mask_off as usize;
                            self.stamp_leaf(li as usize, &part.masks[mo..mo + words]);
                        }
                        blocks_par.push((li, vals));
                        l1s.push(l1i as usize);
                    }
                    None => {
                        if prof_enabled() {
                            use std::sync::atomic::Ordering::Relaxed;
                            PROF_NEW_LEAF.fetch_add(1, Relaxed);
                        }
                        self.blit_leaf(block.leaf, vals, &mut wc);
                        if let Some(li) = wc.leaf_index() {
                            touched.push(li);
                            if block.mask_off != u32::MAX {
                            let words = self.creator_words();
                            let mo = block.mask_off as usize;
                            self.stamp_leaf(li as usize, &part.masks[mo..mo + words]);
                        }
                        }
                        l1s.push(wc.l1());
                    }
                }
            }
        }
        let t_collect = t1.elapsed();

        // **无锁并行落盘**：按叶索引排序 → 切成互不重叠的段 → 每线程写自己那段。
        // 没有锁、原子操作或 CAS 重试；blind 元数据留给 `refresh_blind` 统一算。
        let t2 = std::time::Instant::now();
        if !blocks_par.is_empty() {
            blocks_par.sort_unstable_by_key(|it| it.0);
        }
        let t_sort = t2.elapsed();
        let t3 = std::time::Instant::now();
        if !blocks_par.is_empty() {
            self.blit_leaves_parallel(&blocks_par);
        }
        let t_blit = t3.elapsed();

        let t4 = std::time::Instant::now();
        if stats.changed > 0 {
            touched.sort_unstable();
            touched.dedup();
            l1s.sort_unstable();
            l1s.dedup();
            self.prune_leaves(&touched, &l1s);
        }
        let t_prune = t4.elapsed();

        if prof {
            use std::sync::atomic::Ordering::Relaxed;
            for (i, d) in [
                t_whole,
                t_collect,
                t_sort,
                t_blit,
                t_prune,
            ]
            .iter()
            .enumerate()
            {
                PROF_W[i].fetch_add(d.as_nanos() as u64, Relaxed);
            }
        }
    }

    /// 求值一个子块（**只读**，因此可以并行）。写回由调用方串行/并行完成。
    ///
    /// **按叶遍历**：先把"这一段能碰到的叶"筛出来（AABB 相交 + 1-Lipschitz 剔除），
    /// 再逐叶处理。批量时一个叶只取一次、一次性套完所有段。
    fn cut_box(
        &self,
        lo: [i32; 3],
        hi: [i32; 3],
        segs: &[Seg],
        profile: &Profile,
        band: f64,
    ) -> CutPart<S> {
        let mut part = CutPart::empty();

        let l0 = [lo[0] >> LEAF_LOG2, lo[1] >> LEAF_LOG2, lo[2] >> LEAF_LOG2];
        let l1 = [
            (hi[0] + LEAF_DIM - 1) >> LEAF_LOG2,
            (hi[1] + LEAF_DIM - 1) >> LEAF_LOG2,
            (hi[2] + LEAF_DIM - 1) >> LEAF_LOG2,
        ];

        // **一次把中转缓冲按上限要够**。不预留的话 `pool` 从 0 翻倍增长到
        // 300 KB，中途要搬 600 KB：铣光实测 200 刀合计 **61 ms 花在 realloc**，
        // 比真正写叶还多。上限就是 AABB 里的叶数（每叶最多一份 512 个值）。
        let box_vox = ((hi[0] - lo[0]) as usize)
            * ((hi[1] - lo[1]) as usize)
            * ((hi[2] - lo[2]) as usize);
        let max_leaves = box_vox / LEAF_VOXELS + 1;
        part.pool.reserve(max_leaves * LEAF_VOXELS);
        part.blocks.reserve(max_leaves);
        part.whole.reserve(max_leaves / 8 + 1);

        // 叶的半对角线：叶内任一点到叶心的距离上界。叶级剔除的松紧全靠它。
        let half_diag = 0.5 * LEAF_DIM as f64 * self.config.h * 3.0f64.sqrt();

        // 候选段下标（复用，不每叶分配）
        let mut cand: Vec<usize> = Vec::with_capacity(segs.len().max(8));

        for lz in l0[2]..l1[2] {
            for ly in l0[1]..l1[1] {
                for lx in l0[0]..l1[0] {
                    let bl = [lx << LEAF_LOG2, ly << LEAF_LOG2, lz << LEAF_LOG2];
                    let bh = [bl[0] + LEAF_DIM, bl[1] + LEAF_DIM, bl[2] + LEAF_DIM];
                    let a = [bl[0].max(lo[0]), bl[1].max(lo[1]), bl[2].max(lo[2])];
                    let b = [bh[0].min(hi[0]), bh[1].min(hi[1]), bh[2].min(hi[2])];
                    if a[0] >= b[0] || a[1] >= b[1] || a[2] >= b[2] {
                        continue;
                    }

                    // ── 叶级剔除（1-Lipschitz 上界）──
                    // |sd(p) − sd(center)| ≤ |p − center| ≤ half_diag，所以中心距离
                    // > band + half_diag 就能断定整叶在窄带之外，这一刀碰不到。
                    // 批量时只留下"真能碰到"的那几段——铣光实测一个叶平均 2 段。
                    let center = self.leaf_center(bl);
                    cand.clear();
                    for (si, s) in segs.iter().enumerate() {
                        if !s.touches(bl, bh) {
                            continue;
                        }
                        if s.sd(center, profile) > band + half_diag {
                            continue;
                        }
                        cand.push(si);
                    }
                    if cand.is_empty() {
                        continue;
                    }

                    // 整叶同值（tile / Constant）：刀切不到就整叶跳过，切得到就直接当初始值
                    let (view, slot) = self.leaf_view_with_slot(bl);
                    let slot = slot.map(|(li, l1i)| (li as u32, l1i as u32));
                    let uniform = match &view {
                        None => Some(self.background),
                        Some(LeafView::Tile(v)) => Some(*v),
                        Some(LeafView::Leaf(Leaf::Constant(v))) => Some(*v),
                        Some(LeafView::Leaf(Leaf::Dense(_))) => None,
                    };
                    if let Some(v) = uniform
                        && self.to_distance(v) >= band
                    {
                        continue;
                    }

                    // 整叶短路（凸性）：SDF 是凸函数，凸函数在立方体上的最大值取在
                    // 顶点——8 个角都 < −band ⟹ 整叶都深 ⟹ 整叶饱和成空气。
                    // 批量里只要**有一段**把整叶埋进去就够了：那一刀之后这片全是
                    // 空气，`max` 只会让它更空，后续段不可能再改回来。
                    //
                    // **只对完整叶做**：短路是"整叶填同一个值"，被 AABB 切开的残叶
                    // 整块填会把 AABB 外的体素也写脏。
                    //
                    // **有下标的叶走整块通道**：512 个相同值交给 `write_leaf_block`
                    //（它识别全同值，落成 Constant 叶，不碰分配器），于是这个叶的
                    // 写入混进并行的 blit 队列，不再占串行的 `fill_leaf`。只有
                    // "槽位还是 tile / 路径不存在"的叶才留给串行落盘创建。
                    if a == bl && b == bh
                        && cand.iter().any(|&si| self.leaf_deep_inside(bl, bh, &segs[si], profile, band))
                    {
                        // 深于 band 的空气 = 正的饱和值。写成 −band 会把材料留住
                        //（测试 `cut_makes_slot_of_the_right_width_and_depth` 当场抓到）。
                        let v = self.to_value(band);
                        match slot {
                            Some((li, l1i)) => {
                                // 真正变了的体素数：Dense 叶里可能已有饱和体素
                                let n = match &view {
                                    Some(LeafView::Leaf(Leaf::Dense(arr))) => {
                                        arr.iter().filter(|&&x| x != v).count()
                                    }
                                    _ => LEAF_VOXELS,
                                };
                                let offset = part.pool.len();
                                part.pool.extend_from_slice(&[v; LEAF_VOXELS]);
                                part.blocks.push(LeafBlock {
                                    leaf: [lx, ly, lz],
                                    slot: Some((li, l1i)),
                                    offset,
                                    n: n.min(u16::MAX as usize) as u16,
                                    // 整叶填成 +band（空气）→ 新值离面 0.4 mm，
                                    // 按"变了且贴面"的判据**一个子块都不该盖**：
                                    // 这一刀没有在这里造出面（它把面切掉了）。
                                    // `words = 0`：位图占 0 个字，盖章时什么都不写。
                                    mask_off: u32::MAX,
                                });
                            }
                            None => part.whole.push(([lx, ly, lz], v)),
                        }
                        part.visited += LEAF_VOXELS;
                        continue;
                    }

                    // 本地缓冲放在**栈上**：循环里的 `vals[idx]` 就是一次直接寻址。
                    //
                    // 试过 `Option<Box<[S; 512]>>` 延迟分配（想省掉"没改动的叶"的分配），
                    // 结果更慢——`buf` 在循环内可能被赋值，编译器无法把"是否已分配"的判断
                    // 提出循环，每体素多一次分支：空刀从 2.2 涨到 5.3 ns/体素。
                    // 栈数组的初始化（一次 512 字节 copy）远比循环里的分支便宜。
                    let mut vals: [S; LEAF_VOXELS] = match &view {
                        Some(LeafView::Leaf(Leaf::Dense(arr))) => **arr,
                        _ => [uniform.unwrap_or(self.background); LEAF_VOXELS],
                    };

                    // 逐段套用：同一个叶缓冲，段与段之间不出栈、不回树里找。
                    let mut visited = 0usize;
                    for &si in &cand {
                        let s = &segs[si];
                        let aa = [
                            bl[0].max(s.lo[0]).max(lo[0]),
                            bl[1].max(s.lo[1]).max(lo[1]),
                            bl[2].max(s.lo[2]).max(lo[2]),
                        ];
                        let bb = [
                            bh[0].min(s.hi[0]).min(hi[0]),
                            bh[1].min(s.hi[1]).min(hi[1]),
                            bh[2].min(s.hi[2]).min(hi[2]),
                        ];
                        if aa[0] >= bb[0] || aa[1] >= bb[1] || aa[2] >= bb[2] {
                            continue;
                        }
                        visited += self.eval_into(&mut vals, aa, bb, s, profile, band);
                    }
                    part.visited += visited;

                    // **改动数在叶落地时比一次 512 字节**，不在逐点循环里累加。
                    // 深腔里 75% 的体素走"整行填空气"，那里每体素多一条
                    // load+compare+add 就是纯亏；而这里编译器会把 512 字节
                    // 比成 8 条 64 字节的 AVX2 比较。
                    let (n, mask, words) = self.changed_mask(&vals, &view);
                    if n > 0 {
                        let offset = part.pool.len();
                        part.pool.extend_from_slice(&vals);
                        // `words == 0` 表示这一趟没有要求溯源（`pending_creator == 0`），
                        // 位图一个字节都不写；用 `u32::MAX` 标成"没有位图"。
                        let mask_off = if words == 0 {
                            u32::MAX
                        } else {
                            let o = part.masks.len() as u32;
                            part.masks.extend_from_slice(&mask[..words]);
                            o
                        };
                        part.blocks.push(LeafBlock {
                            leaf: [lx, ly, lz],
                            slot,
                            offset,
                            n: n.min(u16::MAX as usize) as u16,
                            mask_off,
                        });
                    }
                }
            }
        }
        part
    }

    /// 叶缓冲里真正变了值的体素数。
    /// 改动过的子块位图 + 改动体素数。
    ///
    /// `sub = 1` 时走原来的 `count_changed`（SIMD 友好），位图只置第 0 位；
    /// `sub > 1` 时按子块分块比较——块内整块比，比逐体素比快得多
    /// （`sub = 2` 是 8 次 64 字节比较，`sub = 8` 退化成 512 次单字节比较）。
    fn changed_mask(
        &self,
        vals: &[S; LEAF_VOXELS],
        view: &Option<LeafView<'_, S>>,
    ) -> (usize, [u64; 8], usize) {
        // **没要求溯源就整个跳过**：位图只服务于盖章，不盖章时算它是白花钱。
        // 铣光 200 刀基准里没有 `set_creator`，少了这条会慢 25%（48 → 60 ms）。
        if self.pending_creator == 0 {
            return (self.count_changed(vals, view), [0u64; 8], 0);
        }
        let sub = self.creator_sub as usize;
        // **只有"这一刀真的搬动了这一格的材料"才继承颜色**：
        // 场值必须变了（CSG 是 max，移除材料 ⟺ 值变大），且新值离切面不到一格。
        //
        // 不能要求"材料 → 空气"：留量 0.02–0.05 mm 的精加工只会把表面格从
        // −0.14 抬到 −0.01，格心仍在材料侧，但那一刀确实把面削出来了。
        // 早先按"跨零"判，赤道一圈就成了 9/11 的椒盐（实测 −14 → −1 被丢掉）。
        // 反过来，只擦过、没去料的刀（值不变，`max` 不动）仍然保留原色。
        let h = self.config.h;
        if sub == 1 {
            let n = self.count_changed(vals, view);
            let hit = self.any_changed_near_surface(vals, view, h);
            return (n, if hit { [1, 0, 0, 0, 0, 0, 0, 0] } else { [0; 8] }, 1);
        }
        let old = match view {
            Some(LeafView::Leaf(Leaf::Dense(a))) => a.as_slice(),
            // 整叶同值：和常量比，逐体素
            other => {
                let v = match other {
                    Some(LeafView::Leaf(Leaf::Constant(v))) => *v,
                    Some(LeafView::Tile(v)) => *v,
                    _ => self.background,
                };
                let mut n = 0usize;
                let mut mask = [0u64; 8];
                let words = (sub * sub * sub).div_ceil(64);
                let bits = sub.trailing_zeros();
                let shift = 3 - bits;
                for (i, &new) in vals.iter().enumerate() {
                    // `n` 必须数**全部**改动：它决定这个块要不要写回几何。
                    let changed = new != v;
                    if changed {
                        n += 1;
                    }
                    // 盖章只认"这一刀真的搬动了这一格"：值变了、且新面仍在一格内。
                    let stamp = changed && self.to_distance(new).abs() < h;
                    if stamp {
                        let (x, y, z) = (i & 7, (i >> 3) & 7, i >> 6);
                        let s = ((x >> shift) & (sub - 1))
                            | (((y >> shift) & (sub - 1)) << bits)
                            | (((z >> shift) & (sub - 1)) << (2 * bits));
                        mask[s / 64] |= 1u64 << (s % 64);
                    }
                }
                return (n, mask, words);
            }
        };
        // Dense：**一次连续扫描**。
        //
        // 原来写成 `for sz/sy/sx { for dz/dy/dx }` 的四层嵌套，下标全靠
        // `x | y<<3 | z<<6` 现算——铣光 200 刀实测比"不盖色的旧路径"慢 23%
        // （36.8 → 45.4 ms）。连续扫描省掉下标的乘加，`to_distance` 也只对
        // 真正变了的体素做（绝大多数体素没变，直接一次整数比较就跳过）。
        let bits = sub.trailing_zeros();
        let shift = 3 - bits;
        let m = sub - 1;
        let words = (sub * sub * sub).div_ceil(64);
        let mut n = 0usize;
        let mut mask = [0u64; 8];
        for (i, (&nv, &ov)) in vals.iter().zip(old).enumerate() {
            let changed = nv != ov;
            if changed {
                n += 1;
            }
            let stamp = changed && self.to_distance(nv).abs() < h;
            if stamp {
                let x = (i & 7) >> shift;
                let y = ((i >> 3) & 7) >> shift;
                let z = (i >> 6) >> shift;
                let s = (x & m) | ((y & m) << bits) | ((z & m) << (2 * bits));
                mask[s / 64] |= 1u64 << (s % 64);
            }
        }
        (n, mask, words)
    }

    /// 叶里有没有「这一刀真的搬动了、且新面仍在一格内」的体素（整叶粒度用）。
    fn any_changed_near_surface(
        &self,
        vals: &[S; LEAF_VOXELS],
        view: &Option<LeafView<'_, S>>,
        h: f64,
    ) -> bool {
        let old: &[S] = match view {
            Some(LeafView::Leaf(Leaf::Dense(a))) => a.as_slice(),
            _ => return true, // 整叶同值：保守地认为有（下面会被 n 的判据兜住）
        };
        for i in 0..LEAF_VOXELS {
            if vals[i] != old[i] && self.to_distance(vals[i]).abs() < h {
                return true;
            }
        }
        false
    }

    fn count_changed(&self, vals: &[S; LEAF_VOXELS], view: &Option<LeafView<'_, S>>) -> usize {
        match view {
            Some(LeafView::Leaf(Leaf::Dense(arr))) => {
                let mut n = 0usize;
                for i in 0..LEAF_VOXELS {
                    n += (vals[i] != arr[i]) as usize;
                }
                n
            }
            // 三种"整叶同值"的情形都走同一个 8 字节一组计数
            //（`filter().count()` 在 [i8; 512] 上不会被向量化，见 `count_ne`）
            Some(LeafView::Leaf(Leaf::Constant(v))) => crate::tree::count_ne(vals, *v),
            Some(LeafView::Tile(v)) => crate::tree::count_ne(vals, *v),
            None => crate::tree::count_ne(vals, self.background),
        }
    }

    /// 把一段扫掠的 CSG 应用到叶缓冲的 `a..b` 子块（**只读网格，可并行**），
    /// 返回参与求值的体素数。
    ///
    /// 三种行命运（见模块文档）：够不着 → 整行跳过；埋在刀体深处 → 整行填；
    /// 表面行 → 逐点量化。批量时这个函数会被同一个叶连续调用 k 次，
    /// `vals` 一直待在 L1 里。
    ///
    /// 坐标用**增量**而不是逐点 `origin + (ijk+0.5)h`：省掉每体素 3 次乘法。
    #[inline]
    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::explicit_counter_loop)] // idx 与 px/py/pz 是两个独立的游标
    fn eval_into(
        &self,
        vals: &mut [S; LEAF_VOXELS],
        a: [i32; 3],
        b: [i32; 3],
        seg: &Seg,
        profile: &Profile,
        band: f64,
    ) -> usize {
        let h = self.config.h;
        let mut visited = 0usize;

        // **量化换算的两个系数提到循环外**。
        //
        // `to_distance` / `to_value` 每次都调 `band_mm()`（一次乘）再做一次除法；
        // 逐体素两次调用 + 一次除法，VTune 上表现为 `f64::min`/`max` 占 19%
        // （`from_f64_round` 里的 `clamp`）。这里换成两次乘法。
        let band_mm = self.config.band_mm();
        let k_dist = band_mm / S::MAX_MAG; // 存储值 → 距离
        let k_value = S::MAX_MAG / band_mm; // 距离 → 存储值

        // 求值范围（叶内与 AABB 求交后的块）与它的世界坐标起点
        let nx = (b[0] - a[0]) as usize;
        let ny = (b[1] - a[1]) as usize;
        let nz = (b[2] - a[2]) as usize;
        let (i0, j0, k0) = (
            (a[0] - (a[0] >> LEAF_LOG2 << LEAF_LOG2)) as usize,
            (a[1] - (a[1] >> LEAF_LOG2 << LEAF_LOG2)) as usize,
            (a[2] - (a[2] >> LEAF_LOG2 << LEAF_LOG2)) as usize,
        );
        let x0 = self.origin.x + (a[0] as f64 + 0.5) * h;
        let y0 = self.origin.y + (a[1] as f64 + 0.5) * h;
        let z0 = self.origin.z + (a[2] as f64 + 0.5) * h;

        // **叶整个落在走刀段内** → 沿走刀方向 SDF 不变，截面算一次、沿轴复用 8 次。
        // 立铣刀的正交直刀（沿 x 或 y、刀轴竖直）都满足；斜向进给、倾斜刀轴、
        // 沿 z 下刀都自动回退到通用路径。
        let share = seg.sweep.filter(|&(ax, lo, hi)| {
            let u = self.origin.component(ax) + a[ax] as f64 * h;
            let v = self.origin.component(ax) + b[ax] as f64 * h;
            u >= lo - 1e-9 && v <= hi + 1e-9
        });

        if let Some((ax, _, _)) = share {
            // **内联无分支的 SDF 公式**（叶已在走刀段内）。
            //
            // `sd_flat_prism` 里 `origin_to_segment` 的 `t.clamp(0,1)` 在叶位于段内时
            // 恒不生效——径向距离直接就是 `|radial − tip_radial|`。把这层 clamp 和
            // `sd_sweep_with` 的一串条件判断（`axis_delta_sq` / `step_len2` / `sz.abs()`）
            // 都省掉，剩下的纯算术更容易被向量化。
            //
            // 只在**平底刀 + 刀轴正 Z** 时启用，其它姿态回退通用路径。
            let simple = match seg.flat {
                Some((r, len)) if seg.from.axis.z > 0.0 => Some((r, len)),
                _ => None,
            };
            // 走刀轴上的刀心坐标（径向基准）
            let tip_radial = if ax == 0 { seg.from.tip.y } else { seg.from.tip.x };
            // `(radial, pz)` 截面上的扫掠体距离
            let sd_at = |radial: f64, pz: f64| -> f64 {
                match simple {
                    Some((r, len)) => {
                        let dr = (radial - tip_radial).abs() - r;
                        let zw = pz - seg.from.tip.z;
                        let dz = (-zw).max(zw - len);
                        let (a, b) = (dr.max(0.0), dz.max(0.0));
                        (a * a + b * b).sqrt() + dr.max(dz).min(0.0)
                    }
                    None => {
                        let p = if ax == 0 {
                            DVec3::new(x0, radial, pz)
                        } else {
                            DVec3::new(radial, y0, pz)
                        };
                        sd_sweep_with(p, &seg.from, &seg.to, profile, seg.flat)
                    }
                }
            };

            // 沿 x 走刀：每个 (y, z) 截面算一次，内层 8 个 x 复用
            if ax == 0 {
                // 沿 x 走刀：每个 (y, z) 截面算一次，内层 8 个 x 复用
                for k in 0..nz {
                    let pz = z0 + k as f64 * h;
                    for j in 0..ny {
                        let sd = sd_at(y0 + j as f64 * h, pz);
                        // 刀够不着这一整行：8 个体素全在窄带外侧，一个字节都不用碰
                        if sd >= band {
                            continue;
                        }
                        let base = ((k0 + k) << (LEAF_LOG2 * 2)) | ((j0 + j) << LEAF_LOG2) | i0;
                        let row = &mut vals[base..base + nx];
                        visited += nx;
                        // 整行埋在刀体深处：一次 8 字节存储（对已是 MAX 的体素幂等）
                        if -sd >= band {
                            row.fill(S::MAX);
                            continue;
                        }
                        // 表面行：**新值是行常量**（`-sd > d_cur` 时结果就是 `round(-sd)`），
                        // 逐点只需要一次比较 + 一次存储，`round` 每行算一次而不是每体素一次。
                        let nv = S::from_f64_round(-sd * k_value);
                        let neg_sd = -sd;
                        for v in row.iter_mut() {
                            // d_cur < −sd ⟹ 这一刀把它推到 −sd 处；否则 max 取 d_cur 本身，
                            // 量化回来还是原值（round(cur·k_dist·k_value) = cur）
                            if (*v).to_f64() * k_dist < neg_sd {
                                *v = nv;
                            }
                        }
                    }
                }
            } else {
                // 沿 y 走刀：同一 (x, z) 的 SDF 在该列所有 y 上相同。
                // 先按层算 8 个 x 的 SDF + 每列的新值，再整层复用。
                let mut sd_row = [0.0f64; LEAF_DIM as usize];
                let mut nv_row = [S::MAX; LEAF_DIM as usize];
                for k in 0..nz {
                    let pz = z0 + k as f64 * h;
                    let mut all_deep = true;
                    let mut all_skip = true;
                    for i in 0..nx {
                        // 沿 y 走刀：径向是 x，轴向是 z——每个 x 一条独立的 sd
                        let sd = sd_at(x0 + i as f64 * h, pz);
                        sd_row[i] = sd;
                        // 表面处的新值是列常量：`round(-sd)`，每列算一次
                        nv_row[i] = if -sd >= band {
                            S::MAX
                        } else {
                            S::from_f64_round(-sd * k_value)
                        };
                        all_deep &= -sd >= band;
                        all_skip &= sd >= band;
                    }
                    if all_skip {
                        continue;
                    }
                    for j in 0..ny {
                        let base =
                            ((k0 + k) << (LEAF_LOG2 * 2)) | ((j0 + j) << LEAF_LOG2) | i0;
                        let row = &mut vals[base..base + nx];
                        visited += nx;
                        if all_deep {
                            row.fill(S::MAX);
                            continue;
                        }
                        for (i, v) in row.iter_mut().enumerate() {
                            let sd = sd_row[i];
                            if sd >= band {
                                continue; // 刀够不着这一列
                            }
                            let neg_sd = -sd;
                            // 埋在刀体深处（新值必然饱和成 MAX）或表面处推得动
                            if neg_sd >= band || (*v).to_f64() * k_dist < neg_sd {
                                *v = nv_row[i];
                            }
                        }
                    }
                }
            }
        } else {
            // ── 通用路径：逐体素求值 ──
            // 叶内布局是 x | y<<3 | z<<6（x 最内层）→ vals 是顺序写的
            //
            // 平底刀特化（[`FlatSweep`]）在段预处理时已备好：短段走刀时
            // 单轴共享够不着（叶不在刀心路径正上方），这里就是主战场。
            if let Some(f) = seg.fast {
                let mut pz = z0;
                for k in 0..nz {
                    let mut py = y0;
                    for j in 0..ny {
                        let mut px = x0;
                        let mut idx =
                            ((k0 + k) << (LEAF_LOG2 * 2)) | ((j0 + j) << LEAF_LOG2) | i0;
                        for _ in 0..nx {
                            let cur = vals[idx];
                            // 已经是饱和空气：任何一刀都只会让它更空，等于没变
                            if cur != S::MAX {
                                visited += 1;
                                let sd = f.sd(px, py, pz);
                                vals[idx] = if -sd >= band {
                                    S::MAX
                                } else {
                                    S::from_f64_round((cur.to_f64() * k_dist).max(-sd) * k_value)
                                };
                            }
                            idx += 1;
                            px += h;
                        }
                        py += h;
                    }
                    pz += h;
                }
                return visited;
            }

            let mut pz = z0;
            for k in 0..nz {
                let mut py = y0;
                for j in 0..ny {
                    let mut px = x0;
                    let mut idx = ((k0 + k) << (LEAF_LOG2 * 2)) | ((j0 + j) << LEAF_LOG2) | i0;
                    for _ in 0..nx {
                        let cur = vals[idx];
                        // 已经是饱和空气：任何一刀都只会让它更空，等于没变
                        if cur != S::MAX {
                            visited += 1;
                            let d_cur = cur.to_f64() * k_dist;
                            let d_sweep = sd_sweep_with(
                                DVec3::new(px, py, pz),
                                &seg.from,
                                &seg.to,
                                profile,
                                seg.flat,
                            );
                            vals[idx] = if -d_sweep >= band {
                                S::MAX
                            } else {
                                S::from_f64_round(d_cur.max(-d_sweep) * k_value)
                            };
                        }
                        idx += 1;
                        px += h;
                    }
                    py += h;
                }
                pz += h;
            }
        }
        visited
    }

    /// 叶的几何中心（世界坐标）。参数是叶的**起始体素索引**（`leaf << 3`）。
    fn leaf_center(&self, bl: [i32; 3]) -> DVec3 {
        let h = self.config.h;
        let half = 0.5 * LEAF_DIM as f64 * h;
        DVec3::new(
            self.origin.x + bl[0] as f64 * h + half,
            self.origin.y + bl[1] as f64 * h + half,
            self.origin.z + bl[2] as f64 * h + half,
        )
    }

    /// 叶的 8 个角（体素中心）是否都落在扫掠体内部且深度超过一个窄带。
    fn leaf_deep_inside(
        &self,
        bl: [i32; 3],
        bh: [i32; 3],
        seg: &Seg,
        profile: &Profile,
        band: f64,
    ) -> bool {
        for &z in &[bl[2], bh[2] - 1] {
            for &y in &[bl[1], bh[1] - 1] {
                for &x in &[bl[0], bh[0] - 1] {
                    let sd = seg.sd(self.voxel_center([x, y, z]), profile);
                    if sd > -band {
                        return false;
                    }
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cncsim_motion::Pose;
    use cncsim_tool::flat_end_mill;

    #[test]
    fn sweep_aabb_covers_tool_at_both_ends() {
        let prof = flat_end_mill(6.0, 20.0);
        let a = Pose {
            tip: DVec3::new(0.0, 5.0, 3.0),
            axis: DVec3::Z,
        };
        let b = Pose {
            tip: DVec3::new(10.0, 5.0, 3.0),
            axis: DVec3::Z,
        };
        let (lo, hi) = sweep_aabb(&a, &b, &prof);
        assert!((lo.x - (-3.0)).abs() < 1e-12, "x 下界 {lo:?}");
        assert!((hi.x - 13.0).abs() < 1e-12, "x 上界 {hi:?}");
        assert!((lo.y - 2.0).abs() < 1e-12);
        assert!((hi.y - 8.0).abs() < 1e-12);
        assert!((lo.z - 3.0).abs() < 1e-12, "刀尖在 z=3，下界就是 3");
        assert!((hi.z - 23.0).abs() < 1e-12, "刀长 20，上界 23");
    }

    /// 窄带必须**双边完整**：槽底以下 0.15 mm 的存储值应是真实距离 `−0.15`，
    /// 而不是切前的饱和值 `−band`。
    ///
    /// 少了这一段，任何跨过切面的插值都会被饱和值拽偏——显示 LOD 在 k=4 时
    /// 槽底偏 97 µm（这条断言就是它的最小复现）。
    #[test]
    fn band_is_complete_below_the_cut_surface() {
        let mut g = SparseGrid::<i8>::new(crate::GridConfig::default(), DVec3::ZERO);
        g.fill_box([0, 0, 0], [120, 120, 80], -127);
        let prof = flat_end_mill(6.0, 20.0);
        g.cut_segment(
            &Pose { tip: DVec3::new(0.0, 6.0, 7.0), axis: DVec3::Z },
            &Pose { tip: DVec3::new(12.0, 6.0, 7.0), axis: DVec3::Z },
            &prof,
        );
        // 槽底 z=7；体素中心 6.85 在槽底以下 0.15（band=0.4 之内）
        let d = g.sdf(DVec3::new(6.0, 6.0, 6.85));
        assert!(
            (d + 0.15).abs() < 5e-3,
            "槽底以下 0.15 mm 处应为 −0.150，实际 {d:+.4}（饱和值 −0.4 说明 AABB 没外扩 band）"
        );
        // 再深一点（0.35 mm）仍在带内
        let d = g.sdf(DVec3::new(6.0, 6.0, 6.65));
        assert!((d + 0.35).abs() < 5e-3, "槽底以下 0.35 mm 处应为 −0.350，实际 {d:+.4}");
    }

    /// 深腔（切深 5 mm >> 叶高 0.8 mm）会触发**整叶短路**：
    /// 整个 8³ 叶落在扫掠体深处时，一次填充代替 512 次求值+写入。
    /// 这里验证短路之后几何仍然精确——槽底、槽壁、槽外材料一个都不能错。
    #[test]
    fn deep_pocket_shortcut_keeps_the_slot_geometry() {
        let mut g = SparseGrid::<i8>::new(crate::GridConfig::default(), DVec3::ZERO);
        // 10×10×10 mm 实心毛坯
        g.fill_box([0, 0, 0], [100, 100, 100], -127);
        let prof = flat_end_mill(6.0, 20.0);
        // 刀尖 z=5：从顶面 10 切到 5，深 5 mm（跨 6 个叶高）
        let st = g.cut_segment(
            &Pose { tip: DVec3::new(2.0, 5.0, 5.0), axis: DVec3::Z },
            &Pose { tip: DVec3::new(8.0, 5.0, 5.0), axis: DVec3::Z },
            &prof,
        );
        assert!(st.changed > 0);

        // 槽底 z=5：体素中心 4.95 在带内 → −0.05
        let d = g.sdf(DVec3::new(5.0, 5.0, 4.95));
        assert!((d + 0.05).abs() < 6e-3, "槽底应在 z=5：{d:+.4}");

        // 深腔内部（z=6）完全切空 → 正的饱和值
        let d = g.sdf(DVec3::new(5.0, 5.0, 6.0));
        assert!(d > 0.3, "深腔内部应是空气：{d:+.4}");

        // 槽壁 y=2（刀半径 3、刀心 y=5）
        let d = g.sdf(DVec3::new(5.0, 2.0, 6.0));
        assert!(d.abs() < 0.02, "槽壁应在 y=2：{d:+.4}");

        // 槽外材料一动不动
        let d = g.sdf(DVec3::new(5.0, 0.5, 6.0));
        assert!(d < -0.1, "槽外应还是材料：{d:+.4}");
    }

    /// 微基准：`cut_segment` 全流程（含求值 / 并行落盘 / prune），
    /// 以及 `cut_batch` 同一批的耗时对照。
    ///
    /// ```bash
    /// cargo test --release -p cncsim-voxel --lib bench_cut -- --ignored --nocapture
    /// CNCSIM_THREADS=1 cargo test --release -p cncsim-voxel --lib bench_cut -- --ignored --nocapture
    /// ```
    ///
    /// 每个刀长跑两遍取第二遍：第一遍含冷启动与叶创建（首切要把 tile 展开成叶）。
    #[test]
    #[ignore = "微基准，手动跑"]
    fn bench_cut() {
        use crate::par;
        let prof = flat_end_mill(6.0, 20.0);
        let mut g = SparseGrid::<i8>::new(crate::GridConfig::default(), DVec3::ZERO);
        g.fill_box([0, 0, 0], [300, 300, 120], -127);

        println!(
            "\n【cut_segment 全流程】30×30×12 毛坯 h=0.1 φ6 平底刀 切深 0.5mm，{} 线程",
            par::worker_count()
        );
        println!("{:<10} {:>10} {:>10} {:>10}", "刀长", "合计 ms", "遍历", "ns/体素");

        // 先跑一刀小切削把线程池和叶创建的热身做掉（y=0.5，与后面四刀都不重叠）
        let _ = g.cut_segment(
            &Pose { tip: DVec3::new(2.0, 0.5, 11.0), axis: DVec3::Z },
            &Pose { tip: DVec3::new(3.0, 0.5, 11.0), axis: DVec3::Z },
            &prof,
        );

        for (len, x0, y) in [
            (1.0, 2.0, 3.0),
            (10.0, 2.0, 10.0),
            (20.0, 2.0, 15.0),
            (28.0, 1.0, 22.0),
        ] {
            let from = Pose { tip: DVec3::new(x0, y, 11.0), axis: DVec3::Z };
            let to = Pose { tip: DVec3::new(x0 + len, y, 11.0), axis: DVec3::Z };
            // **不预热这一刀**：四个刀长各在不同 y，都是首切。
            // （预热同一段会让第二遍变成"重复切"，遍历数掉三成，数字虚高。）
            let t0 = std::time::Instant::now();
            let st = g.cut_segment(&from, &to, &prof);
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            println!(
                "{:<10} {:>10.2} {:>10} {:>10.1}",
                format!("{len} mm"),
                ms,
                st.visited,
                ms * 1e6 / st.visited.max(1) as f64
            );
        }
    }
}

#[cfg(test)]
mod int_threshold_tests {
    //! **反向测试**：表面行的判据"整数化"会改变结果，这里把它钉死。
    //!
    //! 表面行的原判据是浮点：`(cur as f64) * k_dist < neg_sd`。
    //! 看起来可以等价换成整数：`cur < ceil(neg_sd * k_value)`——
    //! 因为 `k_dist * k_value == 1`，两边乘 `k_value` 就得到 `cur < neg_sd * k_value`。
    //!
    //! **实测不等价。** 反例（`band = 0.8`、`max_mag = 127`）：
    //!
    //! ```text
    //! sd = -0.510236220472, cur = 81
    //! 浮点：81 * (0.8/127) = 0.51023622047...  <  0.510236220472  →  false（不改）
    //! 整数：0.510236220472 * (127/0.8) = 81.0000000001 → ceil = 82，81 < 82  →  true（要改）
    //! ```
    //!
    //! 根因是两条路径的**舍入不同**：`81 * (0.8/127)` 是"一次除法 + 一次乘法"两次舍入，
    //! 而 `0.510236220472 * (127/0.8)` 是反推回档位。在量化档边界上（`neg_sd` 恰好
    //! 落在某档附近）两者会分歧。分歧只发生在临界点上，但**逐位可复现是硬门槛**，
    //! 所以这个优化不能做。
    //!
    //! 这个测试断言"**确实存在分歧**"。哪天它变成绿（不再有分歧），说明浮点路径
    //! 本身被改了，需要重新评估这个优化是否安全。

    fn count_divergence(band_mm: f64, max_mag: f64) -> usize {
        let k_dist = band_mm / max_mag;
        let k_value = max_mag / band_mm;
        let mut sds = Vec::new();
        for i in 1..=200 {
            sds.push(-(band_mm * i as f64 / 201.0));
        }
        // 恰好落在量化档上的临界值——分歧就在这里
        for i in 1..max_mag as i32 {
            sds.push(-(i as f64) / k_value);
        }
        let mut n = 0;
        for sd in sds {
            let neg_sd = -sd;
            let thr = (neg_sd * k_value).ceil() as i32;
            for cur in -127i8..=127 {
                let by_float = (cur as f64) * k_dist < neg_sd;
                let by_int = (cur as i32) < thr;
                if by_float != by_int {
                    n += 1;
                }
            }
        }
        n
    }

    #[test]
    fn integer_threshold_diverges_from_float_at_quantization_boundaries() {
        for (band, mag) in [(0.8, 127.0), (0.4, 127.0), (2.0, 127.0)] {
            let n = count_divergence(band, mag);
            assert!(
                n > 0,
                "band={band}：整数判据与浮点判据竟然完全等价了——\
                 要么浮点路径被改了，要么这个优化其实安全，需要重新评估"
            );
        }
    }

    /// 分歧**只**发生在量化档边界上：把 `sd` 刻意放在档位中间时，两者一致。
    #[test]
    fn integer_threshold_agrees_away_from_boundaries() {
        let (band_mm, max_mag) = (0.8, 127.0);
        let k_dist = band_mm / max_mag;
        let k_value = max_mag / band_mm;
        for i in 1..200 {
            // 落在档位正中（i + 0.5），远离任何临界点
            let sd = -((i as f64) + 0.5) / k_value;
            let neg_sd = -sd;
            let thr = (neg_sd * k_value).ceil() as i32;
            for cur in -127i8..=127 {
                assert_eq!(
                    (cur as f64) * k_dist < neg_sd,
                    (cur as i32) < thr,
                    "sd={sd} cur={cur} 在档位中间不该分歧"
                );
            }
        }
    }
}
