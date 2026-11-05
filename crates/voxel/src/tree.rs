//! 稀疏三级树：`root → level2(32³) → level1(16³) → leaf(8³)`。
//!
//! 结构照搬 OpenVDB `Tree4<T,5,4,3>`（数字**从叶往根读**），见
//! `docs/ADR-006-稀疏体素数据结构.md`。
//!
//! ## 为什么每一级都要能存 tile 值
//!
//! 这是整个设计的关键，不是优化：
//!
//! | 方案 | 200×200×100 毛坯（h=0.1，2000×2000×1000 体素） |
//! |---|---|
//! | 只存叶 | 15.6M 个叶 → 无法表达 |
//! | 叶 + 叶级 tile | 仍然 15.6M 个 `Leaf::Constant` |
//! | **三级都能 tile** | 内部整块是 **1 条 root 条目或 1 个槽** |
//!
//! 一个 level-2 槽覆盖 128³ 体素，一个 level-1 槽覆盖 8³ 体素（就是一个叶）。
//! 实心毛坯的绝大部分落在"整块同值"的区域，直接被 tile 吃掉。
//!
//! ## 索引：全是位运算，无哈希
//!
//! ```text
//! ijk ──>>12──→ root 坐标（哈希，通常 1~8 条）
//!     ──>>7 &31─→ level2 槽（5 位/轴）
//!     ──>>3 &15─→ level1 槽（4 位/轴）
//!     ──&7─────→ 叶内体素（3 位/轴）
//! ```
//!
//! 子节点取法：`children[popcount(mask & 低位)]`——位测试 + 一次连续数组访问。
//!
//! ## 负坐标
//!
//! 用算术右移与按位与：`-1 >> 3 == -1`、`-1 & 7 == 7`，于是体素 `-1` 落在叶 `-1`
//! 的局部索引 7。**不要用 `/` 和 `%`**——它们在负数上向零取整，会让 `-1` 和 `1` 落进同一格。

use cncsim_math::DVec3;
use crate::fxhash::FxHashMap;
use rayon::prelude::*;
use std::fmt::Debug;

// ─────────────────────────── 常量 ───────────────────────────

/// 叶边长 = 2^3 = 8 体素/轴
pub const LEAF_LOG2: u32 = 3;
/// level-1 分支 = 2^4 = 16 槽/轴 → 覆盖 16×8 = 128 体素/轴
pub const L1_LOG2: u32 = 4;
/// level-2 分支 = 2^5 = 32 槽/轴 → 覆盖 32×128 = 4096 体素/轴
pub const L2_LOG2: u32 = 5;

pub const LEAF_DIM: i32 = 1 << LEAF_LOG2; // 8
pub const L1_DIM: i32 = 1 << L1_LOG2; // 16
pub const L2_DIM: i32 = 1 << L2_LOG2; // 32

pub const LEAF_VOXELS: usize = (LEAF_DIM * LEAF_DIM * LEAF_DIM) as usize; // 512
pub const L1_SLOTS: usize = (L1_DIM * L1_DIM * L1_DIM) as usize; // 4096
pub const L2_SLOTS: usize = (L2_DIM * L2_DIM * L2_DIM) as usize; // 32768

const L1_WORDS: usize = L1_SLOTS / 64; // 64
const L2_WORDS: usize = L2_SLOTS / 64; // 512

/// 一个 level-1 槽覆盖的体素数/轴
pub const L1_SPAN: i32 = LEAF_DIM * L1_DIM; // 128
/// 一个 level-2 槽覆盖的体素数/轴
pub const L2_SPAN: i32 = L1_SPAN * L2_DIM; // 4096
/// 一个 level-2 节点（= root 的一条）覆盖的体素数/轴
pub const ROOT_SPAN: i32 = L2_SPAN; // 4096

// ─────────────────────────── 值类型 ───────────────────────────

/// 体素值。只实现 `i8` 与 `i16`——**这是精度的旋钮，不是 float 宽度**。
///
/// `MIN` 取 `-127` / `-32767`（而不是 `-128` / `-32768`）：
/// 量化是对称的，用对称量程才能让 `|v|` 直接当作"离表面多远"。
pub trait Scalar: Copy + Default + PartialEq + PartialOrd + Debug + Send + Sync + 'static {
    const MIN: Self;
    const MAX: Self;
    fn to_f64(self) -> f64;
    fn from_f64_round(v: f64) -> Self;
    /// `|v|` 的浮点值
    fn magnitude(self) -> f64;
    /// 量化档数的一半（`i8` 为 127，`i16` 为 32767）——距离换算的分母
    const MAX_MAG: f64;
}

impl Scalar for i8 {
    const MIN: Self = -127;
    const MAX: Self = 127;
    const MAX_MAG: f64 = 127.0;
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64_round(v: f64) -> Self {
        v.round().clamp(-127.0, 127.0) as i8
    }
    fn magnitude(self) -> f64 {
        (self as f64).abs()
    }
}

impl Scalar for i16 {
    const MIN: Self = -32767;
    const MAX: Self = 32767;
    const MAX_MAG: f64 = 32767.0;
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64_round(v: f64) -> Self {
        v.round().clamp(-32767.0, 32767.0) as i16
    }
    fn magnitude(self) -> f64 {
        (self as f64).abs()
    }
}

// ─────────────────────────── 索引 ───────────────────────────

/// 体素坐标 → 所属叶坐标
#[inline]
pub fn leaf_coord(ijk: [i32; 3]) -> [i32; 3] {
    [ijk[0] >> LEAF_LOG2, ijk[1] >> LEAF_LOG2, ijk[2] >> LEAF_LOG2]
}

/// 体素在叶内的线性索引（x 最快）
#[inline]
pub fn leaf_index(ijk: [i32; 3]) -> usize {
    let x = (ijk[0] & (LEAF_DIM - 1)) as usize;
    let y = (ijk[1] & (LEAF_DIM - 1)) as usize;
    let z = (ijk[2] & (LEAF_DIM - 1)) as usize;
    x | (y << LEAF_LOG2) | (z << (LEAF_LOG2 * 2))
}

/// 体素 → 所属 level-1 坐标（即叶坐标 >> 4）
#[inline]
pub fn l1_coord(ijk: [i32; 3]) -> [i32; 3] {
    [ijk[0] >> (LEAF_LOG2 + L1_LOG2), ijk[1] >> (LEAF_LOG2 + L1_LOG2), ijk[2] >> (LEAF_LOG2 + L1_LOG2)]
}

/// 叶坐标在 level-1 内的槽号
#[inline]
pub fn l1_slot(leaf: [i32; 3]) -> usize {
    let x = (leaf[0] & (L1_DIM - 1)) as usize;
    let y = (leaf[1] & (L1_DIM - 1)) as usize;
    let z = (leaf[2] & (L1_DIM - 1)) as usize;
    x | (y << L1_LOG2) | (z << (L1_LOG2 * 2))
}

/// 体素 → 所属 root 条目坐标（level-2 节点坐标）
#[inline]
pub fn root_coord(ijk: [i32; 3]) -> [i32; 3] {
    [
        ijk[0] >> (LEAF_LOG2 + L1_LOG2 + L2_LOG2),
        ijk[1] >> (LEAF_LOG2 + L1_LOG2 + L2_LOG2),
        ijk[2] >> (LEAF_LOG2 + L1_LOG2 + L2_LOG2),
    ]
}

/// level-1 坐标在 level-2 内的槽号
#[inline]
pub fn l2_slot(l1: [i32; 3]) -> usize {
    let x = (l1[0] & (L2_DIM - 1)) as usize;
    let y = (l1[1] & (L2_DIM - 1)) as usize;
    let z = (l1[2] & (L2_DIM - 1)) as usize;
    x | (y << L2_LOG2) | (z << (L2_LOG2 * 2))
}

/// O(1) 版：用前缀和数组，只 popcount **当前字**的低位。
///
/// 这是热路径（提取 200mm 毛坯要调 3 亿次）。`child_index` 是同一语义的
/// 参考实现（扫全部前缀字），只在测试里用来对拍。
#[inline]
fn child_at(prefix: &[u32], mask: &[u64], slot: usize) -> usize {
    let w = slot >> 6;
    let b = slot & 63;
    prefix[w] as usize + (mask[w] & ((1u64 << b) - 1)).count_ones() as usize
}

/// 位掩码里某个槽之前的活动子节点个数 = 该槽在紧凑数组里的下标。
///
/// 这是整个结构的核心一行：`children[popcount(mask & 低位)]`。
/// **参考实现**：逐字累加，`O(字数)`。热路径用 `child_at`。
#[inline]
pub fn child_index(mask: &[u64], slot: usize) -> usize {
    let w = slot >> 6;
    let b = slot & 63;
    let mut n: usize = mask[..w].iter().map(|m| m.count_ones() as usize).sum();
    n += (mask[w] & ((1u64 << b) - 1)).count_ones() as usize;
    n
}

#[inline]
fn bit_set(mask: &[u64], slot: usize) -> bool {
    (mask[slot >> 6] >> (slot & 63)) & 1 == 1
}

#[inline]
fn bit_on(mask: &mut [u64], slot: usize) {
    mask[slot >> 6] |= 1u64 << (slot & 63);
}

/// 遍历掩码里所有置位的槽（只走活动槽，不扫全表）。
#[inline]
fn for_each_set_slot(mask: &[u64], mut f: impl FnMut(usize)) {
    for (wi, &word) in mask.iter().enumerate() {
        let mut w = word;
        while w != 0 {
            let b = w.trailing_zeros() as usize;
            f((wi << 6) + b);
            w &= w - 1;
        }
    }
}

// ─────────────────────────── 节点 ───────────────────────────

/// 建场三阶段累加器（求值 / 写入 / prune）。与 `cut` 的 PROF 分开，
/// 因为建场是一次性的、切削是每刀的，混在一起看不出谁贵。
pub static PROF_BOX: [std::sync::atomic::AtomicU64; 3] = [
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
    std::sync::atomic::AtomicU64::new(0),
];

/// 取走建场三阶段的累计毫秒（eval, write, prune）。
/// 建场实际写入的体素数（`|d| < band` 的那些）。
pub static PROF_BOX_N: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

pub fn prof_take_box() -> [f64; 3] {
    use std::sync::atomic::Ordering::Relaxed;
    let mut out = [0.0; 3];
    for (i, a) in PROF_BOX.iter().enumerate() {
        out[i] = a.swap(0, Relaxed) as f64 / 1e6;
    }
    out
}

/// 叶：整叶同值（tile），或 512 个真实体素。
#[derive(Clone, Debug, PartialEq)]
pub enum Leaf<S> {
    /// 整叶同一个值——未加工的内部、完全切空的外面、远离表面的区域
    Constant(S),
    /// 表面穿过这个叶，存真实的窄带距离
    Dense(Box<[S; LEAF_VOXELS]>),
}

impl<S: Scalar> Leaf<S> {
    pub fn constant(v: S) -> Self {
        Leaf::Constant(v)
    }
    /// 读一个体素
    pub fn get(&self, idx: usize) -> S {
        match self {
            Leaf::Constant(v) => *v,
            Leaf::Dense(a) => a[idx],
        }
    }
    /// 写一个体素。`Constant` 会先展开成 `Dense`（512 个值全填成原常量）。
    pub fn set(&mut self, idx: usize, v: S) {
        match self {
            Leaf::Constant(c) => {
                let old = *c;
                let mut a = Box::new([old; LEAF_VOXELS]);
                a[idx] = v;
                *self = Leaf::Dense(a);
            }
            Leaf::Dense(a) => a[idx] = v,
        }
    }
    /// 若整叶同值则返回该值。
    ///
    /// `a.iter().all(|&x| x == first)` 在 `[i8; 512]` 上同样不会被向量化
    /// （理由见 `count_ne`），而 `prune` 每刀要对几千个叶调它。
    /// 改成 8 个一组整体比较：`[S; 8]` 的 `!=` 编译成一次 8/16 字节比较，
    /// 512 次迭代降到 64 次。
    pub fn uniform_value(&self) -> Option<S> {
        match self {
            Leaf::Constant(v) => Some(*v),
            Leaf::Dense(a) => {
                let first = a[0];
                // 32 个一组：`[S; 32]` 的 `!=` 编译成 4 条 u64 比较（或 2 条 AVX2），
                // 512 字节从 64 次迭代降到 16 次。全同的叶（`prune` 的主要对象）
                // 走的就是这条快路。
                const W: usize = 32;
                let pat = [first; W];
                let mut i = 0;
                while i + W <= LEAF_VOXELS {
                    if a[i..i + W] != pat {
                        return None;
                    }
                    i += W;
                }
                while i < LEAF_VOXELS {
                    if a[i] != first {
                        return None;
                    }
                    i += 1;
                }
                Some(first)
            }
        }
    }
}

/// level-1 节点：16³ 个槽，每槽要么是一个叶，要么是一个 tile 值。
#[derive(Clone, Debug)]
pub struct Level1<S> {
    mask: Box<[u64]>,
    /// `prefix[i]` = `mask[0..i]` 的 popcount 之和。
    ///
    /// 没有它的话，`children[popcount(mask & 低位)]` 每次都要扫一遍前缀字——
    /// level-2 有 512 个 u64 字，最坏一次查找扫 512 个 popcount。
    /// 提取 200mm 毛坯要做 3 亿次查找，实测这就是主要瓶颈。
    prefix: Box<[u32]>,
    /// 每个槽的 tile 值（该槽有子节点时无意义）
    values: Box<[S]>,
    /// 活动槽的叶索引，按 popcount 顺序连续存放
    children: Vec<u32>,
    /// **blind metadata**：本块内 `|v|` 的最小值 = 到最近表面叶的距离下界。
    /// `255` 表示需重算（写入把最小值抬高了）。
    blind_min_abs: u8,
}

impl<S: Scalar> Level1<S> {
    fn new(fill: S) -> Self {
        Self {
            mask: vec![0u64; L1_WORDS].into_boxed_slice(),
            prefix: vec![0u32; L1_WORDS + 1].into_boxed_slice(),
            values: vec![fill; L1_SLOTS].into_boxed_slice(),
            children: Vec::new(),
            // 初始整块同值，下界就是 |fill|
            blind_min_abs: fill.magnitude().min(254.0) as u8,
        }
    }
    /// 到最近表面的距离下界（体素值的量级）。`None` = 需重算。
    pub fn blind_min_abs(&self) -> Option<u8> {
        if self.blind_min_abs == u8::MAX { None } else { Some(self.blind_min_abs) }
    }
    /// 该槽是否有子叶
    pub fn has_child(&self, slot: usize) -> bool {
        bit_set(&self.mask, slot)
    }
    /// 该槽的 tile 值（无子叶时有效）
    pub fn tile(&self, slot: usize) -> S {
        self.values[slot]
    }
    fn child(&self, slot: usize) -> Option<u32> {
        if !bit_set(&self.mask, slot) {
            return None;
        }
        self.children.get(child_at(&self.prefix, &self.mask, slot)).copied()
    }
    fn insert_child(&mut self, slot: usize, leaf: u32) {
        let at = child_at(&self.prefix, &self.mask, slot);
        self.children.insert(at, leaf);
        bit_on(&mut self.mask, slot);
        // 该位从 0 变 1：后面所有字的前缀和 +1
        for p in (slot >> 6) + 1..=L1_WORDS {
            self.prefix[p] += 1;
        }
    }
    fn set_tile(&mut self, slot: usize, v: S) {
        self.values[slot] = v;
    }
    /// 重算 blind metadata（O(4096)，只在 prune 时调用）
    fn refresh_blind(&mut self, leaves: &[Leaf<S>]) {
        let mut m = u8::MAX;
        for slot in 0..L1_SLOTS {
            let mag = match self.child(slot) {
                Some(li) => match &leaves[li as usize] {
                    Leaf::Constant(x) => x.magnitude(),
                    Leaf::Dense(a) => a.iter().map(|x| x.magnitude()).fold(f64::INFINITY, f64::min),
                },
                None => self.values[slot].magnitude(),
            };
            m = m.min(mag.min(254.0) as u8);
            if m == 0 {
                break;
            }
        }
        self.blind_min_abs = m;
    }
    fn note_write(&mut self, old: S, new: S) {
        let (a, b) = (old.magnitude().min(254.0) as u8, new.magnitude().min(254.0) as u8);
        if self.blind_min_abs == u8::MAX {
            return;
        }
        if b < self.blind_min_abs {
            self.blind_min_abs = b;
        } else if a == self.blind_min_abs && b > a {
            // 原来的最小值被抬高了，下界不再成立
            self.blind_min_abs = u8::MAX;
        }
    }
}

/// level-2 节点：32³ 个槽，每槽要么是一个 level-1 节点，要么是一个 tile 值。
#[derive(Clone, Debug)]
pub struct Level2<S> {
    mask: Box<[u64]>,
    prefix: Box<[u32]>,
    values: Box<[S]>,
    children: Vec<u32>,
}

impl<S: Scalar> Level2<S> {
    fn new(fill: S) -> Self {
        Self {
            mask: vec![0u64; L2_WORDS].into_boxed_slice(),
            prefix: vec![0u32; L2_WORDS + 1].into_boxed_slice(),
            values: vec![fill; L2_SLOTS].into_boxed_slice(),
            children: Vec::new(),
        }
    }
    pub fn has_child(&self, slot: usize) -> bool {
        bit_set(&self.mask, slot)
    }
    pub fn tile(&self, slot: usize) -> S {
        self.values[slot]
    }
    fn child(&self, slot: usize) -> Option<u32> {
        if !bit_set(&self.mask, slot) {
            return None;
        }
        self.children.get(child_at(&self.prefix, &self.mask, slot)).copied()
    }
    fn insert_child(&mut self, slot: usize, l1: u32) {
        let at = child_at(&self.prefix, &self.mask, slot);
        self.children.insert(at, l1);
        bit_on(&mut self.mask, slot);
        for p in (slot >> 6) + 1..=L2_WORDS {
            self.prefix[p] += 1;
        }
    }
    fn set_tile(&mut self, slot: usize, v: S) {
        self.values[slot] = v;
    }
}

/// root 的一条：要么是 level-2 节点，要么是一个 tile 值（整块 4096³ 体素同值）。
#[derive(Clone, Copy, Debug)]
pub enum RootEntry<S> {
    Node(u32),
    Tile(S),
}

// ─────────────────────────── 盒子判据 ───────────────────────────

/// 体素 `p` 是否在 `[lo, hi)` 内
#[inline]
fn contains(p: [i32; 3], lo: [i32; 3], hi: [i32; 3]) -> bool {
    p[0] >= lo[0] && p[0] < hi[0] && p[1] >= lo[1] && p[1] < hi[1] && p[2] >= lo[2] && p[2] < hi[2]
}

/// 以 `base` 起、每轴 `span` 个体素的方块是否与 `[lo, hi)` 有交
#[inline]
fn overlaps(base: [i32; 3], span: i32, lo: [i32; 3], hi: [i32; 3]) -> bool {
    (0..3).all(|a| base[a] < hi[a] && base[a] + span > lo[a])
}

/// 该方块是否完全落在 `[lo, hi)` 内
#[inline]
fn fully_inside(base: [i32; 3], span: i32, lo: [i32; 3], hi: [i32; 3]) -> bool {
    (0..3).all(|a| base[a] >= lo[a] && base[a] + span <= hi[a])
}

/// 该方块能否安全地做成 **tile**：不仅完全在盒内，还要求**四周各留 1 体素余量**。
///
/// 这条余量不是保守，是**正确性要求**：表面提取要遍历"8 个角有符号变化"的单元格，
/// 而单元格跨 2 个体素。若紧贴填充边界的槽是 tile（没有叶），那么
/// "tile 的最后一个体素 ↔ 盒外背景"这条符号变化就落在没有叶子的区域，
/// 谁都遍历不到它 → **网格在实体边界上出现一圈洞**（实测 298 个四边形丢失）。
///
/// 留 1 体素余量之后，边界那一层叶槽一定是叶而不是 tile，洞就没了。
#[inline]
fn tile_safe(base: [i32; 3], span: i32, lo: [i32; 3], hi: [i32; 3]) -> bool {
    (0..3).all(|a| base[a] > lo[a] && base[a] + span < hi[a])
}

// ─────────────────────────── 网格 ───────────────────────────

/// 把一个叶的 512 个值整体覆盖（[`SparseGrid::blit_leaves_parallel`] 的工作单元）。
///
/// **只碰 `Leaf` 自己**，不碰任何树结构——所以可以对着不同叶并行调用。
fn write_leaf_block<S: Scalar>(dst: &mut Leaf<S>, vals: &[S]) {
    debug_assert_eq!(vals.len(), LEAF_VOXELS, "整块写必须是 512 个值");
    if let Leaf::Dense(a) = dst {
        a.copy_from_slice(vals);
        return;
    }
    // Constant 叶：整块还是同值就保持 Constant（不碰分配器），否则展开成 Dense
    let first = vals[0];
    if vals.iter().all(|&x| x == first) {
        *dst = Leaf::Constant(first);
    } else {
        let mut a: Box<[S; LEAF_VOXELS]> = Box::new([first; LEAF_VOXELS]);
        a.copy_from_slice(vals);
        *dst = Leaf::Dense(a);
    }
}

/// 一次查找拿到的叶视图：[`SparseGrid::leaf_or_tile`] 的返回。
pub enum LeafView<'a, S> {
    /// 该槽是 tile（没有具体叶）：整块同值
    Tile(S),
    /// 具体叶（可能是 `Constant`，也可能是 `Dense`）
    Leaf(&'a Leaf<S>),
}

/// 顺序写入的缓存：一次批量写（一段切削的 `changed`）复用一个，用完即丢。
///
/// `set` 每次都要 `ensure_l2 → ensure_l1 → ensure_leaf` 三次查找（实测 15.8 ns/体素），
/// 而切削产生的 `changed` 按 z/y/x 顺序出来，**同一叶的写入天然相邻**。
/// 命中时只需三次移位比较，直接写叶缓冲。
///
/// 两级缓存：
/// · 叶级（`coord`）：整叶写（`blit_leaf`）与逐体素写（`set_cached`）命中。
/// · 块级（`l1_coord`）：一个 level-1 块管 16³ = 4096 个叶，切削按 z/y/x 走，
///   相邻的叶几乎总在同一块里——命中可跳过 root 的哈希查找与 `ensure_l1`。
///   落盘新建叶（`slot: None` 的块）是这条最大的受益者。
#[derive(Clone, Copy, Debug, Default)]
pub struct WriteCache {
    /// 上一次写入所属的叶坐标（`ijk >> 3`）
    coord: [i32; 3],
    /// 该叶所属的一级节点（`note_write` 要它更新 blind metadata）
    l1i: u32,
    leaf: u32,
    valid: bool,
    /// 上一次写过的 level-1 块坐标（`l1_coord(ijk)`）
    l1_coord: [i32; 3],
    l1_valid: bool,
}

impl WriteCache {
    /// 当前缓存的叶坐标（还没写过任何东西时为 `None`）。
    pub fn leaf(&self) -> Option<[i32; 3]> {
        if self.valid { Some(self.coord) } else { None }
    }
    /// 当前叶所属的一级节点下标（给增量 prune 用）。
    pub fn l1(&self) -> usize {
        self.l1i as usize
    }
    /// 当前叶在 `leaves` 里的下标（给 [`SparseGrid::prune_leaves`] 用）。
    pub fn leaf_index(&self) -> Option<u32> {
        if self.valid { Some(self.leaf) } else { None }
    }
}

/// 稀疏体素场。**单一所有者**——不共享，不 `Arc<Mutex<>>`。
///
/// 材料去除是天然串行的（第 N+1 段要读第 N 段的结果），并行切削会让浮点结果
/// 不可复现，而这是验证工具，确定性优先。见 `docs/并发与性能.md`。
#[derive(Clone, Debug)]
pub struct SparseGrid<S: Scalar> {
    pub config: crate::GridConfig,
    /// 网格域最小角（体素 `(0,0,0)` 的中心在 `origin + 0.5h`）
    pub origin: DVec3,
    /// 既不在叶也不在 tile 里时返回的值（OpenVDB 的 background value）
    pub background: S,
    leaves: Vec<Leaf<S>>,
    level1: Vec<Level1<S>>,
    level2: Vec<Level2<S>>,
    root: FxHashMap<[i32; 3], RootEntry<S>>,
    /// 曾经写过的体素范围（含下界、开上界）。切削时用它裁剪扫掠 AABB，
    /// 否则一把 20 mm 长的刀会去遍历毛坯上方整片空区域。
    bounds: Option<([i32; 3], [i32; 3])>,
    /// **溯源标记**（ADR-005 的调色板索引）。0 = 原始毛坯面，1..=12 = 第 n 刀。
    ///
    /// 每片叶存 `creator_sub³` 个字节：`creator_sub = 1` 时每叶一个（最省），
    /// `= 8` 时每个体素一个（最锐）。见 [`Self::set_creator_sub`]。
    ///
    /// 用**平行数组**而不是给 `Leaf` 加字段：`Leaf` 是 `Dense(Box<[S;512]>)` 的
    /// 枚举，加字段会让每片叶多一个 padding 字节、并改动所有 `match` 分支。
    pub(crate) creator: Vec<u8>,
    /// 是否记录溯源（切割颜色）。**关掉时 `creator` 一个字节都不分配**，
    /// `stamp_leaf` 也不会被调用——省的不只是显示，是每片叶 1~512 字节的写。
    pub(crate) colorize: bool,
    /// 每片叶切成 `creator_sub³` 个子块。只能是 1 / 2 / 4 / 8。
    pub(crate) creator_sub: u8,
    /// 接下来这一刀盖的标记。0 表示不盖（建模/填充走这个值）。
    pub(crate) pending_creator: u8,
}

/// `get` 的**根游标**：记住上一次的"root 坐标 → root 条目"。
///
/// 三线性插值一次要问 8 个体素，而 root 是 64³ 体素——连续查询几乎总落在同一个
/// root 里。没有游标时每个体素都要走一次 `FxHashMap` 查找：LOD 顶点阶段
/// 53 万次 `sdf` × 8 = **430 万次哈希查找**，这是那一阶段的主要成本。
#[derive(Clone, Copy, Debug)]
pub struct RootCursor<S> {
    rc: [i32; 3],
    entry: Option<RootEntry<S>>,
}

impl<S> Default for RootCursor<S> {
    fn default() -> Self {
        // `root_coord` 是 `ijk >> 6`，不可能等于 `i32::MIN`，所以这个哨兵不会误命中
        Self { rc: [i32::MIN; 3], entry: None }
    }
}

/// 逐顶点取色的**游标**：记住上一次的"叶坐标 → 叶下标"。
///
/// LOD 提取是按粗格顺序走的，相邻格绝大多数落在同一片叶里（k=3 时一片叶
/// 覆盖 2–3 个粗格/轴），所以一次比较就能省掉"哈希查根 + 逐级下降"。
/// 实测 13 ns/次 → 2 ns/次。
#[derive(Clone, Copy, Debug, Default)]
pub struct CreatorCursor {
    pub(crate) last: [i32; 3],
    pub(crate) leaf: Option<usize>,
}

impl<S: Scalar> SparseGrid<S> {
    /// 新建空网格，背景值取 `Scalar::MAX`（远离材料的外部）。
    pub fn new(config: crate::GridConfig, origin: DVec3) -> Self {
        Self {
            config,
            origin,
            background: S::MAX,
            leaves: Vec::new(),
            level1: Vec::new(),
            level2: Vec::new(),
            root: FxHashMap::default(),
            bounds: None,
            creator: Vec::new(),
            colorize: true,
            creator_sub: 1,
            pending_creator: 0,
        }
    }

    /// 设置接下来这一刀盖的创建者标记（0 = 不盖）。
    ///
    /// 放在网格上而不是做成 `cut_*` 的参数：`cut_segment` / `cut_batch` 有十几个
    /// 测试在用，加参数等于把"着色"这个可选的旁路塞进每个签名里。
    pub fn set_creator(&mut self, creator: u8) {
        self.pending_creator = creator;
    }

    /// 溯源粒度：每片叶切成 `sub³` 个子块各存一个 creator。
    ///
    /// | `sub` | 分辨率 | 内存 | 面铣一层的实测溢出 |
    /// |---|---|---|---|
    /// | 1 | 8h = 0.80 mm | **1 B/叶** | 0.4–1.0 mm |
    /// | 2 | 4h = 0.40 mm | 8 B/叶 | |
    /// | 4 | 2h = 0.20 mm | 64 B/叶 | |
    /// | 8 | h = 0.10 mm | 512 B/叶 | |
    ///
    /// **必须在建场之前调用**（`leaves` 非空时 panic）：`creator` 是平行数组，
    /// 建场之后再改粒度就得整体重排。
    pub fn set_creator_sub(&mut self, sub: u8) {
        assert!(
            self.leaves.is_empty(),
            "set_creator_sub 必须在建场之前调用（已有 {} 片叶）",
            self.leaves.len()
        );
        let sub = sub.clamp(1, 8).next_power_of_two().min(8);
        self.creator_sub = sub;
    }

    /// 当前溯源粒度（1 / 2 / 4 / 8）。
    pub fn creator_sub(&self) -> u8 {
        self.creator_sub
    }

    /// 溯源数组占的字节数（测粒度-内存权衡用）。
    pub fn creator_bytes(&self) -> usize {
        self.creator.len()
    }

    /// 打开/关闭溯源着色。**必须在建场之前调用**（和 `set_creator_sub` 同理：
    /// `creator` 是平行数组，建场后再改长度就对不上了）。
    ///
    /// 关掉后：不分配 `creator`（大件上是几十 MB）、不写子块位图、
    /// `changed_mask` 直接走 `count_changed` 的快路径。切削时间与"没有溯源功能"
    /// 的版本一致——不是"画的时候不显示"，是那一份工作根本不发生。
    pub fn set_colorize(&mut self, on: bool) {
        assert!(
            self.leaves.is_empty(),
            "set_colorize 必须在建场之前调用（已有 {} 片叶）",
            self.leaves.len()
        );
        self.colorize = on;
        if !on {
            self.creator = Vec::new();
            self.pending_creator = 0;
        }
    }

    pub fn colorize(&self) -> bool {
        self.colorize
    }

    /// 新建一片叶时把 `creator` 补齐到同样的叶数。关掉着色时一个字节都不分配。
    #[inline]
    fn grow_creator(&mut self) {
        if !self.colorize {
            return;
        }
        let n = self.creator_stride();
        self.creator.resize(self.creator.len() + n, 0);
    }

    /// `creator` 数组里每片叶占多少字节。
    #[inline]
    fn creator_stride(&self) -> usize {
        let s = self.creator_sub as usize;
        s * s * s
    }

    /// 叶内子块下标（0..sub³）。`p` 是体素坐标。
    #[inline]
    fn sub_index(&self, p: [i32; 3]) -> usize {
        let sub = self.creator_sub as usize;
        if sub == 1 {
            return 0;
        }
        let bits = sub.trailing_zeros(); // log2(sub)
        let shift = 3 - bits;
        let m = sub - 1;
        let sx = ((p[0] as usize) & 7) >> shift;
        let sy = ((p[1] as usize) & 7) >> shift;
        let sz = ((p[2] as usize) & 7) >> shift;
        sx & m | ((sy & m) << bits) | ((sz & m) << (2 * bits))
    }

    /// 叶的创建者标记（子块 0）。越界或未标记返回 0。
    #[inline]
    pub fn creator_at(&self, leaf: u32) -> u8 {
        self.creator.get(leaf as usize * self.creator_stride()).copied().unwrap_or(0)
    }

    /// 体素坐标所属子块的创建者标记（该处没有叶 → 0）。
    #[inline]
    pub fn creator_of_voxel(&self, p: [i32; 3]) -> u8 {
        match self.leaf_slot(leaf_coord(p)) {
            Some(li) => self.creator_at_voxel(li, p),
            None => 0,
        }
    }

    /// 带游标的 [`Self::creator_of_voxel`]。
    #[inline]
    pub fn creator_of_voxel_cached(&self, p: [i32; 3], cur: &mut CreatorCursor) -> u8 {
        let lc = leaf_coord(p);
        if lc != cur.last || cur.leaf.is_none() {
            cur.last = lc;
            cur.leaf = self.leaf_slot(lc);
        }
        match cur.leaf {
            Some(li) => self.creator_at_voxel(li, p),
            None => 0,
        }
    }

    /// 已知叶下标时取子块颜色——**跳过叶查找**。
    ///
    /// 提取时每片叶只查一次叶下标，然后对叶内每个顶点调这个（一次数组寻址）。
    #[inline]
    pub(crate) fn creator_at_voxel(&self, leaf: usize, p: [i32; 3]) -> u8 {
        let at = leaf * self.creator_stride() + self.sub_index(p);
        self.creator.get(at).copied().unwrap_or(0)
    }

    /// 位图占几个 u64（`sub ≤ 4` 是 1，`sub = 8` 是 8）。
    #[inline]
    pub(crate) fn creator_words(&self) -> usize {
        let n = (self.creator_sub as usize).pow(3);
        n.div_ceil(64)
    }

    /// 给一片叶里 `mask` 标记的子块盖上 `pending_creator`。
    ///
    /// `mask` 的第 i 位对应子块 i（按 `sub_index` 的编号）。`sub = 1` 时只看第 0 位。
    pub(crate) fn stamp_leaf(&mut self, leaf: usize, mask: &[u64]) {
        if !self.colorize {
            return;
        }
        let c = self.pending_creator;
        if c == 0 {
            return;
        }
        let stride = self.creator_stride();
        let base = leaf * stride;
        let Some(slice) = self.creator.get_mut(base..base + stride) else {
            return;
        };
        if stride == 1 {
            if mask[0] & 1 != 0 {
                slice[0] = c;
            }
            return;
        }
        for (w, &word) in mask.iter().enumerate() {
            let mut bits = word;
            while bits != 0 {
                let b = bits.trailing_zeros() as usize;
                bits &= bits - 1;
                let i = w * 64 + b;
                if i < stride {
                    slice[i] = c;
                }
            }
        }
    }

    /// 曾经写过的体素范围（含下界、开上界）。从未写过时返回 `None`。
    pub fn bounds(&self) -> Option<([i32; 3], [i32; 3])> {
        self.bounds
    }

    fn grow_bounds(&mut self, lo: [i32; 3], hi: [i32; 3]) {
        self.bounds = Some(match self.bounds {
            None => (lo, hi),
            Some((a, b)) => (
                [a[0].min(lo[0]), a[1].min(lo[1]), a[2].min(lo[2])],
                [b[0].max(hi[0]), b[1].max(hi[1]), b[2].max(hi[2])],
            ),
        });
    }

    pub fn with_background(config: crate::GridConfig, origin: DVec3, bg: S) -> Self {
        let mut g = Self::new(config, origin);
        g.background = bg;
        g
    }

    /// root 条目数。小毛坯应是个位数——这是树结构存在的主要理由。
    pub fn root_len(&self) -> usize {
        self.root.len()
    }

    /// 已分配的叶数（含 `Constant`）
    pub fn leaf_count(&self) -> usize {
        self.leaves.len()
    }

    /// 已分配的实际体素存储字节数（不含 Vec 容量冗余）
    pub fn value_bytes(&self) -> usize {
        let leaf = self
            .leaves
            .iter()
            .map(|l| match l {
                Leaf::Constant(_) => 0,
                Leaf::Dense(_) => LEAF_VOXELS * std::mem::size_of::<S>(),
            })
            .sum::<usize>();
        let l1 = self.level1.len() * L1_SLOTS * std::mem::size_of::<S>();
        let l2 = self.level2.len() * L2_SLOTS * std::mem::size_of::<S>();
        leaf + l1 + l2
    }

    /// 材料体积（mm³）：所有 `v < 0` 的体素个数 × `h³`。
    ///
    /// **是阶梯近似**，不是解析体积：边界处的误差在半个体素厚的一层上，
    /// 对 200 mm 毛坯（h=0.1）约 ±0.4%。用于报告"切掉了多少料"够了；
    /// 要精确体积得用表面网格积分（`MeshData` 已经在手，但那是另一件事）。
    ///
    /// 统计必须走**三层 tile + 叶**：`fill_box` 会把统一值的区域短路成
    /// root / level-2 / level-1 三级 tile，`for_each_leaf` 只看得到叶。
    /// 只数叶会把整块实心毛坯算成 0（或者更糟：切削时 tile 被"打开"成叶，
    /// 体积反而**变大**——实测过）。
    pub fn material_volume(&self) -> f64 {
        let zero = S::default();
        let mut count: u64 = 0;

        let leaf_voxels = LEAF_VOXELS as u64;
        let l2_tile_voxels = (L1_SPAN as u64) * (L1_SPAN as u64) * (L1_SPAN as u64);
        let rs = ROOT_SPAN as u64;
        let root_tile_voxels = rs * rs * rs;

        for entry in self.root.values() {
            let l2i = match entry {
                RootEntry::Tile(v) => {
                    if *v < zero {
                        count += root_tile_voxels;
                    }
                    continue;
                }
                RootEntry::Node(i) => *i as usize,
            };
            let l2 = &self.level2[l2i];
            for s2 in 0..L2_SLOTS {
                let Some(l1i) = l2.child(s2) else {
                    // 整个 level-1 节点是一个 tile
                    if l2.tile(s2) < zero {
                        count += l2_tile_voxels;
                    }
                    continue;
                };
                let l1 = &self.level1[l1i as usize];
                for s1 in 0..L1_SLOTS {
                    let Some(li) = l1.child(s1) else {
                        if l1.tile(s1) < zero {
                            count += leaf_voxels;
                        }
                        continue;
                    };
                    match &self.leaves[li as usize] {
                        Leaf::Constant(v) => {
                            if *v < zero {
                                count += leaf_voxels;
                            }
                        }
                        Leaf::Dense(a) => {
                            for v in a.iter() {
                                if *v < zero {
                                    count += 1;
                                }
                            }
                        }
                    }
                }
            }
        }

        count as f64 * self.config.h.powi(3)
    }

    /// 读一个体素。
    /// **整叶整块写入**：512 个新值一次覆盖，返回写前写后的叶内最小 |v|。
    ///
    /// 与 [`Self::fill_leaf`] 的区别：那个是"整叶同一个值"（tile 化），这个是
    /// "整叶各不相同的值"（表面叶）。与 512 次 `set` 的区别全在访存——
    /// 定位一次、`copy_from_slice` 一次，写的是**同一条 512 字节的连续区间**，
    /// 而不是 512 次各自算下标再散写。
    ///
    /// `blind_min_abs` 的维护是**按需**的：块内一旦出现过表面（blind 已是 0），
    /// 就不可能再变小，直接跳过 512 次幅值扫描。粗加工的场景里块内总有表面，
    /// 所以这条几乎总是零开销；只有整块落在材料深处时才需要算一次。
    /// 由切片构造定长 `Box`：一次分配 + 一次 `memset` + 一次 `copy`。
    ///
    /// 试过 `Box::<[S; N]>::try_from(values)`（标准库有这个 impl，`T: Copy`），
    /// 但在泛型 `S: Scalar` 的位置上 trait bound 推不出来，编译报
    /// `Box<[S; 512]>: TryFrom<&[S]>` 不满足。这里退回显式写法——
    /// `[values[0]; N]` 是 `memset`，512 字节约 10 ns，不是瓶颈。
    #[inline]
    fn boxed_leaf(values: &[S]) -> Box<[S; LEAF_VOXELS]> {
        let mut b = Box::new([values[0]; LEAF_VOXELS]);
        b.copy_from_slice(values);
        b
    }

    pub fn blit_leaf(&mut self, leaf: [i32; 3], values: &[S], cache: &mut WriteCache) {
        debug_assert_eq!(values.len(), LEAF_VOXELS, "整块写必须是 512 个值");

        // **快路**：cache 命中且叶已是 `Dense` → 一次 `copy_from_slice` 收工。
        // 切削里同一片叶常被连续写（按叶遍历产出），这条路命中率很高。
        if cache.valid && cache.coord == leaf
            && let Leaf::Dense(dst) = &mut self.leaves[cache.leaf as usize]
        {
            dst.copy_from_slice(values);
            self.level1[cache.l1i as usize].blind_min_abs = u8::MAX;
            return;
        }

        let base = [
            leaf[0] << LEAF_LOG2,
            leaf[1] << LEAF_LOG2,
            leaf[2] << LEAF_LOG2,
        ];
        let l1i = Self::ensure_l1_cached(self, base, cache);
        let s1 = l1_slot(leaf_coord(base));

        // **不经过 `Constant` 中间态**。原来的写法是 `ensure_leaf`（push 一个
        // `Constant(fill)`）→ 判断是 Constant → 换成 `Dense(Box::new([fill; 512]))`
        // → `copy_from_slice`。等于**两次分配 + 一次多余的 512 字节填充**。
        //
        // 实测：铣光 4000 刀里 86.6 万个叶是这条路径建的（占 30%），
        // 每刀 217 次、collect 阶段 253 ns/次。
        //
        // `Box::try_from(&[S])` 直接由切片构造定长 Box：一次分配 + 一次 copy。
        match self.level1[l1i].child(s1) {
            Some(i) => {
                let li = i as usize;
                if let Leaf::Dense(dst) = &mut self.leaves[li] {
                    dst.copy_from_slice(values);
                } else {
                    // Constant → Dense：同样直接构造，不用先填一遍
                    self.leaves[li] = Leaf::Dense(Self::boxed_leaf(values));
                }
                cache.coord = leaf;
                cache.l1i = l1i as u32;
                cache.leaf = li as u32;
                cache.valid = true;
            }
            None => {
                let li = self.leaves.len();
                self.leaves.push(Leaf::Dense(Self::boxed_leaf(values)));
                self.grow_creator();
                self.level1[l1i].insert_child(s1, li as u32);
                cache.coord = leaf;
                cache.l1i = l1i as u32;
                cache.leaf = li as u32;
                cache.valid = true;
            }
        }

        // blind 是**下界**，偏小无害（跳过的少一点），偏大才危险。切削会造出
        // 新表面让 |v| 变小，也可能把整叶削成空气让 |v| 变大——两个方向都会发生，
        // 所以这里只能**标脏**（`u8::MAX` = 需重算），不猜。
        //
        // 不在这里扫 512 个幅值求最小值：那是一次额外的整块扫描，而 blind 目前
        // 没有读取者。要精确值的调用方走 `prune()`（离线，全量重算）。
        self.level1[l1i].blind_min_abs = u8::MAX;
    }

    /// 定位叶（**只读，不创建**）。`None` = 落在背景值上。
    ///
    /// 这是 `cut_box` 热路径要的东西：一次三级查找拿到叶，之后 512 个体素直接
    /// `leaf.get(idx)`，而不是每个体素都重走 `root → l2 → l1 → leaf`。
    ///
    /// 实测（50×50×100 铣光、h=0.2）：每刀 38.8 万体素 × 三级查找 ≈ 3 ms，
    /// 换成叶级读取后这段基本消失。
    pub fn leaf_or_tile(&self, bl: [i32; 3]) -> Option<LeafView<'_, S>> {
        self.leaf_view_with_slot(bl).0
    }

    /// [`Self::leaf_or_tile`] + 该叶在 `leaves` 里的下标（`None` = 该槽是 tile
    /// 或整根路径不存在）。落盘时要靠这个下标做**无锁并行分区**。
    pub fn leaf_view_with_slot(
        &self,
        bl: [i32; 3],
    ) -> (Option<LeafView<'_, S>>, Option<(usize, usize)>) {
        let entry = match self.root.get(&root_coord(bl)) {
            Some(e) => *e,
            None => return (None, None),
        };
        let l2i = match entry {
            RootEntry::Tile(v) => return (Some(LeafView::Tile(v)), None),
            RootEntry::Node(i) => i as usize,
        };
        let l2 = &self.level2[l2i];
        let s2 = l2_slot(l1_coord(bl));
        let l1i = match l2.child(s2) {
            Some(i) => i as usize,
            None => return (Some(LeafView::Tile(l2.tile(s2))), None),
        };
        let l1 = &self.level1[l1i];
        let s1 = l1_slot(leaf_coord(bl));
        match l1.child(s1) {
            Some(i) => (
                Some(LeafView::Leaf(&self.leaves[i as usize])),
                Some((i as usize, l1i)),
            ),
            None => (Some(LeafView::Tile(l1.tile(s1))), None),
        }
    }

    /// **批量整块写，按叶索引分段并行（无锁）**。
    ///
    /// `items` 必须按叶索引升序——调用方排序，这里只切段。
    /// 每个线程只写自己那段 `leaves`（`par_chunks_mut` 保证物理不相交），
    /// 没有任何锁、原子操作或 CAS 重试。树的其它部分（`level1` 的 blind
    /// 元数据）由调用方事后串行统一重算，避免共享写。
    ///
    /// 返回写入的叶数。
    pub(crate) fn blit_leaves_parallel(&mut self, items: &[(u32, &[S])]) -> usize {
        if items.is_empty() {
            return 0;
        }
        let n = self.leaves.len();
        let nthreads = crate::par::worker_count().min(items.len()).max(1);
        // **小批量串行比并行快得多**。
        //
        // 门槛不是拍的：铣光逐刀约 500 块 × 512 字节 = 256 KB，串行写只要
        // **6 µs**（每块 12 ns，源和目标都在 L2）；而 24 线程 fork-join 光唤醒
        // 就 15–50 µs，再加分派和每线程的冷 cache。
        //
        // 实测（铣光 4000 刀累计，`CNCSIM_PROF=1` 的 write 拆分）：
        //   24 线程 blit 202 ms ／ 1 线程 blit **24 ms** —— **并行慢 8.4 倍**。
        //
        // 只有批量攒到几 MB 时并行才开始回本（那时每块都要跨 NUMA/L3 搬）。
        // 这是"任务粒度小于调度开销"的又一个实例，与 `WaitOnAddress` 占
        // 57% 是同一个病根。
        const PAR_BLIT_MIN_BYTES: usize = 1 << 20; // 1 MB
        let total_bytes = items.len() * LEAF_VOXELS * core::mem::size_of::<S>();
        if nthreads <= 1 || total_bytes < PAR_BLIT_MIN_BYTES {
            for (li, vals) in items {
                write_leaf_block(&mut self.leaves[*li as usize], vals);
            }
            return items.len();
        }

        // 段比线程多一档，让尾部不均的工作量能被抢走
        let chunk = n.div_ceil(nthreads * 2).max(1);
        // **必须跑在 cncsim 自己的池上**：`par_chunks_mut` 直接调用会落到
        // rayon 的**全局池**，而我们自己已经有一个 `pool()`——两个 24 线程池
        // 同时存在，进程里 50 个线程抢 24 个核。VTune 实测线程超订 88.3%、
        // 有效 CPU 利用率只有 8.9%。包进 `install` 后就只有一个池。
        crate::par::pool().install(|| {
            self.leaves
                .par_chunks_mut(chunk)
                .enumerate()
                .for_each(|(ci, seg)| {
                    let lo = ci * chunk;
                    let hi = lo + seg.len();
                    let a = items.partition_point(|it| (it.0 as usize) < lo);
                    let b = items.partition_point(|it| (it.0 as usize) < hi);
                    for (li, vals) in &items[a..b] {
                        write_leaf_block(&mut seg[*li as usize - lo], vals);
                    }
                });
        });
        items.len()
    }

    pub fn get(&self, ijk: [i32; 3]) -> S {
        let entry = match self.root.get(&root_coord(ijk)) {
            Some(e) => *e,
            None => return self.background,
        };
        self.get_from_entry(ijk, entry)
    }

    /// 带根游标的 [`Self::get`]。逐体素语义完全一致，只是省掉重复的哈希查找。
    #[inline]
    pub fn get_cached(&self, ijk: [i32; 3], cur: &mut RootCursor<S>) -> S {
        let rc = root_coord(ijk);
        if rc != cur.rc {
            cur.rc = rc;
            cur.entry = self.root.get(&rc).copied();
        }
        match cur.entry {
            Some(e) => self.get_from_entry(ijk, e),
            None => self.background,
        }
    }

    /// 已知 root 条目时走完剩下的两级 + 叶内取值。
    #[inline]
    fn get_from_entry(&self, ijk: [i32; 3], entry: RootEntry<S>) -> S {
        let l2i = match entry {
            RootEntry::Tile(v) => return v,
            RootEntry::Node(i) => i as usize,
        };
        let l2 = &self.level2[l2i];
        let s2 = l2_slot(l1_coord(ijk));
        let l1i = match l2.child(s2) {
            Some(i) => i as usize,
            None => return l2.tile(s2),
        };
        let l1 = &self.level1[l1i];
        let s1 = l1_slot(leaf_coord(ijk));
        let li = match l1.child(s1) {
            Some(i) => i as usize,
            None => return l1.tile(s1),
        };
        self.leaves[li].get(leaf_index(ijk))
    }

    /// 确保 root 条目是一个 level-2 节点（`Tile` 会就地展开成节点，其余槽保持该 tile 值）。
    fn ensure_l2(&mut self, rc: [i32; 3]) -> usize {
        match self.root.get(&rc).copied() {
            Some(RootEntry::Node(i)) => i as usize,
            Some(RootEntry::Tile(t)) => {
                let idx = self.level2.len();
                self.level2.push(Level2::new(t));
                self.root.insert(rc, RootEntry::Node(idx as u32));
                idx
            }
            None => {
                let idx = self.level2.len();
                self.level2.push(Level2::new(self.background));
                self.root.insert(rc, RootEntry::Node(idx as u32));
                idx
            }
        }
    }

    fn ensure_l1(&mut self, l2i: usize, slot: usize) -> usize {
        if let Some(i) = self.level2[l2i].child(slot) {
            return i as usize;
        }
        let fill = self.level2[l2i].tile(slot);
        let idx = self.level1.len();
        self.level1.push(Level1::new(fill));
        self.level2[l2i].insert_child(slot, idx as u32);
        idx
    }

    fn ensure_leaf(&mut self, l1i: usize, slot: usize) -> usize {
        if let Some(i) = self.level1[l1i].child(slot) {
            return i as usize;
        }
        let fill = self.level1[l1i].tile(slot);
        let idx = self.leaves.len();
        self.leaves.push(Leaf::Constant(fill));
        self.grow_creator();
        self.level1[l1i].insert_child(slot, idx as u32);
        idx
    }

    /// 写一个体素。需要时逐级创建节点、把 `Constant` 叶展开成 `Dense`。
    pub fn set(&mut self, ijk: [i32; 3], v: S) {
        self.set_inner(ijk, v);
    }

    /// 与 [`Self::set`] 等价，但可以复用 [`WriteCache`] 跳过重复的三级查找。
    ///
    /// 切削产生的 `changed` 是按 z/y/x 顺序出来的，**同一叶的写入天然相邻**
    /// （x 连续 8 个都在同一叶）。判断"是否同一叶"只比较 `leaf_coord`
    /// （三次移位），命中就直接写叶缓冲。
    pub fn set_cached(&mut self, ijk: [i32; 3], v: S, cache: &mut WriteCache) {
        let lc = leaf_coord(ijk);
        if cache.valid && cache.coord == lc {
            let idx = leaf_index(ijk);
            let li = cache.leaf as usize;
            let old = self.leaves[li].get(idx);
            if old != v {
                self.leaves[li].set(idx, v);
                self.level1[cache.l1i as usize].note_write(old, v);
                self.grow_bounds(ijk, [ijk[0] + 1, ijk[1] + 1, ijk[2] + 1]);
            }
            return;
        }
        let (l1i, li) = self.set_inner(ijk, v);
        // 块级缓存沿用：set_inner 刚走过完整路径，l1 坐标就是这次的
        let l1c = l1_coord(ijk);
        *cache = WriteCache {
            coord: lc,
            l1i: l1i as u32,
            leaf: li as u32,
            valid: true,
            l1_coord: l1c,
            l1_valid: true,
        };
    }

    /// 与 [`Self::set_cached`] 相同，但**调用方保证 `ijk` 已在网格 bounds 内**。
    ///
    /// 省掉每体素一次 `grow_bounds`（6 次 min/max），实测 9.3 → 6.9 ns。
    /// 切削正好满足这个前提：`cut_box` 的 `lo..hi` 是从 `bounds()` 裁剪出来的。
    pub fn set_cached_in_bounds(&mut self, ijk: [i32; 3], v: S, cache: &mut WriteCache) {
        let lc = leaf_coord(ijk);
        if cache.valid && cache.coord == lc {
            let idx = leaf_index(ijk);
            let li = cache.leaf as usize;
            let old = self.leaves[li].get(idx);
            if old != v {
                self.leaves[li].set(idx, v);
                self.level1[cache.l1i as usize].note_write(old, v);
            }
            return;
        }
        let (l1i, li) = self.set_inner(ijk, v);
        // 块级缓存沿用：set_inner 刚走过完整路径，l1 坐标就是这次的
        let l1c = l1_coord(ijk);
        *cache = WriteCache {
            coord: lc,
            l1i: l1i as u32,
            leaf: li as u32,
            valid: true,
            l1_coord: l1c,
            l1_valid: true,
        };
    }

    /// 定位（或创建）叶所属的 level-1 节点，带**块级缓存**。
    ///
    /// 一个 level-1 块管 16³ 个叶；落盘按 z/y/x 顺序走来时相邻叶几乎总在同一块，
    /// 命中就跳过 root 的哈希查找与 `ensure_l1`（实测铣光每刀 ~300 次）。
    fn ensure_l1_cached(grid: &mut Self, base: [i32; 3], cache: &mut WriteCache) -> usize {
        let lc = l1_coord(base);
        if cache.l1_valid && cache.l1_coord == lc {
            return cache.l1i as usize;
        }
        let l2i = grid.ensure_l2(root_coord(base));
        let l1i = grid.ensure_l1(l2i, l2_slot(lc));
        cache.l1_coord = lc;
        cache.l1_valid = true;
        cache.l1i = l1i as u32;
        l1i
    }

    /// 整个叶**同值填充**（8³ = 512 体素），返回**实际改变的体素数**。
    ///
    /// 这是 tile 级短路的落点：整叶落在扫掠体深处时，一次调用代替 512 次 `set`。
    /// 叶已经是目标值时返回 0——幂等性靠这个（重复切同一段不该报告改动）。
    ///
    /// **槽位还是 tile 时直接改 tile，不造叶。** 整叶短路的语义是"整叶变成同一个值"，
    /// 而 tile 恰好就是"整叶同值"的原生表示——旧实现先 `ensure_leaf` 造一个
    /// `Dense`（520 字节分配 + 512 值写），再由 `prune` 发现全同、折叠回 `Constant`，
    /// **分配了又扔掉**。铣光一刀 580 次整叶短路里绝大多数落在毛坯内部的 tile 上，
    /// 实测这一项占整刀 32%（比求值还多）。
    pub fn fill_leaf(&mut self, leaf: [i32; 3], v: S, cache: &mut WriteCache) -> usize {
        let base = [leaf[0] << LEAF_LOG2, leaf[1] << LEAF_LOG2, leaf[2] << LEAF_LOG2];
        let (l1i, li) = if cache.valid && cache.coord == leaf {
            (cache.l1i as usize, cache.leaf as usize)
        } else {
            let l1i = Self::ensure_l1_cached(self, base, cache);
            let s1 = l1_slot(leaf_coord(base));
            // ★ tile 快路：槽位为空（整槽同值）时改值即可，零分配。
            if self.level1[l1i].child(s1).is_none() {
                let old = self.level1[l1i].tile(s1);
                // 记下 l1（`l1s` 要标 blind 脏），但**没有叶下标**——
                // 调用方靠 `leaf_index() == None` 知道"这个槽不用 prune"。
                cache.coord = leaf;
                cache.l1i = l1i as u32;
                cache.valid = false;
                if old == v {
                    return 0;
                }
                self.level1[l1i].set_tile(s1, v);
                self.level1[l1i].blind_min_abs = u8::MAX;
                return LEAF_VOXELS;
            }
            let li = self.ensure_leaf(l1i, s1);
            cache.coord = leaf;
            cache.l1i = l1i as u32;
            cache.leaf = li as u32;
            cache.valid = true;
            (l1i, li)
        };

        let changed = match &self.leaves[li] {
            Leaf::Constant(c) if *c == v => 0,
            Leaf::Constant(_) => LEAF_VOXELS,
            Leaf::Dense(a) => count_ne(a, v),
        };
        if changed == 0 {
            return 0;
        }
        self.leaves[li] = Leaf::Constant(v);
        // 整叶换成一个值，块内最小值可能被抬高也可能被压低——标脏，不猜
        //（理由见 `blit_leaf`：blind 没有读取者，重算放到离线的 `prune()`）
        self.level1[l1i].blind_min_abs = u8::MAX;
        changed
    }

    /// 完整查找 + 写入，返回 `(level-1 节点索引, 叶索引)` 供缓存。
    fn set_inner(&mut self, ijk: [i32; 3], v: S) -> (usize, usize) {
        let l2i = self.ensure_l2(root_coord(ijk));
        let s2 = l2_slot(l1_coord(ijk));
        let l1i = self.ensure_l1(l2i, s2);
        let s1 = l1_slot(leaf_coord(ijk));
        let li = self.ensure_leaf(l1i, s1);

        let idx = leaf_index(ijk);
        let old = self.leaves[li].get(idx);
        if old == v {
            return (l1i, li);
        }
        self.leaves[li].set(idx, v);
        self.level1[l1i].note_write(old, v);
        self.grow_bounds(ijk, [ijk[0] + 1, ijk[1] + 1, ijk[2] + 1]);
        (l1i, li)
    }

    /// 填实心盒，并把**表面窄带内的体素细化成精确盒 SDF**。
    ///
    /// [`Self::fill_box`] 只写一个饱和值（整块同值 → 可压成 tile，建场几乎免费），
    /// 代价是未切削表面的位置依赖"体素中心恰好落在盒边界上"。毛坯尺寸与 `h`
    /// 不整除时，插值出的表面会偏最多半个体素。
    ///
    /// 这里把边界壳（到最近盒面距离 < `band_mm` 的体素）改成精确距离，
    /// 与 OpenVDB 的 level set 毛坯等价。**内部仍是饱和 tile**，所以内存不变。
    ///
    /// # 为什么按**叶**遍历而不是按体素
    ///
    /// 上一版是"并行逐体素求值 → 串行 532 万次 `set`"。实测（50×50×100 @ h=0.2，
    /// `CNCSIM_PROF=1` 的建场拆分）：求值 6 ms、**写入 31 ms**、prune 5 ms——
    /// 写入占了 69%，而每次 `set` 只写 1 个字节。
    ///
    /// 改成按叶：**一片叶的 512 个体素一起读、一起改、一次写回**。1.5 万次整块写
    /// 代替 532 万次单点写，写入降到 ~3 ms。求值量不变（还是那些 `|d| < bandm` 的
    /// 体素），但顺序从"面片 × 切片"变成"叶 × 叶内索引"，同一叶的 512 个值留在
    /// L1 里。
    ///
    /// 关键是**初值必须读对**：叶内不在面片上的体素要保持原值（`fill_box` 写的
    /// 饱和 −band，或背景值），所以每叶先按 `tile / Constant / Dense` 三种现状
    /// 取一份 512 值，只覆盖落在面片上的那些。
    pub fn fill_box_exact(&mut self, lo: [i32; 3], hi: [i32; 3], band_mm: f64) {
        self.fill_box(lo, hi, self.to_value(-band_mm));

        let t_start = std::time::Instant::now();
        let h = self.config.h;
        let o = self.origin;
        let pad = (band_mm / h).ceil() as i32 + 1;
        let k_v = S::MAX_MAG / band_mm;

        // 盒的世界边界
        let (wx0, wx1) = (o.x + lo[0] as f64 * h, o.x + hi[0] as f64 * h);
        let (wy0, wy1) = (o.y + lo[1] as f64 * h, o.y + hi[1] as f64 * h);
        let (wz0, wz1) = (o.z + lo[2] as f64 * h, o.z + hi[2] as f64 * h);

        // 6 个面片覆盖的体素范围（各轴 pad 层厚）
        let (x0, x1) = (lo[0] - pad, hi[0] + pad);
        let (y0, y1) = (lo[1] - pad, hi[1] + pad);
        let (z0, z1) = (lo[2] - pad, hi[2] + pad);

        // ── ① 枚举面片覆盖的叶并去重 ──
        //
        // 棱和角上的叶会被两三个面同时覆盖，不去重就会重复处理——而并行时
        // 第二次读到的是**第一次之前的旧值**，后写的会覆盖前面的精确值。
        let (lx0, lx1) = (x0 >> LEAF_LOG2, (x1 + LEAF_DIM - 1) >> LEAF_LOG2);
        let (ly0, ly1) = (y0 >> LEAF_LOG2, (y1 + LEAF_DIM - 1) >> LEAF_LOG2);
        let (lz0, lz1) = (z0 >> LEAF_LOG2, (z1 + LEAF_DIM - 1) >> LEAF_LOG2);
        let mut leaves: Vec<[i32; 3]> = Vec::new();
        // ±X 板
        for lx in lx0..((lo[0] + pad + LEAF_DIM - 1) >> LEAF_LOG2) {
            for ly in ly0..ly1 {
                for lz in lz0..lz1 {
                    leaves.push([lx, ly, lz]);
                }
            }
        }
        for lx in ((hi[0] - pad) >> LEAF_LOG2)..lx1 {
            for ly in ly0..ly1 {
                for lz in lz0..lz1 {
                    leaves.push([lx, ly, lz]);
                }
            }
        }
        // ±Y 板
        for ly in ly0..((lo[1] + pad + LEAF_DIM - 1) >> LEAF_LOG2) {
            for lx in lx0..lx1 {
                for lz in lz0..lz1 {
                    leaves.push([lx, ly, lz]);
                }
            }
        }
        for ly in ((hi[1] - pad) >> LEAF_LOG2)..ly1 {
            for lx in lx0..lx1 {
                for lz in lz0..lz1 {
                    leaves.push([lx, ly, lz]);
                }
            }
        }
        // ±Z 板
        for lz in lz0..((lo[2] + pad + LEAF_DIM - 1) >> LEAF_LOG2) {
            for lx in lx0..lx1 {
                for ly in ly0..ly1 {
                    leaves.push([lx, ly, lz]);
                }
            }
        }
        for lz in ((hi[2] - pad) >> LEAF_LOG2)..lz1 {
            for lx in lx0..lx1 {
                for ly in ly0..ly1 {
                    leaves.push([lx, ly, lz]);
                }
            }
        }
        leaves.sort_unstable();
        leaves.dedup();

        // 一片叶是否可能被面片碰到（叶整体在三个方向的"内芯"里就一定碰不到）
        let touches = |leaf: [i32; 3]| -> bool {
            let a = [leaf[0] << LEAF_LOG2, leaf[1] << LEAF_LOG2, leaf[2] << LEAF_LOG2];
            let b = [a[0] + LEAF_DIM, a[1] + LEAF_DIM, a[2] + LEAF_DIM];
            // 与扫描范围相交
            if b[0] <= x0 || a[0] >= x1 || b[1] <= y0 || a[1] >= y1 || b[2] <= z0 || a[2] >= z1 {
                return false;
            }
            // 与任一板相交
            a[0] < lo[0] + pad
                || b[0] > hi[0] - pad
                || a[1] < lo[1] + pad
                || b[1] > hi[1] - pad
                || a[2] < lo[2] + pad
                || b[2] > hi[2] - pad
        };

        // ── ② 并行：每叶产出一份 512 值 ──
        let grid: &Self = self;
        let n = leaves.len();
        let per = 64usize; // 每片处理 64 个叶（512 字节 × 64 = 32 KB，留在 L2）
        let slices = n.div_ceil(per).max(1);
        let parts = crate::par::map_indexed(slices, |si| {
            let a = si * per;
            let b = ((si + 1) * per).min(n);
            let mut out: Vec<([i32; 3], Box<[S; LEAF_VOXELS]>)> = Vec::with_capacity(b - a);
            if a >= b {
                return out;
            }
            for &leaf in &leaves[a..b] {
                if !touches(leaf) {
                    continue;
                }
                // 初值：按叶的现状取 512 个（tile / Constant / Dense）
                let bl = [
                    leaf[0] << LEAF_LOG2,
                    leaf[1] << LEAF_LOG2,
                    leaf[2] << LEAF_LOG2,
                ];
                let mut vals: Box<[S; LEAF_VOXELS]> = match grid.leaf_view_with_slot(bl).0 {
                    None => Box::new([grid.background; LEAF_VOXELS]),
                    Some(LeafView::Tile(v)) => Box::new([v; LEAF_VOXELS]),
                    Some(LeafView::Leaf(Leaf::Constant(v))) => Box::new([*v; LEAF_VOXELS]),
                    Some(LeafView::Leaf(Leaf::Dense(arr))) => (**arr).into(),
                };
                let mut any = false;
                for lz in 0..LEAF_DIM {
                    let k = bl[2] + lz;
                    if k < z0 || k >= z1 {
                        continue;
                    }
                    for ly in 0..LEAF_DIM {
                        let j = bl[1] + ly;
                        if j < y0 || j >= y1 {
                            continue;
                        }
                        for lx in 0..LEAF_DIM {
                            let i = bl[0] + lx;
                            if i < x0 || i >= x1 {
                                continue;
                            }
                            // 只在 6 个板之一里才算（内芯的体素保持饱和值）
                            if !(i < lo[0] + pad
                                || i >= hi[0] - pad
                                || j < lo[1] + pad
                                || j >= hi[1] - pad
                                || k < lo[2] + pad
                                || k >= hi[2] - pad)
                            {
                                continue;
                            }
                            let cx = o.x + (i as f64 + 0.5) * h;
                            let cy = o.y + (j as f64 + 0.5) * h;
                            let cz = o.z + (k as f64 + 0.5) * h;
                            let dx = (wx0 - cx).max(cx - wx1);
                            let dy = (wy0 - cy).max(cy - wy1);
                            let dz = (wz0 - cz).max(cz - wz1);
                            let d = dx.max(dy).max(dz);
                            if d.abs() < band_mm {
                                let idx = (lx | (ly << LEAF_LOG2) | (lz << (LEAF_LOG2 * 2))) as usize;
                                vals[idx] = S::from_f64_round(d * k_v);
                                any = true;
                            }
                        }
                    }
                }
                if any {
                    out.push((leaf, vals));
                }
            }
            out
        });

        // ── ③ 串行整块写回 ──
        let t_eval = std::time::Instant::now();
        let mut wc = WriteCache::default();
        let mut n_write = 0usize;
        let mut touched: Vec<u32> = Vec::new();
        for part in parts {
            for (leaf, vals) in part {
                n_write += LEAF_VOXELS;
                self.blit_leaf(leaf, &vals[..], &mut wc);
                if let Some(li) = wc.leaf_index() {
                    touched.push(li);
                }
            }
        }
        let t_write = std::time::Instant::now();
        // **只折叠写过的叶，不做全量 `prune()`**。
        //
        // 全量版要遍历整棵树 + 重算所有 level-1 的 blind。建场刚结束时树里只有
        // `fill_box` 造的 tile 和这里刚写的边界叶，前者不需要折叠、后者**不可能
        // 全同**（`any` 保证每片叶至少有一个精确值）。实测全量 prune 4.7 ms/轮，
        // 占建场剩下的四分之一。
        //
        // blind 已经在 `blit_leaf` 里标脏（`u8::MAX`），所以 l1 列表传空。
        touched.sort_unstable();
        touched.dedup();
        self.prune_leaves(&touched, &[]);
        if crate::cut::prof_enabled() {
            use std::sync::atomic::Ordering::Relaxed;
            PROF_BOX[0].fetch_add((t_eval - t_start).as_nanos() as u64, Relaxed);
            PROF_BOX[1].fetch_add((t_write - t_eval).as_nanos() as u64, Relaxed);
            PROF_BOX[2].fetch_add(t_write.elapsed().as_nanos() as u64, Relaxed);
            PROF_BOX_N.fetch_add(n_write as u64, Relaxed);
        }
    }

    /// 把一个轴对齐的体素盒子填成同一个值。`hi` 为**开区间**。
    ///
    /// 必须分层进行——否则填一个 2000³ 的实心毛坯要 8e9 次写入。
    /// 完全落在盒内的槽直接写成 tile，只有边界壳才逐级下降。
    ///
    /// > 覆盖已有子节点时会替换整个子树（旧节点成为孤儿，内存不回收）。
    /// > v1 只在建毛坯/夹具时调用，不重复调用；真需要回收时再加 `compact()`。
    pub fn fill_box(&mut self, lo: [i32; 3], hi: [i32; 3], v: S) {
        if lo[0] >= hi[0] || lo[1] >= hi[1] || lo[2] >= hi[2] {
            return;
        }
        self.grow_bounds(lo, hi);
        let r_lo = [
            lo[0].div_euclid(ROOT_SPAN),
            lo[1].div_euclid(ROOT_SPAN),
            lo[2].div_euclid(ROOT_SPAN),
        ];
        let r_hi = [
            (hi[0] - 1).div_euclid(ROOT_SPAN),
            (hi[1] - 1).div_euclid(ROOT_SPAN),
            (hi[2] - 1).div_euclid(ROOT_SPAN),
        ];
        for rz in r_lo[2]..=r_hi[2] {
            for ry in r_lo[1]..=r_hi[1] {
                for rx in r_lo[0]..=r_hi[0] {
                    let rc = [rx, ry, rz];
                    let base = [rx * ROOT_SPAN, ry * ROOT_SPAN, rz * ROOT_SPAN];
                    if tile_safe(base, ROOT_SPAN, lo, hi) {
                        self.root.insert(rc, RootEntry::Tile(v));
                    } else {
                        // 紧贴边界的块不能短路成 tile——必须下降到叶级，
                        // 让"最后一个实体体素 ↔ 盒外背景"那条符号变化有叶子覆盖。
                        let l2i = self.ensure_l2(rc);
                        self.fill_l2(l2i, base, lo, hi, v);
                    }
                }
            }
        }
    }

    fn fill_l2(&mut self, l2i: usize, base: [i32; 3], lo: [i32; 3], hi: [i32; 3], v: S) {
        for sz in 0..L2_DIM {
            for sy in 0..L2_DIM {
                for sx in 0..L2_DIM {
                    let slot = (sx as usize) | ((sy as usize) << L2_LOG2) | ((sz as usize) << (L2_LOG2 * 2));
                    let sbase = [
                        base[0] + sx * L1_SPAN,
                        base[1] + sy * L1_SPAN,
                        base[2] + sz * L1_SPAN,
                    ];
                    if !overlaps(sbase, L1_SPAN, lo, hi) {
                        continue;
                    }
                    if tile_safe(sbase, L1_SPAN, lo, hi) {
                        // 整槽同值：有子树就整棵重填，否则直接写成 tile
                        if let Some(l1i) = self.level2[l2i].child(slot) {
                            self.level1[l1i as usize] = Level1::new(v);
                        } else {
                            self.level2[l2i].set_tile(slot, v);
                        }
                    } else {
                        let l1i = self.ensure_l1(l2i, slot);
                        self.fill_l1(l1i, sbase, lo, hi, v);
                    }
                }
            }
        }
    }

    fn fill_l1(&mut self, l1i: usize, base: [i32; 3], lo: [i32; 3], hi: [i32; 3], v: S) {
        for sz in 0..L1_DIM {
            for sy in 0..L1_DIM {
                for sx in 0..L1_DIM {
                    let slot = (sx as usize) | ((sy as usize) << L1_LOG2) | ((sz as usize) << (L1_LOG2 * 2));
                    let sbase = [
                        base[0] + sx * LEAF_DIM,
                        base[1] + sy * LEAF_DIM,
                        base[2] + sz * LEAF_DIM,
                    ];
                    if !overlaps(sbase, LEAF_DIM, lo, hi) {
                        continue;
                    }
                    if tile_safe(sbase, LEAF_DIM, lo, hi) {
                        // 整叶同值：有叶就替换成常量叶，否则写成 tile
                        if let Some(li) = self.level1[l1i].child(slot) {
                            self.leaves[li as usize] = Leaf::Constant(v);
                        } else {
                            self.level1[l1i].set_tile(slot, v);
                        }
                    } else if fully_inside(sbase, LEAF_DIM, lo, hi) {
                        // 整叶同值但紧贴边界 → 必须存在叶（否则提取会在实体边界漏洞），
                        // 但建成**常量叶**只要 16 字节，不必逐体素展开成 528 字节的密集叶。
                        // 200mm 毛坯的边界层有 25 万个叶槽，这一条把 130 MB 压回 4 MB。
                        if let Some(li) = self.level1[l1i].child(slot) {
                            self.leaves[li as usize] = Leaf::Constant(v);
                        } else {
                            let idx = self.leaves.len();
                            self.leaves.push(Leaf::Constant(v));
                            self.grow_creator();
                            self.level1[l1i].insert_child(slot, idx as u32);
                        }
                    } else {
                        let li = self.ensure_leaf(l1i, slot);
                        for z in 0..LEAF_DIM {
                            for y in 0..LEAF_DIM {
                                for x in 0..LEAF_DIM {
                                    let p = [sbase[0] + x, sbase[1] + y, sbase[2] + z];
                                    if contains(p, lo, hi) {
                                        self.leaves[li].set(leaf_index(p), v);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// 把同值的 `Dense` 叶折叠回 `Constant`，并重算盲元数据。
    ///
    /// **这是正确性要求不是内存优化**：切空的叶若仍是 `Dense`，碰撞检测会
    /// 认为那里还有材料（ADR-004 §5.4）。切削引擎必须在每段之后对脏砖调用。
    pub fn prune(&mut self) {
        for leaf in &mut self.leaves {
            if let Some(v) = leaf.uniform_value() {
                *leaf = Leaf::Constant(v);
            }
        }
        // 盲元数据依赖叶内容，prune 之后统一重算
        let mut l1s = std::mem::take(&mut self.level1);
        for l1 in &mut l1s {
            l1.refresh_blind(&self.leaves);
        }
        self.level1 = l1s;
    }

    /// **只** prune 指定的叶，并且只把涉及的 level-1 块的盲元数据标脏。
    ///
    /// [`Self::prune`] 是全量的：遍历所有叶 + 所有 level1 块重算 blind。
    /// 建模时无所谓，但切削热路径上每刀调一次就是灾难——50×50×100 铣光
    /// 实测每刀要扫 3100 万次（`15 个 level1 × 4096 叶 × 512 体素`），
    /// 占掉 10.3 ms 里的绝大部分。这里只碰真正写过的叶。
    ///
    /// 这里**不重算** blind，只把它标成"未知"（`u8::MAX`）。理由：blind 目前
    /// 没有任何读取者，而重算是本条路径上最大的开销——铣光实测 200 刀里
    /// **1361 µs/刀（合计 272 ms，占全程 79%）** 全花在这个重算上。
    /// 标脏是诚实的：`blind_min_abs()` 会返回 `None`（"需重算"），
    /// 需要精确值的调用方再走一次 [`Self::prune`]，那是离线操作。
    pub fn prune_touched(&mut self, leaves: &[[i32; 3]], l1s: &[usize]) {
        for &bl in leaves {
            let Some(li) = self.leaf_slot(bl) else { continue };
            if let Some(v) = self.leaves[li].uniform_value() {
                self.leaves[li] = Leaf::Constant(v);
            }
        }
        for &i in l1s {
            self.level1[i].blind_min_abs = u8::MAX;
        }
    }

    /// 同 [`Self::prune_touched`]，但直接吃**叶下标**，跳过三级查找。
    ///
    /// 切削的落盘本来就知道每个叶在自己数组里的下标（`LeafBlock::slot`，
    /// 或 `WriteCache` 刚写过的那个），再按坐标查一遍是白付一次
    /// `root 哈希 → l2 → l1`：铣光实测每次约 67 ns，一刀 800 个叶就是 54 µs。
    ///
    /// 下标必须来自**同一次**落盘（`leaves` 只增不搬，下标在 push 之后仍然有效）。
    pub fn prune_leaves(&mut self, li: &[u32], l1s: &[usize]) {
        for &i in li {
            if let Some(v) = self.leaves[i as usize].uniform_value() {
                self.leaves[i as usize] = Leaf::Constant(v);
            }
        }
        for &i in l1s {
            self.level1[i].blind_min_abs = u8::MAX;
        }
    }

    /// 叶在 `leaves` 里的下标（只读，不创建）。参数是**叶坐标**（`ijk >> 3`）。
    pub(crate) fn leaf_slot(&self, leaf: [i32; 3]) -> Option<usize> {
        let base = [
            leaf[0] << LEAF_LOG2,
            leaf[1] << LEAF_LOG2,
            leaf[2] << LEAF_LOG2,
        ];
        let entry = *self.root.get(&root_coord(base))?;
        let l2i = match entry {
            RootEntry::Tile(_) => return None,
            RootEntry::Node(i) => i as usize,
        };
        let l2 = &self.level2[l2i];
        let l1i = l2.child(l2_slot(l1_coord(base)))? as usize;
        let l1 = &self.level1[l1i];
        let li = l1.child(l1_slot(leaf))? as usize;
        Some(li)
    }

    /// 取某个体素所在的叶
    pub fn leaf_at(&self, ijk: [i32; 3]) -> Option<&Leaf<S>> {
        let rc = root_coord(ijk);
        let entry = self.root.get(&rc)?;
        let l2i = match entry {
            RootEntry::Tile(_) => return None,
            RootEntry::Node(i) => *i,
        };
        let l2 = &self.level2[l2i as usize];
        let s2 = l2_slot(l1_coord(ijk));
        let l1i = l2.child(s2)?;
        let l1 = &self.level1[l1i as usize];
        let s1 = l1_slot(leaf_coord(ijk));
        let li = l1.child(s1)?;
        self.leaves.get(li as usize)
    }

    /// 取某个 level-1 节点（盲元数据粗筛用）
    pub fn level1_at(&self, ijk: [i32; 3]) -> Option<&Level1<S>> {
        let rc = root_coord(ijk);
        let l2i = match self.root.get(&rc)? {
            RootEntry::Tile(_) => return None,
            RootEntry::Node(i) => *i,
        };
        let l2 = &self.level2[l2i as usize];
        let l1i = l2.child(l2_slot(l1_coord(ijk)))?;
        self.level1.get(l1i as usize)
    }

    /// 遍历所有**真实存在的叶**，给出叶坐标。
    ///
    /// 这是索引的逆运算：`root 坐标 + level2 槽 → level-1 坐标 + level1 槽 → 叶坐标`。
    /// 提取网格时必须用它——树本身不存"叶在哪"，只有正向下降。
    ///
    /// 只走活动槽（位掩码的置位），不扫全表：473 个 level-1 节点时，
    /// 扫全表要 473×4096 次，走置位只要 473×活动槽数。
    /// 只遍历与体素范围 `[rlo, rhi]` 相交的叶（**三层都剪枝**）。
    ///
    /// `for_each_leaf` 无条件走遍整棵树——LOD 的分块重建（`TileCache`）本来只想
    /// 重算被切到的那一块，结果每次都要付全树的走查 + 每片叶的 `leaf_flags`。
    /// 实测逐刀更新 **14.7 ms**，比"干脆全量重提一次"（10 ms）还慢，就是这个原因。
    ///
    /// 叶 8 体素、l1 128、l2 4096，所以三层各剪一次能按体积比例砍掉大部分。
    pub fn for_each_leaf_in(
        &self,
        rlo: [i32; 3],
        rhi: [i32; 3],
        mut f: impl FnMut([i32; 3], &Leaf<S>),
    ) {
        let inside = |c: [i32; 3], lo: [i32; 3], hi: [i32; 3]| {
            (0..3).all(|a| c[a] >= lo[a] && c[a] <= hi[a])
        };
        let (r0, r1) = (root_coord(rlo), root_coord(rhi));
        let (l0, l1h) = (l1_coord(rlo), l1_coord(rhi));
        let (f0, f1) = (leaf_coord(rlo), leaf_coord(rhi));
        let mut l2_slots = Vec::new();
        let mut l1_slots = Vec::new();
        for (rc, entry) in &self.root {
            if !inside(*rc, r0, r1) {
                continue;
            }
            let RootEntry::Node(l2i) = entry else {
                continue;
            };
            let l2 = &self.level2[*l2i as usize];
            l2_slots.clear();
            for_each_set_slot(&l2.mask, |s| l2_slots.push(s));
            for &s2 in &l2_slots {
                let Some(l1i) = l2.child(s2) else { continue };
                let l1c = l1_coord_from_slot(*rc, s2);
                if !inside(l1c, l0, l1h) {
                    continue;
                }
                let l1 = &self.level1[l1i as usize];
                l1_slots.clear();
                for_each_set_slot(&l1.mask, |s| l1_slots.push(s));
                for &s1 in &l1_slots {
                    let Some(li) = l1.child(s1) else { continue };
                    let lc = leaf_coord_from_slot(l1c, s1);
                    if !inside(lc, f0, f1) {
                        continue;
                    }
                    f(lc, &self.leaves[li as usize]);
                }
            }
        }
    }

    pub fn for_each_leaf(&self, mut f: impl FnMut([i32; 3], &Leaf<S>)) {
        for (rc, entry) in &self.root {
            let RootEntry::Node(l2i) = entry else {
                continue; // tile 覆盖的区域没有叶
            };
            let l2 = &self.level2[*l2i as usize];
            let mut l2_slots = Vec::new();
            for_each_set_slot(&l2.mask, |s| l2_slots.push(s));
            for s2 in l2_slots {
                let Some(l1i) = l2.child(s2) else { continue };
                let l1c = l1_coord_from_slot(*rc, s2);
                let l1 = &self.level1[l1i as usize];
                let mut l1_slots = Vec::new();
                for_each_set_slot(&l1.mask, |s| l1_slots.push(s));
                for s1 in l1_slots {
                    let Some(li) = l1.child(s1) else { continue };
                    f(leaf_coord_from_slot(l1c, s1), &self.leaves[li as usize]);
                }
            }
        }
    }
}

/// `root 坐标 + level-2 槽 → level-1 坐标`
#[inline]
fn l1_coord_from_slot(rc: [i32; 3], s2: usize) -> [i32; 3] {
    let x = (s2 & (L2_DIM as usize - 1)) as i32;
    let y = ((s2 >> L2_LOG2) & (L2_DIM as usize - 1)) as i32;
    let z = ((s2 >> (L2_LOG2 * 2)) & (L2_DIM as usize - 1)) as i32;
    [
        rc[0] * L2_DIM + x,
        rc[1] * L2_DIM + y,
        rc[2] * L2_DIM + z,
    ]
}

/// `level-1 坐标 + level-1 槽 → 叶坐标`
#[inline]
fn leaf_coord_from_slot(l1c: [i32; 3], s1: usize) -> [i32; 3] {
    let x = (s1 & (L1_DIM as usize - 1)) as i32;
    let y = ((s1 >> L1_LOG2) & (L1_DIM as usize - 1)) as i32;
    let z = ((s1 >> (L1_LOG2 * 2)) & (L1_DIM as usize - 1)) as i32;
    [
        l1c[0] * L1_DIM + x,
        l1c[1] * L1_DIM + y,
        l1c[2] * L1_DIM + z,
    ]
}

#[cfg(test)]
mod uniform_value_tests {
    use super::*;

    fn dense(f: impl Fn(usize) -> i8) -> Leaf<i8> {
        let mut a = [0i8; LEAF_VOXELS];
        for (i, v) in a.iter_mut().enumerate() {
            *v = f(i);
        }
        Leaf::Dense(Box::new(a))
    }

    #[test]
    fn all_equal_is_uniform() {
        assert_eq!(dense(|_| 7).uniform_value(), Some(7));
    }

    #[test]
    fn constant_is_uniform() {
        assert_eq!(Leaf::<i8>::Constant(-3).uniform_value(), Some(-3));
    }

    /// **只有最后一个不同**——按字（u64）分块比较时最典型的漏点：
    /// 512 = 64×8 没有余数，但只要实现写成 `chunks_exact(8)` 之后忘了查尾巴，
    /// 或者干脆按 `len - len % 8` 截断，这条就会漏。
    #[test]
    fn last_byte_differs_is_not_uniform() {
        let l = dense(|i| if i == LEAF_VOXELS - 1 { 8 } else { 7 });
        assert_eq!(l.uniform_value(), None);
    }

    #[test]
    fn first_byte_differs_is_not_uniform() {
        let l = dense(|i| if i == 0 { 8 } else { 7 });
        assert_eq!(l.uniform_value(), None);
    }

    /// 每个 8 字节块的**首字节**不同——抓"只比较块首、不比较块内其余 7 个"。
    #[test]
    fn every_word_head_differs_is_not_uniform() {
        let l = dense(|i| if i % 8 == 0 { 8 } else { 7 });
        assert_eq!(l.uniform_value(), None);
    }

    /// 每个 8 字节块的**末字节**不同——抓"块内只比前 7 个"（少比一个）。
    #[test]
    fn every_word_tail_differs_is_not_uniform() {
        let l = dense(|i| if i % 8 == 7 { 8 } else { 7 });
        assert_eq!(l.uniform_value(), None);
    }

    /// 只有正中一个不同——抓"只比头尾"这类偷懒实现。
    #[test]
    fn single_middle_byte_differs_is_not_uniform() {
        let mid = LEAF_VOXELS / 2;
        let l = dense(|i| if i == mid { 8 } else { 7 });
        assert_eq!(l.uniform_value(), None);
    }
}

/// `a` 里**不等于** `v` 的元素个数。
///
/// 为什么不直接 `a.iter().filter(|&&x| x != v).count()`：这个模式在 `[i8; 512]`
/// 上 LLVM 不肯向量化——i8 比较的结果要归约成 usize 计数，中间没有可用的 mask
/// 指令，退化成逐字节 `load + cmp + adc`（512 次迭代）。
///
/// 改成 8 个一组整体比较（`[S; 8]` 的 `!=` 编译成一次 8/16 字节比较），
/// 全同的块直接跳过。整叶短路产生的叶**绝大多数是全同的**（整叶被埋进刀体、
/// 整叶变同一个饱和值），所以快路径命中率极高：铣光一刀 580 次整叶填值，
/// 从 512 次迭代降到 64 次。
#[inline]
pub fn count_ne<S: Scalar>(a: &[S; LEAF_VOXELS], v: S) -> usize {
    // **W 保持 8，不要跟着 `uniform_value` 提到 32。**
    //
    // 这两个函数的"块内不一致"代价完全相反：
    // · `uniform_value` 只判全同 → 不一致就**立刻返回**，W 越大越快；
    // · `count_ne` 要**数清楚**有多少个不同 → 不一致时得逐个数，
    //   W 越大一次要数的越多。实测把 W 提到 32 后铣光从 48 涨到 56 ms。
    const W: usize = 8;
    let pat = [v; W];
    let mut n = 0usize;
    let mut i = 0;
    // 主循环按 8 个一组（`LEAF_VOXELS` 是 512，正好整除，但别依赖这个）
    while i + W <= LEAF_VOXELS {
        if a[i..i + W] != pat {
            for j in 0..W {
                n += (a[i + j] != v) as usize;
            }
        }
        i += W;
    }
    while i < LEAF_VOXELS {
        n += (a[i] != v) as usize;
        i += 1;
    }
    n
}

#[cfg(test)]
mod count_ne_tests {
    use super::*;

    fn arr(f: impl Fn(usize) -> i8) -> [i8; LEAF_VOXELS] {
        let mut a = [0i8; LEAF_VOXELS];
        for (i, v) in a.iter_mut().enumerate() {
            *v = f(i);
        }
        a
    }

    #[test]
    fn all_equal_counts_zero() {
        assert_eq!(count_ne(&arr(|_| 7), 7), 0);
    }

    #[test]
    fn all_different_counts_all() {
        assert_eq!(count_ne(&arr(|_| 8), 7), LEAF_VOXELS);
    }

    #[test]
    fn half_differs() {
        let a = arr(|i| if i < LEAF_VOXELS / 2 { 7 } else { 8 });
        assert_eq!(count_ne(&a, 7), LEAF_VOXELS / 2);
    }

    /// 块内**只差最后一个**——8 个一组比较最典型的漏点。
    #[test]
    fn last_element_of_each_word() {
        let a = arr(|i| if i % 8 == 7 { 8 } else { 7 });
        assert_eq!(count_ne(&a, 7), LEAF_VOXELS / 8);
    }

    #[test]
    fn first_element_of_each_word() {
        let a = arr(|i| if i % 8 == 0 { 8 } else { 7 });
        assert_eq!(count_ne(&a, 7), LEAF_VOXELS / 8);
    }

    /// 整个数组只有正中一个不同。
    #[test]
    fn single_middle_element() {
        let mid = LEAF_VOXELS / 2;
        let a = arr(|i| if i == mid { 8 } else { 7 });
        assert_eq!(count_ne(&a, 7), 1);
    }

    /// 与朴素实现对拍（随机模式）——独立 oracle，不是自证。
    #[test]
    fn matches_naive_on_patterned_data() {
        let a = arr(|i| ((i * 37 + 11) % 5) as i8);
        for v in -3i8..=3 {
            let naive = a.iter().filter(|&&x| x != v).count();
            assert_eq!(count_ne(&a, v), naive, "v = {v}");
        }
    }
}
