//! 运动学性质测试：**FK ∘ IK = 恒等**。
//!
//! 与 `anchors.rs` 互补：
//! - `anchors.rs` 用**绝对数值**抓"整体偏了"（如刀轴取反、摆长基准错）
//! - 本文件用**性质**抓"局部区间错"（如某个象限的符号反了）
//!
//! 两者缺一不可：只做往返测试，FK 和 IK 一起错但自洽时会全绿；
//! 只做锚点，覆盖不到整条区间。
//!
//! **`q` 是链顺序 `[C, A, X, Y, Z]`**，一律用 `m.q(&[("X", ..), ..])` 构造。
//!
//! ## 为什么不用 proptest
//!
//! 用过一个版本，后来撤了：`proptest → rusty-fork → quick-error` 与
//! `proptest → tempfile → windows-sys → windows-link` 两条链把 **三个上游锁死的包**
//! 带进依赖图（`quick-error` 1.x 被 rusty-fork 钉死，`windows-link` 被 windows-sys 钉死），
//! 版本守卫因此长期红着。为了三个数值区间引一棵依赖树不划算。
//!
//! 换来的是**更好的性质**：proptest 默认每次随机种子，失败了要等缩例才拿到反例；
//! 这里的种子是常数，**同一份代码永远跑同一批数**，失败信息直接带上第几例与全部关节值。
//! 代价是没有自动缩例——500 例里挑出的反例已经足够小，人眼看得出。

use cncsim_kin::{forward, inverse, MachineTopology};

// ─────────────────────────── 确定性伪随机 ───────────────────────────

/// xorshift64*：够随机、够短、**跨平台逐位一致**（不依赖 `RandomState` 的随机种子）。
struct Rng(u64);

impl Rng {
    const fn new(seed: u64) -> Self {
        // 种子不能是 0（xorshift 的固定点）
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// `[0, 1)`，53 位精度（与 f64 尾数同宽）
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn bool(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }
}

/// 固定种子：改了这个数等于换了测试语料，**不要为了"过"而改**。
const SEED: u64 = 0x5EED_2026_1029;

/// 参数范围覆盖真实加工区间，且避开 A≈0 的奇异区（那里 C 无定义，往返不唯一）。
fn sample(rng: &mut Rng) -> (f64, f64, f64, f64, f64) {
    let x = rng.range(-300.0, 300.0);
    let y = rng.range(-200.0, 200.0);
    let z = rng.range(-250.0, 80.0);
    // A 取 ±[5°, 105°]：跨过 0 的奇异区两侧都覆盖，但样本不落在里面
    let a = if rng.bool() {
        rng.range(-105.0, -5.0)
    } else {
        rng.range(5.0, 105.0)
    };
    let c = rng.range(-180.0, 180.0);
    (x, y, z, a, c)
}

fn q_of(m: &MachineTopology, x: f64, y: f64, z: f64, a: f64, c: f64) -> Vec<f64> {
    m.q(&[("X", x), ("Y", y), ("Z", z), ("A", a), ("C", c)])
}

const CASES: usize = 500;

/// 任意可达位姿：逆解后再正解，必须回到原位姿。
#[test]
fn ik_then_fk_is_identity() {
    let m = MachineTopology::ac_trunnion();
    let mut rng = Rng::new(SEED);

    for case in 0..CASES {
        let (x, y, z, a, c) = sample(&mut rng);
        let q = q_of(&m, x, y, z, a, c);
        let pose0 = forward(&m, &q);

        let q2 = inverse(&m, &pose0, &q).expect("可达位姿应能逆解");
        let pose1 = forward(&m, &q2);

        let tip = pose0.tip.abs_diff(pose1.tip);
        assert!(
            tip < 1e-9,
            "第 {case} 例：刀尖漂移 {tip} mm\nX={x:.9} Y={y:.9} Z={z:.9} A={a:.9} C={c:.9}\nq  = {q:?}\nq2 = {q2:?}"
        );
        let axis = pose0.axis.abs_diff(pose1.axis);
        assert!(
            axis < 1e-9,
            "第 {case} 例：刀轴漂移 {axis}\nX={x:.9} Y={y:.9} Z={z:.9} A={a:.9} C={c:.9}"
        );
    }
}

/// 刀轴恒为单位向量——否则 sd_profile 的距离会有系统性偏差。
#[test]
fn axis_stays_normalized() {
    let m = MachineTopology::ac_trunnion();
    let mut rng = Rng::new(SEED ^ 0xA5A5);

    for case in 0..CASES {
        let (x, y, z, a, c) = sample(&mut rng);
        let pose = forward(&m, &q_of(&m, x, y, z, a, c));
        let len = pose.axis.length();
        assert!(
            (len - 1.0).abs() < 1e-12,
            "第 {case} 例：刀轴长度 {len}\nX={x:.9} Y={y:.9} Z={z:.9} A={a:.9} C={c:.9}"
        );
    }
}

/// 只转 C（A=0）时刀尖不动——刀尖在 C 轴上，绕它转不该改变位置。
#[test]
fn rotating_c_alone_keeps_tip_fixed() {
    let m = MachineTopology::ac_trunnion();
    let p0 = forward(&m, &m.q(&[("C", 0.0)]));
    let mut rng = Rng::new(SEED ^ 0x5A5A);

    for case in 0..CASES {
        let c = rng.range(-180.0, 180.0);
        let p1 = forward(&m, &m.q(&[("C", c)]));
        let d = p0.tip.abs_diff(p1.tip);
        assert!(d < 1e-9, "第 {case} 例：C={c:.9} 时刀尖漂移 {d}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 生成器本身要可复现：同种子同序列，否则"逐位一致"是空话。
    #[test]
    fn rng_is_reproducible_and_in_range() {
        let mut a = Rng::new(SEED);
        let mut b = Rng::new(SEED);
        for _ in 0..1000 {
            let (va, vb) = (a.unit(), b.unit());
            assert_eq!(va, vb, "同种子的序列必须逐位相同");
            assert!((0.0..1.0).contains(&va), "unit() 越界：{va}");
        }
        // 零种子会退化成固定点，构造时要替换掉
        let mut z = Rng::new(0);
        assert!(z.next_u64() != 0, "零种子没有兜底");
    }
}
