//! 整数键哈希：替掉标准库的 SipHash。
//!
//! ## 为什么自己写
//!
//! 标准库 `HashMap` 默认用 `RandomState`（SipHash-1-3），它是**抗 HashDoS 的设计**——
//! 每次进程启动随机种子，并对每个字节做多轮混合。对"从网络来的不可信键"这是对的；
//! 对我们这种"键就是自己算出来的体素坐标"是纯浪费：**一次查找约 50–100 ns**，
//! 而提取一次 200mm 毛坯要做**上亿次**查找。
//!
//! 项目有"L0–L3 零外部依赖"的硬约束（见 docs/架构设计.md §2.1），所以不引 `hashbrown`/`rustc-hash`，
//! 自己实现 FxHash 风格的一轮混合：**一次查找约 5–10 ns**。
//!
//! ## FxHash 是什么
//!
//! 来自 rustc 的 `rustc_hash`：`h = (h.rotl(5) ^ word) * SEED`。
//! 每个机器字只做一次旋转 + 一次异或 + 一次乘法，乘法提供雪崩。
//! 对整数键分布很均匀，实测碰撞率与 SipHash 相当。
//!
//! **安全性**：不抗 HashDoS。这里的键全部来自程序内部（体素索引、叶坐标），
//! 不存在"攻击者构造碰撞"的场景。**不要把 `FxHashMap` 用在解析外部输入的地方。**
//!
//! ## 附带好处：迭代顺序确定
//!
//! `RandomState` 让 `HashMap` 的迭代顺序每次运行都不同——这会破坏
//! "仿真结果逐位可复现"的硬要求。换成固定种子之后，同样的插入序列给出同样的顺序。

use std::hash::{BuildHasher, Hasher};

/// FxHash 的乘法常数（来自 rustc）。
const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

/// 单轮混合哈希器。
#[derive(Clone, Copy, Debug, Default)]
pub struct FxHasher {
    hash: u64,
}

impl FxHasher {
    #[inline]
    fn add(&mut self, word: u64) {
        self.hash = (self.hash.rotate_left(5) ^ word).wrapping_mul(SEED);
    }
}

impl Hasher for FxHasher {
    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        // 通用回退：按 8 字节一组吃
        let mut chunks = bytes.chunks_exact(8);
        for c in &mut chunks {
            let mut w = [0u8; 8];
            w.copy_from_slice(c);
            self.add(u64::from_le_bytes(w));
        }
        let rest = chunks.remainder();
        if !rest.is_empty() {
            let mut w = [0u8; 8];
            w[..rest.len()].copy_from_slice(rest);
            self.add(u64::from_le_bytes(w));
        }
    }

    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add(i as u64);
    }
    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add(i as u64);
    }
    #[inline]
    fn write_i32(&mut self, i: i32) {
        self.add(i as u64);
    }
    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add(i);
    }
    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add(i as u64);
    }

    #[inline]
    fn finish(&self) -> u64 {
        self.hash
    }
}

/// 固定种子的 `BuildHasher`。
#[derive(Clone, Copy, Debug, Default)]
pub struct FxBuildHasher;

impl BuildHasher for FxBuildHasher {
    type Hasher = FxHasher;
    #[inline]
    fn build_hasher(&self) -> FxHasher {
        FxHasher::default()
    }
}

/// 整数键哈希表。**不抗 HashDoS，只用于程序内部生成的键。**
pub type FxHashMap<K, V> = std::collections::HashMap<K, V, FxBuildHasher>;

/// 整数键哈希集。**不抗 HashDoS，只用于程序内部生成的键。**
pub type FxHashSet<K> = std::collections::HashSet<K, FxBuildHasher>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::hash::Hash;

    fn h(k: [i32; 3]) -> u64 {
        let mut s = FxHasher::default();
        k.hash(&mut s);
        s.finish()
    }

    #[test]
    fn hash_is_deterministic() {
        // 固定种子 → 同键同值。这是"逐位可复现"的前提。
        assert_eq!(h([1, 2, 3]), h([1, 2, 3]));
        assert_eq!(h([-1, -2, -3]), h([-1, -2, -3]));
    }

    #[test]
    fn nearby_keys_do_not_collide_in_low_bits() {
        // 体素坐标的相邻键必须散开，否则线性探测会退化成长链
        let mut low = std::collections::HashSet::new();
        for x in 0..64 {
            low.insert(h([x, 0, 0]) & 0xFF);
        }
        assert!(
            low.len() >= 60,
            "相邻键的低 8 位应基本不重复，实际只有 {} 种",
            low.len()
        );
    }

    #[test]
    fn axis_order_matters() {
        // [1,0,0] 与 [0,1,0] 不能哈希到同一个值（否则大量碰撞）
        assert_ne!(h([1, 0, 0]), h([0, 1, 0]));
        assert_ne!(h([0, 1, 0]), h([0, 0, 1]));
    }

    #[test]
    fn map_roundtrips() {
        let mut m: FxHashMap<[i32; 3], u32> = FxHashMap::default();
        for i in 0..1000i32 {
            m.insert([i, -i, i * 2], i as u32);
        }
        for i in 0..1000i32 {
            assert_eq!(m.get(&[i, -i, i * 2]), Some(&(i as u32)));
        }
        assert_eq!(m.len(), 1000);
        assert_eq!(m.get(&[1, 1, 1]), None);
    }
}
