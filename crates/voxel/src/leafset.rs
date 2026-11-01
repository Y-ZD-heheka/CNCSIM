//! 叶坐标集合：**位图**（有界时）或哈希集（回退）。
//!
//! ## 为什么不用 `HashSet`
//!
//! 表面提取里每片叶要处理 729 个候选单元格，每个都问一次"这个单元格的最小角
//! 所在的叶是否已分配"。200mm 毛坯有 25 万片叶 → **1.8 亿次查询**。
//!
//! 叶坐标被网格范围限死，所以可以直接映射到位图：200mm @ h=0.1 → 250×250×125 个叶
//! = **7.8 M bit = 1 MB**，查询是"三次减法 + 越界判断 + 一次位测试"，
//! 没有哈希、没有探测链。
//!
//! 万一网格很稀疏（叶分散在很大范围），位图会膨胀——超过 32 M bit（4 MB）时
//! 自动退回哈希集。**两条路径的语义完全一致**，测试覆盖两者。

use crate::fxhash::FxHashSet;

/// 叶坐标集合。
pub struct LeafSet {
    kind: Kind,
}

enum Kind {
    Bits(Bits),
    Set(FxHashSet<[i32; 3]>),
}

struct Bits {
    lo: [i32; 3],
    dim: [usize; 3],
    words: Vec<u64>,
}

/// 位图上限：32 M bit = 4 MB
const MAX_BITS: usize = 32 << 20;

impl LeafSet {
    pub fn from_leaves(leaves: &[[i32; 3]]) -> Self {
        if leaves.is_empty() {
            return Self {
                kind: Kind::Set(FxHashSet::default()),
            };
        }
        let mut lo = [i32::MAX; 3];
        let mut hi = [i32::MIN; 3];
        for k in leaves {
            for a in 0..3 {
                lo[a] = lo[a].min(k[a]);
                hi[a] = hi[a].max(k[a]);
            }
        }
        let mut dim = [0usize; 3];
        let mut total: usize = 1;
        for a in 0..3 {
            dim[a] = (hi[a] as i64 - lo[a] as i64 + 1) as usize;
            total = total.saturating_mul(dim[a]);
        }

        if total > MAX_BITS {
            // 太稀疏，位图不划算
            let mut s = FxHashSet::default();
            for k in leaves {
                s.insert(*k);
            }
            return Self {
                kind: Kind::Set(s),
            };
        }

        let mut words = vec![0u64; total.div_ceil(64)];
        for k in leaves {
            let idx = Self::index(&lo, &dim, *k);
            words[idx >> 6] |= 1u64 << (idx & 63);
        }
        Self {
            kind: Kind::Bits(Bits { lo, dim, words }),
        }
    }

    #[inline]
    fn index(lo: &[i32; 3], dim: &[usize; 3], k: [i32; 3]) -> usize {
        ((k[0] - lo[0]) as usize) * dim[1] * dim[2]
            + ((k[1] - lo[1]) as usize) * dim[2]
            + ((k[2] - lo[2]) as usize)
    }

    #[inline]
    pub fn contains(&self, k: [i32; 3]) -> bool {
        match &self.kind {
            Kind::Bits(b) => {
                for a in 0..3 {
                    let d = k[a] as i64 - b.lo[a] as i64;
                    if d < 0 || d as usize >= b.dim[a] {
                        return false;
                    }
                }
                let idx = Self::index(&b.lo, &b.dim, k);
                (b.words[idx >> 6] >> (idx & 63)) & 1 == 1
            }
            Kind::Set(s) => s.contains(&k),
        }
    }

    /// 是否走了位图路径（测试用）
    pub fn is_bitmap(&self) -> bool {
        matches!(self.kind, Kind::Bits(_))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 位图与哈希集两条路径必须给出**完全相同**的答案。
    fn assert_same(leaves: &[[i32; 3]], probes: &[[i32; 3]]) {
        let bits = LeafSet::from_leaves(leaves);
        let mut set = FxHashSet::default();
        for k in leaves {
            set.insert(*k);
        }
        for p in probes {
            assert_eq!(
                bits.contains(*p),
                set.contains(p),
                "叶集合查询不一致：{p:?}"
            );
        }
    }

    #[test]
    fn bitmap_matches_hashset_on_dense_grid() {
        let mut leaves = Vec::new();
        for x in 0..12 {
            for y in 0..9 {
                for z in 0..5 {
                    leaves.push([x, y, z]);
                }
            }
        }
        let mut probes = Vec::new();
        for x in -2..15 {
            for y in -2..12 {
                for z in -2..8 {
                    probes.push([x, y, z]);
                }
            }
        }
        assert_same(&leaves, &probes);
        assert!(LeafSet::from_leaves(&leaves).is_bitmap());
    }

    #[test]
    fn bitmap_matches_hashset_with_negative_coords() {
        let leaves = vec![[-5, -3, 0], [-4, -3, 0], [7, 9, -11], [0, 0, 0]];
        let mut probes = Vec::new();
        for x in -8..10 {
            for y in -6..12 {
                probes.push([x, y, -11]);
                probes.push([x, y, 0]);
            }
        }
        assert_same(&leaves, &probes);
    }

    #[test]
    fn sparse_falls_back_to_hashset() {
        // 两个相距极远的叶 → 包围盒体积超上限 → 退回哈希集
        let leaves = vec![[0, 0, 0], [1_000_000, 1_000_000, 1_000_000]];
        let s = LeafSet::from_leaves(&leaves);
        assert!(!s.is_bitmap(), "稀疏时应退回哈希集");
        assert!(s.contains([0, 0, 0]));
        assert!(s.contains([1_000_000, 1_000_000, 1_000_000]));
        assert!(!s.contains([500_000, 500_000, 500_000]));
    }

    #[test]
    fn empty_set_contains_nothing() {
        let s = LeafSet::from_leaves(&[]);
        assert!(!s.contains([0, 0, 0]));
        assert!(!s.contains([-1, -1, -1]));
    }
}
