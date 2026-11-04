//! 连通块清理：把切下来却还"挂"在场的**悬浮碎屑**当成切屑丢掉。
//!
//! ## 为什么需要
//!
//! 体素 CSG 只做"材料去除"，它不管去除之后的材料还有没有连着。真实切削里，
//! 一刀切过去，被切断的那一小块会**掉下来**（变成切屑）；但在体素场里它会
//! 原地飘着，于是预览里出现一层"悬浮的小方块/薄片"，看起来像没切干净。
//! 越是五轴摆动、大偏置、薄壁的地方越明显。
//!
//! ## 做什么
//!
//! 对材料（`v < 0`）做 6 邻域连通块标记，保留**最大**的那一块（工件本体），
//! 其余整块写成空气。返回被清掉的体素数，调用方可以把它算进去料体积。
//!
//! ## 代价
//!
//! 只跑一次，O(毛坯域体素数)。花瓶 112×112×250 约 310 万格、~0.3 s，
//! 所以放在整段程序跑完之后，而不是每切一刀都做。

use crate::tree::{Scalar, SparseGrid};

impl<S: Scalar> SparseGrid<S> {
    /// 去掉与主体不连通的材料块，返回清掉的体素数。
    ///
    /// 判定"材料"用 `v < 0`（负 = 材料内），与 SDF 约定一致。
    pub fn drop_floating_islands(&mut self) -> usize {
        let Some((lo, hi)) = self.bounds() else {
            return 0;
        };
        let d = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        if d.iter().any(|v| *v <= 1) {
            return 0;
        }
        let total = (d[0] as usize) * (d[1] as usize) * (d[2] as usize);
        let at = |x: i32, y: i32, z: i32| -> usize {
            ((z as usize) * d[1] as usize + y as usize) * d[0] as usize + x as usize
        };

        // 材料位图：true = 材料。顺便统计总量，没有材料就直接返回。
        let mut solid = vec![false; total];
        let mut n_solid = 0usize;
        for z in 0..d[2] {
            for y in 0..d[1] {
                let row = at(0, y, z);
                for x in 0..d[0] {
                    let v = self.get([lo[0] + x, lo[1] + y, lo[2] + z]);
                    if v.to_f64() < 0.0 {
                        solid[row + x as usize] = true;
                        n_solid += 1;
                    }
                }
            }
        }
        if n_solid == 0 {
            return 0;
        }

        // 连通块标记（6 邻域，显式栈，避免递归爆栈）
        let mut label = vec![0u32; total];
        let mut sizes: Vec<usize> = vec![0];
        let mut stack: Vec<usize> = Vec::with_capacity(4096);
        let mut next = 1u32;
        for start in 0..total {
            if !solid[start] || label[start] != 0 {
                continue;
            }
            let id = next;
            next += 1;
            let mut n = 0usize;
            label[start] = id;
            stack.push(start);
            while let Some(i) = stack.pop() {
                n += 1;
                let x = (i % d[0] as usize) as i32;
                let y = ((i / d[0] as usize) % d[1] as usize) as i32;
                let z = (i / (d[0] as usize * d[1] as usize)) as i32;
                let mut push = |xx: i32, yy: i32, zz: i32, stack: &mut Vec<usize>| {
                    if xx < 0 || yy < 0 || zz < 0 || xx >= d[0] || yy >= d[1] || zz >= d[2] {
                        return;
                    }
                    let j = at(xx, yy, zz);
                    if solid[j] && label[j] == 0 {
                        label[j] = id;
                        stack.push(j);
                    }
                };
                push(x - 1, y, z, &mut stack);
                push(x + 1, y, z, &mut stack);
                push(x, y - 1, z, &mut stack);
                push(x, y + 1, z, &mut stack);
                push(x, y, z - 1, &mut stack);
                push(x, y, z + 1, &mut stack);
            }
            sizes.push(n);
        }

        // 保留最大块
        let mut best = 1u32;
        for (id, &s) in sizes.iter().enumerate().skip(1) {
            if s > sizes[best as usize] {
                best = id as u32;
            }
        }
        if sizes.len() <= 2 {
            return 0; // 只有一块
        }

        let clear = S::MAX;
        let mut dropped = 0usize;
        let mut cache = crate::tree::WriteCache::default();
        for z in 0..d[2] {
            for y in 0..d[1] {
                let row = at(0, y, z);
                for x in 0..d[0] {
                    let i = row + x as usize;
                    if solid[i] && label[i] != best {
                        self.set_cached_in_bounds(
                            [lo[0] + x, lo[1] + y, lo[2] + z],
                            clear,
                            &mut cache,
                        );
                        dropped += 1;
                    }
                }
            }
        }
        dropped
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{GridConfig, ValueType};
    use cncsim_math::DVec3;

    #[test]
    fn floating_chip_is_dropped_and_body_kept() {
        let cfg = GridConfig {
            h: 1.0,
            band_voxels: 4,
            value_type: ValueType::I8,
        };
        let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
        // 主体：20×20×10 的块
        g.fill_box_exact([0, 0, 0], [20, 20, 10], cfg.band_mm());
        // 碎屑：一块 3×3×3，与主体隔开 5 格
        g.fill_box_exact([10, 10, 20], [13, 13, 23], cfg.band_mm());

        let before = g.material_volume();
        let n = g.drop_floating_islands();
        let after = g.material_volume();
        assert_eq!(n, 27, "应当正好清掉 3×3×3 的碎屑，实际 {n}");
        assert!(before - after > 20.0, "体积没减少：{before} → {after}");
        // 主体还在
        assert!(g.sdf(DVec3::new(10.0, 10.0, 5.0)) < 0.0, "主体被误删");
        // 碎屑没了
        assert!(
            g.sdf(DVec3::new(11.0, 11.0, 21.0)) > 0.0,
            "碎屑还在：{}",
            g.sdf(DVec3::new(11.0, 11.0, 21.0))
        );
    }
}
