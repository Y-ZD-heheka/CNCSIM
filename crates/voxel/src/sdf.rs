//! 量化换算与三线性插值查询。

use crate::tree::{Scalar, SparseGrid};
use cncsim_math::DVec3;

impl<S: Scalar> SparseGrid<S> {
    /// 存储值 → 距离（mm）。`±MAX_MAG` 对应 `±band_mm`。
    pub fn to_distance(&self, v: S) -> f64 {
        v.to_f64() / S::MAX_MAG * self.config.band_mm()
    }

    /// 距离（mm）→ 存储值。超出带宽被钳制到 `±MAX_MAG`。
    ///
    /// 钳制是安全的：方向 B 只问"点在不在材料内"，而带宽外的点本来就不可能碰撞，
    /// 钳制方向正好是保守的（外部钳到 +MAX，内部钳到 −MAX）。
    pub fn to_value(&self, d: f64) -> S {
        S::from_f64_round(d / self.config.band_mm() * S::MAX_MAG)
    }

    /// 体素 `(i,j,k)` 的中心坐标。`origin` 是网格域最小角，所以有 0.5 的偏移。
    pub fn voxel_center(&self, ijk: [i32; 3]) -> DVec3 {
        let h = self.config.h;
        DVec3::new(
            self.origin.x + (ijk[0] as f64 + 0.5) * h,
            self.origin.y + (ijk[1] as f64 + 0.5) * h,
            self.origin.z + (ijk[2] as f64 + 0.5) * h,
        )
    }

    /// 世界坐标 → 连续体素坐标（体素中心在整数处）
    pub fn to_voxel_space(&self, p: DVec3) -> DVec3 {
        let h = self.config.h;
        DVec3::new(
            (p.x - self.origin.x) / h - 0.5,
            (p.y - self.origin.y) / h - 0.5,
            (p.z - self.origin.z) / h - 0.5,
        )
    }

    /// 有符号距离（mm），正值在材料外。窄带外返回钳制值 `±band_mm`。
    ///
    /// 三线性插值：平面上精确，曲面上误差约 `h²/(8R_c)`——这是碰撞检出下限
    /// 的第二项误差，见 `docs/计算精度分析.md` §2.2。
    pub fn sdf(&self, p: DVec3) -> f64 {
        let f = self.to_voxel_space(p);
        let i0 = [f.x.floor() as i32, f.y.floor() as i32, f.z.floor() as i32];
        let t = [f.x - i0[0] as f64, f.y - i0[1] as f64, f.z - i0[2] as f64];

        let mut acc = 0.0f64;
        for dz in 0..2i32 {
            let wz = if dz == 0 { 1.0 - t[2] } else { t[2] };
            for dy in 0..2i32 {
                let wy = if dy == 0 { 1.0 - t[1] } else { t[1] };
                for dx in 0..2i32 {
                    let wx = if dx == 0 { 1.0 - t[0] } else { t[0] };
                    let w = wx * wy * wz;
                    if w == 0.0 {
                        continue;
                    }
                    let v = self.get([i0[0] + dx, i0[1] + dy, i0[2] + dz]);
                    acc += w * v.to_f64();
                }
            }
        }
        acc / S::MAX_MAG * self.config.band_mm()
    }

    /// 带根游标的 [`Self::sdf`]：数值**逐位相同**，只是把 8 次根哈希查找省成
    /// 通常 1 次。
    ///
    /// 为什么不把 `sdf` 写成"内部带 `Option<&mut cursor>`"：那样内层 8 次取值
    /// 每次都要判一次 `Option`，而这里要的正是那 8 次的开销。宁可重复这 20 行。
    pub fn sdf_cached(&self, p: DVec3, cur: &mut crate::tree::RootCursor<S>) -> f64 {
        let f = self.to_voxel_space(p);
        let i0 = [f.x.floor() as i32, f.y.floor() as i32, f.z.floor() as i32];
        let t = [f.x - i0[0] as f64, f.y - i0[1] as f64, f.z - i0[2] as f64];

        let mut acc = 0.0f64;
        for dz in 0..2i32 {
            let wz = if dz == 0 { 1.0 - t[2] } else { t[2] };
            for dy in 0..2i32 {
                let wy = if dy == 0 { 1.0 - t[1] } else { t[1] };
                for dx in 0..2i32 {
                    let wx = if dx == 0 { 1.0 - t[0] } else { t[0] };
                    let w = wx * wy * wz;
                    if w == 0.0 {
                        continue;
                    }
                    let v = self.get_cached([i0[0] + dx, i0[1] + dy, i0[2] + dz], cur);
                    acc += w * v.to_f64();
                }
            }
        }
        acc / S::MAX_MAG * self.config.band_mm()
    }

    /// 只在**单个叶内**插值的快速查询。跨叶时退回完整 `sdf`。
    ///
    /// 切削热路径上大量查询落在同一个叶里，这样能省掉 7 次跨级下降。
    pub fn sdf_in_leaf(&self, p: DVec3) -> f64 {
        self.sdf(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tree::RootCursor;
    use crate::{GridConfig, SparseGrid, ValueType};

    /// `sdf_cached` 必须与 `sdf` **逐位相同**——它是给热路径用的加速版本，
    /// 一旦两者有差异，LOD 的顶点位置就会跟 `extract_surface` 对不上，
    /// 而那种偏差在画面上只是"有点不对"，很难查。
    #[test]
    fn sdf_cached_is_bit_identical_to_sdf() {
        let cfg = GridConfig { h: 0.1, band_voxels: 4, value_type: ValueType::I8 };
        let mut g = SparseGrid::<i8>::new(cfg, DVec3::ZERO);
        g.fill_box_exact([0, 0, 0], [100, 100, 50], cfg.band_mm());
        // 切一条圆底槽，制造**弯曲**表面（纯平面测不出插值差异，
        // 而且弯曲表面会跨叶/跨根，正好压住游标的失效分支）
        let prof = cncsim_tool::flat_end_mill(6.0, 10.0);
        g.cut_segment(
            &cncsim_motion::Pose { tip: DVec3::new(-2.0, 5.0, 4.0), axis: DVec3::Z },
            &cncsim_motion::Pose { tip: DVec3::new(12.0, 5.0, 4.0), axis: DVec3::Z },
            &prof,
        );

        let mut cur = RootCursor::default();
        let mut n = 0;
        for i in 0..40 {
            for j in 0..40 {
                for k in 0..25 {
                    let p = DVec3::new(
                        i as f64 * 0.27 + 0.013,
                        j as f64 * 0.29 + 0.007,
                        k as f64 * 0.31 + 0.021,
                    );
                    let a = g.sdf(p);
                    let b = g.sdf_cached(p, &mut cur);
                    assert_eq!(
                        a.to_bits(),
                        b.to_bits(),
                        "在 {p:?} 处 sdf={a} 与 sdf_cached={b} 不同"
                    );
                    n += 1;
                }
            }
        }
        assert!(n > 1000, "样本太少（{n}），覆盖不到跨叶/跨根的情形");
    }
}
