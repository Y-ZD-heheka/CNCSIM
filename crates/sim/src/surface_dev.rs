//! 名义曲面 ↔ 实际毛坯场：逐点有符号偏差。
//!
//! 这是"加工完离原版差多少"最直接的口径：把**设计面**按 `step` 采成点云，
//! 每点查一次毛坯场的 SDF，得到
//!
//! - `d > 0`：该点已经是空气 → 材料被切掉了，`d` 就是**过切深度**；
//! - `d < 0`：该点还在材料里 → 没切到位，`-d` 就是**残料厚度**；
//! - `|d| ≥ band`：超出窄带，读到的是钳制值，只表示"至少差这么多"。
//!
//! 和两种旧口径的分工见 `docs/项目记忆.md` §27：
//! [`crate::compare::compare_with_design`] 比的是**整实体体积**（含内腔、
//! 含"该切没切"的整块料），适合"哪里多切了/少切了"的定位；
//! 本模块比的是**曲面自身的位置**，适合"形状保真度"的验收。
//!
//! 单边采样的固有盲区：它只看设计面上的点，看不到"离设计面很远的地方还留着
//! 一块料"（例如方料四角）。那种问题要么看 `residual_volume_mm3`，要么反向
//! 采样实际曲面，不能只看本模块。

use cncsim_math::DVec3;
use cncsim_voxel::tree::SparseGrid;

/// 逐点有符号偏差（mm）。正值在材料外（已切掉），负值在材料内（没切到）。
///
/// 用 [`SparseGrid::sdf`] 的三线性插值；曲面上的插值误差约 `h²/(8R_c)`，
/// 对 `h=1 mm`、`R=4 mm` 的球头约 0.03 mm，够验收 0.1 mm 级的形状偏差。
pub fn signed_deviation(grid: &SparseGrid<i8>, points: &[DVec3]) -> Vec<f64> {
    points.iter().map(|p| grid.sdf(*p)).collect()
}

/// 偏差统计。占比一律相对 [`Self::points`]。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SurfaceStats {
    pub points: usize,
    /// `|d| ≤ tol`：落在公差带内。
    pub ok: usize,
    /// `d > tol`：过切（材料被切掉太多）。
    pub gouge: usize,
    /// `d < -tol`：残料（没切到位）。
    pub residual: usize,
    /// `|d|` 已到窄带边缘，读数被钳制——真实偏差更大。
    pub saturated: usize,
    pub mean_abs: f64,
    /// `|d|` 的 95 分位。**不带符号**：带符号序列的分位数读出来是
    /// "有 5% 比它更负"，和"偏差多大"不是一回事。
    pub p95_abs: f64,
    pub max_gouge: f64,
    pub max_residual: f64,
}

impl SurfaceStats {
    pub fn ok_frac(&self) -> f64 {
        if self.points == 0 {
            0.0
        } else {
            self.ok as f64 / self.points as f64
        }
    }

    pub fn gouge_frac(&self) -> f64 {
        if self.points == 0 {
            0.0
        } else {
            self.gouge as f64 / self.points as f64
        }
    }

    pub fn residual_frac(&self) -> f64 {
        if self.points == 0 {
            0.0
        } else {
            self.residual as f64 / self.points as f64
        }
    }
}

/// 统计偏差序列。`tol` 是公差带半宽（mm），`band` 是毛坯场的窄带宽度（mm），
/// 用来识别"读数是钳制值"的样本。
pub fn stats(vals: &[f64], tol: f64, band: f64) -> SurfaceStats {
    let mut s = SurfaceStats {
        points: vals.len(),
        ..Default::default()
    };
    if vals.is_empty() {
        return s;
    }
    let mut abs: Vec<f64> = Vec::with_capacity(vals.len());
    let mut sum = 0.0;
    let sat = (band - 1e-6).max(0.0);
    for &d in vals {
        let a = d.abs();
        abs.push(a);
        sum += a;
        if a <= tol {
            s.ok += 1;
        } else if d > 0.0 {
            s.gouge += 1;
            s.max_gouge = s.max_gouge.max(d);
        } else {
            s.residual += 1;
            s.max_residual = s.max_residual.max(a);
        }
        if a >= sat {
            s.saturated += 1;
        }
    }
    s.mean_abs = sum / vals.len() as f64;
    abs.sort_by(f64::total_cmp);
    let k = ((vals.len() as f64 * 0.95) as usize).min(abs.len() - 1);
    s.p95_abs = abs[k];
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stats_separates_gouge_residual_and_saturation() {
        // 正值 = 过切，负值 = 残料；±4 是窄带钳制值
        let v = [0.0, 0.05, -0.05, 0.3, -0.8, 4.0, -4.0];
        let s = stats(&v, 0.1, 4.0);
        assert_eq!(s.points, 7);
        assert_eq!(s.ok, 3, "|d|≤0.1 的三个");
        assert_eq!(s.gouge, 2, "0.3 与 +4");
        assert_eq!(s.residual, 2, "−0.8 与 −4");
        assert_eq!(s.saturated, 2, "±4 都是钳制值");
        assert!((s.max_gouge - 4.0).abs() < 1e-12);
        assert!((s.max_residual - 4.0).abs() < 1e-12);
        assert!((s.mean_abs - (0.0 + 0.05 + 0.05 + 0.3 + 0.8 + 4.0 + 4.0) / 7.0).abs() < 1e-12);
        // P95 取 |d| 的分位：7 个里第 6 小 = 4.0
        assert!((s.p95_abs - 4.0).abs() < 1e-12);
    }

    #[test]
    fn empty_input_is_not_a_panic() {
        let s = stats(&[], 0.1, 4.0);
        assert_eq!(s.points, 0);
        assert_eq!(s.ok_frac(), 0.0);
        assert_eq!(s.gouge_frac(), 0.0);
        assert_eq!(s.residual_frac(), 0.0);
        assert!(s.mean_abs == 0.0);
    }

    /// P95 必须是**绝对值**分位：带符号序列的 P95 会把"过切很深但很窄"
    /// 报成 0，正是旧版 `|Δr|≤0.5` 掉到 48% 却看不出问题的那种误读。
    #[test]
    fn p95_is_unsigned() {
        let mut v = vec![0.02; 95];
        v.push(-3.0);
        let s = stats(&v, 0.1, 4.0);
        assert!(
            s.p95_abs < 0.05,
            "绝对值 P95 应落在 0.02 附近：{}",
            s.p95_abs
        );
        assert!((s.max_residual - 3.0).abs() < 1e-12, "但最大值必须报出来");
    }
}
