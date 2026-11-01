//! # cncsim-math
//!
//! f64 线性代数。**零依赖**，是 L2 领域层唯一的数学基础。
//!
//! ## 为什么自研而不直接用 `glam`
//!
//! 1. **零依赖**——这一层要被 `kin` / `voxel` / `collide` 共用，而它们必须能在无 Bevy、
//!    无 GPU 的环境下跑 `cargo test`。
//! 2. **f64 优先**——`glam` 的 `DVec3` 存在，但默认路径是 f32；我们要让 f64 成为自然选择，
//!    而不是每次都要记得写 `D` 前缀。
//! 3. **只实现用得到的**——不引入 SIMD、不引入 swizzle、不引入泛型标量，
//!    换取可读性和可验证性。
//!
//! ## 约定
//!
//! - 矩阵**列主序**：`cols[j][i]` 是第 `i` 行第 `j` 列。
//! - 旋转矩阵为右手系，角度单位**弧度**（度转弧度在 `kin` 里做）。
//! - 所有变换都是**刚体**（旋转 + 平移），`inverse()` 走 `inverse_affine` 快路径。

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// 判零容差。浮点比较一律用它，不要写 `== 0.0`。
pub const EPS: f64 = 1e-12;

// ───────────────────────────── DVec2 ─────────────────────────────

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DVec2 {
    pub x: f64,
    pub y: f64,
}

impl DVec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn length_squared(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }
}

impl Add for DVec2 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for DVec2 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for DVec2 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s)
    }
}
impl Div<f64> for DVec2 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s)
    }
}

// ───────────────────────────── DVec3 ─────────────────────────────

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl DVec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Self = Self { x: 0.0, y: 0.0, z: 1.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn splat(v: f64) -> Self {
        Self { x: v, y: v, z: v }
    }

    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Self) -> Self {
        Self {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }

    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    pub fn normalize(self) -> Self {
        let l = self.length();
        debug_assert!(l > EPS, "normalize 零长度向量");
        self / l
    }

    pub fn lerp(self, o: Self, t: f64) -> Self {
        self + (o - self) * t
    }

    /// 分量级比较，用于测试
    pub fn abs_diff(self, o: Self) -> f64 {
        (self - o).length()
    }
}

impl Add for DVec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for DVec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for DVec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl Mul<f64> for DVec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Div<f64> for DVec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}
impl AddAssign for DVec3 {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}
impl SubAssign for DVec3 {
    fn sub_assign(&mut self, o: Self) {
        *self = *self - o;
    }
}

// ───────────────────────────── DMat3 ─────────────────────────────

/// 3×3 矩阵，列主序：`cols[j][i]` = 第 i 行第 j 列。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DMat3 {
    pub cols: [DVec3; 3],
}

impl DMat3 {
    pub const IDENTITY: Self = Self {
        cols: [DVec3::X, DVec3::Y, DVec3::Z],
    };

    pub const fn from_cols(c0: DVec3, c1: DVec3, c2: DVec3) -> Self {
        Self { cols: [c0, c1, c2] }
    }

    /// `m[i][j]`：第 i 行第 j 列
    pub fn at(&self, i: usize, j: usize) -> f64 {
        self.cols[j].component(i)
    }

    pub fn mul_vec3(&self, v: DVec3) -> DVec3 {
        self.cols[0] * v.x + self.cols[1] * v.y + self.cols[2] * v.z
    }

    pub fn transpose(&self) -> Self {
        let m = self;
        // 新矩阵的 col j = 原矩阵的 row j
        Self::from_cols(
            DVec3::new(m.at(0, 0), m.at(0, 1), m.at(0, 2)),
            DVec3::new(m.at(1, 0), m.at(1, 1), m.at(1, 2)),
            DVec3::new(m.at(2, 0), m.at(2, 1), m.at(2, 2)),
        )
    }

    pub fn determinant(&self) -> f64 {
        let (a, b, c) = (self.at(0, 0), self.at(0, 1), self.at(0, 2));
        let (d, e, f) = (self.at(1, 0), self.at(1, 1), self.at(1, 2));
        let (g, h, i) = (self.at(2, 0), self.at(2, 1), self.at(2, 2));
        a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    }

    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        if det.abs() < EPS {
            return None;
        }
        let (a, b, c) = (self.at(0, 0), self.at(0, 1), self.at(0, 2));
        let (d, e, f) = (self.at(1, 0), self.at(1, 1), self.at(1, 2));
        let (g, h, i) = (self.at(2, 0), self.at(2, 1), self.at(2, 2));
        // 伴随矩阵的**列**：adj[i][j] = 代数余子式(j,i)
        Some(Self::from_cols(
            DVec3::new(e * i - f * h, -(d * i - f * g), d * h - e * g) / det,
            DVec3::new(-(b * i - c * h), a * i - c * g, -(a * h - b * g)) / det,
            DVec3::new(b * f - c * e, -(a * f - c * d), a * e - b * d) / det,
        ))
    }
}

// ───────────────────────────── DMat4 ─────────────────────────────

/// 4×4 刚体变换，列主序：`cols[j][i]` = 第 i 行第 j 列。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DMat4 {
    pub cols: [[f64; 4]; 4],
}

impl DMat4 {
    pub const IDENTITY: Self = Self {
        cols: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    };

    /// `m[i][j]`：第 i 行第 j 列
    pub fn at(&self, i: usize, j: usize) -> f64 {
        self.cols[j][i]
    }

    pub fn from_translation(t: DVec3) -> Self {
        let mut m = Self::IDENTITY;
        m.cols[3] = [t.x, t.y, t.z, 1.0];
        m
    }

    /// 绕**过原点的单位轴**旋转 `angle` 弧度（Rodrigues 公式）。
    // 这里就是一次显式转置（行主序的 r 写进列主序的 cols），下标写法比迭代器清楚。
    #[allow(clippy::needless_range_loop)]
    pub fn from_axis_angle(axis: DVec3, angle: f64) -> Self {
        let a = axis.normalize();
        let (s, c) = angle.sin_cos();
        let t = 1.0 - c;
        let (x, y, z) = (a.x, a.y, a.z);
        // 行主序写入（下面按列赋值）
        let r = [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ];
        let mut m = Self::IDENTITY;
        for j in 0..3 {
            for i in 0..3 {
                m.cols[j][i] = r[i][j];
            }
        }
        m
    }

    pub fn mul_mat4(&self, o: &Self) -> Self {
        let mut m = Self::IDENTITY;
        for j in 0..4 {
            for i in 0..4 {
                let mut s = 0.0;
                for k in 0..4 {
                    s += self.at(i, k) * o.at(k, j);
                }
                m.cols[j][i] = s;
            }
        }
        m
    }

    pub fn transform_point3(&self, p: DVec3) -> DVec3 {
        DVec3::new(
            self.at(0, 0) * p.x + self.at(0, 1) * p.y + self.at(0, 2) * p.z + self.at(0, 3),
            self.at(1, 0) * p.x + self.at(1, 1) * p.y + self.at(1, 2) * p.z + self.at(1, 3),
            self.at(2, 0) * p.x + self.at(2, 1) * p.y + self.at(2, 2) * p.z + self.at(2, 3),
        )
    }

    pub fn transform_vector3(&self, v: DVec3) -> DVec3 {
        DVec3::new(
            self.at(0, 0) * v.x + self.at(0, 1) * v.y + self.at(0, 2) * v.z,
            self.at(1, 0) * v.x + self.at(1, 1) * v.y + self.at(1, 2) * v.z,
            self.at(2, 0) * v.x + self.at(2, 1) * v.y + self.at(2, 2) * v.z,
        )
    }

    /// 旋转部分（左上 3×3）
    pub fn rotation(&self) -> DMat3 {
        DMat3::from_cols(
            DVec3::new(self.at(0, 0), self.at(1, 0), self.at(2, 0)),
            DVec3::new(self.at(0, 1), self.at(1, 1), self.at(2, 1)),
            DVec3::new(self.at(0, 2), self.at(1, 2), self.at(2, 2)),
        )
    }

    pub fn translation(&self) -> DVec3 {
        DVec3::new(self.at(0, 3), self.at(1, 3), self.at(2, 3))
    }

    /// 刚体变换的逆：`[R|t]⁻¹ = [Rᵀ | −Rᵀt]`。
    ///
    /// 比通用 4×4 求逆快，且**没有数值放大**——我们所有变换都是刚体。
    pub fn inverse_affine(&self) -> Self {
        let r = self.rotation();
        let rt = r.transpose();
        let t = self.translation();
        let nt = -rt.mul_vec3(t);
        let mut m = Self::IDENTITY;
        for j in 0..3 {
            for i in 0..3 {
                m.cols[j][i] = rt.at(i, j);
            }
        }
        m.cols[3] = [nt.x, nt.y, nt.z, 1.0];
        m
    }
}

impl Mul for DMat4 {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        self.mul_mat4(&o)
    }
}

impl Mul<DVec3> for DMat4 {
    type Output = DVec3;
    fn mul(self, v: DVec3) -> DVec3 {
        self.transform_point3(v)
    }
}

impl DVec3 {
    /// 按索引取分量（0=x,1=y,2=z）。给矩阵代码用，避免三处重复 match。
    pub fn component(self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

// ───────────────────────────── 测试 ─────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::FRAC_PI_2;

    fn assert_vec(a: DVec3, b: DVec3, tol: f64) {
        assert!(a.abs_diff(b) < tol, "{a:?} != {b:?} (diff {})", a.abs_diff(b));
    }

    #[test]
    fn vec_basics() {
        assert_vec(DVec3::X.cross(DVec3::Y), DVec3::Z, EPS);
        assert!((DVec3::new(3.0, 4.0, 0.0).length() - 5.0).abs() < EPS);
        assert_vec(DVec3::new(2.0, 0.0, 0.0).normalize(), DVec3::X, EPS);
    }

    #[test]
    fn rotation_about_z_maps_x_to_y() {
        let m = DMat4::from_axis_angle(DVec3::Z, FRAC_PI_2);
        assert_vec(m.transform_vector3(DVec3::X), DVec3::Y, 1e-15);
        assert_vec(m.transform_vector3(DVec3::Y), -DVec3::X, 1e-15);
        // 旋转不动原点
        assert_vec(m.transform_point3(DVec3::ZERO), DVec3::ZERO, 1e-15);
    }

    #[test]
    fn translation_only_moves_points_not_vectors() {
        let m = DMat4::from_translation(DVec3::new(10.0, 20.0, 30.0));
        assert_vec(m.transform_point3(DVec3::ZERO), DVec3::new(10.0, 20.0, 30.0), EPS);
        assert_vec(m.transform_vector3(DVec3::X), DVec3::X, EPS);
    }

    #[test]
    fn mat3_inverse_roundtrip() {
        let m = DMat3::from_cols(
            DVec3::new(2.0, 1.0, 0.0),
            DVec3::new(1.0, 3.0, 1.0),
            DVec3::new(0.0, 1.0, 2.0),
        );
        let inv = m.inverse().expect("可逆");
        let v = DVec3::new(1.5, -2.0, 3.25);
        assert_vec(inv.mul_vec3(m.mul_vec3(v)), v, 1e-12);
    }

    #[test]
    fn mat3_inverse_of_nonsymmetric_matrix() {
        // 对称矩阵抓不到"返回 (Mᵀ)⁻¹ 而非 M⁻¹"这个 bug——必须用非对称矩阵。
        let m = DMat3::from_cols(
            DVec3::new(1.0, 2.0, 3.0),
            DVec3::new(4.0, 5.0, 6.0),
            DVec3::new(7.0, 8.0, 10.0), // 非对称、非奇异
        );
        assert!(m.determinant().abs() > 1e-12, "确认非奇异（det={}）", m.determinant());
        let inv = m.inverse().expect("可逆");

        // M·M⁻¹ = I
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 {
                    s += m.at(i, k) * inv.at(k, j);
                }
                let expect = if i == j { 1.0 } else { 0.0 };
                assert!((s - expect).abs() < 1e-12, "M·M⁻¹ 的 ({i},{j}) = {s}，应 {expect}");
            }
        }

        // 且 M⁻¹·M = I（转置错的话这一条也会挂）
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 {
                    s += inv.at(i, k) * m.at(k, j);
                }
                let expect = if i == j { 1.0 } else { 0.0 };
                assert!((s - expect).abs() < 1e-12, "M⁻¹·M 的 ({i},{j}) = {s}，应 {expect}");
            }
        }
    }

    #[test]
    fn mat3_singular_returns_none() {
        let m = DMat3::from_cols(DVec3::X, DVec3::X, DVec3::Y);
        assert!(m.inverse().is_none(), "共面列应判为奇异");
    }

    #[test]
    fn affine_inverse_roundtrip() {
        let m = DMat4::from_translation(DVec3::new(100.0, -50.0, 180.0))
            * DMat4::from_axis_angle(DVec3::X, 0.7)
            * DMat4::from_translation(DVec3::new(0.0, 0.0, -180.0));
        let inv = m.inverse_affine();
        let p = DVec3::new(3.0, -7.0, 11.0);
        assert_vec(inv.transform_point3(m.transform_point3(p)), p, 1e-10);
        // 不改变长度（刚体）
        let d = DVec3::new(1.0, 2.0, 3.0);
        assert!((m.transform_vector3(d).length() - d.length()).abs() < 1e-12);
    }

    #[test]
    fn composition_order_matters() {
        // 先转 90° 再平移 vs 先平移再转 90°，结果不同
        let r = DMat4::from_axis_angle(DVec3::Z, FRAC_PI_2);
        let t = DMat4::from_translation(DVec3::X);
        // t·r：先转后移 → X 转到 Y，再沿 X 平移 1 → (1,1,0)
        assert_vec((t * r).transform_point3(DVec3::X), DVec3::new(1.0, 1.0, 0.0), 1e-15);
        // r·t：先移后转 → X 移到 (2,0,0)，再转 90° → (0,2,0)
        assert_vec((r * t).transform_point3(DVec3::X), DVec3::new(0.0, 2.0, 0.0), 1e-15);
    }

    #[test]
    fn transpose_is_involutive_and_correct() {
        let m = DMat3::from_cols(
            DVec3::new(1.0, 2.0, 3.0),
            DVec3::new(4.0, 5.0, 6.0),
            DVec3::new(7.0, 8.0, 9.0),
        );
        let t = m.transpose();
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(t.at(i, j), m.at(j, i), "转置后 ({i},{j}) 不对");
            }
        }
        assert_eq!(t.transpose(), m, "转置两次应回到原矩阵");
    }
}
