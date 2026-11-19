//! 用所属壳的切平面校准 Surface Nets 顶点。平面投影保留薄壁的两侧；
//! 多个法向共同约束折边或尖角的位置。索引和颜色均由原提取器给出。

use super::Crease;
use crate::{MeshData, meshvol::closest_point_on_triangle, stl::Tri};
use cncsim_math::{DMat3, DVec3};
use std::collections::{BTreeMap, HashMap};
mod creases;

struct Sample {
    point: DVec3,
    target: DVec3,
    distance2: f64,
    normal: DVec3,
    planes: [DVec3; 3],
    rhs: DVec3,
    weight: f64,
}

impl Sample {
    fn add_plane(&mut self, nearest: DVec3, normal: DVec3, double_area: f64, radius2: f64) {
        let offset = nearest - self.point;
        let d2 = offset.length_squared();
        if d2 >= radius2 {
            return;
        }
        // 两个相对的薄壁表面可能同时落在搜索球内。优先使用朝向对应的面，
        // 对应面尚未出现时暂存普通最近点，供完全没有可用切平面的节点回退。
        let facing = self.normal.dot(normal) > 0.0;
        if (facing && self.weight == 0.0) || (d2 < self.distance2 && (facing || self.weight == 0.0))
        {
            self.target = nearest;
            self.distance2 = d2;
        }
        if !facing {
            return;
        }
        let taper = 1.0 - d2 / radius2;
        let weight = taper * taper * double_area.min(radius2);
        self.planes[0] += normal * (weight * normal.x);
        self.planes[1] += normal * (weight * normal.y);
        self.planes[2] += normal * (weight * normal.z);
        self.rhs += normal * (weight * normal.dot(offset));
        self.weight += weight;
    }

    fn fitted(&self, radius: f64) -> DVec3 {
        if self.weight <= 0.0 {
            return self.target;
        }
        let mut matrix = self.planes.map(|col| col / self.weight);
        // 向最近面点正则化。平坦面在切向上没有方程，保留该方向原位置。
        let regularization = 0.02;
        matrix[0].x += regularization;
        matrix[1].y += regularization;
        matrix[2].z += regularization;
        let rhs = self.rhs / self.weight + (self.target - self.point) * regularization;
        let Some(inverse) = (DMat3 { cols: matrix }).inverse() else {
            return self.target;
        };
        let delta = inverse.mul_vec3(rhs);
        if delta.length_squared() <= radius * radius {
            self.point + delta
        } else {
            self.target
        }
    }
}

pub(super) fn align(
    mesh: &mut MeshData,
    tris: &[Tri],
    creases: &[Crease],
    width: f64,
    inward: bool,
) {
    // 搜索尺度取提取格正方形面的半对角线。
    let radius = std::f64::consts::FRAC_1_SQRT_2 * width;
    let mut ids = HashMap::new();
    let mut samples = Vec::new();
    // 塌缩格或多个轮廓环可能给出相同位置。相同原位置只校准一次，以免拉开接缝。
    let corners: Vec<_> = mesh
        .positions
        .iter()
        .zip(&mesh.normals)
        .map(|(p, n)| {
            let key = p.map(|v| if v == 0.0 { 0 } else { v.to_bits() });
            *ids.entry(key).or_insert_with(|| {
                let [x, y, z] = p.map(f64::from);
                let point = DVec3::new(x, y, z);
                let [nx, ny, nz] = n.map(f64::from);
                let normal = DVec3::new(nx, ny, nz);
                samples.push(Sample {
                    point,
                    target: point,
                    distance2: radius * radius,
                    normal: if inward { -normal } else { normal },
                    planes: [DVec3::ZERO; 3],
                    rhs: DVec3::ZERO,
                    weight: 0.0,
                });
                samples.len() - 1
            })
        })
        .collect();
    drop(ids);

    // 按 YZ 条带索引输出点；条带内 X 有序。三角 AABB 只查询已有条带，
    // 斜长薄面无需在三维包围盒内为每个格子分配索引。
    let bin = |v: f64| (v / (2.0 * width)).floor() as i64;
    let mut rows = BTreeMap::<i64, BTreeMap<i64, Vec<usize>>>::new();
    for (i, sample) in samples.iter().enumerate() {
        let p = sample.point;
        rows.entry(bin(p.z))
            .or_default()
            .entry(bin(p.y))
            .or_default()
            .push(i);
    }
    for row in rows.values_mut().flat_map(|z| z.values_mut()) {
        row.sort_unstable_by(|&a, &b| samples[a].point.x.total_cmp(&samples[b].point.x));
    }
    for &[a, b, c] in tris {
        let cross = (b - a).cross(c - a);
        let double_area = cross.length();
        if double_area == 0.0 {
            continue;
        }
        let normal = cross / double_area;
        let lo = DVec3::new(
            a.x.min(b.x).min(c.x),
            a.y.min(b.y).min(c.y),
            a.z.min(b.z).min(c.z),
        ) - DVec3::splat(radius);
        let hi = DVec3::new(
            a.x.max(b.x).max(c.x),
            a.y.max(b.y).max(c.y),
            a.z.max(b.z).max(c.z),
        ) + DVec3::splat(radius);
        for (_, z) in rows.range(bin(lo.z)..=bin(hi.z)) {
            for (_, row) in z.range(bin(lo.y)..=bin(hi.y)) {
                let start = row.partition_point(|&i| samples[i].point.x < lo.x);
                for &i in &row[start..] {
                    let s = &mut samples[i];
                    let p = s.point;
                    if p.x > hi.x {
                        break;
                    }
                    if p.y < lo.y || p.y > hi.y || p.z < lo.z || p.z > hi.z {
                        continue;
                    }
                    let q = closest_point_on_triangle(p, a, b, c);
                    s.add_plane(q, normal, double_area, radius * radius);
                }
            }
        }
    }

    drop(rows);
    // 依照节点顺序移动，逐面检查最终 f32 值。新面在原法向上的有向投影面积
    // 必须保留至少 1/8；冲突时对半缩步，八次均失败就保留原节点。
    let mut placed: Vec<_> = samples.iter().map(|s| s.point).collect();
    let mut incident = vec![Vec::new(); samples.len()];
    let mut faces = Vec::new();
    for tri in mesh.indices.as_chunks::<3>().0 {
        let v = tri.map(|i| corners[i as usize]);
        if v[0] == v[1] || v[1] == v[2] || v[0] == v[2] {
            continue;
        }
        let [a, b, c] = v.map(|i| placed[i]);
        let face = faces.len();
        faces.push((v, (b - a).cross(c - a)));
        for i in v {
            incident[i].push(face);
        }
    }
    for (i, s) in samples.iter().enumerate() {
        placed[i] = guarded_position(i, s.fitted(radius), &placed, &incident[i], &faces, |_| true);
    }
    let targets = creases::targets(creases, width, &placed, &samples);
    if !targets.is_empty() {
        // 不相邻的节点投到同一个棱端点时，有向面积测试不能发现新粘连。
        // 按导出的实际 f32 坐标计数，保留原本已有的重合。
        let key =
            |p: DVec3| [p.x, p.y, p.z].map(|v| if v == 0.0 { 0 } else { (v as f32).to_bits() });
        let mut occupied = HashMap::with_capacity(placed.len());
        for &p in &placed {
            *occupied.entry(key(p)).or_insert(0usize) += 1;
        }
        for (i, target) in targets {
            let old = key(placed[i]);
            let q = guarded_position(i, target, &placed, &incident[i], &faces, |p| {
                key(p) == old || !occupied.contains_key(&key(p))
            });
            let next = key(q);
            if old != next {
                let count = occupied.get_mut(&old).unwrap();
                *count -= 1;
                if *count == 0 {
                    occupied.remove(&old);
                }
                occupied.insert(next, 1);
                placed[i] = q;
            }
        }
    }
    for (p, i) in mesh.positions.iter_mut().zip(corners) {
        let q = placed[i];
        *p = [q.x as f32, q.y as f32, q.z as f32];
    }
}

fn guarded_position(
    i: usize,
    mut q: DVec3,
    placed: &[DVec3],
    incident: &[usize],
    faces: &[([usize; 3], DVec3)],
    available: impl Fn(DVec3) -> bool,
) -> DVec3 {
    for _ in 0..8 {
        let [x, y, z] = [q.x, q.y, q.z].map(|v| f64::from(v as f32));
        let candidate = DVec3::new(x, y, z);
        let valid = incident.iter().all(|&f| {
            let (face, n) = faces[f];
            let [a, b, c] = face.map(|v| if v == i { candidate } else { placed[v] });
            (b - a).cross(c - a).dot(n) > 0.125 * n.length_squared()
        });
        if valid && available(candidate) {
            return candidate;
        }
        q = placed[i].lerp(q, 0.5);
    }
    placed[i]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrapolates_a_supported_acute_crease_beyond_the_query_ball() {
        // 两片面夹角约 44°，交线沿 Z。粗格上的圆钝尖端距真棱 0.9 格，
        // 两个支持平面均在查询球内，真实交线则在球外。
        let a = DVec3::new(0.0, 0.0, -3.0);
        let b = DVec3::new(0.0, 0.0, 3.0);
        let top = DVec3::new(3.0, 1.2, 0.0);
        let bottom = DVec3::new(3.0, -1.2, 0.0);
        let surfaces = [[a, b, top], [b, a, bottom]];
        let mut mesh = MeshData {
            positions: vec![[0.9, 0.0, 0.0]],
            normals: vec![[-1.0, 0.0, 0.0]],
            ..MeshData::default()
        };
        align(
            &mut mesh,
            &surfaces,
            &[Crease {
                start: a,
                end: b,
                bisector: -DVec3::X,
            }],
            1.0,
            false,
        );
        assert!(
            mesh.positions[0][0].abs() < 0.15,
            "ridge {:?}",
            mesh.positions[0]
        );
    }

    #[test]
    fn thin_wall_uses_the_sheet_matching_the_extract_normal() {
        // 同一薄壁的两面相距 0.02 mm。初始格点恰在另一面附近；
        // 最近点正则项和切平面方程均应来自法向对应的一侧。
        let mut mesh = MeshData {
            positions: vec![
                [0.0199, -0.1, -0.1],
                [0.0199, 0.0, 0.1],
                [0.0199, 0.1, -0.1],
            ],
            normals: vec![[-1.0, 0.0, 0.0]; 3],
            indices: vec![0, 1, 2],
            ..MeshData::default()
        };
        let wall = |x| {
            [
                DVec3::new(x, -1.0, -1.0),
                DVec3::new(x, 0.0, 1.0),
                DVec3::new(x, 1.0, -1.0),
            ]
        };
        let mut back = wall(0.02);
        back.swap(1, 2);
        align(&mut mesh, &[wall(0.0), back], &[], 0.2, false);
        for p in &mesh.positions {
            assert!(p[0].abs() < 1e-6, "thin face shifted to x={}", p[0]);
        }
    }
}
