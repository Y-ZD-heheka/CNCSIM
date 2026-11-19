//! 不同壳的间隙可能比距离编码的一步还小。用所属壳的三角形校准
//! 已提取顶点的位置；拓扑、颜色和体素/LOD 格距由原提取器决定。

use crate::{MeshData, meshvol::closest_point_on_triangle, stl::Tri};
use cncsim_math::DVec3;
use std::collections::{BTreeMap, HashMap};

struct Sample {
    point: DVec3,
    target: DVec3,
    distance2: f64,
    limit: f64,
}

pub(super) fn align(mesh: &mut MeshData, tris: &[Tri], h: f64) {
    let radius = 0.5 * h;
    let mut ids = HashMap::new();
    let mut samples = Vec::new();
    // 塌缩格或多个轮廓环可能给出相同位置。相同原位置只校准一次，以免拉开接缝。
    let corners: Vec<_> = mesh
        .positions
        .iter()
        .map(|p| {
            let key = p.map(|v| if v == 0.0 { 0 } else { v.to_bits() });
            *ids.entry(key).or_insert_with(|| {
                let [x, y, z] = p.map(f64::from);
                let point = DVec3::new(x, y, z);
                samples.push(Sample {
                    point,
                    target: point,
                    distance2: radius * radius,
                    limit: radius,
                });
                samples.len() - 1
            })
        })
        .collect();
    drop(ids);

    // 每个角的最大位移 ≤ 其相邻三角形的最小高 / 8。两条边各自最多
    // 改变最小高 / 4，两个扰动的一阶项与二阶项之和小于原叉积长度。
    for face in mesh.indices.as_chunks::<3>().0 {
        let v = face.map(|i| corners[i as usize]);
        if v[0] == v[1] || v[1] == v[2] || v[2] == v[0] {
            continue;
        }
        let [a, b, c] = v.map(|i| samples[i].point);
        let longest = (a - b)
            .length_squared()
            .max((b - c).length_squared())
            .max((c - a).length_squared());
        let limit = ((b - a).cross(c - a).length_squared() / longest).sqrt() / 8.0;
        for i in v {
            samples[i].limit = samples[i].limit.min(limit);
        }
    }

    // 按 YZ 条带索引输出点；条带内 X 有序。三角 AABB 只查询已有条带，
    // 斜长薄面无需在三维包围盒内为每个格子分配索引。
    let bin = |v: f64| (v / (4.0 * h)).floor() as i64;
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
                    let d2 = (q - p).length_squared();
                    if d2 < s.distance2 {
                        s.distance2 = d2;
                        s.target = q;
                    }
                }
            }
        }
    }

    for (p, i) in mesh.positions.iter_mut().zip(corners) {
        let s = &samples[i];
        let mut q = s.target;
        if s.distance2 > s.limit * s.limit {
            q = s.point.lerp(q, s.limit / s.distance2.sqrt());
        }
        let exact = [q.x, q.y, q.z];
        let rounded = exact.map(|v| v as f32);
        let [x, y, z] = rounded.map(f64::from);
        if (DVec3::new(x, y, z) - s.point).length_squared() <= s.limit * s.limit {
            *p = rounded;
        } else {
            // f32 舍入可能把极瘦面的限位多跨一个 ULP。只把越过目标的分量
            // 改为朝原位置取整，因此保存进网格的实际位移也满足同一界限。
            for axis in 0..3 {
                let v = rounded[axis];
                p[axis] = if v > p[axis] && f64::from(v) > exact[axis] {
                    v.next_down()
                } else if v < p[axis] && f64::from(v) < exact[axis] {
                    v.next_up()
                } else {
                    v
                };
            }
        }
    }
}
