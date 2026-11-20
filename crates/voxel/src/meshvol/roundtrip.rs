//! 未切削 STL 的体素往返重建。

use super::mesh_to_field;
use crate::{GridConfig, MeshData, stl::Tri};
use cncsim_math::DVec3;
use std::borrow::Cow;
use std::collections::HashMap;
use std::time::{Duration, Instant};
mod fit;

/// 未切削表面的网格及各壳累计的统计。相交壳的体积会按壳分别计入。
pub struct Reconstruction {
    pub mesh: MeshData,
    pub shells: usize,
    pub leaves: usize,
    pub signed_material_mm3: f64,
    pub voxel_time: Duration,
    pub extract_time: Duration,
}

fn root(parents: &mut [usize], mut face: usize) -> usize {
    while parents[face] != face {
        parents[face] = parents[parents[face]];
        face = parents[face];
    }
    face
}

struct Crease {
    start: DVec3,
    end: DVec3,
    bisector: DVec3,
}

struct Shell<'a> {
    tris: Cow<'a, [Tri]>,
    creases: Vec<Crease>,
}

#[derive(Default)]
struct Incidence {
    first_half: usize,
    other_face: usize,
    count: usize,
    balance: isize,
}

/// 同一几何边上的面归为一组。每条边的正、反向次数必须相等；
/// 开口网格或绕向不一致时，整份输入仍交给原来的奇偶场处理。
fn closed_shells(tris: &[Tri]) -> (Vec<Shell<'_>>, bool) {
    let mut parents: Vec<_> = (0..tris.len()).collect();
    // 闭合三角网格通常有约 F/2 个顶点和 3F/2 条边。
    let mut vertices = HashMap::with_capacity(tris.len() / 2);
    let mut edges = HashMap::with_capacity(tris.len().saturating_add(tris.len() / 2));
    let mut groups = tris.len();
    for (face, tri) in tris.iter().enumerate() {
        let ids = tri.map(|p| {
            let key = [p.x, p.y, p.z].map(|v| if v == 0.0 { 0 } else { v.to_bits() });
            let next = vertices.len();
            *vertices.entry(key).or_insert(next)
        });
        for k in 0..3 {
            let (a, b) = (ids[k], ids[(k + 1) % 3]);
            if a == b {
                continue;
            }
            let edge = edges.entry((a.min(b), a.max(b))).or_insert(Incidence {
                first_half: 3 * face + k,
                ..Incidence::default()
            });
            edge.count += 1;
            edge.other_face = face;
            edge.balance += if a < b { 1 } else { -1 };
            let owner = edge.first_half / 3;
            if owner != face {
                let r = root(&mut parents, owner);
                let s = root(&mut parents, face);
                if r != s {
                    parents[r.max(s)] = r.min(s);
                    groups -= 1;
                }
            }
        }
    }
    drop(vertices);
    let oriented = !edges.is_empty() && edges.values().all(|e| e.balance == 0);
    if !oriented {
        return (
            vec![Shell {
                tris: Cow::Borrowed(tris),
                creases: vec![],
            }],
            false,
        );
    }
    // 沿用连通性检查的边表。开边、多于两面的边及退化法向均不提供棱线约束。
    let normals: Vec<_> = tris
        .iter()
        .map(|t| {
            let n = (t[1] - t[0]).cross(t[2] - t[0]);
            let length = n.length();
            if length > 0.0 {
                n / length
            } else {
                DVec3::ZERO
            }
        })
        .collect();
    let mut sharp = Vec::new();
    for edge in edges.into_values().filter(|e| e.count == 2) {
        let half = edge.first_half;
        let owner = half / 3;
        let (n, m) = (normals[owner], normals[edge.other_face]);
        if n != DVec3::ZERO && m != DVec3::ZERO && n.dot(m) <= 0.5 {
            let t = tris[owner];
            sharp.push((
                half,
                Crease {
                    start: t[half % 3],
                    end: t[(half + 1) % 3],
                    bisector: n + m,
                },
            ));
        }
    }
    sharp.sort_unstable_by_key(|e| e.0);
    if groups <= 1 {
        return (
            vec![Shell {
                tris: Cow::Borrowed(tris),
                creases: sharp.into_iter().map(|e| e.1).collect(),
            }],
            true,
        );
    }
    // 遍历源面，在每组首面处创建输出，哈希表的迭代顺序不参与网格排序。
    let mut slots = HashMap::with_capacity(groups);
    let mut shells = Vec::with_capacity(groups);
    for (face, tri) in tris.iter().enumerate() {
        let owner = root(&mut parents, face);
        let slot = slots.entry(owner).or_insert_with(|| {
            shells.push(Shell {
                tris: Cow::Owned(Vec::new()),
                creases: vec![],
            });
            shells.len() - 1
        });
        shells[*slot].tris.to_mut().push(*tri);
    }
    for (half, crease) in sharp {
        let owner = root(&mut parents, half / 3);
        shells[slots[&owner]].creases.push(crease);
    }
    (shells, true)
}

fn is_inward(tris: &[Tri]) -> bool {
    // 局部原点减少远离世界原点的平移模型求和时的抵消。
    let origin = tris[0][0];
    tris.iter()
        .map(|t| (t[0] - origin).dot((t[1] - origin).cross(t[2] - origin)))
        .sum::<f64>()
        < 0.0
}

/// 用同一个全局格点原点、体素宽度和 LOD 重建 STL 各闭合壳。
/// 用于还原未加工的输入表面；切削、布尔和余量查询使用整体的 [mesh_to_field]。
pub fn reconstruct_surface(
    tris: &[Tri],
    config: GridConfig,
    origin: DVec3,
    domain: [i32; 3],
    k: u32,
) -> Reconstruction {
    let start = Instant::now();
    let (shells, oriented) = closed_shells(tris);
    let mut result = Reconstruction {
        mesh: MeshData::default(),
        shells: shells.len(),
        leaves: 0,
        signed_material_mm3: 0.0,
        voxel_time: start.elapsed(),
        extract_time: Duration::ZERO,
    };
    for Shell {
        tris: shell,
        creases,
    } in shells
    {
        let start = Instant::now();
        let source_inward = oriented && is_inward(&shell);
        let inward = result.shells > 1 && source_inward;
        let grid = mesh_to_field(&shell, config, origin, domain);
        result.voxel_time += start.elapsed();
        result.leaves += grid.leaf_count();
        let material = grid.material_volume();
        result.signed_material_mm3 += if inward { -material } else { material };

        let start = Instant::now();
        let mut part = if k > 1 && k <= grid.max_lod_factor() {
            grid.extract_surface_lod(k)
        } else {
            grid.extract_surface()
        };
        // 截断单壳的域边界是人工生成的断面，不能用未裁剪的源三角形去拟合。
        let fit_shell = oriented
            && (result.shells > 1
                || shell.iter().flatten().all(|p| {
                    (0..3).all(|a| {
                        let x = p.component(a) - origin.component(a);
                        x >= 0.0 && x <= domain[a] as f64 * config.h
                    })
                }));
        if fit_shell {
            let step = if k > 1 && k <= grid.max_lod_factor() {
                k
            } else {
                1
            };
            fit::align(
                &mut part,
                &shell,
                &creases,
                step as f64 * config.h,
                source_inward,
            );
        }
        if inward {
            for face in part.indices.as_chunks_mut::<3>().0 {
                face.swap(1, 2);
            }
            for normal in &mut part.normals {
                *normal = normal.map(|v| -v);
            }
        }
        if result.mesh.positions.is_empty() {
            result.mesh = part;
        } else {
            let base = result.mesh.positions.len();
            result.mesh.indices.extend(
                part.indices
                    .iter()
                    .map(|i| u32::try_from(base + *i as usize).expect("STL 重建顶点索引超出 u32")),
            );
            result.mesh.positions.append(&mut part.positions);
            result.mesh.normals.append(&mut part.normals);
            result.mesh.colors.append(&mut part.colors);
            result.mesh.skipped_quads += part.skipped_quads;
        }
        result.extract_time += start.elapsed();
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(lo: f64, hi: f64) -> Vec<Tri> {
        let vertex = |i: usize| {
            let a = |bit| {
                if i & (1usize << bit) == 0usize {
                    lo
                } else {
                    hi
                }
            };
            DVec3::new(a(0), a(1), a(2))
        };
        let mut tris = Vec::new();
        for [a, b, c, d] in [
            [0, 4, 6, 2],
            [1, 3, 7, 5],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 2, 3, 1],
            [4, 5, 7, 6],
        ] {
            tris.push([a, b, c].map(vertex));
            tris.push([a, c, d].map(vertex));
        }
        tris
    }

    fn run(tris: &[Tri]) -> Reconstruction {
        reconstruct_surface(tris, GridConfig::default(), DVec3::ZERO, [30; 3], 2)
    }

    fn volume(mesh: &MeshData) -> f64 {
        mesh.indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|face| {
                let [a, b, c] = face.map(|i| {
                    let [x, y, z] = mesh.positions[i as usize].map(f64::from);
                    DVec3::new(x, y, z)
                });
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }

    #[test]
    fn only_two_incident_faces_define_a_source_crease() {
        let tris = cube(0.6, 2.4);
        let (parts, oriented) = closed_shells(&tris);
        assert!(oriented);
        assert_eq!(parts[0].creases.len(), 12);
        let doubled = tris.repeat(2);
        let (parts, oriented) = closed_shells(&doubled);
        assert!(oriented);
        assert!(parts[0].creases.is_empty());
    }

    #[test]
    fn recovers_the_tip_of_a_closed_wedge_at_the_requested_lod() {
        let vertices = [
            (0.67, 1.44, 0.51),
            (2.45, 0.72, 0.51),
            (2.45, 2.16, 0.51),
            (0.67, 1.44, 2.51),
            (2.45, 0.72, 2.51),
            (2.45, 2.16, 2.51),
        ]
        .map(|(x, y, z)| DVec3::new(x, y, z));
        let tris = [
            [0, 2, 1],
            [3, 4, 5],
            [0, 1, 4],
            [0, 4, 3],
            [1, 2, 5],
            [1, 5, 4],
            [2, 0, 3],
            [2, 3, 5],
        ]
        .map(|t| t.map(|i| vertices[i]));
        let mesh = run(&tris).mesh;
        for z in [1.02, 1.19, 1.41, 1.78, 1.93] {
            let p = DVec3::new(0.67, 1.44, z);
            let error = mesh
                .indices
                .as_chunks::<3>()
                .0
                .iter()
                .map(|f| {
                    let [a, b, c] = f.map(|i| {
                        let [x, y, z] = mesh.positions[i as usize].map(f64::from);
                        DVec3::new(x, y, z)
                    });
                    (super::super::closest_point_on_triangle(p, a, b, c) - p).length()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(error < 0.025, "wedge tip distance {error} at {z}");
        }
    }

    #[test]
    fn close_crossing_shells_survive_on_the_original_lattice() {
        let mut tris = cube(0.6, 2.4);
        tris.extend(cube(0.62, 2.42));
        let result = run(&tris);
        // 两个实体均横跨许多粗格；相交只产生 0.02 mm 的错位，小于 h=0.1。
        let actual = volume(&result.mesh);
        assert!(
            (actual - 2.0 * 1.8_f64.powi(3)).abs() < 0.4,
            "signed volume {actual}"
        );
        assert_eq!(result.shells, 2);
        assert_eq!(result.mesh.skipped_quads, 0);
    }

    #[test]
    fn quantized_parallel_faces_recover_their_source_spacing() {
        let mut tris = cube(0.601, 2.401);
        let outer = run(&tris);
        tris.extend(cube(0.6012, 2.4012));
        let result = run(&tris);
        let middle =
            |p: &&[f32; 3]| p[0] < 1.0 && (1.0..2.0).contains(&p[1]) && (1.0..2.0).contains(&p[2]);
        for (points, x) in [
            (
                &result.mesh.positions[..outer.mesh.positions.len()],
                0.601f32,
            ),
            (
                &result.mesh.positions[outer.mesh.positions.len()..],
                0.6012f32,
            ),
        ] {
            let mut count = 0;
            for p in points.iter().filter(middle) {
                assert!(
                    (p[0] - x).abs() < 0.00002,
                    "shell face x={} target {x}",
                    p[0]
                );
                count += 1;
            }
            assert!(count > 10);
        }
    }

    #[test]
    fn fitted_corner_follows_all_three_source_planes() {
        let mut tris = cube(0.601, 2.401);
        let first_len = run(&tris).mesh.positions.len();
        tris.extend(cube(0.63, 2.43));
        let result = run(&tris);
        let distance = result.mesh.positions[..first_len]
            .iter()
            .map(|p| {
                let [x, y, z] = p.map(f64::from);
                (DVec3::new(x, y, z) - DVec3::splat(0.601)).length()
            })
            .fold(f64::INFINITY, f64::min);
        assert!(distance < 0.004, "cube corner missed by {distance} mm");
    }

    #[test]
    fn one_closed_shell_recovers_its_crease_in_either_winding() {
        let mut tris = cube(0.601, 2.401);
        for _ in 0..2 {
            let model = run(&tris).mesh;
            let corner = model
                .positions
                .iter()
                .map(|p| {
                    let [x, y, z] = p.map(f64::from);
                    (DVec3::new(x, y, z) - DVec3::splat(0.601)).length()
                })
                .fold(f64::INFINITY, f64::min);
            assert!(corner < 0.004, "single-shell crease missed by {corner} mm");
            assert!(volume(&model) > 5.5, "stock surface must face outward");
            assert_eq!(model.skipped_quads, 0);
            for tri in &mut tris {
                tri.swap(1, 2);
            }
        }
    }

    #[test]
    fn cropped_single_shell_preserves_the_field_boundary_faces() {
        let tris = cube(0.607, 3.413);
        let restored = run(&tris);
        let original = mesh_to_field(&tris, GridConfig::default(), DVec3::ZERO, [30; 3])
            .extract_surface_lod(2);
        assert_eq!(restored.mesh.positions, original.positions);
        assert_eq!(restored.mesh.normals, original.normals);
        assert_eq!(restored.mesh.indices, original.indices);
        assert_eq!(restored.mesh.colors, original.colors);
    }

    #[test]
    fn closed_slab_thinner_than_the_lod_spacing_keeps_both_faces() {
        let mut tris = cube(0.6, 2.4);
        for p in tris.iter_mut().flatten() {
            p.x = if p.x < 1.0 { 0.56 } else { 0.74 };
        }
        let single = run(&tris).mesh;
        tris.extend(cube(1.0, 1.4));
        let restored = run(&tris);
        let n = single.positions.len();
        let mut sides = [0, 0];
        for (p, normal) in restored.mesh.positions[..n]
            .iter()
            .zip(&restored.mesh.normals[..n])
        {
            if (1.0..2.0).contains(&p[1]) && (1.0..2.0).contains(&p[2]) {
                let right = normal[0] > 0.0;
                let expected = if right { 0.74 } else { 0.56 };
                assert!(
                    (p[0] - expected).abs() < 1e-5,
                    "slab face x={} expected {expected}",
                    p[0]
                );
                sides[usize::from(right)] += 1;
            }
        }
        assert!(sides.into_iter().all(|n| n > 10));
        assert_eq!(restored.mesh.skipped_quads, 0);
        assert_eq!(
            &restored.mesh.indices[..single.indices.len()],
            &single.indices
        );
        assert_eq!(&restored.mesh.colors[..single.colors.len()], &single.colors);
    }

    #[test]
    fn open_or_inconsistently_wound_components_use_the_original_field() {
        let shell = cube(0.6, 2.4);
        let mut broken = cube(0.8, 2.2);
        broken[1].swap(1, 2);
        for faces in [broken.as_slice(), &broken[1..]] {
            let tris = [shell.as_slice(), faces].concat();
            let actual = run(&tris);
            let original = mesh_to_field(&tris, GridConfig::default(), DVec3::ZERO, [30; 3])
                .extract_surface_lod(2);
            assert_eq!(actual.shells, 1);
            assert_eq!(actual.mesh.indices, original.indices);
            assert_eq!(actual.mesh.positions, original.positions);
            assert_eq!(actual.mesh.colors, original.colors);
        }
    }

    #[test]
    fn inner_cavity_keeps_inward_winding_and_normals() {
        let mut tris = cube(0.4, 2.6);
        let outer = run(&tris).mesh;
        let mut inner = cube(0.8, 2.2);
        for t in &mut inner {
            t.swap(1, 2);
        }
        tris.extend(inner);
        let result = run(&tris);
        let actual = volume(&result.mesh);
        assert!(
            (actual - (2.2_f64.powi(3) - 1.4_f64.powi(3))).abs() < 0.4,
            "signed volume {actual}"
        );
        assert_eq!(result.shells, 2);
        let center = 1.5;
        let mut count = 0;
        for (p, n) in result
            .mesh
            .positions
            .iter()
            .zip(&result.mesh.normals)
            .skip(outer.positions.len())
        {
            assert!((0..3).map(|a| (p[a] - center) * n[a]).sum::<f32>() < 0.0);
            count += 1;
        }
        assert!(count > 100);
    }
}
