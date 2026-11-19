//! 一个粗格可包含多条互不相连的等值轮廓。沿六个面配对跨零棱，每个闭环生成一个对偶顶点。

use crate::extract::CORNER;
use cncsim_math::DVec3;
use std::sync::LazyLock;

const EDGES: [(usize, usize); 12] = [
    (0, 1),
    (2, 3),
    (4, 5),
    (6, 7),
    (0, 2),
    (1, 3),
    (4, 6),
    (5, 7),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
];

// 面顺序 x−、x+、y−、y+、z−、z+；在每个面上按环绕顺序列出棱。
const FACES: [([usize; 4], usize); 6] = [
    ([4, 10, 6, 8], 0),
    ([5, 11, 7, 9], 1),
    ([0, 9, 2, 8], 0),
    ([1, 11, 3, 10], 2),
    ([0, 5, 1, 4], 0),
    ([2, 7, 3, 6], 4),
];

pub(super) fn edge_index(axis: usize, corner: usize) -> usize {
    // 删除角点编号里的轴位，将两个横向轴的取值压成 0..4。
    let lower = (1 << axis) - 1;
    axis * 4 + ((corner & lower) | ((corner >> 1) & !lower))
}

#[derive(Clone, Copy, Default)]
struct Contours {
    loops: [u16; 4], // 至多 12 条棱，每个环至少有 3 条。
    ambiguous: u8,
}

fn trace(mask: usize, center_inside: u8) -> Contours {
    let mut result = Contours::default();
    let mut linked = [0u16; 12];
    let mut active = 0u16;
    for (e, (a, b)) in EDGES.iter().copied().enumerate() {
        if ((mask >> a) ^ (mask >> b)) & 1 != 0 {
            active |= 1 << e;
        }
    }
    for (face, (edges, start)) in FACES.iter().enumerate() {
        let mut pair = |a: usize, b: usize| {
            linked[a] |= 1 << b;
            linked[b] |= 1 << a;
        };
        let mut crossed = edges.iter().copied().filter(|&e| active & (1 << e) != 0);
        let (Some(a), Some(b)) = (crossed.next(), crossed.next()) else {
            continue;
        };
        if crossed.next().is_none() {
            pair(a, b);
        } else {
            // 四条棱都跨零，以共享面的细场中心判定哪一对对角区域连通。
            result.ambiguous |= 1 << face;
            if (mask & (1 << start) != 0) == (center_inside & (1 << face) != 0) {
                pair(edges[0], edges[1]);
                pair(edges[2], edges[3]);
            } else {
                pair(edges[3], edges[0]);
                pair(edges[1], edges[2]);
            }
        }
    }
    for ring in &mut result.loops {
        if active == 0 {
            break;
        }
        let mut pending = 1u16 << active.trailing_zeros();
        while pending != 0 {
            let e = pending.trailing_zeros() as usize;
            *ring |= 1 << e;
            pending = (pending | linked[e]) & !*ring;
        }
        active &= !*ring;
    }
    result
}

static CASES: LazyLock<[Contours; 256]> = LazyLock::new(|| std::array::from_fn(|m| trace(m, 0)));

pub(super) fn loops(corners: &[f64; 8], mut center_inside: impl FnMut(usize) -> bool) -> [u16; 4] {
    let mask = corners
        .iter()
        .enumerate()
        .fold(0, |m, (i, &d)| m | (usize::from(d < 0.0) << i));
    let case = CASES[mask];
    let mut inside = 0;
    let mut faces = case.ambiguous;
    while faces != 0 {
        let face = faces.trailing_zeros() as usize;
        inside |= u8::from(center_inside(face)) << face;
        faces &= faces - 1;
    }
    if inside == 0 {
        case.loops
    } else {
        trace(mask, inside).loops
    }
}

pub(super) fn vertex(
    mut ring: u16,
    values: &[f64; 8],
    origin: DVec3,
    width: f64,
) -> (DVec3, DVec3) {
    let mut point = DVec3::ZERO;
    let mut slope = [0.0; 3];
    let count = ring.count_ones();
    while ring != 0 {
        let edge = ring.trailing_zeros() as usize;
        ring &= ring - 1;
        let (a, b) = EDGES[edge];
        let dv = values[b] - values[a];
        let mut local = CORNER[a].map(f64::from);
        local[edge / 4] = -values[a] / dv;
        point += DVec3::new(local[0], local[1], local[2]);
        slope[edge / 4] += dv;
    }
    let normal = DVec3::new(slope[0], slope[1], slope[2]);
    let length = normal.length();
    (
        origin + point * (width / f64::from(count)),
        if length > 0.0 {
            normal / length
        } else {
            DVec3::Z
        },
    )
}
