//! 用源网格中确实由两张面共享的锐棱，定位粗格遗漏的棱线。

use super::{Crease, DVec3, Sample};
use std::collections::{BTreeMap, HashMap};

fn visible_span(edge: &Crease, lo: [f64; 3], hi: [f64; 3]) -> Option<(f64, f64)> {
    let ab = edge.end - edge.start;
    let (mut first, mut last) = (0.0_f64, 1.0_f64);
    for k in 0..3 {
        let (a, d) = (edge.start.component(k), ab.component(k));
        if d == 0.0 {
            if a < lo[k] || a > hi[k] {
                return None;
            }
        } else {
            let t = (lo[k] - a) / d;
            let u = (hi[k] - a) / d;
            first = first.max(t.min(u));
            last = last.min(t.max(u));
        }
    }
    (first < last).then_some((first, last))
}

pub(super) fn targets(
    edges: &[Crease],
    width: f64,
    placed: &[DVec3],
    samples: &[Sample],
) -> BTreeMap<usize, DVec3> {
    if edges.is_empty() || placed.is_empty() {
        return BTreeMap::new();
    }
    // 最远允许在一个提取格体对角线内找回棱。每个采样点只认领一个朝向相符的
    // 节点，防止把尖端两侧各一整排面同时挤到棱线上。
    let radius = 3.0_f64.sqrt() * width;
    let bin = |p: DVec3| [p.x, p.y, p.z].map(|v| (v / radius).floor() as i64);
    let mut buckets = HashMap::<_, Vec<usize>>::new();
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for (i, &p) in placed.iter().enumerate() {
        buckets.entry(bin(p)).or_default().push(i);
        for k in 0..3 {
            lo[k] = lo[k].min(p.component(k) - radius);
            hi[k] = hi[k].max(p.component(k) + radius);
        }
    }
    let mut chosen = BTreeMap::<usize, (f64, DVec3)>::new();
    for edge in edges {
        let ab = edge.end - edge.start;
        // 裁去网格之外的线段，以免极长且仅露出一角的输入边产生大量空查询。
        let Some((first, last)) = visible_span(edge, lo, hi) else {
            continue;
        };
        let count = (ab.length() * (last - first) / (0.5 * width)).ceil() as usize;
        for k in 0..count {
            let site =
                edge.start + ab * (first + (last - first) * ((k as f64 + 0.5) / count as f64));
            let [x, y, z] = bin(site);
            let mut best = radius * radius;
            let mut found = None;
            for iz in z.saturating_sub(1)..=z.saturating_add(1) {
                for iy in y.saturating_sub(1)..=y.saturating_add(1) {
                    for ix in x.saturating_sub(1)..=x.saturating_add(1) {
                        if let Some(nodes) = buckets.get(&[ix, iy, iz]) {
                            for &i in nodes {
                                let d2 = (placed[i] - site).length_squared();
                                if d2 < best && samples[i].normal.dot(edge.bisector) > 0.0 {
                                    best = d2;
                                    found = Some(i);
                                }
                            }
                        }
                    }
                }
            }
            if let Some(i) = found {
                let old = chosen.entry(i).or_insert((f64::INFINITY, placed[i]));
                if best < old.0 {
                    let t =
                        ((placed[i] - edge.start).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
                    *old = (best, edge.start + ab * t);
                }
            }
        }
    }
    chosen.into_iter().map(|(i, (_, p))| (i, p)).collect()
}
