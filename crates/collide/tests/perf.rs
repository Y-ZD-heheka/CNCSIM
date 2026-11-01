//! 凸体碰撞的性能基准（默认跳过）。
//!
//! ```bash
//! cargo test --release -p cncsim-collide --test perf -- --ignored --nocapture
//! ```

use cncsim_collide::convex::*;
use cncsim_math::{DMat3, DMat4, DVec3};

struct Rng(u64);
impl Rng {
    fn next_f64(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
    fn vec(&mut self, s: f64) -> DVec3 {
        DVec3::new(self.range(-s, s), self.range(-s, s), self.range(-s, s))
    }
}

/// 整把刀（母线分解成圆台）对同一个盒：这是"刀具 vs 夹具"的真实形状。
#[test]
#[ignore = "性能基准，默认不跑"]
fn tool_vs_fixture() {
    let prof = cncsim_tool::flat_end_mill(6.0, 60.0);
    let cones = cones_of(&prof);
    let fix = Box3 {
        c: DVec3::new(40.0, 0.0, 0.0),
        half: DVec3::new(20.0, 30.0, 15.0),
        rot: DMat3::IDENTITY,
    };

    let n = 50_000;
    let mut rng = Rng(0x5EED_2026_1029);
    let poses: Vec<DMat4> = (0..n)
        .map(|_| DMat4::from_translation(rng.vec(25.0) + DVec3::new(0.0, 0.0, 30.0)))
        .collect();

    let t0 = std::time::Instant::now();
    let mut acc = 0.0;
    let mut iters = 0u64;
    let mut hits = 0;
    for &xf in &poses {
        let mut best = f64::INFINITY;
        for &c in &cones {
            let g = distance(&Placed::new(c, xf), &fix);
            iters += g.iterations as u64;
            if g.touching() {
                hits += 1;
                best = 0.0;
                break;
            }
            best = best.min(g.distance);
        }
        acc += best;
    }
    let us = t0.elapsed().as_secs_f64() * 1e6 / n as f64;
    println!(
        "整把刀（{} 段圆台）vs 夹具：{us:.2} µs/次（含 {} 次 GJK，平均 {:.1} 迭代），命中 {hits}/{n}，校验和 {acc:.1}",
        cones.len(),
        n * cones.len(),
        iters as f64 / (n * cones.len()) as f64
    );
    assert!(acc > 0.0);
}

/// 单个凸体对：看 GJK 本身的成本与迭代数。
#[test]
#[ignore = "性能基准，默认不跑"]
fn single_pair() {
    let a = Cone { c: DVec3::ZERO, axis: DVec3::Z, half_h: 20.0, r_lo: 3.0, r_hi: 3.0 };
    let n = 200_000;
    let mut rng = Rng(0x1234_5678);
    let spheres: Vec<Sphere> = (0..n)
        .map(|_| Sphere { c: rng.vec(40.0), r: rng.range(0.5, 5.0) })
        .collect();

    let t0 = std::time::Instant::now();
    let mut acc = 0.0;
    let mut iters = 0u64;
    for s in &spheres {
        let g = distance(&a, s);
        iters += g.iterations as u64;
        acc += g.distance;
    }
    let ns = t0.elapsed().as_secs_f64() * 1e9 / n as f64;
    println!(
        "圆柱 vs 球：{ns:.0} ns/次，平均 {:.1} 迭代，校验和 {acc:.1}",
        iters as f64 / n as f64
    );

    // AABB 粗筛的成本，用来对比"该不该先粗筛"
    let t0 = std::time::Instant::now();
    let mut acc2 = 0.0;
    for s in &spheres {
        acc2 += aabb_gap(&a, s);
    }
    let ns2 = t0.elapsed().as_secs_f64() * 1e9 / n as f64;
    println!("AABB 粗筛：{ns2:.1} ns/次（{:.0}× 便宜），校验和 {acc2:.1}", ns / ns2.max(0.001));
}

/// 穿透查询：GJK + EPA 的合计成本（只在真的撞上时才跑 EPA）。
#[test]
#[ignore = "性能基准，默认不跑"]
fn penetration_cost() {
    use cncsim_collide::penetration::penetration;

    let prof = cncsim_tool::flat_end_mill(6.0, 60.0);
    let cones = cones_of(&prof);
    let fix = Box3 {
        c: DVec3::new(0.0, 0.0, 10.0),
        half: DVec3::new(40.0, 40.0, 10.0),
        rot: DMat3::IDENTITY,
    };

    let n = 20_000;
    let mut rng = Rng(0x99);
    // 全部落在夹具内部，保证每次都真的进 EPA
    let poses: Vec<DMat4> = (0..n)
        .map(|_| DMat4::from_translation(rng.vec(6.0) + DVec3::new(0.0, 0.0, 12.0)))
        .collect();

    let t0 = std::time::Instant::now();
    let mut depth_sum = 0.0;
    let mut iters = 0u64;
    let mut hits = 0;
    for &xf in &poses {
        let mut worst: Option<f64> = None;
        for &c in &cones {
            if let Some(p) = penetration(&Placed::new(c, xf), &fix) {
                iters += p.iterations as u64;
                worst = Some(worst.map_or(p.depth, |w: f64| w.max(p.depth)));
            }
        }
        if let Some(d) = worst {
            hits += 1;
            depth_sum += d;
        }
    }
    let us = t0.elapsed().as_secs_f64() * 1e6 / n as f64;
    println!(
        "穿透查询（{} 段圆台 vs 盒）：{us:.2} µs/次，{} 次进 EPA，平均 {:.1} 迭代，平均深度 {:.3} mm",
        cones.len(),
        hits,
        iters as f64 / hits.max(1) as f64,
        depth_sum / hits.max(1) as f64
    );
    assert!(hits > n / 2, "样本应该大多穿透");
}
