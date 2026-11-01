//! 扫掠 SDF 的微基准（默认跳过）。
//!
//! ```bash
//! cargo test --release -p cncsim-voxel --test sweep_perf -- --ignored --nocapture
//! ```

use cncsim_math::DVec3;
use cncsim_motion::Pose;
use cncsim_tool::flat_end_mill;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
}

const N: usize = 2_000_000;

fn bench(name: &str, from: Pose, to: Pose, evals_per_call: u32) {
    let prof = flat_end_mill(6.0, 20.0);
    let mut rng = Rng(0x5EED);
    let pts: Vec<DVec3> = (0..N)
        .map(|_| {
            DVec3::new(
                rng.next() * 12.0 - 3.0,
                rng.next() * 12.0 - 3.0,
                rng.next() * 4.0 + 5.0,
            )
        })
        .collect();
    // 预热
    let mut acc = 0.0;
    for p in &pts[..N / 10] {
        acc += cncsim_voxel::sd_sweep(*p, &from, &to, &prof);
    }
    let t0 = std::time::Instant::now();
    for p in &pts {
        acc += cncsim_voxel::sd_sweep(*p, &from, &to, &prof);
    }
    let ns = t0.elapsed().as_secs_f64() * 1e9 / N as f64;
    println!(
        "{name}: {ns:.1} ns/次（每次 {evals_per_call} 个求值 → {:.1} ns/求值）  校验和 {acc:.3}",
        ns / evals_per_call as f64
    );
}

#[test]
#[ignore = "性能基准，默认不跑"]
fn sweep_cost() {
    let flat = Pose { tip: DVec3::new(0.0, 0.0, 7.0), axis: DVec3::Z };
    let flat2 = Pose { tip: DVec3::new(0.05, 0.0, 7.0), axis: DVec3::Z };
    let iters = cncsim_voxel::golden_iters(0.05, 0.0);
    bench("平移 0.05 mm", flat, flat2, 2 + 2 + iters);

    let long = Pose { tip: DVec3::new(200.0, 0.0, 7.0), axis: DVec3::Z };
    let iters = cncsim_voxel::golden_iters(200.0, 0.0);
    bench("平移 200 mm", flat, long, 2 + 2 + iters);

    let tilted = Pose {
        tip: DVec3::new(0.0, 0.0, 7.0),
        axis: DVec3::new(0.0, 0.087_155_657_465_999_35, 0.996_194_698_091_462_6),
    };
    let iters = cncsim_voxel::golden_iters(0.05, 5.0_f64.to_radians() * 3.0);
    bench("平移+转 5°", flat, tilted, 2 + 2 + iters);
}
