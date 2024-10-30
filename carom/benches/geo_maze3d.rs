#![feature(portable_simd)]

use core::hint::black_box;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::{rrt_connect, Prm},
    metric::{Metric, SquaredEuclidean},
    nn::{KdTreeMap, KiddoMap},
    sample::{Rectangle, Sample},
    space::Vector,
    time::Solved,
    valid::{GeoValidate, SampleInterpolate, Validate},
};
use std::{array, simd::Simd};

use brunch::{Bench, Benches};
use carom::env::World3d;

type F = f32;

const L: usize = 8;

struct RakeValidate<'a> {
    radius: F,
    rake_width: F,
    env: &'a World3d<F>,
}

impl Validate<Vector<3, F>> for RakeValidate<'_> {
    fn is_valid_configuration(&self, &Vector([x, y, z]): &Vector<3, F>) -> bool {
        !self.env.collides_ball(x, y, z, self.radius)
    }
}

impl GeoValidate<Vector<3, F>> for RakeValidate<'_> {
    fn is_valid_transition(&self, v0: &Vector<3, F>, v1: &Vector<3, F>) -> bool {
        let distsq = SquaredEuclidean.distance(v0, v1);
        let rakesq = self.rake_width;
        let start_offsets =
            Simd::<F, L>::from_array(array::from_fn(|i| i as F)) / Simd::splat(L as F);
        let rake_frac = rakesq / distsq;
        let [x_add, y_add, z_add] = array::from_fn(|i| Simd::splat((v1[i] - v0[i]) * rake_frac));
        let n_steps = (rake_frac * L as F).recip().ceil() as u32;
        let [mut x, mut y, mut z] =
            array::from_fn(|i| Simd::splat(v0[i]) + Simd::splat(v1[i] - v0[i]) * start_offsets);
        let rs = Simd::splat(self.radius);
        // println!("{n_steps}");
        for _ in 0..n_steps {
            if self.env.collides_balls(x, y, z, rs) {
                return false;
            }
            x += x_add;
            y += y_add;
            z += z_add;
        }
        true
    }
}

fn main() {
    let h: F = 1.0;
    let start = Vector([2.5, 5.0, h / 2.0]);
    let goal = Vector([2.5, 0.0, h / 2.0]);
    let mut env = World3d::new();
    let r = 0.25;

    let h = 1.0;
    let wall_hw = 0.05;

    let mut add_vert = |x: F, start_y: F, end_y: F| {
        env.add_aabb(x - wall_hw, start_y, 0.0, x + wall_hw, end_y, 1.0);
    };

    add_vert(0.0, 0.0, 1.0);
    add_vert(1.0, 1.0, 2.0);
    add_vert(2.0, 0.0, 1.0);
    add_vert(3.0, 0.0, 2.0);
    add_vert(4.0, 1.0, 4.0);
    add_vert(5.0, 0.0, 5.0);

    let mut add_horz = |y, start_x, end_x| {
        env.add_aabb(start_x, y - wall_hw, 0.0, end_x, y + wall_hw, h);
    };

    add_horz(0.0, 0.0, 2.0);
    add_horz(0.0, 3.0, 5.0);
    add_horz(2.0, 1.0, 3.0);
    add_horz(3.0, 0.0, 4.0);
    add_horz(4.0, 1.0, 3.0);
    add_horz(5.0, 0.0, 2.0);
    add_horz(5.0, 3.0, 5.0);

    let step_size = 0.1 * 0.1; // measured in distsq
    let valid = SampleInterpolate::new(
        |&Vector([x, y, z]): &Vector<3, F>| !env.collides_ball(x, y, z, r),
        step_size,
    );

    let sampler = Rectangle {
        min: Vector([0.0, 0.0, 0.0]),
        max: Vector([5.0, 5.0, 0.5]),
    };

    let rrt_radius = 1.0;

    let mut benches = Benches::default();
    benches.push(Bench::new("geo_maze3d_prm").run(|| {
        prm_bench(
            black_box(start),
            black_box(goal),
            &valid,
            &sampler,
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));
    benches.push(Bench::new("geo_maze3d_rrtc").run(|| {
        rrt_connect(
            black_box(start),
            black_box(goal),
            KdTreeMap::new(SquaredEuclidean),
            &valid,
            &sampler,
            rrt_radius,
            &mut Solved::new(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));

    benches.push(Bench::new("geo_maze3d_prm_kiddo").run(|| {
        prm_bench_kiddo(
            black_box(start),
            black_box(goal),
            &valid,
            &sampler,
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));
    benches.push(Bench::new("geo_maze3d_rrtc_kiddo").run(|| {
        rrt_connect(
            black_box(start),
            black_box(goal),
            rumple::nn::KiddoMap::<_, 3, _, SquaredEuclidean>::new(),
            &valid,
            &sampler,
            rrt_radius,
            &mut Solved::new(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));

    let rake_valid = RakeValidate {
        radius: r,
        rake_width: step_size,
        env: &env,
    };
    benches.push(Bench::new("geo_maze3d_prm_simd").run(|| {
        prm_bench(
            black_box(start),
            black_box(goal),
            &rake_valid,
            &sampler,
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));
    benches.push(Bench::new("geo_maze3d_rrtc_simd").run(|| {
        rrt_connect(
            black_box(start),
            black_box(goal),
            KdTreeMap::new(SquaredEuclidean),
            &rake_valid,
            &sampler,
            rrt_radius,
            &mut Solved::new(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));

    benches.push(Bench::new("geo_maze3d_prm_simd_kiddo").run(|| {
        prm_bench_kiddo(
            black_box(start),
            black_box(goal),
            &rake_valid,
            &sampler,
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));
    benches.push(Bench::new("geo_maze3d_rrtc_simd_kiddo").run(|| {
        rrt_connect(
            black_box(start),
            black_box(goal),
            rumple::nn::KiddoMap::<_, 3, _, SquaredEuclidean>::new(),
            &rake_valid,
            &sampler,
            rrt_radius,
            &mut Solved::new(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));
    benches.finish();
}

#[inline(never)]
fn prm_bench<V: GeoValidate<Vector<3, F>>, S, R>(
    start: Vector<3, F>,
    goal: Vector<3, F>,
    valid: &V,
    sampler: &S,
    rng: &mut R,
) -> Vec<Vector<3, F>>
where
    S: Sample<Vector<3, F>, R>,
{
    let mut prm = Prm::new(KdTreeMap::new(SquaredEuclidean), valid);
    let s = prm.insert_r(start, 0.0).unwrap();
    let g = prm.insert_r(goal, 0.0).unwrap();
    prm.grow_r_solve(1.0, &mut Solved::new(), sampler, rng, s, g);
    prm.path(s, g, &SquaredEuclidean)
        .unwrap()
        .into_iter()
        .map(|n| *prm.configuration(n).unwrap())
        .collect()
}

#[inline(never)]
fn prm_bench_kiddo<V: GeoValidate<Vector<3, F>>, S, R>(
    start: Vector<3, F>,
    goal: Vector<3, F>,
    valid: &V,
    sampler: &S,
    rng: &mut R,
) -> Vec<Vector<3, F>>
where
    S: Sample<Vector<3, F>, R>,
{
    let mut prm = Prm::new(KiddoMap::<_, 3, _, SquaredEuclidean>::new(), valid);
    let s = prm.insert_r(start, 0.0).unwrap();
    let g = prm.insert_r(goal, 0.0).unwrap();
    prm.grow_r_solve(1.0, &mut Solved::new(), sampler, rng, s, g);
    prm.path(s, g, &SquaredEuclidean)
        .unwrap()
        .into_iter()
        .map(|n| *prm.configuration(n).unwrap())
        .collect()
}
