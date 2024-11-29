#![feature(portable_simd)]

use core::hint::black_box;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::{rrt_connect, Prm},
    metric::SquaredEuclidean,
    nn::{KdTreeMap, KiddoMap},
    sample::{Rectangle, Sample},
    space::Vector,
    time::Solved,
    valid::{GeoValidate, SampleInterpolate},
};

use brunch::{Bench, Benches};
use carom::{env::World3d, robot::Sphere, Rake};

type F = f32;
const L: usize = 8;

fn main() {
    let h: F = 1.0;
    let start = Vector([2.5, 5.0, h / 2.0]);
    let goal = Vector([2.5, 0.0, h / 2.0]);
    let mut env = World3d::new();
    let r = 0.25;

    let h = 1.0;
    let wall_hw = 0.05;

    let step_size = 0.05; // measured in euclidean distance

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
            rumple::nn::KiddoMap::<_, 3, SquaredEuclidean>::new(),
            &valid,
            &sampler,
            rrt_radius,
            &mut Solved::new(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));

    let rake_valid = Rake::<_, _, L> {
        robot: Sphere {
            r,
            resolution: step_size,
        },
        world: env.clone(),
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
            rumple::nn::KiddoMap::<_, 3, SquaredEuclidean>::new(),
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
    let mut prm = Prm::new(KiddoMap::<_, 3, SquaredEuclidean>::new(), valid);
    let s = prm.insert_r(start, 0.0).unwrap();
    let g = prm.insert_r(goal, 0.0).unwrap();
    prm.grow_r_solve(1.0, &mut Solved::new(), sampler, rng, s, g);
    prm.path(s, g, &SquaredEuclidean)
        .unwrap()
        .into_iter()
        .map(|n| *prm.configuration(n).unwrap())
        .collect()
}
