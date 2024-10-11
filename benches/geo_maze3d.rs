use core::hint::black_box;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    env::World3d,
    geo::rrt_connect,
    geo::Prm,
    metric::SquaredEuclidean,
    nn::KdTreeMap,
    sample::Rectangle,
    space::Vector,
    time::Solved,
    valid::{GeoValidate, SampleInterpolate},
    Sample,
};

use brunch::Bench;

fn main() {
    let h = 1.0;
    let start = Vector([2.5, 5.0, h / 2.0]);
    let goal = Vector([2.5, 0.0, h / 2.0]);
    let mut env = World3d::new();
    let r = 0.25;

    let h = 1.0;
    let wall_hw = 0.05;

    let mut add_vert = |x, start_y, end_y| {
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
        |&Vector([x, y, z]): &Vector<3>| !env.collides_ball(x, y, z, r),
        0.02,
    );

    let sampler = Rectangle {
        min: Vector([0.0, 0.0, 0.0]),
        max: Vector([5.0, 5.0, 0.5]),
    };

    brunch::benches!(
        inline:
        Bench::new("geo_maze3d_prm").run(|| black_box(prm_bench(start, goal, &valid, &sampler, &mut ChaCha20Rng::seed_from_u64(2707)))),
        Bench::new("geo_maze3d_rrtc").run(|| black_box(rrt_connect(start, goal, KdTreeMap::new(SquaredEuclidean), &valid, &sampler, 1.0, &mut Solved::new(), &mut ChaCha20Rng::seed_from_u64(2707))))
    )
}

fn prm_bench<V: GeoValidate<Vector<3>>, S, R>(
    start: Vector<3>,
    goal: Vector<3>,
    valid: &V,
    sampler: &S,
    rng: &mut R,
) -> Vec<Vector<3>>
where
    S: Sample<Vector<3>, R>,
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
