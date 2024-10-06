#![cfg(feature = "std")]

use std::f64::consts::PI;

use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    env::World2d,
    geo::{rrt, DiscreteValidate},
    metric::SquaredEuclidean,
    nn::KdTreeMap,
    sample::Rectangle,
    space::{Angle, Pose2d, PoseRadius, Vector, WeightedPoseDistance},
    time::Solved,
};

use criterion::{criterion_group, criterion_main, Criterion};

fn bench_geo_car(c: &mut Criterion) {
    c.bench_function("geo_car", |b| b.iter(geo_car));
}

criterion_group!(gcar, bench_geo_car);
criterion_main!(gcar);

fn geo_car() -> Vec<Pose2d> {
    let mut env = World2d::new();
    // left bug trap
    env.add_aabb(0.0, 0.0, 4.0, 1.0);
    env.add_aabb(0.0, 1.0, 1.0, 4.0);
    env.add_aabb(3.0, 1.0, 4.0, 8.0);
    env.add_aabb(0.0, 5.0, 1.0, 9.0);
    env.add_aabb(1.0, 8.0, 4.0, 9.0);

    // right bug trap
    env.add_aabb(5.0, 0.0, 9.0, 1.0);
    env.add_aabb(8.0, 1.0, 9.0, 4.0);
    env.add_aabb(5.0, 1.0, 6.0, 8.0);
    env.add_aabb(5.0, 8.0, 9.0, 9.0);
    env.add_aabb(8.0, 5.0, 9.0, 8.0);

    let start = Pose2d {
        position: Vector([2.0, 3.0]),
        angle: Angle::new(PI / 2.0),
    };
    let goal = Pose2d {
        position: Vector([7.0, 3.0]),
        angle: Angle::new(PI / 2.0),
    };

    let half_w = 0.5;
    let half_h = 0.25;

    let valid = DiscreteValidate::new(
        |&Pose2d {
             position: Vector([x, y]),
             angle,
         }: &Pose2d| { !env.collides_rect(x, y, angle.get(), half_w, half_h) },
        PoseRadius {
            angle_dist: PI / 180.0,
            position_dist: 0.01,
        },
    );

    let grow_radius = PoseRadius {
        angle_dist: PI / 4.0,
        position_dist: 2.0,
    };

    rrt(
        start,
        KdTreeMap::new(WeightedPoseDistance {
            position_metric: SquaredEuclidean,
            position_weight: 1.0,
            angle_metric: SquaredEuclidean,
            angle_weight: 1.0,
        }),
        &valid,
        &Rectangle {
            min: Vector::new([-2.0; 2]),
            max: Vector::new([11.0; 2]),
        },
        &goal,
        grow_radius,
        &mut Solved::new(),
        &Bernoulli::new(0.2).unwrap(),
        &mut ChaCha20Rng::seed_from_u64(2707),
    )
    .unwrap()
}
