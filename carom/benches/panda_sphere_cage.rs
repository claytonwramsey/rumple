#![feature(portable_simd)]

use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::rrt_connect,
    nn::KiddoMap,
    sample::Rectangle,
    space::Vector,
    time::{LimitSamples, Solved},
};

use brunch::{Bench, Benches};
use carom::{env::World3d, robot::Panda, Rake};

const L: usize = 8;

fn main() {
    let q_start = Vector([0., -0.785, 0., -2.356, 0., 1.571, 0.785]);
    let q_end = Vector([2.35, 1., 0., -0.8, 0.0, 2.5, 0.785]);

    let sphere_centers = [
        [0.55, 0.0, 0.25],
        [0.35, 0.35, 0.25],
        [0.0, 0.55, 0.25],
        [-0.55, 0.0, 0.25],
        [-0.35, -0.35, 0.25],
        [0.0, -0.55, 0.25],
        [0.35, -0.35, 0.25],
        [0.35, 0.35, 0.8],
        [0.0, 0.55, 0.8],
        [-0.35, 0.35, 0.8],
        [-0.55, 0.0, 0.8],
        [-0.35, -0.35, 0.8],
        [0.0, -0.55, 0.8],
        [0.35, -0.35, 0.8],
    ];

    let r = 0.2;

    let mut world = World3d::new();
    for [x, y, z] in sphere_centers {
        world.add_ball(x, y, z, r);
    }

    let rake: Rake<Panda, World3d<f32>, L> = Rake {
        robot: Panda,
        world,
    };

    let mut benches = Benches::default();
    benches.push(Bench::new("panda_sphere_cage").run(|| {
        rrt_connect(
            q_start,
            q_end,
            KiddoMap::new(),
            &rake,
            &Rectangle {
                min: Panda::BOUNDS[0],
                max: Panda::BOUNDS[1],
            },
            0.1,
            &mut (Solved::new() | LimitSamples::new(1_000_000)),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
    }));
    benches.finish();
}
