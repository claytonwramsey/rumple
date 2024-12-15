#![feature(portable_simd)]

use std::time::Instant;

use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::rrt_connect,
    nn::KiddoMap,
    sample::Rectangle,
    space::Vector,
    time::{LimitNodes, LimitSamples, Solved},
    valid::GeoValidate,
};

use carom::{env::World3d, robot::Panda, Rake};

const L: usize = 8;

#[cfg_attr(test, test)]
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
    // let sphere_centers: [[f32; 3]; 0] = [];

    let r = 0.2;

    let mut world = World3d::new();
    for [x, y, z] in sphere_centers {
        world.add_ball(x, y, z, r);
    }

    assert!(world.collides_ball(
        sphere_centers[0][0],
        sphere_centers[0][1],
        sphere_centers[0][2] + 1.5 * r,
        r
    ));

    let rake: Rake<Panda, World3d<f32>, L> = Rake {
        robot: Panda,
        world,
    };
    let tic = Instant::now();

    let mut sample_limit = LimitSamples::new(1_000_000);
    let mut node_limit = LimitNodes::new(1_000_000);
    let traj = rrt_connect(
        q_start,
        q_end,
        KiddoMap::new(),
        &rake,
        &Rectangle {
            min: Panda::BOUNDS[0],
            max: Panda::BOUNDS[1],
        },
        2.0,
        &mut (Solved::new() | &mut sample_limit | &mut node_limit),
        &mut ChaCha20Rng::seed_from_u64(2707),
    )
    .unwrap();
    let elapsed = Instant::now().duration_since(tic);
    let nsamples = sample_limit.n_sampled();
    let nnodes = node_limit.n_nodes();

    println!(
        "Finished planning in {:?} with {} samples and {} nodes ({} samples/sec, {} nodes/sec)",
        elapsed,
        nsamples,
        nnodes,
        nsamples as f64 / elapsed.as_secs_f64(),
        nnodes as f64 / elapsed.as_secs_f64(),
    );

    println!("plan_panda: traj is {:?}", traj);

    for w in traj.windows(2) {
        let [q1, q2] = *w else { unreachable!() };
        println!("{q1:?} -> {q2:?}");
        assert!(rake.is_valid_transition(&q1, &q2));
    }
}
