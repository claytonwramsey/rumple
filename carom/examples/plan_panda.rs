#![feature(portable_simd)]

use std::{
    array,
    convert::identity,
    thread::sleep,
    time::{Duration, Instant},
};

use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rubullet::{nalgebra::Isometry3, MultiBodyOptions, UrdfOptions};
use rumple::{
    geo::rrt_connect,
    nn::KiddoMap,
    sample::Rectangle,
    space::{Interpolate, Vector},
    time::{LimitNodes, LimitSamples, Solved},
    valid::GeoValidate,
};

use carom::{env::World3d, robot::Panda, Rake};

const L: usize = 8;

#[cfg_attr(test, test)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    // render for debugging
    let mut physics_client = rubullet::PhysicsClient::connect(rubullet::Mode::Gui)?;
    physics_client.set_gravity([0.0, 0.0, -9.81]);

    let panda_id = physics_client.load_urdf(
        "resources/panda/panda.urdf",
        UrdfOptions {
            use_fixed_base: true,
            flags: rubullet::LoadModelFlags::URDF_PRINT_URDF_INFO,
            ..Default::default()
        },
    )?;

    for center in sphere_centers {
        let radius = r as f64;
        let [x, y, z] = center.map(|v| v as f64);
        let vis_shape = physics_client.create_visual_shape(
            rubullet::GeometricVisualShape::Sphere { radius },
            rubullet::VisualShapeOptions {
                frame_offset: rubullet::nalgebra::Isometry3::translation(0.0, 0.0, 0.0),
                rgba_colors: [1.0, 1.0, 1.0, 1.0],
                ..Default::default()
            },
        )?;
        let col_shape = physics_client
            .create_collision_shape(rubullet::GeometricCollisionShape::Sphere { radius }, None)?;
        let _mb_shape = physics_client.create_multi_body(
            col_shape,
            vis_shape,
            MultiBodyOptions {
                base_pose: Isometry3::translation(x, y, z),
                ..Default::default()
            },
        )?;
    }

    // construct interpolated trajectory
    let res = 0.01;
    let mut interp = traj
        .first()
        .copied()
        .into_iter()
        .chain(traj.windows(2).flat_map(|w| {
            let &[q1, q2] = w else { unreachable!() };
            q1.interpolate(&q2, res).chain(Some(q2))
        }))
        .map(|q| q.map(|x| x as f64))
        .collect::<Vec<_>>();

    loop {
        for q in &interp {
            physics_client.set_joint_motor_control_array(
                panda_id,
                &array::from_fn::<_, { Panda::DIM }, _>(identity),
                rubullet::ControlCommandArray::Positions(q),
                None,
            )?;
            physics_client.step_simulation()?;

            sleep(Duration::from_secs_f64(1.0 / 60.0));
        }

        interp.reverse();
    }

    // Ok(())
}
