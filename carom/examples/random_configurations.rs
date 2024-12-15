#![feature(portable_simd)]

use std::{fs::File, io::Write, simd::Simd};

use carom::{env::World3d, robot::Panda, Block, Robot};
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    sample::{Rectangle, Sample},
    space::Vector,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sampler = Rectangle {
        min: Panda::BOUNDS[0],
        max: Panda::BOUNDS[1],
    };

    let mut rng = ChaCha20Rng::seed_from_u64(2707);

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

    let mut cfg_file = File::create("cfgs.csv")?;
    let mut result_file = File::create("valid.csv")?;

    for _ in 0..100_000 {
        let q: Vector<7, f32> = sampler.sample(&mut rng);
        let cfgs = q.map(Simd::<f32, 1>::splat);
        let valid = Panda.is_valid(&Block(cfgs), &world);

        for &v in &q[..q.len() - 1] {
            write!(cfg_file, "{v},")?;
        }
        writeln!(cfg_file, "{}", q.last().unwrap())?;
        writeln!(result_file, "{valid}")?;
    }

    Ok(())
}
