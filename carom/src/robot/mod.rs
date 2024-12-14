use core::simd::{cmp::SimdPartialOrd, LaneCount, Simd, SupportedLaneCount};

mod panda;
mod sphere;

pub use sphere::Sphere;

use crate::env::World3d;

pub use panda::Panda;

#[expect(clippy::many_single_char_names)]
fn sphere_environment_in_collision<const L: usize>(
    e: &World3d<f32>,
    x: Simd<f32, L>,
    y: Simd<f32, L>,
    z: Simd<f32, L>,
    r: Simd<f32, L>,
) -> bool
where
    LaneCount<L>: SupportedLaneCount,
{
    e.collides_balls(x, y, z, r)
}

#[expect(clippy::too_many_arguments)]
fn sphere_sphere_self_collision<const L: usize>(
    x1: Simd<f32, L>,
    y1: Simd<f32, L>,
    z1: Simd<f32, L>,
    r1: Simd<f32, L>,
    x2: Simd<f32, L>,
    y2: Simd<f32, L>,
    z2: Simd<f32, L>,
    r2: Simd<f32, L>,
) -> bool
where
    LaneCount<L>: SupportedLaneCount,
{
    let xdiff = x2 - x1;
    let ydiff = y2 - y1;
    let zdiff = z2 - z1;
    let rplus = r1 + r2;
    (xdiff * xdiff + ydiff * ydiff + zdiff * zdiff)
        .simd_le(rplus * rplus)
        .any()
}
