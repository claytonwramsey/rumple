use core::{
    ops::{Add, Mul, Sub},
    simd::{
        cmp::{SimdPartialEq, SimdPartialOrd},
        prelude::SimdFloat,
        LaneCount, Mask, Simd, SimdElement, SupportedLaneCount,
    },
};

use num_traits::float::FloatCore;

use crate::{env::World3d, Block, Robot};

pub struct Sphere<F> {
    pub r: F,
    pub resolution: F,
}

impl<F: FloatCore + SimdElement> Robot<3, F> for Sphere<F> {
    type World = World3d<F>;

    fn resolution(&self) -> F {
        self.resolution
    }

    fn is_valid<const L: usize>(
        &self,
        &Block([xs, ys, zs]): &Block<3, L, F>,
        world: &Self::World,
    ) -> bool
    where
        LaneCount<L>: SupportedLaneCount,
        F: SimdElement + FloatCore,
        Simd<F, L>: Add<Output = Simd<F, L>>
            + Sub<Output = Simd<F, L>>
            + Mul<Output = Simd<F, L>>
            + SimdPartialOrd
            + SimdPartialEq<Mask = Mask<F::Mask, L>>
            + SimdFloat,
    {
        !world.collides_balls(xs, ys, zs, Simd::splat(self.r))
    }
}
