use core::simd::{LaneCount, Simd, SimdElement, SupportedLaneCount};

use num_traits::float::FloatCore;

use crate::{env::World3d, Block, Robot, SimdArithmetic};

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
        Simd<F, L>: SimdArithmetic<F, L>,
    {
        !world.collides_balls(xs, ys, zs, Simd::splat(self.r))
    }
}
