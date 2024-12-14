use core::simd::Simd;

use crate::{env::World3d, Robot};
use fkcc::interleaved_sphere_fk;
use rumple::space::Vector;

mod fkcc;

pub struct Panda;

impl Panda {
    #[expect(clippy::approx_constant)]
    pub const BOUNDS: [Vector<DIM, f32>; 2] = [
        Vector([
            -2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671,
        ]),
        Vector([2.9671, 1.8326, 2.9671, 0.0873, 2.9671, 0.0873, 2.9671]),
    ];
}

const DIM: usize = 7;

type ConfigurationBlock<const L: usize> = [Simd<f32, L>; DIM];

impl Robot<7, f32> for Panda {
    type World = World3d<f32>;
    fn is_valid<const L: usize>(&self, cfgs: &crate::Block<7, L, f32>, world: &Self::World) -> bool
    where
        std::simd::LaneCount<L>: std::simd::SupportedLaneCount,
        f32: std::simd::SimdElement + num_traits::Float,
        Simd<f32, L>: crate::SimdArithmetic<f32, L>,
    {
        interleaved_sphere_fk(&cfgs.0, world)
    }

    fn resolution(&self) -> f32 {
        0.1
    }
}
