use core::simd::Simd;

use fkcc::interleaved_sphere_fk;
use rumple::{
    space::Vector,
    valid::{GeoValidate, Validate},
};

use crate::env::World3d;

mod fkcc;

pub struct PandaCc<'a> {
    environment: &'a World3d<f32>,
}

pub const DIM: usize = 7;

pub type Configuration = Vector<DIM, f32>;
pub type ConfigurationBlock<const L: usize> = [Simd<f32, L>; DIM];

impl Validate<Configuration> for PandaCc<'_> {
    fn is_valid_configuration(&self, c: &Configuration) -> bool {
        interleaved_sphere_fk(&c.0.map(Simd::<f32, 1>::splat), &self.environment)
    }
}

impl GeoValidate<Configuration> for PandaCc<'_> {
    fn is_valid_transition(&self, _: &Configuration, _: &Configuration) -> bool {
        todo!()
    }
}
