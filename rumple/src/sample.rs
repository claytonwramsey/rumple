use core::array;

use crate::{
    space::{Angle, Pose2d, RealVector},
    Sample,
};
use num_traits::{float::FloatCore, FloatConst};
use rand::{
    distributions::{uniform::SampleUniform, Bernoulli, Distribution},
    Rng,
};

impl<A, B, S, RNG> Sample<(A, B), RNG> for S
where
    S: Sample<A, RNG> + Sample<B, RNG>,
{
    fn sample(&self, _: &mut RNG) -> (A, B) {
        todo!()
    }
}

impl<RNG: Rng> Sample<bool, RNG> for Bernoulli {
    fn sample(&self, rng: &mut RNG) -> bool {
        <Self as Distribution<bool>>::sample(self, rng)
    }
}

pub struct Rectangle<T> {
    pub min: T,
    pub max: T,
}

pub struct Everywhere;

impl<const N: usize, T, RNG: Rng> Sample<RealVector<N, T>, RNG> for Rectangle<RealVector<N, T>>
where
    T: FloatCore + SampleUniform,
{
    fn sample(&self, rng: &mut RNG) -> RealVector<N, T> {
        RealVector::from_floats(array::from_fn(|i| {
            rng.gen_range(self.min[i].into_inner()..=self.max[i].into_inner())
        }))
    }
}

impl<T, RNG: Rng> Sample<Angle<T>, RNG> for Everywhere
where
    T: FloatCore + FloatConst + SampleUniform,
{
    fn sample(&self, rng: &mut RNG) -> Angle<T> {
        unsafe { Angle::new_unchecked(rng.gen_range(T::zero()..T::TAU())) }
    }
}

impl<T, RNG: Rng> Sample<Pose2d<T>, RNG> for Rectangle<RealVector<2, T>>
where
    T: FloatConst + FloatCore + SampleUniform,
{
    fn sample(&self, rng: &mut RNG) -> Pose2d<T> {
        Pose2d {
            position: self.sample(rng),
            angle: Everywhere.sample(rng),
        }
    }
}
