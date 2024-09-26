use core::{
    array,
    ops::{Range, RangeInclusive},
};

use crate::{space::RealVector, Sample};
use num_traits::float::FloatCore;
use rand::{
    distributions::{
        uniform::{SampleRange, SampleUniform},
        Bernoulli, Distribution, Standard,
    },
    Rng,
};

impl<A, B, S, RNG> Sample<(A, B), RNG> for S
where
    S: Sample<A, RNG> + Sample<B, RNG>,
{
    fn sample(&self, rng: &mut RNG) -> (A, B) {
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
