use std::array;

use crate::{space::RealVector, Sample};
use ordered_float::FloatCore;
use rand::{
    distributions::{Bernoulli, Distribution, Standard},
    Rng,
};

impl<RNG: Rng> Sample<bool, RNG> for Bernoulli {
    fn sample(&self, rng: &mut RNG) -> bool {
        <Self as Distribution<bool>>::sample(self, rng)
    }
}

pub struct Everywhere;

impl<RNG: Rng, const N: usize, T> Sample<RealVector<N, T>, RNG> for Everywhere
where
    T: FloatCore,
    Standard: Distribution<T>,
{
    fn sample(&self, rng: &mut RNG) -> RealVector<N, T> {
        RealVector::from_floats(array::from_fn(|_| rng.gen()))
    }
}
