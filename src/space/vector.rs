use crate::{geo::Interpolate, metric::SquaredEuclidean, nn::KdKey, Sample};
use core::{
    array,
    ops::{Deref, DerefMut, Sub},
};
use num_traits::float::FloatCore;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// An `N`-dimensional vector of `T`.
pub struct Vector<const N: usize, T = f64>(pub [T; N]);

impl<T, const N: usize> Vector<N, T> {
    /// Construct a new `Vector`.
    pub const fn new(x: [T; N]) -> Self {
        Self(x)
    }
}

impl<T, const N: usize> Deref for Vector<N, T> {
    type Target = [T; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const N: usize> DerefMut for Vector<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const N: usize> AsRef<[T; N]> for Vector<N, T> {
    fn as_ref(&self) -> &[T; N] {
        &self.0
    }
}

impl<T, const N: usize> From<[T; N]> for Vector<N, T> {
    fn from(value: [T; N]) -> Self {
        Self(value)
    }
}

impl<const N: usize, T> Interpolate for Vector<N, T>
where
    T: FloatCore,
{
    type Distance = T;

    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Result<Self, Self> {
        let dist = SquaredEuclidean.partial_distance(self, end);
        if dist <= radius {
            Err(*end)
        } else {
            let scl = radius / dist;
            let inv_scl = T::one() - scl;
            Ok(Self(array::from_fn(|i| inv_scl * self[i] + scl * end[i])))
        }
    }
}

impl<const N: usize, T, RNG> Sample<Self, RNG> for Vector<N, T>
where
    T: Clone,
{
    fn sample(&self, _: &mut RNG) -> Self {
        self.clone()
    }
}

impl<const N: usize, T: Sub<Output = T> + FloatCore> Sub for Vector<N, T> {
    type Output = Self;
    fn sub(mut self, rhs: Self) -> Self::Output {
        for (a, b) in self.iter_mut().zip(rhs.into_iter()) {
            *a = *a - b;
        }
        self
    }
}

impl<T, const N: usize> KdKey for Vector<N, T>
where
    T: Ord + Clone + FloatCore,
{
    fn assign(&mut self, src: &Self, k: usize) {
        self.0[k] = src.0[k];
    }

    fn compare(&self, rhs: &Self, k: usize) -> core::cmp::Ordering {
        self.0[k].cmp(&rhs.0[k])
    }

    fn dimension() -> usize {
        N
    }

    fn lower_bound() -> Self {
        Self(array::from_fn(|_| T::min_value()))
    }

    fn upper_bound() -> Self {
        Self(array::from_fn(|_| T::max_value()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolate_real() {
        let x = Vector::new([0.0]);
        let y = Vector::new([1.0]);
        let dist = 0.05;
        let z_expected = Vector::new([0.05]);
        let z = x.interpolate(&y, dist).unwrap();
        assert!((z - z_expected)[0].abs() <= 0.001);
    }
}
