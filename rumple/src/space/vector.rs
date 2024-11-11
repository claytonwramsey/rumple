use crate::{metric::SquaredEuclidean, nn::KdKey, sample::Sample};
use core::{
    array,
    ops::{Deref, DerefMut, Sub},
};
use num_traits::{Float, NumCast};

use super::Interpolate;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
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
    T: Float,
{
    /// This distance is the Euclidean distance.
    type Distance = T;

    type Interpolation<'a> = VectorInterpolation<N, T> where T: 'a;

    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Self::Interpolation<'_> {
        let dist = SquaredEuclidean.partial_distance(self, end);

        if dist <= radius * radius {
            VectorInterpolation {
                n: 0,
                start: *self,
                step: *self,
            }
        } else {
            let scl = radius / dist.sqrt();
            let n = <usize as NumCast>::from(scl.recip())
                .expect("cannot interpolate with negative or NaN radius");
            VectorInterpolation {
                n,
                start: *self,
                step: Self(array::from_fn(|i| scl * (end[i] - self[i]))),
            }
        }
    }
}

#[expect(clippy::module_name_repetitions)]
pub struct VectorInterpolation<const N: usize, T> {
    pub(crate) n: usize,
    start: Vector<N, T>,
    pub(crate) step: Vector<N, T>,
}

impl<const N: usize, T: Float> Iterator for VectorInterpolation<N, T> {
    type Item = Vector<N, T>;
    fn next(&mut self) -> Option<Self::Item> {
        (self.n != 0).then(|| {
            self.n -= 1;
            for (x, s) in self.start.iter_mut().zip(self.step.0) {
                *x = *x + s;
            }
            self.start
        })
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

impl<const N: usize, T: Sub<Output = T> + Float> Sub for Vector<N, T> {
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
    T: Clone + Float,
{
    fn assign(&mut self, src: &Self, k: usize) {
        self.0[k] = src.0[k];
    }

    fn compare(&self, rhs: &Self, k: usize) -> core::cmp::Ordering {
        self.0[k].partial_cmp(&rhs.0[k]).unwrap()
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
        let z = x.interpolate(&y, dist).next().unwrap();
        println!("{z:?}");
        assert!((z - z_expected)[0].abs() <= 0.001);
    }
}
