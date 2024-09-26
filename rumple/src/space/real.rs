use crate::{float::Real, metric::SquaredEuclidean, nn::KdKey, Interpolate, Metric, Sample};
use core::{
    array,
    ops::{Deref, DerefMut, Sub},
    ptr::addr_of,
};
use num_traits::{float::FloatCore, One};

#[derive(Clone, Copy, Debug, PartialEq)]
#[expect(clippy::module_name_repetitions)]
pub struct RealVector<const N: usize, T = f64>([Real<T>; N]);

impl<T, const N: usize> RealVector<N, T> {
    pub const fn new(x: [Real<T>; N]) -> Self {
        Self(x)
    }

    /// Convert an array of floating point numbers into a `RealVector`.
    ///
    /// # Panics
    ///
    /// This function will panic of any element of `x` is `NaN`.
    pub fn from_floats(x: [T; N]) -> Self
    where
        T: FloatCore,
    {
        Self(x.map(|v| Real::new(v).unwrap()))
    }
}

impl<T, const N: usize> Deref for RealVector<N, T> {
    type Target = [Real<T>; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const N: usize> DerefMut for RealVector<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const N: usize> AsRef<[Real<T>; N]> for RealVector<N, T> {
    fn as_ref(&self) -> &[Real<T>; N] {
        &self.0
    }
}

impl<T, const N: usize> AsRef<[T; N]> for RealVector<N, T> {
    fn as_ref(&self) -> &[T; N] {
        unsafe {
            // SAFETY: `NotNan` is transparent, so references may safely be cast.
            &*addr_of!(self.0).cast()
        }
    }
}

impl<T, const N: usize> From<[Real<T>; N]> for RealVector<N, T> {
    fn from(value: [Real<T>; N]) -> Self {
        Self(value)
    }
}

impl<const N: usize, T> Interpolate for RealVector<N, T>
where
    T: FloatCore,
{
    type Distance = Real<T>;

    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Result<Self, Self> {
        let dist = SquaredEuclidean.distance(self, end);
        if dist <= radius {
            Err(*end)
        } else {
            let scl = radius / dist;
            let inv_scl = Real::<T>::one() - scl;
            Ok(Self(array::from_fn(|i| inv_scl * self[i] + scl * end[i])))
        }
    }
}

impl<const N: usize, T, RNG> Sample<Self, RNG> for RealVector<N, T>
where
    T: Clone,
{
    fn sample(&self, _: &mut RNG) -> Self {
        self.clone()
    }
}

impl<const N: usize, T: Sub<Output = T> + FloatCore> Sub for RealVector<N, T> {
    type Output = Self;
    fn sub(mut self, rhs: Self) -> Self::Output {
        for (a, b) in self.iter_mut().zip(rhs.into_iter()) {
            *a = Real::new(a.into_inner() - b.into_inner()).unwrap();
        }
        self
    }
}

impl<T, const N: usize> KdKey for RealVector<N, T>
where
    Real<T>: Ord,
    T: Clone,
{
    fn assign(&mut self, src: &Self, k: usize) {
        self.0[k] = src.0[k].clone();
    }

    fn compare(&self, rhs: &Self, k: usize) -> core::cmp::Ordering {
        self.0[k].cmp(&rhs.0[k])
    }

    fn dimension() -> usize {
        N
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::float::r32;

    #[test]
    fn interpolate_real() {
        let x = RealVector::from_floats([0.0]);
        let y = RealVector::from_floats([1.0]);
        let dist = r32(0.05);
        let z_expected = RealVector::from_floats([0.05]);
        let z = x.interpolate(&y, dist).unwrap();
        assert!((z - z_expected)[0].abs().into_inner() <= 0.001);
    }
}
