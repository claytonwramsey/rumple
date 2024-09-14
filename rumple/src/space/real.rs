use crate::{metric::SquaredEuclidean, space::LinearInterpolate, Interpolate, Metric, Sample};
use num_traits::One;
use ordered_float::{FloatCore, NotNan};
use std::{
    array,
    iter::Sum,
    ops::{Deref, DerefMut, Sub},
    ptr::addr_of,
};

#[derive(Clone, Copy, Debug, PartialEq)]
#[expect(clippy::module_name_repetitions)]
pub struct RealVector<const N: usize, T = f64>([NotNan<T>; N]);

impl<T, const N: usize> RealVector<N, T> {
    pub const fn new(x: [NotNan<T>; N]) -> Self {
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
        Self(x.map(|v| NotNan::new(v).expect("x must not have NaN values")))
    }
}

impl<T, const N: usize> Deref for RealVector<N, T> {
    type Target = [NotNan<T>; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const N: usize> DerefMut for RealVector<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const N: usize> AsRef<[NotNan<T>; N]> for RealVector<N, T> {
    fn as_ref(&self) -> &[NotNan<T>; N] {
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

impl<T, const N: usize> From<[NotNan<T>; N]> for RealVector<N, T> {
    fn from(value: [NotNan<T>; N]) -> Self {
        Self(value)
    }
}

impl<const N: usize, T> Interpolate<RealVector<N, T>> for LinearInterpolate
where
    T: Sum + FloatCore,
    NotNan<T>: One,
{
    type Distance = NotNan<T>;

    fn interpolate(
        &self,
        start: &RealVector<N, T>,
        end: &RealVector<N, T>,
        radius: Self::Distance,
    ) -> Result<RealVector<N, T>, RealVector<N, T>> {
        let dist = SquaredEuclidean.distance(start, end);
        if dist <= radius {
            Ok(*end)
        } else {
            let scl: NotNan<T> = radius / dist;
            let inv_scl = NotNan::<T>::one() - scl;
            Err(RealVector(array::from_fn(|i| {
                inv_scl * start[i] + scl * end[i]
            })))
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
            *a = NotNan::new(a.into_inner() - b.into_inner()).unwrap();
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolate_real() {
        let x = RealVector::from_floats([0.0]);
        let y = RealVector::from_floats([1.0]);
        let dist = NotNan::new(0.05).unwrap();
        let z_expected = RealVector::from_floats([0.05]);
        let z = LinearInterpolate.interpolate(&x, &y, dist).unwrap_err();
        println!("{z:?}");
        assert!((z - z_expected)[0].abs() <= 0.001);
    }
}
