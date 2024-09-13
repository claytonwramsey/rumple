use crate::{metric::SquaredEuclidean, space::Interpolate, Grow, Metric, Sample};
use num_traits::One;
use ordered_float::{FloatCore, NotNan};
use std::{
    array,
    iter::Sum,
    ops::{Deref, DerefMut},
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

impl<const N: usize, T> Grow<RealVector<N, T>> for Interpolate
where
    T: Sum + FloatCore,
    NotNan<T>: One,
{
    type Distance = NotNan<T>;

    fn grow_toward(
        &self,
        start: &RealVector<N, T>,
        end: &RealVector<N, T>,
        radius: Self::Distance,
    ) -> RealVector<N, T> {
        let dist = SquaredEuclidean.distance(start, end);
        let scl: NotNan<T> = radius / dist;
        let inv_scl = NotNan::<T>::one() - scl;
        RealVector(array::from_fn(|i| scl * start[i] + inv_scl * end[i]))
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
