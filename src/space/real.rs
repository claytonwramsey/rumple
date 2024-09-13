use ordered_float::{FloatCore, NotNan};
use std::{
    ops::{Deref, DerefMut},
    ptr::addr_of,
};

#[derive(Clone, Copy, Debug, PartialEq)]
#[expect(clippy::module_name_repetitions)]
pub struct RealVector<T, const N: usize>([NotNan<T>; N]);

impl<T, const N: usize> RealVector<T, N> {
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

impl<T, const N: usize> Deref for RealVector<T, N> {
    type Target = [NotNan<T>; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const N: usize> DerefMut for RealVector<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const N: usize> AsRef<[NotNan<T>; N]> for RealVector<T, N> {
    fn as_ref(&self) -> &[NotNan<T>; N] {
        &self.0
    }
}

impl<T, const N: usize> AsRef<[T; N]> for RealVector<T, N> {
    fn as_ref(&self) -> &[T; N] {
        unsafe {
            // SAFETY: `NotNan` is transparent, so references may safely be cast.
            &*addr_of!(self.0).cast()
        }
    }
}

impl<T, const N: usize> From<[NotNan<T>; N]> for RealVector<T, N> {
    fn from(value: [NotNan<T>; N]) -> Self {
        Self(value)
    }
}
