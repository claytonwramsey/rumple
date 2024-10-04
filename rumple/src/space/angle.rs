use num_traits::float::FloatCore;

use crate::{nn::KdKey, Interpolate};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(transparent)]
/// An angle (an element of [0, 2π).).
///
/// Angles can uniquely model the special orthogonal group in 2 dimensions (_SO_(2)), also known as
/// the set of 2D rotations.
///
/// You might be tempted to model orientations in 3D using a product of 3 `Angle`s.
/// **This is a bad idea!**
/// You should instead use a [`crate::space::Orient`] for 3D rotation instead.
pub struct Angle<T>(T);

impl<T> Angle<T> {
    #[cfg(feature = "std")]
    /// # Panics
    ///
    /// This function will panic if `T` is not between -pi and pi.
    pub fn new(value: T) -> Self
    where
        T: num_traits::FloatConst + FloatCore,
    {
        assert!(
            T::zero() <= value && value < T::TAU(),
            "angle must be between 0 and 2pi"
        );
        Self(value)
    }

    /// # Safety
    ///
    /// Will be unsafe if `value` is not in the range [0, 2pi).
    pub const unsafe fn new_unchecked(value: T) -> Self {
        Self(value)
    }

    pub const fn get(self) -> T
    where
        T: Copy,
    {
        self.0
    }
}

impl<T: FloatCore> Eq for Angle<T> {}

impl<T: FloatCore> PartialOrd for Angle<T> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: FloatCore> Ord for Angle<T> {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        unsafe { self.0.partial_cmp(&other.0).unwrap_unchecked() }
    }
}

impl<T: Clone + Ord> KdKey for Angle<T> {
    fn dimension() -> usize {
        1
    }

    fn compare(&self, rhs: &Self, _: usize) -> core::cmp::Ordering {
        self.0.cmp(&rhs.0)
    }

    fn assign(&mut self, src: &Self, _: usize) {
        self.0 = src.0.clone();
    }
}

impl<T> AsRef<T> for Angle<T> {
    fn as_ref(&self) -> &T {
        &self.0
    }
}

impl<T> Interpolate for Angle<T> {
    type Distance = T;
    fn interpolate(&self, _end: &Self, _radius: Self::Distance) -> Result<Self, Self> {
        todo!()
    }
}
