use num_traits::{float::FloatCore, FloatConst, Zero};

use crate::{geo::Interpolate, nn::KdKey};

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

    /// Get the internal value wrapped by this angle.
    pub const fn get(self) -> T
    where
        T: Copy,
    {
        self.0
    }

    /// Compute the signed distance from `self` to `other`.
    /// Will be positive if the closest direction toward `other` is positive, and negative
    /// otherwise.
    pub fn signed_distance(self, other: Self) -> T
    where
        T: FloatCore + FloatConst,
    {
        let diff = other.get() - self.get();
        // manual check is faster than fmod
        if diff > T::PI() {
            diff - T::TAU()
        } else if diff < -T::PI() {
            diff + T::TAU()
        } else {
            diff
        }
    }
}

impl<T: PartialEq> Eq for Angle<T> {}

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

impl<T: Clone + PartialOrd + FloatConst + Zero> KdKey for Angle<T> {
    fn dimension() -> usize {
        1
    }

    fn compare(&self, rhs: &Self, _: usize) -> core::cmp::Ordering {
        unsafe { self.0.partial_cmp(&rhs.0).unwrap_unchecked() }
    }

    fn assign(&mut self, src: &Self, _: usize) {
        self.0 = src.0.clone();
    }

    fn lower_bound() -> Self {
        Self(T::zero())
    }

    fn upper_bound() -> Self {
        Self(T::TAU())
    }
}

impl<T> AsRef<T> for Angle<T> {
    fn as_ref(&self) -> &T {
        &self.0
    }
}

impl<T> Interpolate for Angle<T>
where
    T: FloatCore + FloatConst,
{
    type Distance = T;
    fn interpolate(&self, &end: &Self, radius: Self::Distance) -> Result<Self, Self> {
        let dist = self.signed_distance(end);
        if dist.abs() <= radius {
            Err(end)
        } else {
            Ok(Self((dist.signum() * radius + self.0).rem(T::TAU())))
        }
    }
}

#[cfg(test)]
mod tests {
    use core::f32::consts::TAU;

    use crate::space::Angle;

    #[test]
    fn sign_dist() {
        assert!((Angle::new(0.0).signed_distance(Angle::new(TAU - 0.1)) + 0.1).abs() < 1e-5);

        assert!((Angle::new(TAU - 0.1).signed_distance(Angle::new(0.0)) - 0.1).abs() <= 1e-5);

        assert!((Angle::new(0.25f32).signed_distance(Angle::new(0.5)) - 0.25).abs() <= 1e-5);
    }
}
