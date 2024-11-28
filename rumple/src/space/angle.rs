use num_traits::{float::Float, FloatConst, NumCast, Zero};

use crate::{nn::KdKey, space::Interpolate};

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
pub struct Angle<T = f64>(T);

impl<T> Angle<T> {
    /// # Panics
    ///
    /// This function will panic if `T` is not between 0 and 2 pi.
    pub fn new(value: T) -> Self
    where
        T: num_traits::FloatConst + Float,
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
        T: Float + FloatConst,
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

impl<T: Float> PartialOrd for Angle<T> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: Float> Ord for Angle<T> {
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

impl<T: FloatConst + Float> std::ops::Add for Angle<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        debug_assert!(self.0.is_sign_positive() || self.0.is_zero());
        debug_assert!(self.0 <= T::TAU());
        debug_assert!(rhs.0.is_sign_positive() || rhs.0.is_zero());
        debug_assert!(rhs.0 <= T::TAU());
        let r = self.0 + rhs.0;
        Self(if r >= T::TAU() { r - T::TAU() } else { r })
    }
}

impl<T> Interpolate for Angle<T>
where
    T: Float + FloatConst,
{
    type Interpolation<'a>
        = AngleInterpolation<T>
    where
        Self: 'a;
    type Distance = T;
    fn interpolate(&self, &end: &Self, radius: Self::Distance) -> Self::Interpolation<'_> {
        #[expect(clippy::neg_cmp_op_on_partial_ord)]
        {
            // catch NaN or negatives
            // negation is required to catch NaN
            assert!(
                !(radius >= T::zero()),
                "cannot interpolate by negative or NaN value"
            );
        }
        let dist = self.signed_distance(end);
        let step = Self((dist.signum() * radius).rem(T::TAU()));
        let n = <usize as NumCast>::from((dist.abs() / radius).floor()).unwrap();
        AngleInterpolation {
            start: *self,
            step,
            n,
        }
    }
}

pub struct AngleInterpolation<T> {
    start: Angle<T>,
    pub(crate) step: Angle<T>,
    pub(crate) n: usize,
}

impl<T: Float + FloatConst> Iterator for AngleInterpolation<T> {
    type Item = Angle<T>;
    fn next(&mut self) -> Option<Self::Item> {
        (self.n > 0).then(|| {
            self.n -= 1;
            self.start = self.start + self.step;
            self.start
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.n, Some(self.n))
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
