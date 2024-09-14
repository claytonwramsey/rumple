use std::{
    fmt::Debug,
    iter::Sum,
    ops::{Add, Div, Mul, Sub},
};

use num_traits::Float;

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Hash)]
pub struct Real<T>(T);
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Hash)]
pub struct Ordered<T>(T);

pub type R32 = Real<f32>;
pub type R64 = Real<f64>;
pub type O32 = Ordered<f32>;
pub type O64 = Ordered<f64>;

impl<T> Real<T> {
    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T: Float> Real<T> {
    /// # Panics
    ///
    /// This function will panic if `x` is non-finite.
    pub fn new(x: T) -> Self {
        assert!(x.is_finite(), "type Real must be finite");
        Self(x)
    }

    #[must_use]
    pub fn abs(self) -> Self {
        Self(self.0.abs())
    }

    pub fn is_zero(self) -> bool {
        self.0.is_zero()
    }
}

impl<T: Float> Sub for Real<T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl<T: Float> Mul for Real<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self(self.0 * rhs.0)
    }
}

impl<T: Float> Add for Real<T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl<T: Float> Div for Real<T> {
    type Output = Self;
    fn div(self, rhs: Self) -> Self::Output {
        assert!(!rhs.is_zero(), "cannot divide by zero");
        Self(self.0 / rhs.0)
    }
}

impl<T: Debug> Debug for Real<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl<T: Float> Eq for Real<T> {}

impl<T: Float> PartialOrd for Real<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: Float> Ord for Real<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        unsafe { self.0.partial_cmp(&other.0).unwrap_unchecked() }
    }
}

impl<T: Float> Sum for Real<T> {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        let mut total = T::zero();
        for x in iter {
            total = total + x.0;
        }
        Self(total)
    }
}

impl<T: Float> Eq for Ordered<T> {}
