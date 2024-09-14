use core::{
    cmp::Ordering,
    fmt::Debug,
    iter::Sum,
    ops::{Add, Div, Mul, Sub},
};

use num_traits::float::FloatCore;

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

impl<T: FloatCore> Real<T> {
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

impl<T: FloatCore> Sub for Real<T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl<T: FloatCore> Mul for Real<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self(self.0 * rhs.0)
    }
}

impl<T: FloatCore> Add for Real<T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl<T: FloatCore> Div for Real<T> {
    type Output = Self;
    fn div(self, rhs: Self) -> Self::Output {
        assert!(!rhs.is_zero(), "cannot divide by zero");
        Self(self.0 / rhs.0)
    }
}

impl<T: Debug> Debug for Real<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl<T: FloatCore> Eq for Real<T> {}

impl<T: FloatCore> PartialOrd for Real<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: FloatCore> Ord for Real<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        unsafe { self.0.partial_cmp(&other.0).unwrap_unchecked() }
    }
}

impl<T: FloatCore> Sum for Real<T> {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        let mut total = T::zero();
        for x in iter {
            total = total + x.0;
        }
        Self(total)
    }
}

impl<T: FloatCore> Eq for Ordered<T> {}
