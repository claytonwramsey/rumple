use core::{
    cmp::Ordering,
    fmt::Debug,
    iter::Sum,
    ops::{Add, Div, Mul, Neg, Rem, Sub},
};

use num_traits::{float::FloatCore, FloatConst, Num, NumCast, One, ToPrimitive, Zero};
use rand::distributions::uniform::{SampleBorrow, SampleUniform, UniformFloat, UniformSampler};

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Hash)]
pub struct Real<T>(T);

pub type R32 = Real<f32>;
pub type R64 = Real<f64>;

#[must_use]
/// Construct a new real-valued 32-bit float.
///
/// # Panics
///
/// This function will panic if `x` is non-finite.
pub fn r32(x: f32) -> R32 {
    Real::new(x).unwrap()
}

#[must_use]
/// Construct a new real-valued 64-bit float.
///
/// # Panics
///
/// This function will panic if `x` is non-finite.
pub fn r64(x: f64) -> R64 {
    Real::new(x).unwrap()
}

impl<T> Real<T> {
    pub fn new(x: T) -> Option<Self>
    where
        T: FloatCore,
    {
        x.is_finite().then_some(Self(x))
    }
    pub fn into_inner(self) -> T {
        self.0
    }
}

macro_rules! to_primitive {
    ($(($id: ident, $t: ty)), *) => {
        $(
            fn $id(&self) -> Option<$t> {self.0.$id()})*
    };
}

impl<T> ToPrimitive for Real<T>
where
    T: ToPrimitive,
{
    to_primitive! {
        (to_i64, i64),
        (to_u64, u64)
    }
}

impl<T> NumCast for Real<T>
where
    T: NumCast,
{
    fn from<U: num_traits::ToPrimitive>(n: U) -> Option<Self> {
        Some(Self(T::from(n)?))
    }
}

pub enum FromStrRadixErr<T> {
    Underlying(T),
    NonFinite,
}

impl<T> Num for Real<T>
where
    T: FloatCore,
{
    type FromStrRadixErr = FromStrRadixErr<T::FromStrRadixErr>;
    fn from_str_radix(str: &str, radix: u32) -> Result<Self, Self::FromStrRadixErr> {
        let x = T::from_str_radix(str, radix).map_err(FromStrRadixErr::Underlying)?;
        if x.is_finite() {
            Ok(Self(x))
        } else {
            Err(FromStrRadixErr::NonFinite)
        }
    }
}

impl<T> Rem for Real<T>
where
    T: FloatCore,
{
    type Output = Self;

    fn rem(self, rhs: Self) -> Self::Output {
        let r = self.0.rem(rhs.0);
        assert!(r.is_finite());
        Self(r)
    }
}

impl<T> One for Real<T>
where
    T: FloatCore,
{
    fn one() -> Self {
        Self(T::one())
    }
}

impl<T> Neg for Real<T>
where
    T: FloatCore,
{
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

macro_rules! make_unops {
    ($($id: ident), *) => {
        $(fn $id(self) -> Self {
            Self(self.0.$id())
        })*
    };
}

impl<T> FloatCore for Real<T>
where
    T: FloatCore,
{
    fn infinity() -> Self {
        Self(T::infinity())
    }

    fn nan() -> Self {
        panic!("NaN not supported");
    }

    fn max(self, other: Self) -> Self {
        Self(self.0.max(other.0))
    }

    fn min(self, other: Self) -> Self {
        Self(self.0.min(other.0))
    }

    fn powi(self, exp: i32) -> Self {
        Self(self.0.powi(exp))
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        Self(self.0.clamp(min.0, max.0))
    }

    fn epsilon() -> Self {
        Self(T::epsilon())
    }

    make_unops! { floor, fract, recip, round, trunc, signum, to_degrees, to_radians, ceil, abs}

    fn is_nan(self) -> bool {
        self.0.is_nan()
    }

    fn neg_zero() -> Self {
        Self(T::neg_zero())
    }

    fn max_value() -> Self {
        Self(T::max_value())
    }

    fn min_value() -> Self {
        Self(T::min_value())
    }

    fn classify(self) -> core::num::FpCategory {
        self.0.classify()
    }

    fn is_finite(self) -> bool {
        true
    }

    fn is_normal(self) -> bool {
        self.0.is_normal()
    }

    fn neg_infinity() -> Self {
        panic!("-infinity is not allowed");
    }

    fn min_positive_value() -> Self {
        Self(T::min_positive_value())
    }

    fn is_infinite(self) -> bool {
        false
    }

    fn is_subnormal(self) -> bool {
        self.0.is_subnormal()
    }

    fn integer_decode(self) -> (u64, i16, i8) {
        self.0.integer_decode()
    }

    fn is_sign_negative(self) -> bool {
        self.0.is_sign_negative()
    }

    fn is_sign_positive(self) -> bool {
        self.0.is_sign_positive()
    }
}

impl<T: FloatCore> Real<T> {
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

impl<T: FloatCore> Zero for Real<T> {
    fn zero() -> Self {
        Self(T::zero())
    }

    fn is_zero(&self) -> bool {
        self.0.is_zero()
    }

    fn set_zero(&mut self) {
        self.0.set_zero();
    }
}

macro_rules! add_const {
    ($($name: ident), *) => {
        $(fn $name () -> Self {
            Self(T::$name())
        })*
    };
}

impl<T: FloatConst> FloatConst for Real<T> {
    add_const! {
    E, PI, FRAC_1_PI, FRAC_1_SQRT_2, FRAC_2_PI, FRAC_2_SQRT_PI,
    FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8, LN_10,
    LN_2, LOG10_E, LOG2_E, SQRT_2}
}

impl SampleUniform for R32 {
    type Sampler = UniformReal<f32>;
}
impl SampleUniform for R64 {
    type Sampler = UniformReal<f64>;
}

pub struct UniformReal<X>(UniformFloat<X>);

impl<X> UniformSampler for UniformReal<X>
where
    UniformFloat<X>: UniformSampler<X = X>,
    X: FloatCore + SampleBorrow<X>,
{
    type X = Real<X>;

    fn new<B1, B2>(low: B1, high: B2) -> Self
    where
        B1: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
        B2: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
    {
        Self(UniformFloat::new(low.borrow().0, high.borrow().0))
    }

    fn sample<R: rand::Rng + ?Sized>(&self, rng: &mut R) -> Self::X {
        Real::new(self.0.sample(rng)).unwrap()
    }

    fn new_inclusive<B1, B2>(low: B1, high: B2) -> Self
    where
        B1: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
        B2: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
    {
        Self(UniformFloat::new_inclusive(low.borrow().0, high.borrow().0))
    }

    fn sample_single<R: rand::Rng + ?Sized, B1, B2>(low: B1, high: B2, rng: &mut R) -> Self::X
    where
        B1: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
        B2: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
    {
        Real::new(UniformFloat::<X>::sample_single(
            low.borrow().0,
            high.borrow().0,
            rng,
        ))
        .unwrap()
    }

    fn sample_single_inclusive<R: rand::Rng + ?Sized, B1, B2>(
        low: B1,
        high: B2,
        rng: &mut R,
    ) -> Self::X
    where
        B1: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
        B2: rand::distributions::uniform::SampleBorrow<Self::X> + Sized,
    {
        Real::new(UniformFloat::<X>::sample_single_inclusive(
            low.borrow().0,
            high.borrow().0,
            rng,
        ))
        .unwrap()
    }
}
