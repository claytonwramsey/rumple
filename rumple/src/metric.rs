//! Distance metrics.

use core::array;

use crate::{
    nn::DistanceAabb,
    space::{Angle, Vector},
};
use num_traits::{Float, FloatConst, Zero};

/// A metric between configurations.
pub trait Metric<C> {
    /// The distance between configurations.
    type Distance: PartialOrd + Zero;

    /// Compute the distance between `c1` and `c2`.
    fn distance(&self, c1: &C, c2: &C) -> Self::Distance;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The squared-Euclidean distance metric, i.e. the sum of the squares along each axis.
pub struct SquaredEuclidean;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The Euclidean distance metric, i.e. the length of the line segment connecting two points.
pub struct Euclidean;

impl SquaredEuclidean {
    /// Computer the distance between two vectors without requiring that the result be strictly
    /// ordered.
    pub fn partial_distance<T, const N: usize>(&self, c1: &Vector<N, T>, c2: &Vector<N, T>) -> T
    where
        T: Float,
    {
        let mut total = T::zero();
        for (&a, &b) in c1.iter().zip(c2.iter()) {
            total = total + (a - b) * (a - b);
        }
        total
    }
}

impl<T, const N: usize> Metric<Vector<N, T>> for SquaredEuclidean
where
    T: Float,
{
    type Distance = T;

    fn distance(&self, c1: &Vector<N, T>, c2: &Vector<N, T>) -> Self::Distance {
        self.partial_distance(c1, c2)
    }
}

impl<T, const N: usize> DistanceAabb<Vector<N, T>> for SquaredEuclidean
where
    T: Float,
{
    fn distance_to_aabb(
        &self,
        c: &Vector<N, T>,
        aabb_lo: &Vector<N, T>,
        aabb_hi: &Vector<N, T>,
    ) -> Self::Distance {
        Self.distance(
            c,
            &Vector(array::from_fn(|i| {
                Float::clamp(c[i], aabb_lo[i], aabb_hi[i])
            })),
        )
    }
}

impl<T> Metric<Angle<T>> for Euclidean
where
    T: Float + FloatConst,
{
    type Distance = T;

    fn distance(&self, c1: &Angle<T>, c2: &Angle<T>) -> Self::Distance {
        c1.signed_distance(*c2).abs()
    }
}

impl<const N: usize, T> Metric<Vector<N, T>> for Euclidean
where
    T: Float,
{
    type Distance = T;
    fn distance(&self, c1: &Vector<N, T>, c2: &Vector<N, T>) -> Self::Distance {
        let mut total = T::zero();
        for (&a, &b) in c1.iter().zip(c2.iter()) {
            total = total + (a - b) * (a - b);
        }
        total.sqrt()
    }
}

impl<T> Metric<Angle<T>> for SquaredEuclidean
where
    T: Float + FloatConst,
{
    type Distance = T;

    fn distance(&self, c1: &Angle<T>, c2: &Angle<T>) -> Self::Distance {
        let d = Euclidean.distance(c1, c2);
        d * d
    }
}

impl<T> DistanceAabb<Angle<T>> for Euclidean
where
    T: Float + FloatConst,
{
    fn distance_to_aabb(
        &self,
        c: &Angle<T>,
        aabb_lo: &Angle<T>,
        aabb_hi: &Angle<T>,
    ) -> Self::Distance {
        if aabb_lo <= c && c <= aabb_hi {
            T::zero()
        } else {
            Self.distance(c, aabb_lo).min(Self.distance(c, aabb_hi))
        }
    }
}

impl<T> DistanceAabb<Angle<T>> for SquaredEuclidean
where
    T: Float + FloatConst,
{
    fn distance_to_aabb(
        &self,
        c: &Angle<T>,
        aabb_lo: &Angle<T>,
        aabb_hi: &Angle<T>,
    ) -> Self::Distance {
        Euclidean.distance_to_aabb(c, aabb_lo, aabb_hi).powi(2)
    }
}

impl Default for SquaredEuclidean {
    fn default() -> Self {
        Self
    }
}

impl Default for Euclidean {
    fn default() -> Self {
        Self
    }
}
