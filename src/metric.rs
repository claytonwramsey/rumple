//! Distance metrics.

use core::{array, iter::Sum};

use crate::{
    nn::DistanceAabb,
    space::{Angle, Vector},
    Metric,
};
use num_traits::{float::FloatCore, FloatConst};

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
        T: FloatCore,
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
    T: FloatCore + Sum,
{
    type Distance = T;

    fn distance(&self, c1: &Vector<N, T>, c2: &Vector<N, T>) -> Self::Distance {
        self.partial_distance(c1, c2)
    }
}

impl<T, const N: usize> DistanceAabb<Vector<N, T>> for SquaredEuclidean
where
    T: FloatCore + Sum,
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
                FloatCore::clamp(c[i], aabb_lo[i], aabb_hi[i])
            })),
        )
    }
}

impl<T> Metric<Angle<T>> for Euclidean
where
    T: FloatCore + FloatConst,
{
    type Distance = T;

    fn distance(&self, c1: &Angle<T>, c2: &Angle<T>) -> Self::Distance {
        c1.signed_distance(*c2).abs()
    }
}

impl<T> Metric<Angle<T>> for SquaredEuclidean
where
    T: FloatCore + FloatConst,
{
    type Distance = T;

    fn distance(&self, c1: &Angle<T>, c2: &Angle<T>) -> Self::Distance {
        let d = Euclidean.distance(c1, c2);
        d * d
    }
}

impl<T> DistanceAabb<Angle<T>> for Euclidean
where
    T: FloatCore + FloatConst,
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
    T: FloatCore + FloatConst,
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
