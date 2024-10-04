use core::iter::Sum;

use crate::{
    space::{Angle, Vector},
    Metric,
};
use num_traits::{float::FloatCore, FloatConst};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SquaredEuclidean;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Euclidean;

impl SquaredEuclidean {
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
    T: FloatCore + Ord + Sum,
{
    type Distance = T;

    fn distance(&self, c1: &Vector<N, T>, c2: &Vector<N, T>) -> Self::Distance {
        self.partial_distance(c1, c2)
    }
}

impl<T> Metric<Angle<T>> for Euclidean
where
    T: FloatCore + FloatConst + Ord,
{
    type Distance = T;

    fn distance(&self, c1: &Angle<T>, c2: &Angle<T>) -> Self::Distance {
        c1.signed_distance(*c2).abs()
    }
}

impl<T> Metric<Angle<T>> for SquaredEuclidean
where
    T: FloatCore + FloatConst + Ord,
{
    type Distance = T;

    fn distance(&self, c1: &Angle<T>, c2: &Angle<T>) -> Self::Distance {
        let d = Euclidean.distance(c1, c2);
        d * d
    }
}
