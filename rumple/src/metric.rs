use core::iter::Sum;

use crate::{space::Vector, Metric};
use num_traits::float::FloatCore;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SquaredEuclidean;

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
