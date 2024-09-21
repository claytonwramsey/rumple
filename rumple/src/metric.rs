use crate::{float::Real, space::RealVector, Metric};
use num_traits::float::FloatCore;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SquaredEuclidean;

impl<T, const N: usize> Metric<RealVector<N, T>> for SquaredEuclidean
where
    T: FloatCore,
{
    type Distance = Real<T>;

    fn distance(&self, c1: &RealVector<N, T>, c2: &RealVector<N, T>) -> Self::Distance {
        c1.iter()
            .zip(c2.iter())
            .map(|(&a, &b)| (a - b) * (a - b))
            .sum()
    }
}
