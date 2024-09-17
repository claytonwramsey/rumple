use crate::{
    float::Real,
    nn::{Region, RegionMetric},
    space::RealVector,
    Metric,
};
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

impl<T, const N: usize> RegionMetric<RealVector<N, T>> for SquaredEuclidean
where
    T: FloatCore,
{
    fn compare(
        &self,
        c0: &RealVector<N, T>,
        c1: &RealVector<N, T>,
        k: usize,
    ) -> core::cmp::Ordering {
        c0[k].cmp(&c1[k])
    }

    fn dimension(&self) -> usize {
        N
    }

    fn set_dim(&self, dest: &mut RealVector<N, T>, src: &RealVector<N, T>, k: usize) {
        dest[k] = src[k];
    }

    fn whole_space(&self) -> Region<RealVector<N, T>> {
        Region {
            lo: RealVector::new([Real::new(T::min_value()); N]),
            hi: RealVector::new([Real::new(T::max_value()); N]),
        }
    }

    fn dist_to_region(
        &self,
        point: &RealVector<N, T>,
        region: &Region<RealVector<N, T>>,
    ) -> Self::Distance {
        (**point)
            .iter()
            .zip(&*region.lo)
            .zip(&*region.hi)
            .map(|((&p, &l), &h)| {
                if p < l {
                    l - p
                } else if h < p {
                    p - h
                } else {
                    Real::new(T::zero())
                }
            })
            .map(|d| d * d)
            .sum()
    }
}
