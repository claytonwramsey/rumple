use core::{marker::PhantomData, ops::AddAssign};

use kiddo::{
    float::kdtree::{Axis, KdTree},
    NearestNeighbour,
};
use num_traits::float::Float;

use crate::{nn::NearestNeighborsMap, space::Vector};

use super::{NearestEntry, RangeNearestNeighborsMap};

#[derive(Clone, Debug)]
/// A _k_-d tree map using [`kiddo::KdTree`] as its backing implementation.
///
/// Unlike [`KdTreeMap`](super::KdTreeMap)
pub struct KiddoMap<T: Default + Copy, const N: usize, M> {
    tree: KdTree<T, usize, N, 32, u32>,
    metric: PhantomData<M>,
}

pub struct KiddoEntry<V> {
    value: V,
}

impl<V> NearestEntry<V> for KiddoEntry<V> {
    fn value(&self) -> &V {
        &self.value
    }
}

impl<T: Default + Copy, const N: usize, M> KiddoMap<T, N, M> {
    #[must_use]
    pub fn new() -> Self
    where
        T: Axis,
    {
        Self {
            tree: KdTree::new(),
            metric: PhantomData,
        }
    }
}

impl<T, const N: usize> NearestNeighborsMap<Vector<N, T>, usize>
    for KiddoMap<T, N, crate::metric::SquaredEuclidean>
where
    T: Float + Default + AddAssign + Send + Sync + Axis,
{
    type Entry<'q>
        = KiddoEntry<usize>
    where
        T: 'q;
    fn insert(&mut self, key: Vector<N, T>, value: usize) {
        self.tree.add(&key, value);
    }

    fn nearest<'q>(&'q self, key: &Vector<N, T>) -> Option<Self::Entry<'q>> {
        (self.tree.size() > 0).then(|| {
            let item = self.tree.nearest_one::<kiddo::SquaredEuclidean>(key).item;
            KiddoEntry { value: item }
        })
    }
}

pub struct KiddoNearest<'a, T: Default + Copy, const N: usize, M> {
    iter: Vec<NearestNeighbour<T, usize>>,
    _phantom: PhantomData<(&'a KiddoMap<T, N, M>, M)>,
}

impl<T, const N: usize> RangeNearestNeighborsMap<Vector<N, T>, usize>
    for KiddoMap<T, N, crate::metric::SquaredEuclidean>
where
    T: Float + Default + AddAssign + Send + Sync + Axis,
{
    type Distance = T;
    type RangeNearest<'q>
        = KiddoNearest<'q, T, N, crate::metric::SquaredEuclidean>
    where
        Self: 'q;
    fn nearest_within_r<'q>(
        &'q self,
        key: &'q Vector<N, T>,
        r: Self::Distance,
    ) -> Self::RangeNearest<'q> {
        KiddoNearest {
            iter: self.tree.within_unsorted::<kiddo::SquaredEuclidean>(key, r),
            _phantom: PhantomData,
        }
    }
}

impl<T: Default + Copy, const N: usize, M> Iterator for KiddoNearest<'_, T, N, M> {
    type Item = KiddoEntry<usize>;
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.pop().map(|nbr| KiddoEntry { value: nbr.item })
    }
}

impl<T: Default + Copy + Axis, const N: usize, M> Default for KiddoMap<T, N, M> {
    fn default() -> Self {
        Self::new()
    }
}
