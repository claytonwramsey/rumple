use core::{marker::PhantomData, ops::AddAssign};

use kiddo::{
    float::kdtree::{Axis, KdTree},
    NearestNeighbour,
};
use num_traits::float::FloatCore;

use crate::{nn::NearestNeighborsMap, space::Vector};

use super::RangeNearestNeighborsMap;

#[expect(clippy::module_name_repetitions)]
#[derive(Clone, Debug)]
/// A _k_-d tree map using [`kiddo::KdTree`] as its backing implementation.
///
/// Unlike [`KdTreeMap`](super::KdTreeMap)
pub struct KiddoMap<T: Default + Copy, const N: usize, V, M> {
    tree: KdTree<T, usize, N, 32, u32>,
    keys: Vec<Vector<N, T>>,
    values: Vec<V>,
    metric: PhantomData<M>,
}

impl<T: Default + Copy, const N: usize, V, M> KiddoMap<T, N, V, M> {
    #[must_use]
    pub fn new() -> Self
    where
        T: Axis,
    {
        Self {
            tree: KdTree::new(),
            keys: Vec::new(),
            values: Vec::new(),
            metric: PhantomData,
        }
    }
}

impl<T, const N: usize, V> NearestNeighborsMap<Vector<N, T>, V>
    for KiddoMap<T, N, V, crate::metric::SquaredEuclidean>
where
    T: FloatCore + Default + AddAssign + Send + Sync + Axis,
{
    fn insert(&mut self, key: Vector<N, T>, value: V) {
        self.tree.add(&key, self.values.len());
        self.values.push(value);
        self.keys.push(key);
    }

    fn nearest<'q>(&'q self, key: &Vector<N, T>) -> Option<(&'q Vector<N, T>, &'q V)> {
        (!self.values.is_empty()).then(|| {
            let item = self.tree.nearest_one::<kiddo::SquaredEuclidean>(key).item;
            (&self.keys[item], &self.values[item])
        })
    }
}

#[expect(clippy::module_name_repetitions)]
pub struct KiddoNearest<'a, T, const N: usize, V, M> {
    iter: Vec<NearestNeighbour<T, usize>>,
    keys: &'a [Vector<N, T>],
    values: &'a [V],
    _phantom: PhantomData<M>,
}

impl<T, const N: usize, V> RangeNearestNeighborsMap<Vector<N, T>, V>
    for KiddoMap<T, N, V, crate::metric::SquaredEuclidean>
where
    T: FloatCore + Default + AddAssign + Send + Sync + Axis,
{
    type Distance = T;
    type RangeNearest<'q> = KiddoNearest<'q, T, N, V, crate::metric::SquaredEuclidean> where Self: 'q;
    fn nearest_within_r<'q>(
        &'q self,
        key: &'q Vector<N, T>,
        r: Self::Distance,
    ) -> Self::RangeNearest<'q> {
        KiddoNearest {
            iter: self.tree.within_unsorted::<kiddo::SquaredEuclidean>(key, r),
            keys: &self.keys,
            values: &self.values,
            _phantom: PhantomData,
        }
    }
}

impl<'a, T, const N: usize, V, M> Iterator for KiddoNearest<'a, T, N, V, M> {
    type Item = (&'a Vector<N, T>, &'a V);
    fn next(&mut self) -> Option<Self::Item> {
        self.iter
            .pop()
            .map(|nbr| (&self.keys[nbr.item], &self.values[nbr.item]))
    }
}

impl<T: Default + Copy + Axis, const N: usize, V, M> Default for KiddoMap<T, N, V, M> {
    fn default() -> Self {
        Self::new()
    }
}
