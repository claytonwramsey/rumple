use std::cmp::Ordering;

use crate::{MetricSpace, NearestNeighborsMap};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct KdTreeMap<K, V> {
    root: Option<Node<K, V>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// A prismatic region in a `K`-dimensional space.
/// Assumed to be half-open (i.e. lower-bound inclusive).
pub struct Region<C> {
    /// The "bottom-left" point in the region, containing the minimum value along each axis.
    pub lo: C,
    /// The "top-right" point in the region, containing the maximum value along each axis.
    /// Note that the region technically does not contain `hi` as it is half-open.
    pub hi: C,
}

pub trait RegionMetric: MetricSpace {
    /// Construct a region containing the entire metric space.
    fn whole_space(&self) -> Region<Self::Configuration>;

    /// Compute the distance between a point and a region.
    fn dist_to_region(
        &self,
        point: &Self::Configuration,
        region: &Region<Self::Configuration>,
    ) -> Self::Distance;

    /// Get the dimension of the space.
    /// This quantity should be stable across subsequent calls.
    fn dimension(&self) -> usize;

    /// Compare configurations `c0` and `c1` by their values along dimension `k`.
    /// Returns `Ordering::Less` if `c0[k] < c1[k]`, `Ordering::Equal` if `c0[k] == c1[k]`,
    /// and `Ordering::Greater` if `c0[k] > c1[k]`.
    fn compare(&self, c0: &Self::Configuration, c1: &Self::Configuration, k: usize) -> Ordering;

    /// Set the value of `dest[k]` to be `src[k]`.
    fn set_dim(&self, dest: &mut Self::Configuration, src: &Self::Configuration, k: usize);
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Node<K, V> {
    point: K,
    value: V,
    children: [Option<Box<Self>>; 2],
}

impl<K, V, M> NearestNeighborsMap<K, V, M> for KdTreeMap<K, V>
where
    M: RegionMetric<Configuration = K>,
    M::Configuration: Clone,
{
    fn empty() -> Self {
        Self { root: None }
    }

    fn insert(&mut self, key: K, value: V, metric: &M) {
        let Some(mut parent) = self.root.as_mut() else {
            self.root = Some(Node {
                point: key,
                value,
                children: [None, None],
            });
            return;
        };

        let mut k = 0;
        let mut side: usize;
        while {
            side = metric.compare(&parent.point, &key, k).is_le().into();
            parent.children[side].is_some()
        } {
            parent = parent.children[side]
                .as_mut()
                .expect("parent has already been checked to have this child");
            k = (k + 1) % metric.dimension();
        }
        parent.children[side] = Some(Box::new(Node {
            point: key,
            value,
            children: [None, None],
        }));
    }

    fn nearest<'q>(&'q self, key: &K, metric: &M) -> Option<(&'q K, &'q V)> {
        let region = metric.whole_space();
        let root = self.root.as_ref()?;
        let mut radius = metric.distance(&root.point, key);
        if metric.is_zero(&radius) {
            return Some((&root.point, &root.value));
        }
        let best_node = root
            .nearest_help(key, region, &mut radius, metric, 0)
            .unwrap_or(root);
        Some((&best_node.point, &best_node.value))
    }
}

impl<K, V> Node<K, V> {
    /// Find the node nearest to `key` with distance less than equal to `radius` of `key`.
    /// Updates `radius` to tightest radius found so far.
    /// A radius of `None` implies infinite radius (i.e. no tighter bound).
    fn nearest_help<'q, M>(
        &'q self,
        key: &K,
        mut region: Region<K>,
        radius: &mut M::Distance,
        metric: &M,
        k: usize,
    ) -> Option<&'q Self>
    where
        M: RegionMetric<Configuration = K>,
        M::Configuration: Clone,
    {
        let is_right = metric.compare(&self.point, key, k).is_le();
        let mut best_result = None;
        // search right side first
        let children = if is_right { [1, 0] } else { [0, 1] }.map(|i| self.children[i].as_ref());
        let mut r_shrunk = region.clone();
        metric.set_dim(&mut r_shrunk.lo, &self.point, k);
        if metric.dist_to_region(key, &r_shrunk) >= *radius {
            // out of bounds
            return None;
        }
        if let Some(child) = children[0] {
            let cdist = metric.distance(&child.point, key);
            if cdist <= *radius {
                *radius = cdist;
                if metric.is_zero(radius) {
                    // exact match to query
                    return Some(child);
                }
            }
            best_result =
                child.nearest_help(key, r_shrunk, radius, metric, (k + 1) % metric.dimension());
        }
        if let Some(child) = children[1] {
            let cdist = metric.distance(&child.point, key);
            if cdist <= *radius {
                *radius = cdist;
                if metric.is_zero(radius) {
                    // exact match to query
                    return Some(child);
                }
            }
            metric.set_dim(&mut region.lo, &self.point, k);
            best_result =
                child.nearest_help(key, region, radius, metric, (k + 1) % metric.dimension());
        }

        best_result
    }
}
