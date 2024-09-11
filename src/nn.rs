use std::cmp::Ordering;

use crate::{Metric, NearestNeighborsMap};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct KdTreeMap<K, V, M> {
    root: Option<Node<K, V>>,
    metric: M,
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

pub trait RegionMetric<C>: Metric<C> {
    /// Construct a region containing the entire metric space.
    fn whole_space(&self) -> Region<C>;

    /// Compute the distance between a point and a region.
    fn dist_to_region(&self, point: &C, region: &Region<C>) -> Self::Distance;

    /// Get the dimension of the space.
    /// This quantity should be stable across subsequent calls.
    fn dimension(&self) -> usize;

    /// Compare configurations `c0` and `c1` by their values along dimension `k`.
    /// Returns `Ordering::Less` if `c0[k] < c1[k]`, `Ordering::Equal` if `c0[k] == c1[k]`,
    /// and `Ordering::Greater` if `c0[k] > c1[k]`.
    fn compare(&self, c0: &C, c1: &C, k: usize) -> Ordering;

    /// Set the value of `dest[k]` to be `src[k]`.
    fn set_dim(&self, dest: &mut C, src: &C, k: usize);
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Node<K, V> {
    key: K,
    value: V,
    children: [Option<Box<Self>>; 2],
}

impl<K, V, M> KdTreeMap<K, V, M> {
    pub const fn new(metric: M) -> Self {
        Self { root: None, metric }
    }
}

impl<K, V, M> NearestNeighborsMap<K, V, M> for KdTreeMap<K, V, M>
where
    M: RegionMetric<K>,
    K: Clone,
{
    fn insert(&mut self, key: K, value: V) {
        let Some(mut parent) = self.root.as_mut() else {
            self.root = Some(Node {
                key,
                value,
                children: [None, None],
            });
            return;
        };

        let mut k = 0;
        let mut side: usize;
        while {
            side = self.metric.compare(&parent.key, &key, k).is_le().into();
            parent.children[side].is_some()
        } {
            parent = parent.children[side]
                .as_mut()
                .expect("parent has already been checked to have this child");
            k = (k + 1) % self.metric.dimension();
        }
        parent.children[side] = Some(Box::new(Node {
            key,
            value,
            children: [None, None],
        }));
    }

    fn nearest<'q>(&'q self, key: &K) -> Option<(&'q K, &'q V)> {
        let region = self.metric.whole_space();
        let root = self.root.as_ref()?;
        let mut radius = self.metric.distance(&root.key, key);
        if self.metric.is_zero(&radius) {
            return Some((&root.key, &root.value));
        }
        let best_node =
            Self::nearest_help(root, key, region, &mut radius, &self.metric, 0).unwrap_or(root);
        Some((&best_node.key, &best_node.value))
    }
}

impl<K, V, M> KdTreeMap<K, V, M>
where
    M: RegionMetric<K>,
    K: Clone,
{
    fn nearest_help<'q>(
        node: &'q Node<K, V>,
        key: &K,
        mut region: Region<K>,
        radius: &mut M::Distance,
        metric: &M,
        k: usize,
    ) -> Option<&'q Node<K, V>> {
        let is_right = metric.compare(&node.key, key, k).is_le();
        let mut best_result = None;
        // search right side first
        let children = if is_right { [1, 0] } else { [0, 1] }.map(|i| node.children[i].as_deref());
        let mut r_shrunk = region.clone();
        metric.set_dim(&mut r_shrunk.lo, &node.key, k);
        if metric.dist_to_region(key, &r_shrunk) > *radius {
            // out of bounds
            return None;
        }
        if let Some(child) = children[0] {
            let cdist = metric.distance(&child.key, key);
            if cdist <= *radius {
                *radius = cdist;
                if metric.is_zero(radius) {
                    // exact match to query
                    return Some(child);
                }
                best_result = Some(
                    Self::nearest_help(
                        child,
                        key,
                        r_shrunk,
                        radius,
                        metric,
                        (k + 1) % metric.dimension(),
                    )
                    .unwrap_or(child),
                );
            }
        }
        if let Some(child) = children[1] {
            let cdist = metric.distance(&child.key, key);
            if cdist <= *radius {
                *radius = cdist;
                if metric.is_zero(radius) {
                    // exact match to query
                    return Some(child);
                }
                metric.set_dim(&mut region.lo, &node.key, k);
                return Some(
                    Self::nearest_help(
                        child,
                        key,
                        region,
                        radius,
                        metric,
                        (k + 1) % metric.dimension(),
                    )
                    .unwrap_or(child),
                );
            }
        }

        best_result
    }
}

#[cfg(test)]
mod tests {
    use core::f64;

    use crate::{Metric, NearestNeighborsMap};
    use ordered_float::NotNan;

    use super::{KdTreeMap, Region, RegionMetric};

    type RealVector<const N: usize> = [NotNan<f64>; N];

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct SquaredEuclidean;

    impl<const N: usize> Metric<RealVector<N>> for SquaredEuclidean {
        type Distance = NotNan<f64>;

        fn distance(&self, c1: &RealVector<N>, c2: &RealVector<N>) -> Self::Distance {
            c1.iter()
                .zip(c2.iter())
                .map(|(&a, &b)| (a - b) * (a - b))
                .sum()
        }

        fn is_zero(&self, dist: &Self::Distance) -> bool {
            *dist == 0.0
        }
    }

    impl<const N: usize> RegionMetric<RealVector<N>> for SquaredEuclidean {
        fn compare(&self, c0: &RealVector<N>, c1: &RealVector<N>, k: usize) -> std::cmp::Ordering {
            c0[k].cmp(&c1[k])
        }

        fn dimension(&self) -> usize {
            N
        }

        fn set_dim(&self, dest: &mut RealVector<N>, src: &RealVector<N>, k: usize) {
            dest[k] = src[k];
        }

        fn whole_space(&self) -> Region<RealVector<N>> {
            Region {
                lo: [NotNan::new(f64::NEG_INFINITY).unwrap(); N],
                hi: [NotNan::new(f64::INFINITY).unwrap(); N],
            }
        }

        fn dist_to_region(
            &self,
            point: &RealVector<N>,
            region: &Region<RealVector<N>>,
        ) -> Self::Distance {
            point
                .iter()
                .zip(&region.lo)
                .zip(&region.hi)
                .map(|((p, l), h)| {
                    if p < l {
                        l - p
                    } else if h < p {
                        p - h
                    } else {
                        NotNan::new(0.0).unwrap()
                    }
                })
                .map(|d| d * d)
                .sum()
        }
    }

    fn cvpoint<const N: usize>(point: [f64; N]) -> [NotNan<f64>; N] {
        point.map(|x| NotNan::new(x).unwrap())
    }

    fn build_tree<const N: usize>(
        points: &[[f64; N]],
    ) -> KdTreeMap<RealVector<N>, (), SquaredEuclidean> {
        let mut t = KdTreeMap::new(SquaredEuclidean);
        for &point in points {
            t.insert(cvpoint(point), ());
        }
        t
    }

    #[test]
    fn make_tree() {
        let points = [[0.0, 0.0], [0.5, 0.5]];
        let t = build_tree(&points);
        println!("{t:?}");
    }

    #[test]
    fn get_empty() {
        let t = build_tree(&[]);
        assert_eq!(t.nearest(&[NotNan::new(0.0).unwrap(); 2]), None);
    }

    #[test]
    fn get_one() {
        let t = build_tree(&[[1.0, 1.0]]);
        assert_eq!(
            t.nearest(&cvpoint([0.0; 2])),
            Some((&cvpoint([1.0, 1.0]), &()))
        );
    }

    #[test]
    fn across_border() {
        let t = build_tree(&[[1.0, 1.0], [1.5, 1.1], [-0.5, 0.5]]);
        println!("{t:?}");
        assert_eq!(
            t.nearest(&cvpoint([0.0; 2])),
            Some((&cvpoint([-0.5, 0.5]), &()))
        );
    }
}
