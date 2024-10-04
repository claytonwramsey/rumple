use crate::{Metric, NearestNeighborsMap, RangeNearestNeighborsMap};
use alloc::{boxed::Box, vec::Vec};
use core::{cmp::Ordering, fmt::Debug, marker::PhantomData};
use num_traits::Zero;

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

pub trait KdKey: Clone {
    fn dimension() -> usize;
    fn compare(&self, rhs: &Self, k: usize) -> Ordering;
    fn assign(&mut self, src: &Self, k: usize);
    fn lower_bound() -> Self;
    fn upper_bound() -> Self;
}

pub trait DistanceAabb<C>: Metric<C> {
    fn distance_to_aabb(&self, c: &C, aabb_lo: &C, aabb_hi: &C) -> Self::Distance;
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

impl<K, V, M> NearestNeighborsMap<K, V> for KdTreeMap<K, V, M>
where
    M: DistanceAabb<K>,
    K: KdKey,
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
            side = parent.key.compare(&key, k).is_le().into();
            parent.children[side].is_some()
        } {
            parent = parent.children[side]
                .as_mut()
                .expect("parent has already been checked to have this child");
            k = (k + 1) % K::dimension();
        }
        parent.children[side] = Some(Box::new(Node {
            key,
            value,
            children: [None, None],
        }));
    }

    fn nearest<'q>(&'q self, key: &K) -> Option<(&'q K, &'q V)> {
        let root = self.root.as_ref()?;
        let mut radius = self.metric.distance(&root.key, key);
        if radius.is_zero() {
            return Some((&root.key, &root.value));
        }
        let best_node = self
            .nearest_help(
                root,
                key,
                K::lower_bound(),
                K::upper_bound(),
                &mut radius,
                0,
            )
            .unwrap_or(root);
        Some((&best_node.key, &best_node.value))
    }
}

// TODO make this a resuming iterator
pub struct RangeNearest<'a, K, V, M>(Vec<&'a V>, PhantomData<&'a KdTreeMap<K, V, M>>);

impl<'a, K, V, M> Iterator for RangeNearest<'a, K, V, M> {
    type Item = &'a V;
    fn next(&mut self) -> Option<Self::Item> {
        self.0.pop()
    }
}

impl<K, V, M> RangeNearestNeighborsMap<K, V> for KdTreeMap<K, V, M>
where
    M: DistanceAabb<K>,
    K: KdKey,
{
    type Distance = <M as Metric<K>>::Distance;
    type RangeNearest<'q> = RangeNearest<'q, K, V, M> where K: 'q, V: 'q, M: 'q;

    fn nearest_within_r<'q>(&'q self, key: &K, r: Self::Distance) -> Self::RangeNearest<'q> {
        let mut result = Vec::new();
        if let Some(root) = self.root.as_ref() {
            self.nearest_r_help(
                key,
                &mut result,
                &r,
                root,
                K::lower_bound(),
                K::upper_bound(),
                0,
            );
        }
        RangeNearest(result, PhantomData)
    }
}

impl<K, V, M> KdTreeMap<K, V, M>
where
    M: DistanceAabb<K>,
    K: KdKey,
{
    fn nearest_help<'q>(
        &self,
        node: &'q Node<K, V>,
        key: &K,
        mut reg_lo: K,
        mut reg_hi: K,
        radius: &mut <M as Metric<K>>::Distance,
        k: usize,
    ) -> Option<&'q Node<K, V>> {
        let mut best_result = None;
        let is_right = node.key.compare(key, k).is_le();
        // search right side first
        let children = if is_right { [1, 0] } else { [0, 1] }.map(|i| node.children[i].as_deref());

        if let Some(child) = children[0] {
            let cdist = self.metric.distance(&child.key, key);
            if cdist <= *radius {
                *radius = cdist;
                best_result = Some(child);
                if radius.is_zero() {
                    // exact match to query
                    return best_result;
                }
            }

            best_result = self
                .nearest_help(
                    child,
                    key,
                    reg_lo.clone(),
                    reg_hi.clone(),
                    radius,
                    (k + 1) % K::dimension(),
                )
                .or(best_result);
        }
        if let Some(child) = children[1] {
            let cdist = self.metric.distance(&child.key, key);
            if cdist <= *radius {
                *radius = cdist;
                best_result = Some(child);
                if radius.is_zero() {
                    // exact match to query
                    return best_result;
                }
            }

            if is_right {
                reg_hi.assign(&node.key, k);
            } else {
                reg_lo.assign(&node.key, k);
            }
            if self.metric.distance_to_aabb(key, &reg_lo, &reg_hi) < *radius {
                best_result = self
                    .nearest_help(child, key, reg_lo, reg_hi, radius, (k + 1) % K::dimension())
                    .or(best_result);
            }
        }

        best_result
    }

    #[expect(clippy::too_many_arguments)]
    fn nearest_r_help<'q>(
        &'q self,
        point: &K,
        buf: &mut Vec<&'q V>,
        radius: &<M as Metric<K>>::Distance,
        node: &'q Node<K, V>,
        mut reg_lo: K,
        mut reg_hi: K,
        k: usize,
    ) {
        if &self.metric.distance(point, &node.key) <= radius {
            buf.push(&node.value);
        }

        let is_left = point.compare(&node.key, k).is_lt();
        let [near_child, far_child] = if is_left {
            [node.children[0].as_deref(), node.children[1].as_deref()]
        } else {
            [node.children[1].as_deref(), node.children[0].as_deref()]
        };

        let new_k = (k + 1) % K::dimension();
        if let Some(c) = near_child {
            self.nearest_r_help(point, buf, radius, c, reg_lo.clone(), reg_hi.clone(), new_k);
        }

        if let Some(c) = far_child {
            if is_left {
                reg_lo.assign(&node.key, k);
            } else {
                reg_hi.assign(&node.key, k);
            }
            if &self.metric.distance_to_aabb(point, &reg_lo, &reg_hi) <= radius {
                self.nearest_r_help(point, buf, radius, c, reg_lo, reg_hi, new_k);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    use super::*;
    use crate::{
        float::{r32, r64, R32, R64},
        metric::SquaredEuclidean,
        sample::Rectangle,
        space::{Pose2d, Vector, WeightedPoseDistance},
        AlwaysValid, Sample,
    };

    struct BruteForce<K, V, M> {
        poses: Vec<K>,
        values: Vec<V>,
        metric: M,
    }

    impl<K, V, M> NearestNeighborsMap<K, V> for BruteForce<K, V, M>
    where
        M: Metric<K>,
    {
        fn insert(&mut self, key: K, value: V) {
            self.poses.push(key);
            self.values.push(value);
        }

        fn nearest<'q>(&'q self, key: &K) -> Option<(&'q K, &'q V)> {
            let mut best_i = 0;
            let mut best_dist = self.metric.distance(self.poses.first()?, key);
            for (i, pose) in self.poses.iter().enumerate() {
                let dist = self.metric.distance(pose, key);
                if dist < best_dist {
                    best_i = i;
                    best_dist = dist;
                }
            }

            Some((&self.poses[best_i], &self.values[best_i]))
        }
    }

    fn build_tree<const N: usize>(
        points: &[[f64; N]],
    ) -> KdTreeMap<Vector<N, R64>, (), SquaredEuclidean> {
        let mut t = KdTreeMap::new(SquaredEuclidean);
        for &point in points {
            t.insert(Vector::new(point.map(r64)), ());
        }
        t
    }

    #[test]
    fn make_tree() {
        let points = [[0.0, 0.0], [0.5, 0.5]];
        let _ = build_tree(&points);
    }

    #[test]
    fn get_empty() {
        let t = build_tree(&[]);
        assert_eq!(t.nearest(&Vector::new([0.0, 0.0].map(r64))), None);
    }

    #[test]
    fn get_one() {
        let t = build_tree(&[[1.0, 1.0]]);
        assert_eq!(
            t.nearest(&Vector::new([0.0, 0.0].map(r64))),
            Some((&Vector::new([1.0, 1.0].map(r64)), &()))
        );
    }

    #[test]
    fn across_border() {
        let t = build_tree(&[[1.0, 1.0], [1.5, 1.1], [-0.5, 0.5]]);
        // println!("{t:?}");
        assert_eq!(
            t.nearest(&Vector::new([0.0, 0.0].map(r64))),
            Some((&Vector::new([-0.5, 0.5].map(r64)), &()))
        );
    }

    #[test]
    fn make_rrt() {
        use crate::geo::Rrt;
        let _rrt = Rrt::new(
            Vector::new([r64(0.0)]),
            KdTreeMap::new(SquaredEuclidean),
            &AlwaysValid,
        );
    }

    #[test]
    fn randomized_3d() {
        const N: usize = 3;
        let region = Rectangle {
            min: Vector::new([r32(-10.0); N]),
            max: Vector::new([r32(10.0); N]),
        };

        let mut rng = ChaCha20Rng::seed_from_u64(2707);

        let mut bf = BruteForce {
            poses: Vec::new(),
            values: Vec::new(),
            metric: SquaredEuclidean,
        };
        let mut kdt = KdTreeMap::new(SquaredEuclidean);
        for _ in 0..2_000 {
            let pt: Vector<N, R32> = region.sample(&mut rng);
            println!("insert {pt:?}");
            bf.insert(pt, ());
            kdt.insert(pt, ());
            let q = region.sample(&mut rng);
            // println!("{kdt:#?}");
            println!("query {q:?}");
            let bf_nearest = bf.nearest(&q);
            let kdt_nearest = kdt.nearest(&q);
            assert_eq!(bf_nearest, kdt_nearest);
        }
    }

    #[test]
    fn pose2d() {
        let region = Rectangle {
            min: Vector::new([r32(-10.0); 2]),
            max: Vector::new([r32(10.0); 2]),
        };

        let mut rng = ChaCha20Rng::seed_from_u64(2707);

        let m = WeightedPoseDistance {
            position_metric: SquaredEuclidean,
            position_weight: r32(1.0),
            angle_metric: SquaredEuclidean,
            angle_weight: r32(1.0),
        };
        let mut bf = BruteForce {
            poses: Vec::new(),
            values: Vec::new(),
            metric: m,
        };
        let mut kdt = KdTreeMap::new(m);
        for i in 0..2_000 {
            println!("i={i}");
            let pt: Pose2d<R32> = region.sample(&mut rng);
            println!("insert {pt:?}");
            bf.insert(pt, ());
            kdt.insert(pt, ());
            let q = region.sample(&mut rng);
            // println!("{kdt:#?}");
            println!("query {q:?}");
            let bf_nearest = bf.nearest(&q).unwrap().0;
            let kdt_nearest = kdt.nearest(&q).unwrap().0;
            println!("bf distance {:?}", m.distance(bf_nearest, &q));
            println!("kdt distance {:?}", m.distance(kdt_nearest, &q));
            assert_eq!(bf_nearest, kdt_nearest);
        }
    }
}
