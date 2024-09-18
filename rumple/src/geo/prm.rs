use alloc::collections::BinaryHeap;
use core::{fmt::Debug, hash::Hash, iter, marker::PhantomData, mem::swap, ops::Add};

use alloc::vec::Vec;
use num_traits::Zero;

use crate::{time::Timeout, Metric, RangeNearestNeighborsMap, Sample, Validate};

pub struct Prm<'a, C, NN, V> {
    /// List of configurations for each node.
    configurations: Vec<C>,
    /// Adjacency list of nodes.
    edges: Vec<Vec<usize>>,
    components: SetForest,
    nn: NN,
    valid: &'a V,
}

mod private {
    pub struct Node(pub usize);
}
use private::Node;

#[allow(clippy::module_name_repetitions)]
pub struct PrmNodeId<PRM>(usize, PhantomData<PRM>);

#[derive(Clone, Debug)]
/// A disjoint set forest.
/// todo: should we move away from the S-o-A structure? Probably not, this is likely faster.
struct SetForest {
    /// Integers representing the component each node belongs to.
    /// This is a random set forest, if `components[i] != i`, `i` belongs to the same component as
    /// `components[i]`.
    parents: Vec<usize>,
    /// For some representative node `i`, `component_sizes[i]` is the number of total nodes in its
    /// component. The size is only cached for true representatives, so you must find the
    /// representatitive node ID for a component before checking its size.
    sizes: Vec<usize>,
}

impl<'a, C, NN, V> Prm<'a, C, NN, V> {
    #[must_use]
    /// Construct a new PRM.
    pub const fn new(nn: NN, valid: &'a V) -> Self {
        Self {
            configurations: Vec::new(),
            edges: Vec::new(),
            components: SetForest::new(),
            nn,
            valid,
        }
    }

    pub fn grow_r<R, TC, S, RNG>(&mut self, radius: R, timeout: &mut TC, sample: &S, rng: &mut RNG)
    where
        V: Validate<C>,
        NN: RangeNearestNeighborsMap<C, Node, Distance = R>,
        TC: Timeout,
        S: Sample<C, RNG>,
        C: Clone,
        R: Clone,
    {
        while !timeout.is_over() {
            timeout.update_sample_count(1);
            let c = sample.sample(rng);
            if self.insert_r(c, radius.clone()).is_some() {
                timeout.update_node_count(1);
            }
        }
    }

    /// Grow this PRM while attempting to solve a problem connecting `start` and `goal`, connecting
    /// nodes within a radius of `radius` of each other. This will only terminate when `timeout`
    /// is over.
    pub fn grow_r_solve<R, TC, S, RNG>(
        &mut self,
        radius: R,
        timeout: &mut TC,
        sample: &S,
        rng: &mut RNG,
        start: PrmNodeId<Self>,
        goal: PrmNodeId<Self>,
    ) where
        V: Validate<C>,
        NN: RangeNearestNeighborsMap<C, Node, Distance = R>,
        TC: Timeout,
        S: Sample<C, RNG>,
        C: Clone,
        R: Clone,
    {
        while !timeout.is_over() {
            timeout.update_sample_count(1);
            let c = sample.sample(rng);
            if self.insert_r(c, radius.clone()).is_some() {
                timeout.update_node_count(1);
            }
            if self.components.find_cache(start.0) == self.components.find_cache(goal.0) {
                timeout.notify_solved();
            }
        }
    }

    pub fn insert_r<R>(&mut self, c: C, radius: R) -> Option<PrmNodeId<Self>>
    where
        V: Validate<C>,
        NN: RangeNearestNeighborsMap<C, Node, Distance = R>,
        C: Clone,
    {
        if !self.valid.is_valid_configuration(&c) {
            return None;
        }

        let i = self.edges.len();
        self.edges.push(Vec::new());
        let new_component = self.components.create();
        for n in self
            .nn
            .nearest_within_r(&c, radius)
            .map(|&Node(n)| n)
            .filter(|&n| self.valid.is_valid_transition(&c, &self.configurations[n]))
        {
            self.components.unify(new_component, n);
            // assume bidirectionality
            self.edges[i].push(n);
            self.edges[n].push(i);
        }

        self.nn.insert(c.clone(), Node(i));
        self.configurations.push(c);
        debug_assert_eq!(
            self.edges.len(),
            self.configurations.len(),
            "configuration and edge buffers must have equal length"
        );
        Some(PrmNodeId(i, PhantomData))
    }

    pub fn configuration(&self, id: PrmNodeId<Self>) -> Option<&C> {
        self.configurations.get(id.0)
    }

    /// Compute a path between `start` and `end`.
    ///
    /// # Panics
    ///
    /// This function may panic if `start` or `end` point to nodes which do not exist in `self`.
    pub fn path<M, D>(
        &self,
        start: PrmNodeId<Self>,
        end: PrmNodeId<Self>,
        cost: &M,
    ) -> Option<Vec<PrmNodeId<Self>>>
    where
        M: Metric<C, Distance = D>,
        D: Add + Zero + Ord + Clone,
    {
        // use A*
        assert!(
            (0..self.configurations.len()).contains(&start.0),
            "invalid start configuration ID"
        );
        assert!(
            (0..self.configurations.len()).contains(&end.0),
            "invalid end configuration ID"
        );

        if self.components.find(start.0) != self.components.find(end.0) {
            // different components - no solution exists
            return None;
        }

        // open may include duplicate entries if we find alternate paths to open nodes
        let mut open = BinaryHeap::new();
        let mut parent = vec![0; self.configurations.len()];
        let mut g_score: Vec<_> = iter::repeat_with(|| None)
            .take(self.configurations.len())
            .collect();
        g_score[end.0] = Some(D::zero());

        // plan from goal to start to save a reversal
        let end_c = &self.configurations[end.0];

        open.push(Open {
            node: end.0,
            f_score: D::zero(),
        });

        while let Some(Open { node, .. }) = open.pop() {
            if node == start.0 {
                // done
                let mut traj = vec![PrmNodeId(start.0, PhantomData)];
                let mut n = start.0;
                while n != end.0 {
                    n = parent[n];
                    traj.push(PrmNodeId(n, PhantomData));
                }
                return Some(traj);
            }
            let nc = &self.configurations[node];

            for &neighbor in &self.edges[node] {
                let nbr_c = &self.configurations[neighbor];
                let new_g_score = cost.distance(nbr_c, nc)
                    + g_score[node]
                        .clone()
                        .expect("nodes in open set must have extant g-score");
                if g_score[neighbor]
                    .as_ref()
                    .map_or(true, |d| &new_g_score < d)
                {
                    // found a shorter path to neighbor
                    parent[neighbor] = node;
                    g_score[neighbor] = Some(new_g_score.clone());
                    open.push(Open {
                        node: neighbor,
                        f_score: new_g_score + cost.distance(nbr_c, end_c),
                    });
                }
            }
        }

        unreachable!("if start and goal are in same connected component, A* must find a path");
    }
}

impl SetForest {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            parents: Vec::new(),
            sizes: Vec::new(),
        }
    }

    pub fn create(&mut self) -> usize {
        let i = self.parents.len();
        self.parents.push(i);
        self.sizes.push(1);
        i
    }

    /// Find the representative node ID for the graph component containing `node`,
    /// internally updating the disjoint-set data structure.
    /// TODO make `components` mutexed so this function can be called with `&self`?
    /// Alternately, use atomic operations?
    fn find_cache(&mut self, mut x: usize) -> usize {
        let root = self.find(x);

        // downward pass - simplify path
        while self.parents[x] != root {
            let parent = self.parents[x];
            self.parents[x] = root;
            x = parent;
        }

        root
    }

    /// Find the representative node ID for the graph component containing `node`,
    /// without updating the cache.
    fn find(&self, x: usize) -> usize {
        let mut root = x;

        // upward pass - find root
        while root != self.parents[root] {
            root = self.parents[root];
        }
        root
    }

    /// Force the sets containing `a` and `b` to become unified into one set.
    /// Returns the ID of the newly created set.
    pub fn unify(&mut self, a: usize, b: usize) -> usize {
        let mut a_root = self.find_cache(a);
        let mut b_root = self.find_cache(b);

        if a_root == b_root {
            return a_root;
        }

        if self.sizes[a_root] < self.sizes[b_root] {
            swap(&mut a_root, &mut b_root);
        }
        self.parents[b_root] = a_root;
        self.sizes[a_root] += self.sizes[b_root];

        a_root
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Open<D> {
    f_score: D,
    node: usize,
}

impl<T> Clone for PrmNodeId<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for PrmNodeId<T> {}

impl<T> Debug for PrmNodeId<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

impl<T> Hash for PrmNodeId<T> {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.0.hash(state);
    }
}

impl<T> PartialEq for PrmNodeId<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T> Eq for PrmNodeId<T> {}

#[cfg(test)]
mod tests {
    use crate::{
        float::Real,
        metric::SquaredEuclidean,
        nn::KdTreeMap,
        sample::Everywhere,
        space::RealVector,
        time::{LimitNodes, Solved},
        AlwaysValid, Metric,
    };
    use alloc::vec::Vec;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    use super::Prm;

    #[test]
    fn prm2d() {
        let r = Real::new(0.05);
        let mut prm: Prm<RealVector<2, f32>, _, _> =
            Prm::new(KdTreeMap::new(SquaredEuclidean), &AlwaysValid);
        let start = prm
            .insert_r(RealVector::from_floats([0.0, 0.0]), r)
            .unwrap();
        let end = prm
            .insert_r(RealVector::from_floats([1.0, 1.0]), r)
            .unwrap();
        prm.grow_r(
            r,
            &mut LimitNodes::new(50),
            &Everywhere,
            &mut ChaCha20Rng::seed_from_u64(2707),
        );
        let path = prm
            .path(start, end, &SquaredEuclidean)
            .expect("unable to find path");

        #[cfg(feature = "std")]
        println!("{path:?}");

        let traj = path
            .into_iter()
            .map(|id| {
                prm.configuration(id)
                    .expect("pathfinding must yield valid nodes")
            })
            .collect::<Vec<_>>();

        #[cfg(feature = "std")]
        println!("{traj:?}");

        assert!(
            traj.windows(2)
                .all(|a| SquaredEuclidean.distance(a[0], a[1]) <= r),
            "all transitions must be within growth radius"
        );
    }

    #[test]
    fn prm_solved() {
        let r = Real::new(0.05);
        let mut prm: Prm<RealVector<2, f32>, _, _> =
            Prm::new(KdTreeMap::new(SquaredEuclidean), &AlwaysValid);
        let start = prm
            .insert_r(RealVector::from_floats([0.0, 0.0]), r)
            .unwrap();
        let end = prm
            .insert_r(RealVector::from_floats([1.0, 1.0]), r)
            .unwrap();
        prm.grow_r_solve(
            r,
            &mut Solved::new(),
            &Everywhere,
            &mut ChaCha20Rng::seed_from_u64(2707),
            start,
            end,
        );
        let path = prm
            .path(start, end, &SquaredEuclidean)
            .expect("unable to find path");

        let traj = path
            .into_iter()
            .map(|id| {
                prm.configuration(id)
                    .expect("pathfinding must yield valid nodes")
            })
            .collect::<Vec<_>>();

        #[cfg(feature = "std")]
        println!("{traj:?}");

        assert!(
            traj.windows(2)
                .all(|a| SquaredEuclidean.distance(a[0], a[1]) <= r),
            "all transitions must be within growth radius"
        );
    }
}
