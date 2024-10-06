use alloc::collections::BinaryHeap;
use core::{fmt::Debug, hash::Hash, iter, mem::swap, ops::Add};

use alloc::vec::Vec;
use num_traits::Zero;

use crate::{time::Timeout, Metric, RangeNearestNeighborsMap, Sample};

/// Probabilistic roadmaps; a class of anytime geometric motion planner.
///
/// # Generic parameters
///
/// - `C`: The configurations of the robot.
/// - `NN`: The nearest-neighbor data structure to use. To be useful, `NN` should implement
///   [`RangeNearestNeighborsMap`].
/// - `V`: The state validator. `V` should implement [`EdgeValidate`].
///
/// # Citation
///
/// ```bibtex
/// @article{kavraki1996probabilistic,
///   title={Probabilistic roadmaps for path planning in high-dimensional configuration spaces},
///   author={Kavraki, Lydia E and Svestka, Petr and Latombe, J-C and Overmars, Mark H},
///   journal={IEEE transactions on Robotics and Automation},
///   volume={12},
///   number={4},
///   pages={566--580},
///   year={1996},
///   publisher={IEEE}
/// }
/// ```
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

use super::EdgeValidate;

#[expect(clippy::module_name_repetitions)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The ID for a node in a [`Prm`].
pub struct PrmNodeId(usize);

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

    /// Grow this PRM until `timeout` runs out, connecting all nodes within `radius` of one another.
    /// Generated nodes will only be sampled from `sample` using `rng` as the source of randomness.
    pub fn grow_r<R, TC, S, RNG>(&mut self, radius: R, timeout: &mut TC, sample: &S, rng: &mut RNG)
    where
        V: EdgeValidate<C>,
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
        start: PrmNodeId,
        goal: PrmNodeId,
    ) where
        V: EdgeValidate<C>,
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

    /// Insert a configuration into the graph, connecting it to all other nodes in the graph within
    /// a distance of `radius`. Returns the ID of the node it created, or `None` if the given
    /// configuration was invalid.
    pub fn insert_r<R>(&mut self, c: C, radius: R) -> Option<PrmNodeId>
    where
        V: EdgeValidate<C>,
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
        Some(PrmNodeId(i))
    }

    /// Get the configuration in the graph corresponding to the given node ID.
    ///
    /// Returns `None` if no such node with the given ID exists.
    pub fn configuration(&self, id: PrmNodeId) -> Option<&C> {
        self.configurations.get(id.0)
    }

    /// Compute a path between `start` and `end`.
    ///
    /// # Panics
    ///
    /// This function may panic if `start` or `end` point to nodes which do not exist in `self`.
    pub fn path<M, D>(&self, start: PrmNodeId, end: PrmNodeId, cost: &M) -> Option<Vec<PrmNodeId>>
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
                let mut traj = vec![PrmNodeId(start.0)];
                let mut n = start.0;
                while n != end.0 {
                    n = parent[n];
                    traj.push(PrmNodeId(n));
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

#[cfg(test)]
mod tests {
    use crate::{
        float::{r32, R32},
        metric::SquaredEuclidean,
        nn::KdTreeMap,
        sample::Rectangle,
        space::Vector,
        time::{LimitNodes, Solved},
        AlwaysValid, Metric,
    };
    use alloc::vec::Vec;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    use super::Prm;

    #[test]
    fn prm2d() {
        let r = r32(0.05);
        let mut prm: Prm<Vector<2, R32>, _, _> =
            Prm::new(KdTreeMap::new(SquaredEuclidean), &AlwaysValid);
        let start = prm.insert_r(Vector::new([0.0, 0.0].map(r32)), r).unwrap();
        let end = prm.insert_r(Vector::new([1.0, 1.0].map(r32)), r).unwrap();
        prm.grow_r(
            r,
            &mut LimitNodes::new(50),
            &Rectangle {
                min: Vector::new([0.0; 2].map(r32)),
                max: Vector::new([1.0; 2].map(r32)),
            },
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
        let r = r32(0.05);
        let mut prm: Prm<Vector<2, R32>, _, _> =
            Prm::new(KdTreeMap::new(SquaredEuclidean), &AlwaysValid);
        let start = prm.insert_r(Vector::new([0.0, 0.0].map(r32)), r).unwrap();
        let end = prm.insert_r(Vector::new([1.0, 1.0].map(r32)), r).unwrap();
        prm.grow_r_solve(
            r,
            &mut Solved::new(),
            &Rectangle {
                min: Vector::new([0.0; 2].map(r32)),
                max: Vector::new([1.0; 2].map(r32)),
            },
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