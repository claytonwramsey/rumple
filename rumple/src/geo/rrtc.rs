//! RRT-connect.

use alloc::vec::Vec;

use crate::{
    nn::NearestNeighborsMap, sample::Sample, space::Interpolate, time::Timeout, valid::GeoValidate,
};

mod private {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct Node(pub usize);
}
use private::Node;

#[derive(Clone, Debug)]
/// A planner that combines two [`Rrt`]s growing toward each other.
///
/// # Citation
///
/// ```bibtex
/// @inproceedings{kuffner2000rrt,
///  title={RRT-connect: An efficient approach to single-query path planning},
///  author={Kuffner, James J and LaValle, Steven M},
///  booktitle={Proceedings 2000 ICRA. Millennium Conference. IEEE International Conference on Robotics and Automation. Symposia Proceedings (Cat. No. 00CH37065)},
///  volume={2},
///  pages={995--1001},
///  year={2000},
///  organization={IEEE}
/// }
/// ```
pub struct RrtConnect<'a, C, NN, V> {
    trees: [HalfTree<C, NN>; 2],
    cross_edges: Vec<(usize, usize)>,
    valid: &'a V,
    next: u8,
}

#[derive(Clone, Debug)]
struct HalfTree<C, NN> {
    configurations: Vec<C>,
    parents: Vec<usize>,
    nn: NN,
}

#[expect(clippy::too_many_arguments)]
/// Plan between two configurations using an [`RrtConnect`].
///
/// # Parameters
///
/// - `start`: The start configuration.
/// - `valid`: The state validator.
/// - `space_sampler`: A sampler for states in the configuration space.
/// - `goal`: The goal state or sampler for goal states.
/// - `radius`: The radius by which to expand the RRT.
/// - `timeout`: The timeout condition. The planning algorithm will continue until `timeout` is
///   over.
/// - `target_goal_distn`: A sampler which returns `true` with some probability; every time it
///   returns `true`, the RRT grows toward the goal instead of to fill the space.
/// - `rng`: The source of randomness.
pub fn rrt_connect<C, NN, V, SP, TC, R, RNG>(
    start: C,
    goal: C,
    nn: NN,
    valid: &V,
    space_sampler: &SP,
    radius: R,
    timeout: &mut TC,
    rng: &mut RNG,
) -> Option<Vec<C>>
where
    NN: NearestNeighborsMap<C, Node> + Clone,
    V: GeoValidate<C>,
    SP: Sample<C, RNG>,
    R: Clone,
    C: Clone + Interpolate<Distance = R>,
    TC: Timeout,
{
    let mut trees = RrtConnect::new(nn, start, goal, valid);
    trees.grow(space_sampler, radius, timeout, rng)
}

impl<'a, C, NN, V> RrtConnect<'a, C, NN, V> {
    pub fn new(mut nn: NN, start: C, goal: C, valid: &'a V) -> Self
    where
        NN: Clone + NearestNeighborsMap<C, Node>,
        C: Clone,
    {
        let mut nn1 = nn.clone();
        nn.insert(start.clone(), Node(0));
        nn1.insert(goal.clone(), Node(0));
        Self {
            trees: [
                HalfTree {
                    configurations: vec![start],
                    parents: vec![0],
                    nn,
                },
                HalfTree {
                    configurations: vec![goal],
                    parents: vec![0],
                    nn: nn1,
                },
            ],
            cross_edges: Vec::new(),
            valid,
            next: 0,
        }
    }

    #[expect(clippy::missing_panics_doc)]
    pub fn grow<SP, TC, R, RNG>(
        &mut self,
        space_sampler: &SP,
        radius: R,
        timeout: &mut TC,
        rng: &mut RNG,
    ) -> Option<Vec<C>>
    where
        V: GeoValidate<C>,
        SP: Sample<C, RNG>,
        TC: Timeout,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone + Interpolate<Distance = R>,
    {
        if !self
            .trees
            .iter()
            .all(|t| self.valid.is_valid_configuration(&t.configurations[0]))
        {
            // invalid start/goal
            return None;
        }

        let mut res = None;

        while !timeout.is_over() {
            // begin with RRTC extend procedure
            timeout.update_sample_count(1);
            let target = space_sampler.sample(rng);
            let t = &mut self.trees[self.next as usize];
            let (start_cfg, &Node(start_id)) = t.nn.nearest(&target).expect("NN must be nonempty");
            let end_cfg = match start_cfg.interpolate(&target, radius.clone()) {
                Ok(c) | Err(c) => c,
            };

            if !self.valid.is_valid_configuration(&end_cfg)
                || !self.valid.is_valid_transition(start_cfg, &end_cfg)
            {
                continue;
            }
            timeout.update_node_count(1);
            let new_id = t.configurations.len();
            t.configurations.push(end_cfg.clone());
            t.parents.push(start_id);
            t.nn.insert(end_cfg.clone(), Node(new_id));

            self.next ^= 1;
            let tb = &mut self.trees[self.next as usize];
            let (mut start_cfg, Node(mut start_id)) =
                tb.nn.nearest(&end_cfg).expect("NN must be nonempty");

            let target = end_cfg;

            loop {
                let (end_cfg, reached) = match start_cfg.interpolate(&target, radius.clone()) {
                    Ok(c) => (c, false),
                    Err(c) => (c, true),
                };

                if !self.valid.is_valid_configuration(&end_cfg)
                    || !self.valid.is_valid_transition(start_cfg, &end_cfg)
                {
                    break;
                }

                if reached {
                    // connected!
                    timeout.notify_solved();
                    self.cross_edges.push(if self.next == 1 {
                        (new_id, start_id)
                    } else {
                        (start_id, new_id)
                    });

                    let mut traj = Vec::new();
                    let (mut p0, mut p1) = self.cross_edges.last().unwrap();

                    // extract first half of path
                    while p0 != 0 {
                        traj.push(self.trees[0].configurations[p0].clone());
                        p0 = self.trees[0].parents[p0];
                    }
                    traj.push(self.trees[0].configurations[0].clone());
                    traj.reverse();

                    // extract second half of path
                    while p1 != 0 {
                        traj.push(self.trees[1].configurations[p1].clone());
                        p1 = self.trees[1].parents[p1];
                    }
                    traj.push(self.trees[1].configurations[0].clone());

                    res = Some(traj);
                    break;
                }
                let ext_id = tb.configurations.len();

                tb.configurations.push(end_cfg.clone());
                start_cfg = tb.configurations.last().unwrap();
                tb.parents.push(start_id);
                start_id = ext_id;
                tb.nn.insert(end_cfg.clone(), Node(ext_id));
            }
        }

        res
    }

    pub fn num_nodes(&self) -> usize {
        self.trees.iter().map(|t| t.configurations.len()).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        metric::{Metric, SquaredEuclidean},
        nn::KdTreeMap,
        sample::Rectangle,
        space::Vector,
        time::Solved,
        valid::AlwaysValid,
    };
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    #[test]
    fn rrtc_plane2d() {
        let mut rrtc = RrtConnect::new(
            KdTreeMap::new(SquaredEuclidean),
            Vector::new([0.0, 0.0]),
            Vector::new([1.0, 1.0]),
            &AlwaysValid,
        );
        let radius = 0.05;
        let res = rrtc
            .grow(
                &Rectangle {
                    min: Vector::new([0.0, 1.1]),
                    max: Vector::new([0.0, 1.1]),
                },
                radius,
                &mut Solved::new(),
                &mut ChaCha20Rng::seed_from_u64(2707),
            )
            .unwrap();

        #[cfg(feature = "std")]
        {
            println!("Created {} nodes", rrtc.num_nodes());
            println!("{res:?}");
        }
        assert!(
            res.windows(2)
                .all(|a| SquaredEuclidean.distance(&a[0], &a[1]) <= radius + 1e-5),
            "all transitions must be within growth radius"
        );
    }
}
