//! RRT-connect.

use alloc::vec::Vec;

use crate::{
    nn::NearestNeighborsMap, sample::Sample, space::Interpolate, time::Timeout, valid::GeoValidate,
};

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
    NN: NearestNeighborsMap<C, usize> + Clone,
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
        NN: Clone + NearestNeighborsMap<C, usize>,
        C: Clone,
    {
        let mut nn1 = nn.clone();
        nn.insert(start.clone(), 0);
        nn1.insert(goal.clone(), 0);
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
        NN: NearestNeighborsMap<C, usize>,
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

        'a: while !timeout.is_over() {
            // begin with RRTC extend procedure
            timeout.update_sample_count(1);
            let q_rand = space_sampler.sample(rng);
            let t = &mut self.trees[self.next as usize];
            let (q_near, &q_near_id) = t.nn.nearest(&q_rand).expect("NN must be nonempty");
            let q_new = q_near
                .interpolate(&q_rand, radius.clone())
                .next()
                .unwrap_or_else(|| q_rand.clone());

            if !self.valid.is_valid_configuration(&q_new)
                || !self.valid.is_valid_transition(q_near, &q_new)
            {
                continue;
            }
            timeout.update_node_count(1);
            let q_new_id = t.configurations.len();
            t.configurations.push(q_new.clone());
            t.parents.push(q_near_id);
            t.nn.insert(q_new.clone(), q_new_id);

            self.next ^= 1;

            // attempt to connect the two trees with this newly created node
            let tb = &mut self.trees[self.next as usize];
            let (q_old_connect_r, &id) = tb.nn.nearest(&q_new).expect("NN must be nonempty");
            let mut q_old_connect_id = id;
            let mut q_old_connect = q_old_connect_r.clone();

            for q_new_connect in q_old_connect.clone().interpolate(&q_new, radius.clone()) {
                if !self.valid.is_valid_configuration(&q_new_connect)
                    || !self
                        .valid
                        .is_valid_transition(&q_old_connect, &q_new_connect)
                {
                    continue 'a;
                }

                let q_new_connect_id = tb.configurations.len();

                tb.configurations.push(q_new_connect.clone());
                tb.parents.push(q_old_connect_id);
                tb.nn.insert(q_new_connect.clone(), q_new_connect_id);

                q_old_connect = q_new_connect;
                q_old_connect_id = q_new_connect_id;
            }

            if self.valid.is_valid_transition(&q_old_connect, &q_new) {
                // connected!
                timeout.notify_solved();
                self.cross_edges.push(if self.next == 1 {
                    (q_new_id, q_old_connect_id)
                } else {
                    (q_old_connect_id, q_new_id)
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
