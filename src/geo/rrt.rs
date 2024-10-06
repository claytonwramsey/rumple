use crate::{
    space::Interpolate, time::Timeout, valid::GeoValidate, valid::Validate, NearestNeighborsMap,
    Sample,
};
use alloc::vec::Vec;

/// A rapidly-exploring random tree: a geometric single-query sampling-based motion planner.
///
/// This is a simple implementation; you may want to alternately use [`RrtConnect`].
///
/// # Generic parameters
///
/// - `C` should be the configuration of a robot.
/// - `NN` should be the nearest neighbors data structure, which can use `C` as a key and implement
///   `NearestNeighborsMap`.
/// - `V` should be a state validator; it must implement [`EdgeValidate`] for `C`.
///
/// # Citation
///
/// ```bibtex
/// @article{lavalle1998rapidly,
///   title={Rapidly-exploring random trees: A new tool for path planning},
///   author={LaValle, Steven},
///   journal={Research Report 9811},
///   year={1998},
///   publisher={Department of Computer Science, Iowa State University}
/// }
/// ```
pub struct Rrt<'a, C, NN, V> {
    /// buffer of saved configurations
    /// configurations[0] is the root
    configurations: Vec<C>,
    /// ids for each configuration
    /// `parent_ids[0]` is ignorable
    parent_ids: Vec<usize>,
    /// The nearest neighbors lookup.
    nn: NN,
    /// The state validator.
    valid: &'a V,
}

/// Workaround module to avoid exposing implementation details of `Node` to consumers.
mod private {
    pub struct Node(pub usize);
}
use private::Node;

#[expect(clippy::too_many_arguments)]
/// Plan between two configurations using an [`Rrt`].
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
pub fn rrt<C, NN, V, SP, G, TC, TG, R, RNG>(
    start: C,
    nn: NN,
    valid: &V,
    space_sampler: &SP,
    goal: &G,
    radius: R,
    timeout: &mut TC,
    target_goal_distn: &TG,
    rng: &mut RNG,
) -> Option<Vec<C>>
where
    NN: NearestNeighborsMap<C, Node>,
    V: GeoValidate<C>,
    SP: Sample<C, RNG>,
    G: Sample<C, RNG>,
    R: Clone,
    C: Clone + Interpolate<Distance = R>,
    TC: Timeout,
    TG: Sample<bool, RNG>,
{
    let mut rrt = Rrt::new(start, nn, valid);
    let mut id = rrt.grow_help(space_sampler, goal, radius, timeout, target_goal_distn, rng)?;
    let mut traj = Vec::new();
    while id != 0 {
        // can safely remove the configuration since we are deleting the rrt shortly
        traj.push(rrt.configurations.swap_remove(id));
        id = rrt.parent_ids[id];
    }
    traj.push(rrt.configurations.swap_remove(0));
    traj.reverse();
    Some(traj)
}

impl<'a, C, NN, V> Rrt<'a, C, NN, V> {
    /// Construct a new RRT rooted at `root`, using `nn` as its nearest-neighbor structure and
    /// `valid` as its state validator.
    pub fn new(root: C, mut nn: NN, valid: &'a V) -> Self
    where
        NN: NearestNeighborsMap<C, Node>,
        C: Clone,
        V: Validate<C>,
    {
        nn.insert(root.clone(), Node(0));
        Self {
            configurations: vec![root],
            parent_ids: vec![usize::MAX],
            nn,
            valid,
        }
    }

    fn grow_help<SP, G, TC: Timeout, TG, RNG, R>(
        &mut self,
        space_sampler: &SP,
        goal: &G,
        radius: R,
        timeout: &mut TC,
        target_goal_distn: &TG,
        rng: &mut RNG,
    ) -> Option<usize>
    where
        V: GeoValidate<C>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone + Interpolate<Distance = R>,
        TG: Sample<bool, RNG>,
    {
        if !self.valid.is_valid_configuration(&self.configurations[0]) {
            return None; // invalid configuration
        }
        let mut soln = None;
        while !timeout.is_over() {
            timeout.update_sample_count(1);
            let sample_goal = target_goal_distn.sample(rng);
            let target = if sample_goal {
                goal.sample(rng)
            } else {
                space_sampler.sample(rng)
            };
            let (start_cfg, &Node(start_id)) = self
                .nn
                .nearest(&target)
                .expect("NN must always have elements");
            let (reached, end_cfg) = match start_cfg.interpolate(&target, radius.clone()) {
                Ok(c) => (false, c),
                Err(c) => (true, c),
            };
            if !self.valid.is_valid_transition(start_cfg, &end_cfg) {
                continue;
            }
            timeout.update_node_count(1);
            let new_id = self.configurations.len();
            self.configurations.push(end_cfg.clone());
            self.parent_ids.push(start_id);
            debug_assert_eq!(
                self.configurations.len(),
                self.parent_ids.len(),
                "number of configurations and parents must be equal"
            );
            self.nn.insert(end_cfg, Node(new_id));
            if sample_goal && reached {
                timeout.notify_solved();
                soln = Some(new_id);
            }
        }

        soln
    }

    /// Grow this RRT toward the provided goal `goal`.
    ///
    /// # Parameters
    ///
    /// - `space_sampler`: A sampler for states in the configuration space.
    /// - `goal`: The goal state or sampler for goal states.
    /// - `radius`: The radius by which to expand the RRT.
    /// - `timeout`: The timeout condition. The planning algorithm will continue until `timeout` is
    ///   over.
    /// - `target_goal_distn`: A sampler which returns `true` with some probability; every time it
    ///   returns `true`, the RRT grows toward the goal instead of to fill the space.
    /// - `rng`: The source of randomness.
    pub fn grow_toward<SP, G, TC, TG, R, RNG>(
        &mut self,
        space_sampler: &SP,
        goal: &G,
        radius: R,
        timeout: &mut TC,
        target_goal_distn: &TG,
        rng: &mut RNG,
    ) -> Option<Vec<C>>
    where
        V: GeoValidate<C>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        TG: Sample<bool, RNG>,
        TC: Timeout,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone + Interpolate<Distance = R>,
    {
        let mut id =
            self.grow_help(space_sampler, goal, radius, timeout, target_goal_distn, rng)?;
        let mut traj = Vec::new();
        while id != 0 {
            traj.push(self.configurations[id].clone());
            id = self.parent_ids[id];
        }
        traj.push(self.configurations[0].clone());
        traj.reverse();
        Some(traj)
    }

    /// Get the number of total nodes in this tree.
    pub fn num_nodes(&self) -> usize {
        self.configurations.len()
    }
}

#[expect(clippy::module_name_repetitions)]
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
pub struct RrtConnect {}
