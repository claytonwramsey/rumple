use crate::{time::Timeout, Interpolate, NearestNeighborsMap, Sample, Validate};
use alloc::vec::Vec;

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

#[allow(clippy::too_many_arguments)]
pub fn rrt<C, NN, V, SP, G, TC, TG, R, RNG>(
    start: C,
    valid: &V,
    space_sampler: &SP,
    goal: &G,
    radius: R,
    timeout: &mut TC,
    target_goal_distn: &TG,
    rng: &mut RNG,
) -> Option<Vec<C>>
where
    NN: NearestNeighborsMap<C, Node> + Default,
    V: Validate<C>,
    SP: Sample<C, RNG>,
    G: Sample<C, RNG>,
    R: Clone,
    C: Clone + Interpolate<Distance = R>,
    TC: Timeout,
    TG: Sample<bool, RNG>,
{
    let mut rrt = Rrt::new(start, NN::default(), valid);
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
    pub fn new(root: C, mut nn: NN, valid: &'a V) -> Self
    where
        NN: NearestNeighborsMap<C, Node>,
        C: Clone,
    {
        nn.insert(root.clone(), Node(0));
        Self {
            configurations: vec![root],
            parent_ids: vec![usize::MAX],
            nn,
            valid,
        }
    }

    #[allow(clippy::too_many_arguments)]
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
        V: Validate<C>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone + Interpolate<Distance = R>,
        TG: Sample<bool, RNG>,
    {
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
                Ok(c) => (true, c),
                Err(c) => (false, c),
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

    #[allow(clippy::too_many_arguments)]
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
        V: Validate<C>,
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

    pub fn num_nodes(&self) -> usize {
        self.configurations.len()
    }
}
