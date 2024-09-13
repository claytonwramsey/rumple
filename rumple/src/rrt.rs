use crate::{Grow, NearestNeighborsMap, Sample, Timeout, Validate};

pub struct Rrt<C, NN> {
    /// buffer of saved configurations
    /// configurations[0] is the root
    configurations: Vec<C>,
    /// ids for each configuration
    /// `parent_ids[0]` is ignorable
    parent_ids: Vec<usize>,
    /// The nearest neighbors lookup.
    nn: NN,
}

/// Workaround module to avoid exposing implementation details of `Node` to consumers.
mod private {
    pub struct Node(pub usize);
}
use private::Node;

impl<C, NN> Rrt<C, NN> {
    pub fn new(root: C, mut nn: NN) -> Self
    where
        NN: NearestNeighborsMap<C, Node>,
        C: Clone,
    {
        nn.insert(root.clone(), Node(0));
        Self {
            configurations: vec![root],
            parent_ids: vec![usize::MAX],
            nn,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn grow_help<GR, V, SP, G, TC: Timeout, TG, RNG, R>(
        &mut self,
        grow: &GR,
        valid: &V,
        space_sampler: &SP,
        goal: &G,
        radius: R,
        timeout: &mut TC,
        target_goal_distn: &TG,
        rng: &mut RNG,
    ) -> Option<usize>
    where
        GR: Grow<C, Distance = R>,
        V: Validate<C>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone,
        TG: Sample<bool, RNG>,
    {
        while !timeout.is_over() {
            timeout.update_sample_count(1);
            let sample_goal = target_goal_distn.sample(rng);
            let target = if sample_goal {
                println!("sample goal!");
                goal.sample(rng)
            } else {
                println!("do not sample goal!");
                space_sampler.sample(rng)
            };
            let (start_cfg, &Node(start_id)) = self
                .nn
                .nearest(&target)
                .expect("NN must always have elements");
            let end_cfg = grow.grow_toward(start_cfg, &target, radius.clone());
            if !valid.is_valid_transition(start_cfg, &end_cfg) {
                continue;
            }
            let new_id = self.configurations.len();
            self.configurations.push(end_cfg.clone());
            self.parent_ids.push(start_id);
            self.nn.insert(end_cfg, Node(new_id));
            if sample_goal {
                return Some(new_id);
            }
        }

        None
    }

    #[allow(clippy::too_many_arguments)]
    pub fn grow_toward<GR, V, SP, G, TC, TG, R, RNG>(
        &mut self,
        grow: &GR,
        valid: &V,
        space_sampler: &SP,
        goal: &G,
        radius: R,
        timeout: &mut TC,
        target_goal_distn: &TG,
        rng: &mut RNG,
    ) -> Option<Vec<C>>
    where
        GR: Grow<C, Distance = R>,
        V: Validate<C>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        TG: Sample<bool, RNG>,
        TC: Timeout,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone,
    {
        let mut id = self.grow_help(
            grow,
            valid,
            space_sampler,
            goal,
            radius,
            timeout,
            target_goal_distn,
            rng,
        )?;
        let mut traj = Vec::new();
        while id != 0 {
            traj.push(self.configurations[id].clone());
            id = self.parent_ids[id];
        }
        traj.push(self.configurations[0].clone());
        traj.reverse();
        Some(traj)
    }
}
