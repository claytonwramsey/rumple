use crate::{ConfigurationSpace, NearestNeighborsMap, Sample, TimeoutCondition};

pub trait Space: ConfigurationSpace {
    type Distance;

    fn grow_toward(
        &self,
        start: &Self::Configuration,
        end: &Self::Configuration,
        radius: Self::Distance,
    ) -> Self::Configuration;
}

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

/// Secret internal struct that can be used as a value in a nearest-neighbors map.
struct Node(usize);

impl<C, NN> Rrt<C, NN> {
    pub fn new<M>(root: C, mut nn: NN) -> Self
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
    fn grow_help<SS, SP, G, TC: TimeoutCondition, TG, RNG, R>(
        &mut self,
        space: &SS,
        space_sampler: &SP,
        goal: &G,
        radius: R,
        timeout: &TC,
        target_goal_distn: &TG,
        rng: &mut RNG,
    ) -> Option<usize>
    where
        SS: Space<Configuration = C, Distance = R>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone,
        TG: Sample<bool, RNG>,
    {
        while !timeout.is_over() {
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
            let end_cfg = space.grow_toward(start_cfg, &target, radius.clone());
            if !space.is_valid_transition(start_cfg, &end_cfg) {
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
    pub fn grow_toward<SS, SP, G, TC, TG, R, RNG>(
        &mut self,
        space: &SS,
        space_sampler: &SP,
        goal: &G,
        radius: R,
        timeout: &TC,
        target_goal_distn: &TG,
        rng: &mut RNG,
    ) -> Option<Vec<C>>
    where
        SS: Space<Configuration = C, Distance = R>,
        SP: Sample<C, RNG>,
        G: Sample<C, RNG>,
        TG: Sample<bool, RNG>,
        TC: TimeoutCondition,
        NN: NearestNeighborsMap<C, Node>,
        R: Clone,
        C: Clone,
    {
        let mut id = self.grow_help(
            space,
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
