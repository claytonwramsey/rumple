use rand::{distributions::Uniform, prelude::Distribution, Rng};

use crate::{MetricSpace, NearestNeighborsMap, SampleSpace, TimeoutCondition};

pub trait Space: SampleSpace + MetricSpace {
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
    /// parent_ids[0] is ignorable
    parent_ids: Vec<usize>,
    /// The nearest neighbors lookup.
    nn: NN,
}

impl<C, NN> Rrt<C, NN>
where
    NN: NearestNeighborsMap<C, usize>,
    C: Clone,
{
    pub fn new(root: C) -> Self {
        let mut nn = NN::empty();
        nn.insert(root.clone(), 0);
        Rrt {
            configurations: vec![root],
            parent_ids: vec![0],
            nn,
        }
    }

    fn grow_help<SS, G, TC: TimeoutCondition, RNG: Rng, R>(
        &mut self,
        space: &SS,
        goal: &G,
        radius: R,
        timeout: &TC,
        target_goal_probability: f32,
        rng: &mut RNG,
    ) -> Option<usize>
    where
        G: SampleSpace<Configuration = C>,
        SS: Space<Configuration = C, Distance = R>,
        R: Clone,
    {
        let distn = Uniform::new(0.0, 1.0);
        while !timeout.is_over() {
            let target = if distn.sample(rng) < target_goal_probability {
                goal.sample(rng)
            } else {
                space.sample(rng)
            };
            let start_id = *self
                .nn
                .nearest(&target)
                .expect("NN must always have elements");
            let start_cfg = &self.configurations[start_id];
            let end_cfg = space.grow_toward(start_cfg, &target, radius.clone());
            if !space.is_valid_transition(&start_cfg, &end_cfg) {
                continue;
            }
            let new_id = self.configurations.len();
            self.configurations.push(end_cfg.clone());
            let end_ref = self
                .configurations
                .last()
                .expect("configurations must be nonempty");
            self.parent_ids.push(start_id);
            self.nn.insert(end_cfg, new_id);
            if goal.is_valid_configuration(end_ref) {
                return Some(new_id);
            }
        }

        None
    }

    pub fn grow_toward<SS, G, TC: TimeoutCondition, RNG: Rng, R>(
        &mut self,
        space: &SS,
        goal: &G,
        radius: R,
        timeout: &TC,
        target_goal_probability: f32,
        rng: &mut RNG,
    ) -> Option<Vec<C>>
    where
        G: SampleSpace<Configuration = C>,
        SS: Space<Configuration = C, Distance = R>,
        R: Clone,
    {
        let mut id = self.grow_help(space, goal, radius, timeout, target_goal_probability, rng)?;
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
