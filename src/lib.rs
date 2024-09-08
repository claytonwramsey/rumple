#![warn(clippy::pedantic, clippy::nursery)]

use rand::Rng;

pub mod nn;
pub mod rrt;

pub trait StateSpace {
    type Configuration;

    fn is_valid_configuration(&self, c: &Self::Configuration) -> bool;
    fn is_valid_transition(&self, start: &Self::Configuration, end: &Self::Configuration) -> bool;
}

pub trait SampleSpace: StateSpace {
    fn sample<R: Rng>(&self, rng: &mut R) -> Self::Configuration;
}

pub trait TimeoutCondition {
    fn is_over(&self) -> bool;

    fn update_sample_count(&mut self, _n: usize) {}
}

pub trait Problem<SS: StateSpace> {
    fn start(&self) -> &SS::Configuration;
    fn sample_goal(&self) -> SS::Configuration;
}

pub trait MetricSpace: StateSpace {
    type Distance: Ord;

    fn distance(&self, c1: &Self::Configuration, c2: &Self::Configuration) -> Self::Distance;
    fn is_zero(&self, dist: &Self::Distance) -> bool;
}

pub trait NearestNeighborsMap<K, V, M>
where
    M: MetricSpace<Configuration = K>,
{
    fn empty() -> Self;
    /// Insert a key into the map.
    /// `metric` must be consistent across calls to this function.
    fn insert(&mut self, key: K, value: V, metric: &M);
    /// Get the nearest element of the space to this key.
    /// `metric` must be consistent across calls to this function.
    fn nearest<'q>(&'q self, key: &K, metric: &M) -> Option<(&'q K, &'q V)>;
}
