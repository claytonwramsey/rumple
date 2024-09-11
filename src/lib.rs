#![warn(clippy::pedantic, clippy::nursery)]

pub mod nn;
pub mod rrt;

pub trait ConfigurationSpace {
    type Configuration;
    fn is_valid_configuration(&self, c: &Self::Configuration) -> bool;
    fn is_valid_transition(&self, start: &Self::Configuration, end: &Self::Configuration) -> bool;
}

pub trait Sample<C, RNG> {
    fn sample(&self, rng: &mut RNG) -> C;
}

pub trait TimeoutCondition {
    fn is_over(&self) -> bool;

    fn update_sample_count(&mut self, _n: usize) {}
}

pub trait Problem {
    type Configuration;

    fn start(&self) -> &Self::Configuration;
    fn sample_goal(&self) -> Self::Configuration;
}

pub trait Metric<C> {
    type Distance: Ord;

    fn distance(&self, c1: &C, c2: &C) -> Self::Distance;
    fn is_zero(&self, dist: &Self::Distance) -> bool;
}

pub trait NearestNeighborsMap<K, V> {
    /// Insert a key into the map.
    fn insert(&mut self, key: K, value: V);
    /// Get the nearest element of the space to this key.
    fn nearest<'q>(&'q self, key: &K) -> Option<(&'q K, &'q V)>;
}
