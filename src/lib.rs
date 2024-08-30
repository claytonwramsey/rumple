#![warn(clippy::pedantic, clippy::nursery)]

use rand::Rng;

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
}

pub trait NearestNeighborsMap<K, V> {
    fn empty() -> Self;
    fn insert(&mut self, key: K, value: V);
    fn nearest<'a>(&'a self, key: &K) -> Option<&'a V>;
}
