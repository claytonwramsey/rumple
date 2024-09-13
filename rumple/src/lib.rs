#![warn(clippy::pedantic, clippy::nursery)]

pub mod metric;
pub mod nn;
pub mod rrt;
pub mod sample;
pub mod space;
pub mod time;

pub trait Validate<C> {
    fn is_valid_configuration(&self, c: &C) -> bool;
    fn is_valid_transition(&self, start: &C, end: &C) -> bool;
}

pub struct AlwaysValid;

impl<C> Validate<C> for AlwaysValid {
    fn is_valid_configuration(&self, _: &C) -> bool {
        true
    }

    fn is_valid_transition(&self, _: &C, _: &C) -> bool {
        true
    }
}

pub trait Sample<C, RNG> {
    fn sample(&self, rng: &mut RNG) -> C;
}

pub trait Timeout {
    fn is_over(&self) -> bool;

    fn update_sample_count(&mut self, _n: usize) {}
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

pub trait Grow<C> {
    type Distance;

    fn grow_toward(&self, start: &C, end: &C, radius: Self::Distance) -> C;
}
