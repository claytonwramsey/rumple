#![cfg_attr(not(feature = "std"), no_std)]
#![warn(clippy::pedantic, clippy::nursery)]

use num_traits::Zero;

#[macro_use]
extern crate alloc;

pub mod env;
pub mod float;
pub mod geo;
pub mod metric;
pub mod nn;
pub mod sample;
pub mod space;
pub mod time;
pub mod valid;

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

pub trait Metric<C> {
    type Distance: Ord + Zero;

    fn distance(&self, c1: &C, c2: &C) -> Self::Distance;
}

pub trait NearestNeighborsMap<K, V> {
    /// Insert a key into the map.
    fn insert(&mut self, key: K, value: V);
    /// Get the nearest element of the space to this key.
    fn nearest<'q>(&'q self, key: &K) -> Option<(&'q K, &'q V)>;
}

pub trait RangeNearestNeighborsMap<K, V>: NearestNeighborsMap<K, V> {
    type Distance;

    type RangeNearest<'q>: Iterator<Item = &'q V>
    where
        V: 'q,
        Self: 'q;

    /// Get an iterator over all items in `self` within range `r` of
    fn nearest_within_r<'q>(&'q self, key: &K, r: Self::Distance) -> Self::RangeNearest<'q>;
}

pub trait Interpolate: Sized {
    type Distance;

    #[expect(clippy::missing_errors_doc)]
    /// Attempt to grow from `self` to `goal`.
    ///
    /// Returns `Err(end)` if `self` and `end` are within `radius` of one another.
    /// Returns `Ok(x)`, where `x` is within `radius` distance of `self` but along the direction
    /// toward `end`.
    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Result<Self, Self>;
}
