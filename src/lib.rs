#![cfg_attr(not(feature = "std"), no_std)]
#![warn(clippy::pedantic, clippy::nursery)]

use geo::EdgeValidate;
use kino::DynamicValidate;
use num_traits::Zero;

#[macro_use]
extern crate alloc;

pub mod env;
pub mod float;
pub mod geo;
pub mod kino;
pub mod metric;
pub mod nn;
pub mod sample;
pub mod space;
pub mod time;

pub trait Validate<C> {
    fn is_valid_configuration(&self, c: &C) -> bool;
}

pub struct AlwaysValid;

impl<C> Validate<C> for AlwaysValid {
    fn is_valid_configuration(&self, _: &C) -> bool {
        true
    }
}

impl<C> EdgeValidate<C> for AlwaysValid {
    fn is_valid_transition(&self, _: &C, _: &C) -> bool {
        true
    }
}

impl<P, C, U, D> DynamicValidate<P, C, U, D> for AlwaysValid {
    fn is_valid_transition(&self, _: &P, _: &C, _: &U, _: D, _: &C) -> bool {
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
