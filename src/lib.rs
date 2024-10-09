//! `rumple` is a motion planning library implemented in Rust.
//! It contains generic implementations of common motion planning algorithms with a flexible,
//! zero-cost API.

#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(feature = "simd", feature(portable_simd))]
#![warn(clippy::pedantic, clippy::nursery)]
#![warn(clippy::allow_attributes, reason = "prefer expect over allow")]
// #![warn(missing_docs)]

use num_traits::Zero;

#[macro_use]
extern crate alloc;

pub mod env;
pub mod geo;
pub mod kino;
pub mod metric;
pub mod nn;
pub mod sample;
pub mod space;
pub mod time;
pub mod valid;

/// A sampler for a configuration.
///
/// `C` is the type of the configuration, and `RNG` is a source of randomness.
pub trait Sample<C, RNG> {
    /// Sample a configuration, using `rng` as a source of randomness.
    fn sample(&self, rng: &mut RNG) -> C;
}

/// A metric between configurations.
pub trait Metric<C> {
    /// The distance between configurations.
    type Distance: PartialOrd + Zero;

    /// Compute the distance between `c1` and `c2`.
    fn distance(&self, c1: &C, c2: &C) -> Self::Distance;
}

/// A key-value map which is capable of nearest-neighbor search.
pub trait NearestNeighborsMap<K, V> {
    /// Insert a key into the map.
    fn insert(&mut self, key: K, value: V);
    /// Get the nearest element of the space to this key.
    fn nearest<'q>(&'q self, key: &K) -> Option<(&'q K, &'q V)>;
}

/// A key-value map which is capable of range nearest-neighbor search.
pub trait RangeNearestNeighborsMap<K, V>: NearestNeighborsMap<K, V> {
    /// The radius of a ball to search.
    type Distance;

    /// An iterator over configurations within a fixed radius of a query.
    type RangeNearest<'q>: Iterator<Item = &'q V>
    where
        V: 'q,
        Self: 'q;

    /// Get an iterator over all items in `self` within range `r` of
    fn nearest_within_r<'q>(&'q self, key: &K, r: Self::Distance) -> Self::RangeNearest<'q>;
}
