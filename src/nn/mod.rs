//! Nearest-neighbor search.

mod kdt;
#[cfg(feature = "kiddo")]
mod kiddo;

pub use kdt::{DistanceAabb, KdKey, KdTreeMap, RangeNearest};
#[cfg(feature = "kiddo")]
pub use kiddo::{KiddoMap, KiddoNearest};

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
    type RangeNearest<'q>: Iterator<Item = (&'q K, &'q V)>
    where
        V: 'q,
        K: 'q,
        Self: 'q;

    /// Get an iterator over all items in `self` within range `r` of
    fn nearest_within_r<'q>(&'q self, key: &'q K, r: Self::Distance) -> Self::RangeNearest<'q>;
}
