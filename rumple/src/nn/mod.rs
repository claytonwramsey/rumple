//! Nearest-neighbor search.

mod kdt;
#[cfg(feature = "kiddo")]
mod kiddo;

pub use kdt::{DistanceAabb, KdKey, KdTreeMap, RangeNearest};
#[cfg(feature = "kiddo")]
pub use kiddo::{KiddoMap, KiddoNearest};
pub trait NearestEntry<V> {
    fn value(&self) -> &V;
}

/// A key-value map which is capable of nearest-neighbor search.
pub trait NearestNeighborsMap<K, V> {
    type Entry<'q>: NearestEntry<V> + 'q
    where
        Self: 'q,
        V: 'q;

    /// Insert a key into the map.
    fn insert(&mut self, key: K, value: V);
    /// Get the nearest element of the space to this key.
    fn nearest<'q>(&'q self, key: &K) -> Option<Self::Entry<'q>>;
}

/// A key-value map which is capable of range nearest-neighbor search.
pub trait RangeNearestNeighborsMap<K, V>: NearestNeighborsMap<K, V> {
    /// The radius of a ball to search.
    type Distance;

    /// An iterator over configurations within a fixed radius of a query.
    type RangeNearest<'q>: Iterator<Item = Self::Entry<'q>>
    where
        V: 'q,
        K: 'q,
        Self: 'q;

    /// Get an iterator over all items in `self` within range `r` of
    fn nearest_within_r<'q>(&'q self, key: &'q K, r: Self::Distance) -> Self::RangeNearest<'q>;
}

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct BorrowedEntry<'a, K, V> {
    pub key: &'a K,
    pub value: &'a V,
}

impl<K, V> Clone for BorrowedEntry<'_, K, V> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<K, V> Copy for BorrowedEntry<'_, K, V> {}

impl<K, V> NearestEntry<V> for BorrowedEntry<'_, K, V> {
    fn value(&self) -> &V {
        self.value
    }
}
