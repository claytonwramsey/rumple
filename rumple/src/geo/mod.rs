//! Geometric planning.
//!
//! Geometric planning algorithms assume that a robot can move in any direction at a given time.
//! Accordingly, the state-spaces for geometric algorithms are quite well-behaved, enabling linear
//! interpolation between any state. This module contains implementations of common geometric
//! planning algorithms as well as some useful primitives for working with geometric states.

mod prm;
mod rrt;
mod rrtc;

pub use prm::Prm;
pub use rrt::{rrt, Rrt};
pub use rrtc::{rrt_connect, RrtConnect};

/// A generic trait for planners which are geometric graphs.
pub trait Graph {
    /// A handle for a node in the graph.
    type Node: Copy + Eq;

    /// A configuration in this planner.
    type Configuration;

    /// Get the configuration corresponding to a node.
    ///
    /// # Panics
    ///
    /// This function will panic if `node` corresponds to a node which has been deleted from the
    /// graph.
    fn configuration(&self, node: Self::Node) -> &Self::Configuration;

    /// Get a list of neighbors for a node.
    fn neighbors(&self, node: Self::Node) -> impl IntoIterator<Item = Self::Node>;
}

pub trait Tree {
    type Node: Copy + Eq;

    type Configuration;

    /// Get the configuration corresponding to a node.
    ///
    /// # Panics
    ///
    /// This function will panic if `node` corresponds to a node which has been deleted from the
    /// tree.
    fn configuration(&self, node: Self::Node) -> &Self::Configuration;

    fn children(&self, node: Self::Node) -> impl IntoIterator<Item = Self::Node>;

    /// Returns `None` if `node` is the root.
    fn parent(&self, node: Self::Node) -> Option<Self::Node>;
}
