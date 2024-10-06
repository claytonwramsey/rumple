//! Geometric planning.
//!
//! Geometric planning algorithms assume that a robot can move in any direction at a given time.
//! Accordingly, the state-spaces for geometric algorithms are quite well-behaved, enabling linear
//! interpolation between any state. This module contains implementations of common geometric
//! planning algorithms as well as some useful primitives for working with geometric states.

mod prm;
mod rrt;

pub use prm::{Prm, PrmNodeId};
pub use rrt::{rrt, Rrt, RrtConnect};
