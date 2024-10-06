//! Geometric planning.
//!
//! Geometric planning algorithms assume that a robot can move in any direction at a given time.
//! Accordingly, the state-spaces for geometric algorithms are quite well-behaved, enabling linear
//! interpolation between any state. This module contains implementations of common geometric
//! planning algorithms as well as some useful primitives for working with geometric states.

use crate::{kino::DynamicValidate, Validate};

mod prm;
mod rrt;

pub use prm::{Prm, PrmNodeId};
pub use rrt::{rrt, Rrt, RrtConnect};

/// The trait for validating continous edges between configurations.
/// Used by many geometric planners.
pub trait EdgeValidate<C>: Validate<C> {
    /// Determine whether the continuous transition between `start` and `end` is valid.
    fn is_valid_transition(&self, start: &C, end: &C) -> bool;
}

/// The trait for linear interpolation between configurations.
pub trait Interpolate: Sized {
    /// The radius to which interpolation may be limited.
    type Distance;

    #[expect(clippy::missing_errors_doc)]
    /// Attempt to grow from `self` to `goal`.
    ///
    /// Returns `Err(end)` if `self` and `end` are within `radius` of one another.
    /// Returns `Ok(x)`, where `x` is within `radius` distance of `self` but along the direction
    /// toward `end`.
    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Result<Self, Self>;
}

#[derive(Clone, Copy, Debug)]
/// An edge validator that determines validity by subsampling states along an edge.
/// The user must provide an individual state validator (`F`) and a distance between each sample
/// (`R`).
///
/// `V` should implement `Validate` for a desired configuration, and `R` must be a distance between
/// two configurations.
pub struct DiscreteValidate<V, R> {
    valid: V,
    radius: R,
}

impl<V, R> DiscreteValidate<V, R> {
    /// Construct a new validator.
    pub const fn new(valid: V, radius: R) -> Self {
        Self { valid, radius }
    }
}

impl<V, R, C> Validate<C> for DiscreteValidate<V, R>
where
    V: Validate<C>,
    R: Clone,
    C: Interpolate<Distance = R>,
{
    fn is_valid_configuration(&self, c: &C) -> bool {
        self.valid.is_valid_configuration(c)
    }
}

impl<F, R, C> EdgeValidate<C> for DiscreteValidate<F, R>
where
    F: Validate<C>,
    C: Interpolate<Distance = R>,
    R: Clone,
{
    fn is_valid_transition(&self, start: &C, end: &C) -> bool {
        if !self.is_valid_configuration(start) {
            return false;
        }
        let mut interp = start.interpolate(end, self.radius.clone());
        loop {
            match interp {
                Ok(c) => {
                    if !self.is_valid_configuration(&c) {
                        return false;
                    }
                    interp = c.interpolate(end, self.radius.clone());
                }
                Err(c) => {
                    return self.is_valid_configuration(&c);
                }
            }
        }
    }
}

impl<F, R, C, P, D, U> DynamicValidate<P, C, U, D> for DiscreteValidate<F, R>
where
    F: Fn(&C) -> bool,
    C: Interpolate<Distance = R>,
    R: Clone,
{
    /// Validate a transition by linearly interpolating between start and end states.
    /// This does not respect the dynamics of the controller, but it was good enough for OMPL, so
    /// it's good enough for us.
    fn is_valid_transition(&self, _: &P, start: &C, _: &U, _: D, end: &C) -> bool {
        EdgeValidate::is_valid_transition(self, start, end)
    }
}
