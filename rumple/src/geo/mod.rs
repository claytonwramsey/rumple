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
pub use rrt::{rrt, Rrt};

pub trait EdgeValidate<C>: Validate<C> {
    fn is_valid_transition(&self, start: &C, end: &C) -> bool;
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

#[derive(Clone, Copy, Debug)]
/// An edge validator that determines validity by subsampling states along an edge.
/// The user must provide an individual state validator (`F`) and a distance between each sample
/// (`R`).
pub struct DiscreteValidate<F, R> {
    fun: F,
    radius: R,
}

impl<F, R> DiscreteValidate<F, R> {
    pub const fn new(fun: F, radius: R) -> Self {
        Self { fun, radius }
    }
}

impl<F, R, C> Validate<C> for DiscreteValidate<F, R>
where
    F: Fn(&C) -> bool,
    C: Interpolate<Distance = R>,
    R: Clone,
{
    fn is_valid_configuration(&self, c: &C) -> bool {
        (self.fun)(c)
    }
}

impl<F, R, C> EdgeValidate<C> for DiscreteValidate<F, R>
where
    F: Fn(&C) -> bool,
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
