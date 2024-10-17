//! State and transition validation.

use crate::space::Interpolate;

/// A trait for types that can determine whether a configuration is valid.
///
/// This could be implemented by (for example) a collision checker, joint limit tester, or a
/// manifold constraint checker.
pub trait Validate<C> {
    /// Return `true` if the configuration is valid and `false` otherwise.
    fn is_valid_configuration(&self, c: &C) -> bool;
}

impl<F, C> Validate<C> for F
where
    F: Fn(&C) -> bool,
{
    fn is_valid_configuration(&self, c: &C) -> bool {
        self(c)
    }
}

#[expect(clippy::module_name_repetitions)]
/// A validator for configurations which states that all configurations are valid.
pub struct AlwaysValid;

impl<C> Validate<C> for AlwaysValid {
    fn is_valid_configuration(&self, _: &C) -> bool {
        true
    }
}

impl<C> GeoValidate<C> for AlwaysValid {
    fn is_valid_transition(&self, _: &C, _: &C) -> bool {
        true
    }
}

impl<P, C, U, D> DynamicValidate<P, C, U, D> for AlwaysValid {
    fn is_valid_transition(&self, _: &P, _: &C, _: &U, _: D, _: &C) -> bool {
        true
    }
}

#[derive(Clone, Copy, Debug)]
/// An edge validator that determines validity by subsampling states along an edge.
/// The user must provide an individual state validator (`F`) and a distance between each sample
/// (`R`).
///
/// `V` should implement `Validate` for a desired configuration, and `R` must be a distance between
/// two configurations.
pub struct SampleInterpolate<V, R> {
    valid: V,
    radius: R,
}

impl<V, R> SampleInterpolate<V, R> {
    /// Construct a new validator.
    pub const fn new(valid: V, radius: R) -> Self {
        Self { valid, radius }
    }
}

impl<V, R, C> Validate<C> for SampleInterpolate<V, R>
where
    V: Validate<C>,
    R: Clone,
    C: Interpolate<Distance = R>,
{
    fn is_valid_configuration(&self, c: &C) -> bool {
        self.valid.is_valid_configuration(c)
    }
}

impl<F, R, C> GeoValidate<C> for SampleInterpolate<F, R>
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

impl<F, R, C, P, D, U> DynamicValidate<P, C, U, D> for SampleInterpolate<F, R>
where
    F: Fn(&C) -> bool,
    C: Interpolate<Distance = R>,
    R: Clone,
{
    /// Validate a transition by linearly interpolating between start and end states.
    /// This does not respect the dynamics of the controller, but it was good enough for OMPL, so
    /// it's good enough for us.
    fn is_valid_transition(&self, _: &P, start: &C, _: &U, _: D, end: &C) -> bool {
        GeoValidate::is_valid_transition(self, start, end)
    }
}

/// A validator for dynamic systems.
///
/// `P` is a state propagator, `C` is the configuration type, `U` is the control type, and `D` is
/// the duration type.
pub trait DynamicValidate<P, C, U, D> {
    /// Determine whether a propagated transition is valid.
    fn is_valid_transition(
        &self,
        propagator: &P,
        start: &C,
        control: &U,
        duration: D,
        end: &C,
    ) -> bool;
}

/// The trait for validating continous edges between configurations.
/// Used by many geometric planners.
pub trait GeoValidate<C>: Validate<C> {
    /// Determine whether the continuous transition between `start` and `end` is valid.
    fn is_valid_transition(&self, start: &C, end: &C) -> bool;
}
