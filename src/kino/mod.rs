//! Kinodynamic planning algorithms.

/// A trait for dynamic propagators.
///
/// `C` is the configuration type, `U` is the control type, and `D` is the duration type.
pub trait Propagate<C, U, D> {
    /// Evaluate the resulting configuration from holding control `control` from `state` for
    /// `duration` timesteps.
    fn propagate(&self, state: &C, control: &U, duration: D) -> C;
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
