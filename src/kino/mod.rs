//! Kinodynamic planning algorithms.

pub trait Propagate<C, U, D> {
    fn propagate(&self, state: &C, control: &U, duration: D) -> C;
}

pub trait DynamicValidate<P, C, U, D> {
    fn is_valid_transition(
        &self,
        propagator: &P,
        start: &C,
        control: &U,
        duration: D,
        end: &C,
    ) -> bool;
}
