use crate::{Interpolate, Validate};

#[derive(Clone, Copy, Debug)]
/// An edge validator that determines validity by subsampling states along an edge.
/// The user must provide an individual state validator (`F`) and a distance between each sample
/// (`R`).
pub struct SampleValidate<F, R> {
    fun: F,
    radius: R,
}

impl<F, R> SampleValidate<F, R> {
    pub const fn new(fun: F, radius: R) -> Self {
        Self { fun, radius }
    }
}

impl<F, R, C> Validate<C> for SampleValidate<F, R>
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

    fn is_valid_configuration(&self, c: &C) -> bool {
        (self.fun)(c)
    }
}
