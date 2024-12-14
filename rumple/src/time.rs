//! Timeout conditions.

/// A timeout condition.
pub trait Timeout {
    /// Return `true` if this timeout condition is over.
    fn is_over(&self) -> bool;

    /// Update the number of attempted samples of the configuration space, incrementing by `_n`.
    /// This includes both valid and invalid sampled states.
    fn update_sample_count(&mut self, _n: usize) {}

    /// Update the number of nodes in a configuration-space graph, incrementing by `_n`.
    /// This exclusively includes valid sampled states.
    fn update_node_count(&mut self, _n: usize) {}

    /// Notify that the problem has been solved (i.e. with a satisficing solution).
    fn notify_solved(&mut self) {}
}

/// A helper structure for generating a composite timeout of multiple conditions.
///
/// # Examples
///
/// ```
/// use rumple::time::{Any, Forever, LimitSamples, Timeout};
/// let tc0 = Forever;
/// let tc1 = LimitSamples::new(1000);
/// let composed = Any((tc0, tc1));
/// assert!(!composed.is_over());
/// ```
///
/// You can use the bitwise-or operator on any provided timeout for easy composition.
///
/// ```
/// use rumple::time::{Any, LimitNodes, LimitSamples, Timeout};
/// let tc0 = LimitNodes::new(100);
/// let tc1 = LimitSamples::new(1000);
/// let composed = tc0 | tc1;
/// assert!(!composed.is_over());
/// ```
pub struct Any<T>(pub T);

impl<T, R> BitOr<R> for Any<T> {
    type Output = Any<(Self, R)>;
    fn bitor(self, rhs: R) -> Self::Output {
        Any((self, rhs))
    }
}

impl<T: Timeout> Timeout for &mut T {
    fn is_over(&self) -> bool {
        (**self).is_over()
    }

    fn notify_solved(&mut self) {
        (**self).notify_solved();
    }

    fn update_node_count(&mut self, n: usize) {
        (**self).update_node_count(n);
    }

    fn update_sample_count(&mut self, n: usize) {
        (**self).update_sample_count(n);
    }
}

/// A timeout condition that enables a planner to run forever.
pub struct Forever;

/// A timeout condition that terminates as soon as a problem has been solved.
pub struct Solved(bool);

use core::ops::BitOr;

#[cfg(feature = "std")]
pub use alarm::Alarm;

/// A timeout condition that limits the maximum number of samples that a planner can make.
///
/// A sample may or may not be a valid configuration.
pub struct LimitSamples {
    current: usize,
    limit: usize,
}

/// A timeout condition that limits the maximum number of nodes that a planner can make.
///
/// A node is a valid configuration that is added to a planner's graph.
pub struct LimitNodes {
    current: usize,
    limit: usize,
}

/// Implement Timeout for an Any of some tuple.
macro_rules! any_tuple {
    () => {}; // This case is handled above by the trivial case
    ($($args:ident),*) => {
        #[expect(clippy::allow_attributes)]
        impl<$($args: Timeout),*> Timeout for Any<($($args,)*)> {
            fn is_over(&self) -> bool {
                #[allow(non_snake_case)]
                let &($(ref $args,)*) = &self.0;
                $(
                    if ($args).is_over() {
                        return true;
                    }
                )*
                false
            }

            fn update_sample_count(&mut self, n: usize) {
                #[allow(non_snake_case)]
                let &mut ($(ref mut $args,)*) = &mut self.0;
                $(
                    $args.update_sample_count(n);
                )*
            }


            fn update_node_count(&mut self, n: usize) {
                #[allow(non_snake_case)]
                let &mut ($(ref mut $args,)*) = &mut self.0;
                $(
                    $args.update_node_count(n);
                )*
            }
        }
    }
}

any_tuple!();
any_tuple!(A);
any_tuple!(A, B);
any_tuple!(A, B, C);
any_tuple!(A, B, C, D);
any_tuple!(A, B, C, D, E);
any_tuple!(A, B, C, D, E, F);
any_tuple!(A, B, C, D, E, F, G);
any_tuple!(A, B, C, D, E, F, G, H);
any_tuple!(A, B, C, D, E, F, G, H, I);
any_tuple!(A, B, C, D, E, F, G, H, I, J);

macro_rules! bitor_impl {
    ($t: ident) => {
        impl<R: Timeout> core::ops::BitOr<R> for $t {
            type Output = Any<($t, R)>;
            fn bitor(self, rhs: R) -> Self::Output {
                Any((self, rhs))
            }
        }
    };
}

#[cfg(feature = "std")]
mod alarm {
    use super::Any;
    use crate::time::Timeout;
    use core::time::Duration;
    use std::time::Instant;

    /// A timeout that ends after a fixed amount of time has elapsed.
    pub struct Alarm(Instant);

    impl Alarm {
        #[must_use]
        #[expect(clippy::missing_const_for_fn)]
        /// Construct an alarm that will end at time `t`.
        pub fn ending_at(t: Instant) -> Self {
            Self(t)
        }

        #[must_use]
        /// Construct an alarm that will end `d` time from now.
        pub fn from_now(d: Duration) -> Self {
            Self(Instant::now() + d)
        }

        #[must_use]
        /// Construct an alarm that will end `s` seconds from now.
        pub fn secs_from_now(s: u64) -> Self {
            Self::from_now(Duration::from_secs(s))
        }
    }

    impl Timeout for Alarm {
        fn is_over(&self) -> bool {
            Instant::now() >= self.0
        }
    }

    bitor_impl!(Alarm);

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn fifty_millis() {
            let alarm = Alarm::from_now(Duration::from_millis(50));
            assert!(!alarm.is_over());
            std::thread::sleep(Duration::from_millis(50));
            assert!(alarm.is_over());
        }
    }
}

impl Timeout for Forever {
    fn is_over(&self) -> bool {
        false
    }
}

impl LimitNodes {
    #[must_use]
    /// Construct a timeout that limits a search to `n` nodes.
    pub const fn new(n: usize) -> Self {
        Self {
            current: 0,
            limit: n,
        }
    }

    #[must_use]
    pub const fn n_nodes(&self) -> usize {
        self.current
    }
}

impl LimitSamples {
    #[must_use]
    /// Construct a timeout that limits a search to `n` samples.
    pub const fn new(n: usize) -> Self {
        Self {
            current: 0,
            limit: n,
        }
    }

    #[must_use]
    pub const fn n_sampled(&self) -> usize {
        self.current
    }
}

impl Timeout for LimitNodes {
    fn is_over(&self) -> bool {
        self.current >= self.limit
    }

    fn update_node_count(&mut self, n: usize) {
        self.current += n;
    }
}

impl Timeout for LimitSamples {
    fn is_over(&self) -> bool {
        self.current >= self.limit
    }

    fn update_sample_count(&mut self, n: usize) {
        self.current += n;
    }
}

impl Timeout for Solved {
    fn is_over(&self) -> bool {
        self.0
    }

    fn notify_solved(&mut self) {
        self.0 = true;
    }
}

impl Solved {
    #[must_use]
    /// Construct a timeout that runs until the problem is solved.
    pub const fn new() -> Self {
        Self(false)
    }
}

impl Default for Solved {
    fn default() -> Self {
        Self::new()
    }
}

bitor_impl!(Forever);
bitor_impl!(LimitSamples);
bitor_impl!(LimitNodes);
bitor_impl!(Solved);
