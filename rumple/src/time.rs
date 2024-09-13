use std::time::{Duration, Instant};

use crate::Timeout;

pub struct Alarm(Instant);

pub struct Forever;

impl Alarm {
    #[must_use]
    #[expect(clippy::missing_const_for_fn)]
    pub fn ending_at(t: Instant) -> Self {
        Self(t)
    }

    #[must_use]
    pub fn from_now(d: Duration) -> Self {
        Self(Instant::now() + d)
    }

    #[must_use]
    pub fn secs_from_now(s: u64) -> Self {
        Self::from_now(Duration::from_secs(s))
    }
}

impl Timeout for Alarm {
    fn is_over(&self) -> bool {
        Instant::now() >= self.0
    }
}

impl Timeout for Forever {
    fn is_over(&self) -> bool {
        false
    }
}
