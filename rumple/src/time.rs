use crate::Timeout;

#[cfg(feature = "std")]
mod alarm {
    use crate::Timeout;
    use core::time::Duration;
    use std::time::Instant;

    pub struct Alarm(Instant);

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

#[cfg(feature = "std")]
pub use alarm::Alarm;

pub struct Forever;

impl Timeout for Forever {
    fn is_over(&self) -> bool {
        false
    }
}
