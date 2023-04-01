use crate::{Duration, Instant};

/// Timeout wrapper
#[derive(Debug, Default)]
pub struct Timeout(Option<Instant>);

impl Timeout {
    /// Create a new empty timeout
    pub fn new() -> Self {
        Self::default()
    }

    pub fn new_start(now: Instant, duration: Duration) -> Self {
        let mut celf = Self::default();
        celf.start(now, duration);
        celf
    }

    /// Start a timeout some duration in the future
    pub fn start(&mut self, now: Instant, duration: Duration) {
        self.0 = Some(now + duration);
    }

    /// Cancel a timeout
    pub fn cancel(&mut self) {
        self.0 = None;
    }

    /// Test whether a timeout has expired
    pub fn is_expired(&mut self, now: Instant) -> bool {
        let Some(timeout) = self.0 else {
            return false;
        };

        // is "now" after the timeout?
        let expired = timeout <= now;

        if expired {
            self.cancel();
        }

        expired
    }
}
