use usb_pd::{Duration, Instant};

/// Timeout wrapper
pub struct Timeout {
    now: Instant,
    expiry: Option<Instant>,
}

impl Timeout {
    /// Create a new empty timeout
    pub fn new() -> Self {
        Self {
            now: Instant::from_ticks(0),
            expiry: None,
        }
    }

    /// Update the current time
    pub fn update(&mut self, now: Instant) {
        self.now = now;
    }

    /// Start a timeout some duration in the future
    pub fn start(&mut self, duration: Duration) {
        self.expiry = Some(self.now.checked_add_duration(duration).unwrap());
    }

    /// Cancel a timeout
    pub fn cancel(&mut self) {
        self.expiry = None;
    }

    /// Test whether a timeout has expired
    pub fn is_expired(&mut self) -> bool {
        let Some(timeout) = self.expiry else {
            return false;
        };

        // is "now" after the timeout?
        let expired = timeout <= self.now;

        if expired {
            self.cancel();
        }

        expired
    }
}
