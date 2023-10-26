use {crate::pdo::PowerDataObject, defmt::Format, heapless::Vec};

/// Type of the user's callback function
pub type CallbackFn = &'static (dyn Send + Sync + (Fn(Event) -> Option<Response>));

/// Callback event types
#[derive(Format)]
pub enum Event {
    /// Power delivery protocol has changed
    ProtocolChanged,
    /// Source capabilities have changed (immediately request power)
    SourceCapabilitiesChanged(Vec<PowerDataObject, 8>),
    /// Requested power has been accepted (but not ready yet)
    PowerAccepted,
    /// Requested power has been rejected
    PowerRejected,
    /// Requested power is now ready
    PowerReady,
}

#[derive(Format)]
pub enum Protocol {
    /// No USB-PD communication, USB 2.0 5V only
    _20,
    /// USB-PD communication
    PD,
}

#[derive(Format)]
pub enum Response {
    RequestPower {
        /// Index of the desired PowerDataObject
        index: usize,
        current: u16,
    },
}
