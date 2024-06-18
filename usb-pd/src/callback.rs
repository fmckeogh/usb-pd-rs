use {crate::pdo::PowerDataObject, defmt::Format};

/// Type of the user's callback function
pub type Function = &'static dyn Fn(Event) -> Option<Response>;

#[derive(Format)]
pub enum Event {
    /// Protocol changed
    ProtocolChanged { protocol: Protocol },

    /// Source capabilities have changed
    ///
    /// Requires immediate response
    SourceCapabilities {
        source_capabilities: [Option<PowerDataObject>; 10],
    },

    /// Requested power has been accepted
    PowerAccepted,

    /// Requested power has been rejected
    PowerRejected,

    /// Requested power is ready
    PowerReady { active_voltage_mv: u16 },
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
    Request {
        /// Index of the desired PowerDataObject
        index: usize,
        current: u16,
    },
}
