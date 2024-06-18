#[derive(Debug)]
pub enum Event {
    /// Protocol changed
    ProtocolChanged { protocol: Protocol },

    /// Source capabilities have changed
    ///
    /// Requires immediate response
    SourceCapabilities { source_capabilities: () },

    /// Requested power has been accepted
    PowerAccepted,

    /// Requested power has been rejected
    PowerRejected,

    /// Requested power is ready
    PowerReady { active_voltage_mv: u16 },
}

#[derive(Debug)]
pub enum Protocol {
    /// No USB-PD communication, USB 2.0 5V only
    _20,
    /// USB-PD communication
    PD,
}
