use usb_pd::pdo::PowerDataObject;

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

pub enum Protocol {
    /// No USB-PD communication, USB 2.0 5V only
    _20,
    /// USB-PD communication
    PD,
}
