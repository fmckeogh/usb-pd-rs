use crate::i2c::BbI2c;

/// Event kind
pub enum event_kind {
    none,
    state_changed,
    message_received,
}

/// Event queue by FUSB302 instance for clients (such as `pd_sink`)
pub struct event {
    /// Event kind
    pub kind: event_kind,

    /// Message header (valid if event_kind = `message_received`)
    pub msg_header: u16,

    /// Message payload (valid if event_kind = `message_received`, possibly `null`)
    pub msg_payload: *const u8,
    // event() : kind(event_kind::none) {}

    // event(event_kind evt_kind) : kind(evt_kind) {}

    // event(uint16_t header, const uint8_t* payload = nullptr)
    //     : kind(event_kind::message_received), msg_header(header), msg_payload(payload) {}
}

/// FUSB302 state
#[derive(PartialEq)]
pub enum fusb302_state {
    /// VBUS is present, monitoring for activity on CC1/CC2
    usb_20,
    /// Activity on CC1/CC2 has been detected, waiting for first USB PD message
    usb_pd_wait,
    /// Successful USB PD communication established
    usb_pd,
    /// Wait period after a failure
    usb_retry_wait,
}

pub struct Fsusb302 {
    i2c: BbI2c,
}

impl Fsusb302 {
    pub fn new(i2c: BbI2c) -> Self {
        Self { i2c }
    }

    pub fn init(&mut self) {}

    pub fn start_sink(&mut self) {}

    pub fn state(&mut self) -> fusb302_state {
        todo!()
    }

    pub fn poll(&mut self) {}

    pub fn has_event(&mut self) -> bool {
        todo!()
    }

    pub fn pop_event(&mut self) -> event {
        todo!()
    }

    pub fn send_message(&mut self, header: u16, payload: &[u8]) {}
}
