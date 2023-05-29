use crate::{
    callback::{CallbackFn, Event as CallbackEvent, Response},
    header::{DataMessageType, Header, SpecificationRevision},
    message::Message,
    pdo::FixedVariableRequestDataObject,
    Instant, PowerRole,
};

pub trait Driver {
    fn init(&mut self);

    fn poll(&mut self, now: Instant);

    fn get_event(&mut self) -> Option<DriverEvent>;

    fn send_message(&mut self, header: Header, payload: &[u8]);

    fn state(&mut self) -> State;
}

/// FUSB302 state
#[derive(PartialEq, Clone, Copy)]
pub enum State {
    /// VBUS is present, monitoring for activity on CC1/CC2
    Usb20,
    /// Activity on CC1/CC2 has been detected, waiting for first USB PD message
    UsbPdWait,
    /// Successful USB PD communication established
    UsbPd,
    /// Wait period after a failure
    UsbRetryWait,
}

/// Event queue by FUSB302 instance for clients (such as `pd_sink`)
pub enum DriverEvent {
    StateChanged,
    MessageReceived(Message),
}

pub struct Sink<DRIVER> {
    driver: DRIVER,
    protocol_: Protocol,

    /// Requested voltage (in mV)
    requested_voltage: u16,

    /// Requested maximum current (in mA)
    requested_max_current: u16,

    /// Active voltage (in mV)
    active_voltage: u16,

    /// Active maximum current (in mA)
    active_max_current: u16,

    /// Specification revision (of last message)
    spec_rev: u8,

    callback: CallbackFn,
}

impl<DRIVER: Driver> Sink<DRIVER> {
    pub fn new(driver: DRIVER, callback: CallbackFn) -> Self {
        Self {
            driver,
            protocol_: Protocol::Usb20,
            requested_voltage: 0,
            requested_max_current: 0,
            active_voltage: 5000,
            active_max_current: 900,
            spec_rev: 1,
            callback,
        }
    }

    pub fn init(&mut self) {
        self.driver.init();
        self.update_protocol();
    }

    pub fn poll(&mut self, now: Instant) {
        // process events from PD controller
        loop {
            self.driver.poll(now);

            let Some(evt) = self.driver.get_event() else {
                break;
            };

            match evt {
                DriverEvent::StateChanged => {
                    if self.update_protocol() {
                        self.notify(CallbackEvent::ProtocolChanged);
                    }
                }
                DriverEvent::MessageReceived(message) => {
                    self.handle_msg(message);
                }
            }
        }
    }

    fn update_protocol(&mut self) -> bool {
        let old_protocol = self.protocol_;

        if self.driver.state() == State::UsbPd {
            self.protocol_ = Protocol::UsbPd;
        } else {
            self.protocol_ = Protocol::Usb20;
            self.active_voltage = 5000;
            self.active_max_current = 900;
        }

        self.protocol_ != old_protocol
    }

    fn handle_msg(&mut self, message: Message) {
        match message {
            Message::Accept => self.notify(CallbackEvent::PowerAccepted),
            Message::Reject => {
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                self.notify(CallbackEvent::PowerRejected);
            }
            Message::Ready => {
                self.active_voltage = self.requested_voltage;
                self.active_max_current = self.requested_max_current;
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                self.notify(CallbackEvent::PowerReady);
            }
            Message::SourceCapabilities(caps) => {
                self.notify(CallbackEvent::SourceCapabilitiesChanged(caps))
            }
            Message::Unknown => unimplemented!(),
        }
    }

    fn notify(&mut self, event: CallbackEvent) {
        if let Some(response) = (self.callback)(event) {
            match response {
                Response::RequestPower { index, current } => self.request_power(current, index),
            }
        }
    }

    fn request_power(&mut self, max_current: u16, index: usize) {
        // Create 'request' message
        let mut payload = [0; 4];

        self.set_request_payload_fixed(&mut payload, index as u8, max_current);

        let header = Header(0)
            .with_message_type_raw(DataMessageType::Request as u8)
            .with_num_objects(1)
            .with_spec_revision(SpecificationRevision::from(self.spec_rev))
            .with_port_power_role(PowerRole::Sink);

        // Send message
        self.driver.send_message(header, &payload);
    }

    fn set_request_payload_fixed(&mut self, payload: &mut [u8], obj_pos: u8, mut current: u16) {
        current = (current + 5) / 10;

        if current > 0x3ff {
            current = 0x3ff;
        }

        let obj_pos = obj_pos + 1;
        assert!(obj_pos > 0b0000 && obj_pos <= 0b1110);

        FixedVariableRequestDataObject(0)
            .with_operating_current(current)
            .with_maximum_operating_current(current)
            .with_object_position(obj_pos)
            .with_no_usb_suspend(true)
            .with_usb_communications_capable(true)
            .to_bytes(payload);
    }
}

/// Power supply type
#[derive(Clone, Copy, PartialEq)]
pub enum SupplyType {
    /// Fixed supply (Vmin = Vmax)
    Fixed = 0,
    /// Battery
    Battery = 1,
    /// Variable supply (non-battery)
    Variable = 2,
    /// Programmable power supply
    Pps = 3,
}

/// Power deliver protocol
#[derive(PartialEq, Clone, Copy)]
enum Protocol {
    /// No USB PD communication (5V only)
    Usb20,
    /// USB PD communication
    UsbPd,
}
