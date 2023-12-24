use {
    crate::{
        header::{DataMessageType, Header, SpecificationRevision},
        message::Message,
        pdo::{FixedVariableRequestDataObject, PowerDataObject, VDMHeader},
        PowerRole,
    },
    core::future::Future,
    defmt::{warn, Format},
    embassy_time::Instant,
    heapless::Vec,
};

pub trait Driver {
    fn init(&mut self) -> impl Future<Output = ()>;

    fn poll(&mut self, now: Instant) -> impl Future<Output = ()>;

    fn get_pending_message(&mut self) -> Option<Message>;

    fn did_change_protocol(&mut self) -> bool;

    fn send_message(&mut self, header: Header, payload: &[u8]) -> impl Future<Output = ()>;

    fn state(&mut self) -> DriverState;
}

/// Sink events
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
    /// VDM received
    VDMReceived((VDMHeader, Vec<u32, 7>)),
}

/// Requests made to sink
#[derive(Format)]
pub enum Request {
    RequestPower {
        /// Index of the desired PowerDataObject
        index: usize,
        current: u16,
    },
}

/// Driver state
#[derive(PartialEq, Clone, Copy)]
pub enum DriverState {
    /// VBUS is present, monitoring for activity on CC1/CC2
    Usb20,
    /// Activity on CC1/CC2 has been detected, waiting for first USB PD message
    UsbPdWait,
    /// Successful USB PD communication established
    UsbPd,
    /// Wait period after a failure
    UsbRetryWait,
}

pub struct Sink<DRIVER> {
    driver: DRIVER,

    protocol: Protocol,

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
}

impl<DRIVER: Driver> Sink<DRIVER> {
    pub fn new(driver: DRIVER) -> Self {
        Self {
            driver,
            protocol: Protocol::Usb20,
            requested_voltage: 0,
            requested_max_current: 0,
            active_voltage: 5000,
            active_max_current: 900,
            spec_rev: 1,
        }
    }

    pub async fn init(&mut self) {
        self.driver.init().await;
        self.update_protocol();
    }

    /// Call continously until `None` is returned.
    pub async fn poll(&mut self, now: Instant) -> Option<Event> {
        // poll inner driver
        self.driver.poll(now).await;

        if let Some(message) = self.driver.get_pending_message() {
            return self.handle_msg(message);
        };

        if self.driver.did_change_protocol() {
            if self.update_protocol() {
                Some(Event::ProtocolChanged)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub async fn request(&mut self, request: Request) {
        match request {
            Request::RequestPower { index, current } => self.request_power(current, index).await,
        }
    }

    fn update_protocol(&mut self) -> bool {
        let old_protocol = self.protocol;

        if self.driver.state() == DriverState::UsbPd {
            self.protocol = Protocol::UsbPd;
        } else {
            self.protocol = Protocol::Usb20;
            self.active_voltage = 5000;
            self.active_max_current = 900;
        }

        self.protocol != old_protocol
    }

    fn handle_msg(&mut self, message: Message) -> Option<Event> {
        match message {
            Message::Accept => Some(Event::PowerAccepted),
            Message::Reject => {
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                Some(Event::PowerRejected)
            }
            Message::Ready => {
                self.active_voltage = self.requested_voltage;
                self.active_max_current = self.requested_max_current;
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                Some(Event::PowerReady)
            }
            Message::SourceCapabilities(caps) => Some(Event::SourceCapabilitiesChanged(caps)),
            Message::VendorDefined((hdr, data)) => {
                match hdr {
                    crate::pdo::VDMHeader::Structured(hdr) => {
                        warn!(
                            "UNHANDLED: Structured VDM! CMD_TYPE: {:?}, CMD:
                        {:?}",
                            hdr.command_type(),
                            hdr.command()
                        );
                    }
                    crate::pdo::VDMHeader::Unstructured(hdr) => {
                        warn!(
                            "UNHANDLED: Unstructured VDM! SVID: {:x}, DATA:
                        {:x}",
                            hdr.standard_or_vid(),
                            hdr.data()
                        );
                    }
                }
                Some(Event::VDMReceived((hdr, data)))
            }
            Message::SoftReset => {
                warn!("UNHANDLED: Soft RESET request.");
                None
            }
            Message::Unknown => unimplemented!(),
        }
    }

    async fn request_power(&mut self, max_current: u16, index: usize) {
        // Create 'request' message
        let mut payload = [0; 4];

        self.set_request_payload_fixed(&mut payload, index as u8, max_current);

        let header = Header(0)
            .with_message_type_raw(DataMessageType::Request as u8)
            .with_num_objects(1)
            .with_spec_revision(SpecificationRevision::from(self.spec_rev))
            .with_port_power_role(PowerRole::Sink);

        // Send message
        self.driver.send_message(header, &payload).await;
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
#[derive(Format, PartialEq, Clone, Copy)]
enum Protocol {
    /// No USB PD communication (5V only)
    Usb20,
    /// USB PD communication
    UsbPd,
}
