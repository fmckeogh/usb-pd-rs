use {
    crate::{
        header::{ControlMessageType, DataMessageType, Header, MessageType, SpecificationRevision},
        Instant, PowerRole,
    },
    core::ptr::copy_nonoverlapping,
    defmt::{debug, trace},
};

pub trait Driver {
    fn init(&mut self);

    fn poll(&mut self, now: Instant);

    fn get_event(&mut self) -> Option<Event>;

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
pub enum Event {
    StateChanged,
    MessageReceived {
        /// Message header (valid if event_kind = `message_received`)
        msg_header: u16,

        /// Message payload (valid if event_kind = `message_received`, possibly `null`)
        msg_payload: *const u8,
    },
}

pub struct Sink<DRIVER> {
    pd_controller: DRIVER,
    protocol_: Protocol,
    supports_ext_message: bool,
    /// Number of valid elements in `source_caps` array
    num_source_caps: u8,

    /// Array of supply capabilities
    source_caps: [SourceCapability; 10],

    /// Indicates if the source can deliver unconstrained power (e.g. a wall wart)
    is_unconstrained: bool,

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
            pd_controller: driver,
            protocol_: Protocol::Usb20,
            supports_ext_message: false,
            num_source_caps: 0,
            source_caps: [SourceCapability {
                supply_type: SupplyType::Battery,
                obj_pos: 0,
                max_current: 0,
                voltage: 0,
                min_voltage: 0,
            }; 10],
            is_unconstrained: false,
            requested_voltage: 0,
            requested_max_current: 0,
            active_voltage: 5000,
            active_max_current: 900,
            spec_rev: 1,
        }
    }

    pub fn init(&mut self) {
        self.pd_controller.init();
        self.update_protocol();
    }

    pub fn poll(&mut self, now: Instant) {
        // process events from PD controller
        loop {
            self.pd_controller.poll(now);

            let Some(evt) = self.pd_controller.get_event() else {
                break;
            };

            match evt {
                Event::StateChanged => {
                    if self.update_protocol() {
                        self.notify(CallbackEvent::ProtocolChanged);
                    }
                }
                Event::MessageReceived {
                    msg_header,
                    msg_payload,
                } => {
                    self.handle_msg(msg_header, msg_payload);
                }
            }
        }
    }

    fn update_protocol(&mut self) -> bool {
        let old_protocol = self.protocol_;

        if self.pd_controller.state() == State::UsbPd {
            self.protocol_ = Protocol::UsbPd;
        } else {
            self.protocol_ = Protocol::Usb20;
            self.active_voltage = 5000;
            self.active_max_current = 900;
            self.num_source_caps = 0;
        }

        return self.protocol_ != old_protocol;
    }

    fn handle_msg(&mut self, header: u16, payload: *const u8) {
        let msg_type = Header(header).message_type();

        match msg_type {
            MessageType::Data(DataMessageType::SourceCapabilities) => {
                self.handle_src_cap_msg(header, payload);
            }
            MessageType::Control(ControlMessageType::Accept) => {
                self.notify(CallbackEvent::PowerAccepted);
            }
            MessageType::Control(ControlMessageType::Reject) => {
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                self.notify(CallbackEvent::PowerRejected);
            }
            MessageType::Control(ControlMessageType::PsRdy) => {
                self.active_voltage = self.requested_voltage;
                self.active_max_current = self.requested_max_current;
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                self.notify(CallbackEvent::PowerReady);
            }
            _ => (),
        }
    }

    fn notify(&mut self, event: CallbackEvent) {
        let mut voltage = 5000;

        match event {
            CallbackEvent::SourceCapabilitiesChanged => {
                debug!("Caps changed: {}", self.num_source_caps);

                // Take maximum voltage
                for i in 0..self.num_source_caps as usize {
                    if self.source_caps[i].voltage > voltage {
                        voltage = self.source_caps[i].voltage;
                    }
                }

                // Limit voltage to 20V as the voltage regulator was likely selected to handle 20V max
                if voltage > 20000 {
                    voltage = 20000;
                }

                self.request_power(voltage, 0);
            }

            CallbackEvent::PowerReady => debug!("Voltage: {}", self.active_voltage),

            CallbackEvent::ProtocolChanged => debug!("protocol_changed"),

            _ => (),
        }
    }

    fn handle_src_cap_msg(&mut self, header: u16, payload: *const u8) {
        let n = Header(header).num_objects() as usize;

        self.num_source_caps = 0;
        self.is_unconstrained = false;
        self.supports_ext_message = false;

        for obj_pos in 0..n {
            let payload = unsafe { payload.add(obj_pos * 4) };

            if self.num_source_caps >= self.source_caps.len() as u8 {
                break;
            }

            let mut capability = 0u32;
            unsafe { copy_nonoverlapping(payload, &mut capability as *mut u32 as *mut u8, 4) };

            let supply_type: SupplyType = unsafe { core::mem::transmute((capability >> 30) as u8) };
            let mut max_current = (capability & 0x3ff) * 10;
            let mut min_voltage = ((capability >> 10) & 0x03ff) * 50;
            let mut voltage = ((capability >> 20) & 0x03ff) * 50;

            if supply_type == SupplyType::Fixed {
                voltage = min_voltage;

                // Fixed 5V capability contains additional information
                if voltage == 5000 {
                    self.is_unconstrained = (capability & (1 << 27)) != 0;
                    self.supports_ext_message = (capability & (1 << 24)) != 0;
                }
            } else if supply_type == SupplyType::Pps {
                if (capability & (3 << 28)) != 0 {
                    continue;
                }

                max_current = (capability & 0x007f) * 50;
                min_voltage = ((capability >> 8) & 0x00ff) * 100;
                voltage = ((capability >> 17) & 0x00ff) * 100;
            }

            self.source_caps[self.num_source_caps as usize] = SourceCapability {
                supply_type,
                obj_pos: obj_pos as u8 + 1,
                max_current: max_current as u16,
                voltage: voltage as u16,
                min_voltage: min_voltage as u16,
            };
            self.num_source_caps += 1;
        }

        self.notify(CallbackEvent::SourceCapabilitiesChanged);
    }

    fn request_power(&mut self, voltage: u16, mut max_current: u16) {
        // Lookup fixed voltage capabilities first
        let mut index = usize::MAX;
        for i in 0..self.num_source_caps as usize {
            let cap = self.source_caps[i];
            if cap.supply_type == SupplyType::Fixed
                && voltage >= cap.min_voltage
                && voltage <= cap.voltage
            {
                index = i;
                if max_current == 0 {
                    max_current = cap.max_current;
                }
                break;
            }
        }

        // // Lookup PPS capabilites next
        // if (index == -1) {
        //     for (int i = 0; i < num_source_caps; i++) {
        //         auto cap = source_caps + i;
        //         if (cap->supply_type == pd_supply_type::pps && voltage >= cap->min_voltage && voltage <= cap->voltage) {
        //             if (max_current == 0) {
        //                 max_current = cap->max_current;
        //                 index = i;
        //                 break;
        //             } else if (max_current >= 25 && max_current <= cap->max_current) {
        //                 index = i;
        //                 break;
        //             }
        //         }
        //     }
        // }

        if index == usize::MAX {
            panic!("Unsupported voltage {} requested", voltage);
        }

        let res = self.request_power_from_capability(index, voltage, max_current);
        trace!("got result {} from request power cap", res);
    }

    fn request_power_from_capability(
        &mut self,
        index: usize,
        voltage: u16,
        max_current: u16,
    ) -> i32 {
        if index >= self.num_source_caps as usize {
            return -1;
        }
        let cap = self.source_caps[index];
        if cap.supply_type != SupplyType::Fixed && cap.supply_type != SupplyType::Pps {
            return -1;
        }
        if voltage < cap.min_voltage || voltage > cap.voltage {
            return -1;
        }
        if max_current < 25 || max_current > cap.max_current {
            return -1;
        }

        // Create 'request' message
        let mut payload = [0; 4];
        if cap.supply_type == SupplyType::Fixed {
            self.set_request_payload_fixed(&mut payload, cap.obj_pos, voltage, max_current);
        } else {
            todo!()
        }

        let header = Header(0)
            .with_message_type_raw(DataMessageType::Request as u8)
            .with_num_objects(1)
            .with_spec_revision(SpecificationRevision::from(self.spec_rev))
            .with_port_power_role(PowerRole::Sink);

        // Send message
        self.pd_controller.send_message(header, &payload);

        return cap.obj_pos as i32;
    }

    fn set_request_payload_fixed(
        &mut self,
        payload: &mut [u8],
        obj_pos: u8,
        voltage: u16,
        mut current: u16,
    ) {
        let no_usb_suspend = 1;
        let usb_comm_capable = 2;

        current = (current + 5) / 10;
        if current > 0x3ff {
            current = 0x3ff;
        }
        payload[0] = (current & 0xff) as u8;
        payload[1] = (((current >> 8) & 0x03) | ((current << 2) & 0xfc)) as u8;
        payload[2] = ((current >> 6) & 0x0f) as u8;
        payload[3] = (obj_pos & 0x07) << 4 | no_usb_suspend | usb_comm_capable;

        self.requested_voltage = voltage;
        self.requested_max_current = current * 10;
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

/// Power source capability
#[derive(Clone, Copy)]
struct SourceCapability {
    /// Supply type (fixed, batttery, variable etc.)
    supply_type: SupplyType,
    /// Position within message (don't touch)
    obj_pos: u8,
    /// Maximum current (in mA)
    max_current: u16,
    /// Voltage (in mV)
    voltage: u16,
    /// Minimum voltage for variable supplies (in mV)
    min_voltage: u16,
}

/// Callback event types
enum CallbackEvent {
    /// Power delivery protocol has changed
    ProtocolChanged,
    /// Source capabilities have changed (immediately request power)
    SourceCapabilitiesChanged,
    /// Requested power has been accepted (but not ready yet)
    PowerAccepted,
    /// Requested power has been rejected
    PowerRejected,
    /// Requested power is now ready
    PowerReady,
}
