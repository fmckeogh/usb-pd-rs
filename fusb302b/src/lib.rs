#![no_std]

use {
    crate::fsusb302::{event_kind, fusb302_state, Fsusb302},
    core::ptr::copy_nonoverlapping,
    defmt::{debug, trace},
    embedded_hal::blocking::i2c::{Write, WriteRead},
};

mod fsusb302;
pub mod registers;

type Instant = fugit::Instant<u64, 1, 1000>;

/// I2C address of FUSB302BMPX
const DEVICE_ADDRESS: u8 = 0b0100010;

pub struct Fusb302b<I2C> {
    pd_controller: Fsusb302<I2C>,
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

impl<I2C: Write + WriteRead> Fusb302b<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            pd_controller: Fsusb302::new(i2c),
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
        self.pd_controller.start_sink();
        self.update_protocol();
    }

    pub fn poll(&mut self, now: Instant) {
        // process events from PD controller
        loop {
            self.pd_controller.poll(now);

            if !self.pd_controller.has_event() {
                break;
            }

            let evt = self.pd_controller.pop_event();

            match evt.kind {
                event_kind::state_changed => {
                    if self.update_protocol() {
                        self.notify(CallbackEvent::protocol_changed);
                    }
                }
                event_kind::message_received => {
                    self.handle_msg(evt.msg_header, evt.msg_payload);
                }
            }
        }
    }

    fn update_protocol(&mut self) -> bool {
        let old_protocol = self.protocol_;

        if self.pd_controller.state() == fusb302_state::usb_pd {
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
        let _spec_rev = PdHeader(header).spec_rev();

        let msg_type = PdHeader(header).message_type();

        match msg_type {
            pd_msg_type::pd_msg_type_data_source_capabilities => {
                self.handle_src_cap_msg(header, payload);
            }
            pd_msg_type::pd_msg_type_ctrl_accept => {
                self.notify(CallbackEvent::power_accepted);
            }
            pd_msg_type::pd_msg_type_ctrl_reject => {
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                self.notify(CallbackEvent::power_rejected);
            }
            pd_msg_type::pd_msg_type_ctrl_ps_ready => {
                self.active_voltage = self.requested_voltage;
                self.active_max_current = self.requested_max_current;
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                self.notify(CallbackEvent::power_ready);
            }
            _ => (),
        }
    }

    fn notify(&mut self, event: CallbackEvent) {
        let mut voltage = 5000;

        match event {
            CallbackEvent::source_caps_changed => {
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

            CallbackEvent::power_ready => debug!("Voltage: {}", self.active_voltage),

            CallbackEvent::protocol_changed => debug!("protocol_changed"),

            _ => (),
        }
    }

    fn handle_src_cap_msg(&mut self, header: u16, payload: *const u8) {
        let n = PdHeader(header).num_data_objs();

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

        self.notify(CallbackEvent::source_caps_changed);
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

        let header = PdHeader::create_data(pd_msg_type::pd_msg_type_data_request, 1, self.spec_rev);

        // Send message
        self.pd_controller.send_message(header.0, &payload);

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
enum SupplyType {
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
    protocol_changed,
    /// Source capabilities have changed (immediately request power)
    source_caps_changed,
    /// Requested power has been accepted (but not ready yet)
    power_accepted,
    /// Requested power has been rejected
    power_rejected,
    /// Requested power is now ready
    power_ready,
}

/// USB PD message type
#[derive(PartialEq)]
pub enum pd_msg_type {
    pd_msg_type_ctrl_good_crc = 0x01,
    pd_msg_type_ctrl_goto_min = 0x02,
    pd_msg_type_ctrl_accept = 0x03,
    pd_msg_type_ctrl_reject = 0x04,
    pd_msg_type_ctrl_ping = 0x05,
    pd_msg_type_ctrl_ps_ready = 0x06,
    pd_msg_type_ctrl_get_source_cap = 0x07,
    pd_msg_type_ctrl_get_sink_cap = 0x08,
    pd_msg_type_ctrl_dr_swap = 0x09,
    pd_msg_type_ctrl_pr_swap = 0x0a,
    pd_msg_type_ctrl_vconn_swap = 0x0b,
    pd_msg_type_ctrl_wait = 0x0c,
    pd_msg_type_ctrl_soft_reset = 0x0d,
    pd_msg_type_ctrl_data_reset = 0x0e,
    pd_msg_type_ctrl_data_reset_complete = 0x0f,
    pd_msg_type_ctrl_not_supported = 0x10,
    pd_msg_type_ctrl_get_source_cap_extended = 0x11,
    pd_msg_type_ctrl_get_status = 0x12,
    pd_msg_type_ctrl_fr_swap = 0x13,
    pd_msg_type_ctrl_get_pps_status = 0x14,
    pd_msg_type_ctrl_get_country_codes = 0x15,
    pd_msg_type_ctrl_get_sink_cap_extended = 0x16,
    pd_msg_type_data_source_capabilities = 0x81,
    pd_msg_type_data_request = 0x82,
    pd_msg_type_data_bist = 0x83,
    pd_msg_type_data_sink_capabilities = 0x84,
    pd_msg_type_data_battery_status = 0x85,
    pd_msg_type_data_alert = 0x86,
    pd_msg_type_data_get_country_info = 0x87,
    pd_msg_type_data_enter_usb = 0x88,
    pd_msg_type_data_vendor_defined = 0x8f,
}

/// Helper class to constrcut and decode USB PD message headers
pub struct PdHeader(pub u16);

impl PdHeader {
    pub fn has_extended(&self) -> bool {
        return (self.0 & 0x8000) != 0;
    }

    pub fn num_data_objs(&self) -> usize {
        ((self.0 >> 12) & 0x07) as usize
    }

    pub fn message_id(&self) -> u8 {
        ((self.0 >> 9) & 0x07) as u8
    }

    pub fn message_type(&self) -> pd_msg_type {
        unsafe {
            core::mem::transmute(
                ((((self.num_data_objs() != 0) as u16) << 7) | (self.0 & 0x1f)) as u8,
            )
        }
    }

    pub fn spec_rev(&self) -> u8 {
        ((self.0 >> 6) as u8 & 0x03) + 1
    }

    pub fn create_ctrl(msg_type: pd_msg_type, rev: u8) -> Self {
        Self((msg_type as u16 & 0x1f) | 0x40 | ((rev as u16 - 1) << 6))
    }

    pub fn create_data(msg_type: pd_msg_type, num_data_objs: usize, rev: u8) -> Self {
        Self(
            ((num_data_objs as u16 & 0x07) << 12)
                | (msg_type as u16 & 0x1f)
                | 0x40
                | ((rev as u16 - 1) << 6),
        )
    }
}
