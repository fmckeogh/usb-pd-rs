#![allow(non_camel_case_types)]

use crate::pd_msg_type;

use {
    crate::{
        fsusb302::{
            control1::*, control3::*, interrupt::*, interrupta::*, interruptb::*, mask::*,
            maska::*, maskb::*, power::*, reg::*, reset::*, slice::*, status0::*, status1::*,
            switches0::*, switches1::*, token::*,
        },
        {pd_header, pd_msg_type::*},
    },
    defmt::debug,
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    fixed_queue::VecDeque,
};

const switches0_none: u8 = 0;

/// FUSB302 register addresses
enum reg {
    reg_device_id = 0x01,
    reg_switches0 = 0x02,
    reg_switches1 = 0x03,
    reg_measure = 0x04,
    reg_slice = 0x05,
    reg_control0 = 0x06,
    reg_control1 = 0x07,
    reg_control2 = 0x08,
    reg_control3 = 0x09,
    reg_mask = 0x0a,
    reg_power = 0x0b,
    reg_reset = 0x0c,
    reg_ocpreg = 0x0d,
    reg_maska = 0x0e,
    reg_maskb = 0x0f,
    reg_control4 = 0x10,
    reg_status0a = 0x3c,
    reg_status1a = 0x3d,
    reg_interrupta = 0x3e,
    reg_interruptb = 0x3f,
    reg_status0 = 0x40,
    reg_status1 = 0x41,
    reg_interrupt = 0x42,
    reg_fifos = 0x43,
}

/// FUSB302 register SWITCHES0 values
enum switches0 {
    switches0_pu_en2 = 0x01 << 7,
    switches0_pu_en1 = 0x01 << 6,
    switches0_vconn_cc2 = 0x01 << 5,
    switches0_vconn_cc1 = 0x01 << 4,
    switches0_meas_cc2 = 0x01 << 3,
    switches0_meas_cc1 = 0x01 << 2,
    switches0_pdwn2 = 0x01 << 1,
    switches0_pdwn1 = 0x01 << 0,
}

/// FUSB302 register SWITCHES1 values
enum switches1 {
    switches1_powerrole = 0x01 << 7,
    switches1_specrev_mask = 0x03 << 5,
    switches1_specrev_rev_1_0 = 0x00 << 5,
    switches1_specrev_rev_2_0 = 0x01 << 5,
    switches1_datarole = 0x01 << 4,
    switches1_auto_crc = 0x01 << 2,
    switches1_txcc2 = 0x01 << 1,
    switches1_txcc1 = 0x01 << 0,
}

/// FUSB302 register MEASURE values
enum measure {
    measure_meas_vbus = 0x01 << 6,
    measure_meas_mdac_mask = 0x3f,
}

const slice_sdac_hys_mask: u8 = 0x03 << 6;

/// FUSB302 register SLICE values
enum slice {
    slice_sdac_hys_255mv = 0x03 << 6,
    slice_sdac_hys_170mv = 0x02 << 6,
    slice_sdac_hys_085mv = 0x01 << 6,
    slice_sdac_hys_none = 0x00 << 6,
    slice_sdac_mask = 0x3f << 0,
}

const control0_host_cur_mask: u8 = 0x03 << 2;

/// FUSB302 register CONTROL0 values
enum control0 {
    control0_tx_flush = 0x01 << 6,
    control0_int_mask = 0x01 << 5,
    control0_host_cur_no = 0x00 << 2,
    control0_host_cur_usb_def = 0x01 << 2,
    control0_host_cur_1p5a = 0x02 << 2,
    control0_host_cur_3p0a = 0x03 << 2,
    control0_auto_pre = 0x01 << 1,
    control0_tx_start = 0x01 << 0,
}

/// FUSB302 register CONTROL1 values
enum control1 {
    control1_ensop2db = 0x01 << 6,
    control1_ensop1db = 0x01 << 5,
    control1_bist_mode2 = 0x01 << 4,
    control1_rx_flush = 0x01 << 2,
    control1_ensop2 = 0x01 << 1,
    control1_ensop1 = 0x01 << 0,
}

const control2_mode_mask: u8 = 0x03 << 1;

/// FUSB302 register CONTROL2 values
enum control2 {
    control2_tog_save_pwr_mask = 0x03 << 6,
    control2_tog_rd_only = 0x01 << 5,
    control2_wake_en = 0x01 << 3,
    control2_mode_src_polling = 0x03 << 1,
    control2_mode_snk_polling = 0x02 << 1,
    control2_mode_drp_polling = 0x01 << 1,
    control2_toggle = 0x01 << 0,
}

const control3_n_retries_mask: u8 = 0x03 << 1;

/// FUSB302 register CONTROL3 values
enum control3 {
    control3_send_hard_reset = 0x03 << 6,
    control3_bist_tmode = 0x01 << 5,
    control3_auto_hardreset = 0x01 << 4,
    control3_auto_softreset = 0x01 << 3,
    control3_3_retries = 0x03 << 1,
    control3_2_retries = 0x02 << 1,
    control3_1_retry = 0x01 << 1,
    control3_0_retries = 0x00 << 1,
    control3_auto_retry = 0x01 << 0,
}

/// FUSB302 register MASK values
enum mask {
    mask_m_all = 0xff,
    mask_m_vbusok = 0x01 << 7,
    mask_m_activity = 0x01 << 6,
    mask_m_comp_chng = 0x01 << 5,
    mask_m_crc_chk = 0x01 << 4,
    mask_m_alert = 0x01 << 3,
    mask_m_wake = 0x01 << 2,
    mask_m_collision = 0x01 << 1,
    mask_m_bc_lvl = 0x01 << 0,
}

const power_pwr_mask: u8 = 0x0f << 0;

/// FUSB302 register POWER values
enum power {
    power_pwr_all = 0x0f << 0,
    power_pwr_int_osc = 0x01 << 3,
    power_pwr_receiver = 0x01 << 2,
    power_pwr_measure = 0x01 << 1,
    power_pwr_bandgap = 0x01 << 0,
}

/// FUSB302 register RESET values
enum reset {
    reset_pd_reset = 0x01 << 1,
    reset_sw_res = 0x01 << 0,
}

/// FUSB302 register MASKA values
enum maska {
    maska_m_all = 0xff,
    maska_m_none = 0x00,
    maska_m_ocp_temp = 0x01 << 7,
    maska_m_togdone = 0x01 << 6,
    maska_m_softfail = 0x01 << 5,
    maska_m_retry_fail = 0x01 << 4,
    maska_m_hardsent = 0x01 << 3,
    maska_m_txsent = 0x01 << 2,
    maska_m_softrst = 0x01 << 1,
    maska_m_hardrst = 0x01 << 0,
}

/// FUSB302 register MASKB values
enum maskb {
    maskb_m_gcrcsent = 0x01 << 0,
}

const maskb_m_all: u8 = 0x01;
const maskb_m_none: u8 = 0x00;

const status1a_togss_mask: u8 = 0x07 << 3;
/// FUSB302 register STATUS1A values
enum status1a {
    status1a_togss_toggle_running = 0x00 << 3,
    status1a_togss_src_on_cc1 = 0x01 << 3,
    status1a_togss_src_on_cc2 = 0x02 << 3,
    status1a_togss_snk_on_cc1 = 0x05 << 3,
    status1a_togss_snk_on_cc2 = 0x06 << 3,
    status1a_togss_auto_accessory = 0x07 << 3,

    status1a_rxsop2db = 0x01 << 2,
    status1a_rxsop1db = 0x01 << 1,
    status1a_rxsop = 0x01 << 0,
}

/// FUSB302 register INTERRUPTA values
enum interrupta {
    interrupta_i_ocp_temp = 0x01 << 7,
    interrupta_i_togdone = 0x01 << 6,
    interrupta_i_softfail = 0x01 << 5,
    interrupta_i_retryfail = 0x01 << 4,
    interrupta_i_hardsent = 0x01 << 3,
    interrupta_i_txsent = 0x01 << 2,
    interrupta_i_softrst = 0x01 << 1,
    interrupta_i_hardrst = 0x01 << 0,
}

/// FUSB302 register INTERRUPTB values
enum interruptb {
    interruptb_i_gcrcsent = 0x01 << 0,
}

/// FUSB302 register STATUS0 values
enum status0 {
    status0_vbusok = 0x01 << 7,
    status0_activity = 0x01 << 6,
    status0_comp = 0x01 << 5,
    status0_crc_chk = 0x01 << 4,
    status0_alert_chk = 0x01 << 3,
    status0_wake = 0x01 << 2,
    status0_bc_lvl_mask = 0x03 << 0,
}

/// FUSB302 register STATUS1 values
enum status1 {
    status1_rxsop2_mask = 0x01 << 7,
    status1_rxsop1 = 0x01 << 6,
    status1_rx_empty = 0x01 << 5,
    status1_rx_full = 0x01 << 4,
    status1_tx_empty = 0x01 << 3,
    status1_tx_full = 0x01 << 2,
    status1_ovrtemp = 0x01 << 1,
    status1_ocp = 0x01 << 0,
}

/// FUSB302 register INTERRUPT values
enum interrupt {
    interrupt_none = 0,
    interrupt_i_vbusok = 0x01 << 7,
    interrupt_i_activity = 0x01 << 6,
    interrupt_i_comp_chng = 0x01 << 5,
    interrupt_i_crc_chk = 0x01 << 4,
    interrupt_i_alert = 0x01 << 3,
    interrupt_i_wake = 0x01 << 2,
    interrupt_i_collision = 0x01 << 1,
    interrupt_i_bc_lvl = 0x01 << 0,
}

/// Tokens used in FUSB302B FIFO
enum token {
    token_txon = 0xa1,
    token_sop1 = 0x12,
    token_sop2 = 0x13,
    token_sop3 = 0x1b,
    token_reset1 = 0x15,
    token_reset2 = 0x16,
    token_packsym = 0x80,
    token_jam_crc = 0xff,
    token_eop = 0x14,
    token_txoff = 0xfe,
}

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
#[derive(PartialEq, Clone, Copy)]
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
const NUM_MESSAGE_BUF: usize = 4;

pub struct Fsusb302<I2C> {
    i2c: I2C,
    /// cc line being measured
    measuring_cc: u8,

    /// Indicates if the timeout timer is running
    is_timeout_active: bool,
    /// Time when the current timer expires
    timeout_expiration: u32,

    /// RX message buffers
    rx_message_buf: [[u8; 64]; 4],

    /// Next RX message index
    rx_message_index: usize,

    /// Queue of event that have occurred
    events: VecDeque<event, 6>,

    /// Current attachment state
    state_: fusb302_state,

    /// ID for next USB PD message
    next_message_id: u16,

    timestamp: u32,
}

impl<I2C: Write + Read + WriteRead> Fsusb302<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            measuring_cc: 0,
            is_timeout_active: false,
            timeout_expiration: 0,
            rx_message_buf: [[0u8; 64]; 4],
            rx_message_index: 0,
            events: VecDeque::new(),
            state_: fusb302_state::usb_20,
            next_message_id: 0,
            timestamp: 0,
        }
    }

    pub fn state(&self) -> fusb302_state {
        self.state_
    }

    pub fn init(&mut self) {
        // full reset
        self.write_register(reg_reset, reset_sw_res as u8 | reset_pd_reset as u8);

        for _ in 0..1_000_000 {
            cortex_m::asm::nop();
        }

        // power up everyting except oscillator
        self.write_register(reg_power, power_pwr_all as u8 & !(power_pwr_int_osc as u8));
        // Disable all CC monitoring
        self.write_register(reg_switches0, switches0_none);
        // Mask all interrupts
        self.write_register(reg_mask, mask_m_all as u8);
        // Mask all interrupts
        self.write_register(reg_maska, maska_m_all as u8);
        // Mask all interrupts (incl. good CRC sent)
        self.write_register(reg_maskb, maskb_m_all as u8);

        self.next_message_id = 0;
        self.is_timeout_active = false;
        self.state_ = fusb302_state::usb_20;
        self.events.clear();
    }

    pub fn start_sink(&mut self) {
        // As the interrupt line is also used as SWDIO, the FUSB302B interrupt is
        // not activated until activity on CC1 or CC2 has been detected.
        // Thus, CC1 and CC2 have to be polled manually even though the FUSB302B
        // could do it automatically.

        // BMC threshold: 1.35V with a threshold of 85mV
        self.write_register(reg_slice, slice_sdac_hys_085mv as u8 | 0x20);

        self.start_measurement(1);
    }

    pub fn poll(&mut self, timestamp: u32) {
        self.timestamp = timestamp;
        self.check_for_interrupts();

        if (self.has_timeout_expired()) {
            if (self.state_ == fusb302_state::usb_pd_wait) {
                debug!("{}: No CC activity", self.timestamp);
                self.establish_retry_wait();
            } else if (self.state_ == fusb302_state::usb_20) {
                self.check_measurement();
            } else if (self.state_ == fusb302_state::usb_retry_wait) {
                self.establish_usb_20();
            }
        }
    }

    fn start_measurement(&mut self, cc: u8) {
        let mut sw0 = if cc == 1 {
            switches0_meas_cc1 as u8
        } else {
            switches0_meas_cc2 as u8
        };
        sw0 = sw0 | switches0_pdwn1 as u8 | switches0_pdwn2 as u8;

        // test CC
        self.write_register(reg_switches0, sw0 as u8);
        self.start_timeout(10);
        self.measuring_cc = cc;
    }

    fn check_measurement(&mut self) {
        self.read_register(reg_status0 as u8);
        let status0 = self.read_register(reg_status0 as u8);
        if ((status0 & status0_bc_lvl_mask as u8) == 0) {
            // No CC activity
            self.start_measurement(if self.measuring_cc == 1 { 2 } else { 1 });
            return;
        }

        self.establish_usb_pd_wait(self.measuring_cc);
        self.measuring_cc = 0;
    }

    fn check_for_interrupts(&mut self) {
        let mut may_have_message = false;

        let interrupt = self.read_register(reg_interrupt as u8);
        let interrupta = self.read_register(reg_interrupta as u8);
        let interruptb = self.read_register(reg_interruptb as u8);

        if ((interrupta & interrupta_i_hardrst as u8) != 0) {
            debug!("{}: Hard reset\r\n", self.timestamp);
            self.establish_retry_wait();
            return;
        }
        if ((interrupta & interrupta_i_retryfail as u8) != 0) {
            debug!("Retry failed");
        }
        if ((interrupta & interrupta_i_txsent as u8) != 0) {
            debug!("TX ack");
            // turn off internal oscillator if TX FIFO is empty
            let status1 = self.read_register(reg_status1 as u8);
            if ((status1 & status1_tx_empty as u8) != 0) {
                self.write_register(reg_power, power_pwr_all as u8 & !(power_pwr_int_osc as u8));
            }
        }
        if ((interrupt & interrupt_i_activity as u8) != 0) {
            may_have_message = true;
        }
        if ((interrupt & interrupt_i_crc_chk as u8) != 0) {
            debug!("{}: CRC ok", self.timestamp);
            may_have_message = true;
        }
        if ((interruptb & interruptb_i_gcrcsent as u8) != 0) {
            debug!("Good CRC sent");
            may_have_message = true;
        }
        if (may_have_message) {
            self.check_for_msg();
        }
    }

    fn check_for_msg(&mut self) {
        while (true) {
            let status1 = self.read_register(reg_status1 as u8);
            if ((status1 & status1_rx_empty as u8) == status1_rx_empty as u8) {
                break;
            }

            let mut header = 0;
            let mut payload = self.rx_message_buf[self.rx_message_index];
            self.read_message(&mut header, &mut payload[..]);

            let status0 = self.read_register(reg_status0 as u8);
            if ((status0 & status0_crc_chk as u8) == 0) {
                debug!("Invalid CRC");
            } else if (pd_header(header).message_type() == pd_msg_type_ctrl_good_crc) {
                debug!("Good CRC packet");
            } else {
                if (self.state_ != fusb302_state::usb_pd) {
                    self.establish_usb_pd();
                }
                self.events.push_front(event {
                    msg_header: header,
                    msg_payload: &payload[0] as *const u8,
                    kind: event_kind::message_received,
                });
                self.rx_message_index += 1;
                if (self.rx_message_index >= NUM_MESSAGE_BUF) {
                    self.rx_message_index = 0;
                }
            }
        }
    }

    fn establish_retry_wait(&mut self) {
        debug!("Reset");

        // Reset FUSB302
        self.init();
        self.state_ = fusb302_state::usb_retry_wait;
        self.start_timeout(500);
        self.events.push_front(event {
            kind: event_kind::state_changed,
            msg_header: 0,
            msg_payload: &0u8 as *const u8,
        });
    }

    fn establish_usb_20(&mut self) {
        self.start_sink();
    }

    fn establish_usb_pd_wait(&mut self, cc: u8) {
        // Enable automatic retries
        self.write_register(
            reg_control3,
            control3_auto_retry as u8 | control3_3_retries as u8,
        );
        // Enable interrupts for CC activity and CRC_CHK
        self.write_register(
            reg_mask,
            mask_m_all as u8 & !(mask_m_activity as u8 | mask_m_crc_chk as u8),
        );
        // Unmask all interrupts (toggle done, hard reset, tx sent etc.)
        self.write_register(reg_maska, maska_m_none as u8);
        // Enable good CRC sent interrupt
        self.write_register(reg_maskb, maskb_m_none);
        // Enable pull down and CC monitoring
        self.write_register(
            reg_switches0,
            switches0_pdwn1 as u8
                | switches0_pdwn2 as u8
                | (if cc == 1 {
                    switches0_meas_cc1 as u8
                } else {
                    switches0_meas_cc2 as u8
                }),
        );
        // Configure: auto CRC and BMC transmit on CC pin
        self.write_register(
            reg_switches1,
            switches1_specrev_rev_2_0 as u8
                | switches1_auto_crc as u8
                | (if cc == 1 {
                    switches1_txcc1 as u8
                } else {
                    switches1_txcc2 as u8
                }),
        );

        self.state_ = fusb302_state::usb_pd_wait;
        self.start_timeout(300);
    }

    fn establish_usb_pd(&mut self) {
        self.state_ = fusb302_state::usb_pd;
        self.cancel_timeout();
        debug!("USB PD comm");
        self.events
            .push_front(event {
                kind: event_kind::state_changed,
                msg_header: 0,
                msg_payload: &0u8 as *const u8,
            })
            .ok();
    }

    fn start_timeout(&mut self, ms: u32) {
        self.is_timeout_active = true;
        self.timeout_expiration = self.timestamp + ms;
    }

    fn has_timeout_expired(&mut self) -> bool {
        if (!self.is_timeout_active) {
            return false;
        }

        let delta = self.timeout_expiration - self.timestamp;
        if (delta <= 0x8000000) {
            return false;
        }

        self.is_timeout_active = false;
        return true;
    }

    fn cancel_timeout(&mut self) {
        self.is_timeout_active = false;
    }

    pub fn has_event(&mut self) -> bool {
        !self.events.is_empty()
    }

    pub fn pop_event(&mut self) -> event {
        self.events.pop_back().unwrap()
    }

    fn read_message(&mut self, header: &mut u16, payload: &mut [u8]) -> usize {
        // Read token and header
        let mut buf = [0u8; 3];
        self.pd_ctrl_read(reg_fifos as u8, &mut buf);

        // Check for SOP token
        if ((buf[0] & 0xe0) != 0xe0) {
            // Flush RX FIFO
            self.write_register(reg_control1, control1_rx_flush as u8);
            return 0;
        }

        let mut header_buf = header as *mut u16 as *mut u8;
        unsafe { header_buf.write_unaligned(buf[1]) };
        unsafe { header_buf.add(1).write_unaligned(buf[2]) };

        // Get payload and CRC length
        let len = pd_header(*header).num_data_objs() * 4;
        self.pd_ctrl_read(reg_fifos as u8, &mut payload[..len + 4]);

        return len;
    }

    fn send_header_message(&mut self, msg_type: pd_msg_type) {
        let header = pd_header::create_ctrl(msg_type, 1);
        self.send_message(header.0, &[]);
    }

    pub fn send_message(&mut self, mut header: u16, payload: &[u8]) {
        // Enable internal oscillator
        self.write_register(reg_power, power_pwr_all as u8);

        let payload_len = pd_header(header).num_data_objs() * 4;
        header |= (self.next_message_id << 9);

        let mut buf = [0u8; 40];

        // Create token stream
        buf[0] = reg_fifos as u8;
        buf[1] = token_sop1 as u8;
        buf[2] = token_sop1 as u8;
        buf[3] = token_sop1 as u8;
        buf[4] = token_sop2 as u8;
        buf[5] = (token_packsym as u8 | (payload_len + 2) as u8);
        buf[6] = (header & 0xff) as u8;
        buf[7] = (header >> 8) as u8;
        if (payload_len > 0) {
            for i in 0..payload.len() {
                buf[8 + i] = payload[i];
            }
        }
        let mut n = 8 + payload_len;
        buf[n] = token_jam_crc as u8;
        n += 1;
        buf[n] = token_eop as u8;
        n += 1;
        buf[n] = token_txoff as u8;
        n += 1;
        buf[n] = token_txon as u8;
        n += 1;

        debug!("buf_len: {}", n);

        self.pd_ctrl_write(&mut buf[..n]);

        self.next_message_id += 1;
        if (self.next_message_id == 8) {
            self.next_message_id = 0;
        }
    }

    fn read_register(&mut self, r: u8) -> u8 {
        let mut val = [0];
        self.pd_ctrl_read(r, &mut val);
        val[0]
    }

    fn write_register(&mut self, r: reg, value: u8) {
        self.pd_ctrl_write(&mut [r as u8, value]);
    }

    pub fn pd_ctrl_read(&mut self, reg: u8, data: &mut [u8]) {
        self.i2c.write_read(0x22, &[reg], data).ok();
    }

    pub fn pd_ctrl_write(&mut self, data: &mut [u8]) {
        self.i2c.write(0x22, data).ok();
    }
}
