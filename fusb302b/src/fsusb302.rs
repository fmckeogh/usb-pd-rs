#![allow(non_camel_case_types)]

use {
    crate::{
        registers::{
            Control1, Control3, Mask1, MaskA, MaskB, Power, Register, Registers, Reset, Slice,
            Switches0, Switches1,
        },
        Instant,
    },
    defmt::{debug, warn},
    embedded_hal::blocking::i2c::{Write, WriteRead},
    fixed_queue::VecDeque,
    usb_pd::{
        header::{ControlMessageType, Header, MessageType},
        token::Token,
    },
};

/// Event kind
pub enum event_kind {
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
    i2c: Registers<I2C>,
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

impl<I2C: Write + WriteRead> Fsusb302<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c: Registers::new(i2c),
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
        self.i2c
            .set_reset(Reset::default().with_pd_reset(true).with_sw_reset(true));

        for _ in 0..1_000_000 {
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        }

        // power up everyting except oscillator
        self.i2c.set_power(
            Power::default()
                .with_bandgap_wake(true)
                .with_measure_block(true)
                .with_receiver(true),
        );
        // Disable all CC monitoring
        self.i2c.set_switches0(Switches0(0));

        // Mask all interrupts
        self.i2c.set_mask1(
            Mask1::default()
                .with_m_activity(true)
                .with_m_alert(true)
                .with_m_bc_lvl(true)
                .with_m_collision(true)
                .with_m_comp_chng(true)
                .with_m_crc_chk(true)
                .with_m_vbusok(true)
                .with_m_wake(true),
        );
        // Mask all interrupts
        self.i2c.set_mask_a(
            MaskA::default()
                .with_m_hardrst(true)
                .with_m_hardsent(true)
                .with_m_ocp_temp(true)
                .with_m_retryfail(true)
                .with_m_softfail(true)
                .with_m_softrst(true)
                .with_m_togdone(true)
                .with_m_txsent(true),
        );

        // Mask all interrupts (incl. good CRC sent)
        self.i2c.set_mask_b(MaskB::default().with_m_gcrcsent(true));

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
        self.i2c
            .set_slice(Slice::default().with_sdac(0x20).with_sda_hys(01));

        self.start_measurement(1);
    }

    pub fn poll(&mut self, now: Instant) {
        self.timestamp = now.ticks() as u32;
        self.check_for_interrupts();

        if self.has_timeout_expired() {
            if self.state_ == fusb302_state::usb_pd_wait {
                debug!("{}: No CC activity", self.timestamp);
                self.establish_retry_wait();
            } else if self.state_ == fusb302_state::usb_20 {
                self.check_measurement();
            } else if self.state_ == fusb302_state::usb_retry_wait {
                self.establish_usb_20();
            }
        }
    }

    fn start_measurement(&mut self, cc: u8) {
        let mut switches0 = Switches0::default().with_pdwn1(true).with_pdwn2(true);
        match cc {
            1 => switches0.set_meas_cc1(true),
            2 => switches0.set_meas_cc2(true),
            _ => todo!(),
        }

        // test CC
        self.i2c.set_switches0(switches0);
        self.start_timeout(10);
        self.measuring_cc = cc;
    }

    fn check_measurement(&mut self) {
        let _ = self.i2c.status0();
        if self.i2c.status0().bc_lvl() == 0 {
            // No CC activity
            self.start_measurement(if self.measuring_cc == 1 { 2 } else { 1 });
            return;
        }

        self.establish_usb_pd_wait(self.measuring_cc);
        self.measuring_cc = 0;
    }

    fn check_for_interrupts(&mut self) {
        let mut may_have_message = false;

        let interrupt = self.i2c.interrupt();
        let interrupta = self.i2c.interrupta();
        let interruptb = self.i2c.interruptb();

        if interrupta.i_hardrst() {
            debug!("{}: Hard reset\r\n", self.timestamp);
            self.establish_retry_wait();
            return;
        }
        if interrupta.i_retryfail() {
            debug!("Retry failed");
        }
        if interrupta.i_txsent() {
            debug!("TX ack");
            // turn off internal oscillator if TX FIFO is empty
            if self.i2c.status1().tx_empty() {
                let power = self.i2c.power().with_internal_oscillator(false);
                self.i2c.set_power(power);
            }
        }
        if interrupt.i_activity() {
            may_have_message = true;
        }
        if interrupt.i_crc_chk() {
            debug!("{}: CRC ok", self.timestamp);
            may_have_message = true;
        }
        if interruptb.i_gcrcsent() {
            debug!("Good CRC sent");
            may_have_message = true;
        }
        if may_have_message {
            self.check_for_msg();
        }
    }

    fn check_for_msg(&mut self) {
        loop {
            if self.i2c.status1().rx_empty() {
                break;
            }

            let mut header = 0;
            let mut payload = self.rx_message_buf[self.rx_message_index];
            self.read_message(&mut header, &mut payload[..]);

            if !self.i2c.status0().crc_chk() {
                debug!("Invalid CRC");
            } else if Header(header).message_type()
                == MessageType::Control(ControlMessageType::GoodCRC)
            {
                debug!("Good CRC packet");
            } else {
                if self.state_ != fusb302_state::usb_pd {
                    self.establish_usb_pd();
                }
                self.events
                    .push_front(event {
                        msg_header: header,
                        msg_payload: &payload[0] as *const u8,
                        kind: event_kind::message_received,
                    })
                    .ok()
                    .unwrap();
                self.rx_message_index += 1;
                if self.rx_message_index >= NUM_MESSAGE_BUF {
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
        self.events
            .push_front(event {
                kind: event_kind::state_changed,
                msg_header: 0,
                msg_payload: &0u8 as *const u8,
            })
            .ok()
            .unwrap();
    }

    fn establish_usb_20(&mut self) {
        self.start_sink();
    }

    fn establish_usb_pd_wait(&mut self, cc: u8) {
        // Enable automatic retries
        self.i2c
            .set_control3(Control3::default().with_auto_retry(true).with_n_retries(3));

        // Enable interrupts for CC activity and CRC_CHK
        self.i2c.set_mask1(
            Mask1::default()
                .with_m_activity(false)
                .with_m_alert(true)
                .with_m_bc_lvl(true)
                .with_m_collision(true)
                .with_m_comp_chng(true)
                .with_m_crc_chk(false)
                .with_m_vbusok(true)
                .with_m_wake(true),
        );

        // Unmask all interrupts (toggle done, hard reset, tx sent etc.)
        self.i2c.set_mask_a(MaskA::default());

        // Enable good CRC sent interrupt
        self.i2c.set_mask_b(MaskB::default());

        // Enable pull down and CC monitoring
        let mut switches0 = Switches0(0).with_pdwn1(true).with_pdwn2(true);
        match cc {
            1 => {
                switches0.set_meas_cc1(true);
            }
            2 => {
                switches0.set_meas_cc2(true);
            }
            _ => todo!(),
        }
        self.i2c.set_switches0(switches0);

        // Configure: auto CRC and BMC transmit on CC pin
        let mut switches1 = Switches1(0)
            .with_auto_src(true)
            .with_specrev(crate::registers::Revision::R2_0);
        match cc {
            1 => {
                switches1.set_txcc1(true);
            }
            2 => {
                switches1.set_txcc2(true);
            }
            _ => todo!(),
        }
        self.i2c.set_switches1(switches1);

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
        if !self.is_timeout_active {
            return false;
        }

        let delta = self.timeout_expiration - self.timestamp;
        if delta <= 0x8000000 {
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
        self.i2c.read_fifo(&mut buf);

        // Check for SOP token
        if (buf[0] & 0xe0) != 0xe0 {
            // Flush RX FIFO
            self.i2c
                .set_control1(Control1::default().with_rx_flush(true));
            warn!("Flushed RX buffer");
            return 0;
        }

        let header_buf = header as *mut u16 as *mut u8;
        unsafe { header_buf.write_unaligned(buf[1]) };
        unsafe { header_buf.add(1).write_unaligned(buf[2]) };

        // Get payload and CRC length
        let len = Header(*header).num_objects() as usize * 4;
        self.i2c.read_fifo(&mut payload[..len + 4]);

        return len;
    }

    pub fn send_message(&mut self, mut header: u16, payload: &[u8]) {
        // Enable internal oscillator
        self.i2c.set_power(
            Power::default()
                .with_bandgap_wake(true)
                .with_internal_oscillator(true)
                .with_measure_block(true)
                .with_receiver(true),
        );

        let payload_len = Header(header).num_objects() as usize * 4;
        header |= self.next_message_id << 9;

        let mut buf = [0u8; 40];

        // Create token stream
        buf[0] = Register::Fifo as u8;
        buf[1] = Token::Sop1 as u8;
        buf[2] = Token::Sop1 as u8;
        buf[3] = Token::Sop1 as u8;
        buf[4] = Token::Sop2 as u8;
        buf[5] = Token::PackSym as u8 | (payload_len + 2) as u8;
        buf[6] = (header & 0xff) as u8;
        buf[7] = (header >> 8) as u8;
        if payload_len > 0 {
            for i in 0..payload.len() {
                buf[8 + i] = payload[i];
            }
        }
        let mut n = 8 + payload_len;
        buf[n] = Token::JamCrc as u8;
        n += 1;
        buf[n] = Token::Eop as u8;
        n += 1;
        buf[n] = Token::TxOff as u8;
        n += 1;
        buf[n] = Token::TxOn as u8;
        n += 1;

        debug!("buf_len: {}", n);

        self.i2c.write_raw(&mut buf[..n]);

        self.next_message_id += 1;
        if self.next_message_id == 8 {
            self.next_message_id = 0;
        }
    }
}
