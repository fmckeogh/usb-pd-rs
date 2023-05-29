#![no_std]

use {
    crate::{
        registers::{
            Control1, Control3, Mask1, MaskA, MaskB, Power, Register, Registers, Reset, Slice,
            Switches0, Switches1,
        },
        timeout::Timeout,
    },
    defmt::{debug, trace, warn},
    embedded_hal::blocking::i2c::{Write, WriteRead},
    fixed_queue::VecDeque,
    usb_pd::{
        header::{ControlMessageType, Header, MessageType},
        message::Message,
        sink::{Driver as SinkDriver, Event, State},
        token::Token,
        CcPin, Duration, Instant,
    },
};

pub mod registers;
mod timeout;

/// I2C address of FUSB302BMPX
const DEVICE_ADDRESS: u8 = 0b0100010;

const NUM_MESSAGE_BUF: usize = 4;

pub struct Fusb302b<I2C> {
    registers: Registers<I2C>,

    /// CC line being measured
    cc_pin: CcPin,

    /// Driver timeout logic
    timeout: Timeout,

    /// RX message buffers
    rx_message_buf: [[u8; 64]; NUM_MESSAGE_BUF],

    /// Next RX message index
    rx_message_index: usize,

    /// Queue of event that have occurred
    events: VecDeque<Event, 4>,

    /// Current attachment state
    state_: State,

    /// ID for next USB PD message
    next_message_id: u8,
}

impl<I2C: Write + WriteRead> SinkDriver for Fusb302b<I2C> {
    fn init(&mut self) {
        // full reset
        self.registers
            .set_reset(Reset::default().with_pd_reset(true).with_sw_reset(true));

        for _ in 0..1_000_000 {
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        }

        // power up everyting except oscillator
        self.registers.set_power(
            Power::default()
                .with_bandgap_wake(true)
                .with_measure_block(true)
                .with_receiver(true),
        );
        // Disable all CC monitoring
        self.registers.set_switches0(Switches0(0));

        // Mask all interrupts
        self.registers.set_mask1(
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
        self.registers.set_mask_a(
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
        self.registers
            .set_mask_b(MaskB::default().with_m_gcrcsent(true));

        self.next_message_id = 0;
        self.state_ = State::Usb20;
        self.events.clear();

        self.start_sink();
    }

    fn poll(&mut self, now: Instant) {
        self.timeout.update(now);

        self.check_for_interrupts();

        if self.timeout.is_expired() {
            if self.state_ == State::UsbPdWait {
                debug!("Timeout expired, no CC activity");
                self.establish_retry_wait();
            } else if self.state_ == State::Usb20 {
                self.check_measurement();
            } else if self.state_ == State::UsbRetryWait {
                self.establish_usb_20();
            }
        }
    }

    fn get_event(&mut self) -> Option<Event> {
        self.events.pop_back()
    }

    fn send_message(&mut self, mut header: Header, payload: &[u8]) {
        // Enable internal oscillator
        self.registers.set_power(
            Power::default()
                .with_bandgap_wake(true)
                .with_internal_oscillator(true)
                .with_measure_block(true)
                .with_receiver(true),
        );

        let payload_len = header.num_objects() as usize * 4;

        header.set_message_id(self.next_message_id as u8);

        let mut buf = [0u8; 40];

        // Create token stream
        buf[0] = Register::Fifo as u8;
        buf[1] = Token::Sop1 as u8;
        buf[2] = Token::Sop1 as u8;
        buf[3] = Token::Sop1 as u8;
        buf[4] = Token::Sop2 as u8;
        buf[5] = Token::PackSym as u8 | (payload_len + 2) as u8;
        header.to_bytes(&mut buf[6..=7]);
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

        self.registers.write_raw(&mut buf[..n]);

        self.next_message_id = self.next_message_id.wrapping_add(1);
    }

    fn state(&mut self) -> State {
        self.state_
    }
}

impl<I2C: Write + WriteRead> Fusb302b<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            registers: Registers::new(i2c),
            cc_pin: CcPin::CC1,
            timeout: Timeout::new(),
            rx_message_buf: [[0u8; 64]; NUM_MESSAGE_BUF],
            rx_message_index: 0,
            events: VecDeque::new(),
            state_: State::Usb20,
            next_message_id: 0,
        }
    }

    pub fn start_sink(&mut self) {
        // As the interrupt line is also used as SWDIO, the FUSB302B interrupt is
        // not activated until activity on CC1 or CC2 has been detected.
        // Thus, CC1 and CC2 have to be polled manually even though the FUSB302B
        // could do it automatically.

        // BMC threshold: 1.35V with a threshold of 85mV
        self.registers
            .set_slice(Slice::default().with_sdac(0x20).with_sda_hys(01));

        self.start_measurement(CcPin::CC1);
    }

    fn start_measurement(&mut self, cc: CcPin) {
        let mut switches0 = Switches0::default().with_pdwn1(true).with_pdwn2(true);
        match cc {
            CcPin::CC1 => switches0.set_meas_cc1(true),
            CcPin::CC2 => switches0.set_meas_cc2(true),
        }

        // test CC
        self.registers.set_switches0(switches0);
        self.timeout.start(Duration::millis(10));
        self.cc_pin = cc;
    }

    fn check_measurement(&mut self) {
        let _ = self.registers.status0();
        if self.registers.status0().bc_lvl() == 0 {
            // No CC activity
            self.start_measurement(!self.cc_pin);
            return;
        }

        self.establish_usb_pd_wait(self.cc_pin);
    }

    fn check_for_interrupts(&mut self) {
        let mut may_have_message = false;

        let interrupt = self.registers.interrupt();
        let interrupta = self.registers.interrupta();
        let interruptb = self.registers.interruptb();

        if interrupta.i_hardrst() {
            debug!("Hard reset");
            self.establish_retry_wait();
            return;
        }
        if interrupta.i_retryfail() {
            debug!("Retry failed");
        }
        if interrupta.i_txsent() {
            debug!("TX ack");
            // turn off internal oscillator if TX FIFO is empty
            if self.registers.status1().tx_empty() {
                let power = self.registers.power().with_internal_oscillator(false);
                self.registers.set_power(power);
            }
        }
        if interrupt.i_activity() {
            may_have_message = true;
        }
        if interrupt.i_crc_chk() {
            debug!("CRC ok");
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
            if self.registers.status1().rx_empty() {
                break;
            }

            let mut header = 0;
            let mut payload = self.rx_message_buf[self.rx_message_index];
            self.read_message(&mut header, &mut payload[..]);

            if !self.registers.status0().crc_chk() {
                debug!("Invalid CRC");
            } else if Header(header).message_type()
                == MessageType::Control(ControlMessageType::GoodCRC)
            {
                debug!("Good CRC packet");
            } else {
                if self.state_ != State::UsbPd {
                    self.establish_usb_pd();
                }
                let message = Message::parse(Header(header), &payload[..]);
                trace!("{:?}", message);

                self.events
                    .push_front(Event::MessageReceived(message))
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
        self.state_ = State::UsbRetryWait;
        self.timeout.start(Duration::millis(500));
        self.events.push_front(Event::StateChanged).ok().unwrap();
    }

    fn establish_usb_20(&mut self) {
        self.start_sink();
    }

    fn establish_usb_pd_wait(&mut self, cc: CcPin) {
        // Enable automatic retries
        self.registers
            .set_control3(Control3::default().with_auto_retry(true).with_n_retries(3));

        // Enable interrupts for CC activity and CRC_CHK
        self.registers.set_mask1(
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
        self.registers.set_mask_a(MaskA::default());

        // Enable good CRC sent interrupt
        self.registers.set_mask_b(MaskB::default());

        // Enable pull down and CC monitoring
        let switches0 = {
            let switches0 = Switches0(0).with_pdwn1(true).with_pdwn2(true);
            match cc {
                CcPin::CC1 => switches0.with_meas_cc1(true),
                CcPin::CC2 => switches0.with_meas_cc2(true),
            }
        };

        self.registers.set_switches0(switches0);

        // Configure: auto CRC and BMC transmit on CC pin
        let switches1 = {
            let switches1 = Switches1(0)
                .with_auto_src(true)
                .with_specrev(crate::registers::Revision::R2_0);
            match cc {
                CcPin::CC1 => switches1.with_txcc1(true),
                CcPin::CC2 => switches1.with_txcc2(true),
            }
        };
        self.registers.set_switches1(switches1);

        self.state_ = State::UsbPdWait;
        self.timeout.start(Duration::millis(300u64))
    }

    fn establish_usb_pd(&mut self) {
        self.state_ = State::UsbPd;
        self.timeout.cancel();
        debug!("USB PD comm");
        self.events.push_front(Event::StateChanged).ok();
    }

    fn read_message(&mut self, header: &mut u16, payload: &mut [u8]) -> usize {
        // Read token and header
        let mut buf = [0u8; 3];
        self.registers.read_fifo(&mut buf);

        // Check for SOP token
        if (buf[0] & 0xe0) != 0xe0 {
            // Flush RX FIFO
            self.registers
                .set_control1(Control1::default().with_rx_flush(true));
            warn!("Flushed RX buffer");
            return 0;
        }

        let header_buf = header as *mut u16 as *mut u8;
        unsafe { header_buf.write_unaligned(buf[1]) };
        unsafe { header_buf.add(1).write_unaligned(buf[2]) };

        // Get payload and CRC length
        let len = Header(*header).num_objects() as usize * 4;
        self.registers.read_fifo(&mut payload[..len + 4]);

        return len;
    }
}
