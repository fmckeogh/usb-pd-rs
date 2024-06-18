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
    heapless::spsc::Queue,
    usb_pd::{
        header::{ControlMessageType, Header, MessageType},
        message::Message,
        sink::{Driver as SinkDriver, DriverEvent, DriverState},
        token::Token,
        CcPin, Duration, Instant,
    },
};

pub mod registers;
mod timeout;

/// I2C address of FUSB302BMPX
const DEVICE_ADDRESS: u8 = 0b0100010;

pub struct Fusb302b<I2C> {
    /// FUSB302B registers
    registers: Registers<I2C>,

    /// Driver timeout logic
    timeout: Timeout,

    /// Queue of event that have occurred
    events: Queue<DriverEvent, 4>,

    /// Current driver state
    state: State,

    /// ID for next USB PD message
    message_id: MessageIdCounter,
}

#[derive(PartialEq)]
enum State {
    Measuring {
        /// CC line being measured
        cc_pin: CcPin,
    },
    Ready,
    Connected,
    RetryWait,
}

impl From<&State> for DriverState {
    fn from(value: &State) -> Self {
        match value {
            State::Measuring { .. } => DriverState::Usb20,
            State::Ready => DriverState::UsbPdWait,
            State::Connected => DriverState::UsbPd,
            State::RetryWait => DriverState::UsbRetryWait,
        }
    }
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
        self.registers.set_switches0(Switches0::default());

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

        self.message_id = MessageIdCounter::default();
        self.state = State::Measuring { cc_pin: CcPin::CC1 };
        while self.events.dequeue().is_some() {}

        self.start_sink();
    }

    fn poll(&mut self, now: Instant) {
        self.timeout.update(now);

        self.check_for_interrupts();

        match self.state {
            State::Measuring { .. } => {
                if self.timeout.is_expired() {
                    self.check_measurement();
                }
            }
            State::Ready => {
                if self.timeout.is_expired() {
                    debug!("Timeout expired, no CC activity");
                    self.establish_retry_wait();
                }
            }
            State::Connected => (),
            State::RetryWait => {
                if self.timeout.is_expired() {
                    self.establish_usb_20();
                }
            }
        }
    }

    fn get_event(&mut self) -> Option<DriverEvent> {
        self.events.dequeue()
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

        assert_eq!(header.num_objects() * 4, payload.len());

        header.set_message_id(self.message_id.next());

        let mut buf = [0u8; 40];

        // preamble
        buf[0..=5].copy_from_slice(&[
            Register::Fifo as u8,
            Token::Sop1 as u8,
            Token::Sop1 as u8,
            Token::Sop1 as u8,
            Token::Sop2 as u8,
            Token::PackSym as u8 | (payload.len() + 2) as u8,
        ]);

        // header
        header.to_bytes(&mut buf[6..=7]);

        // payload
        buf[8..(payload.len() + 8)].copy_from_slice(payload);

        // end
        buf[8 + payload.len()..12 + payload.len()].copy_from_slice(&[
            Token::JamCrc as u8,
            Token::Eop as u8,
            Token::TxOff as u8,
            Token::TxOn as u8,
        ]);

        self.registers.write_raw(&mut buf[..12 + payload.len()]);

        self.message_id.inc();
    }

    fn state(&mut self) -> DriverState {
        (&self.state).into()
    }
}

impl<I2C: Write + WriteRead> Fusb302b<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            registers: Registers::new(i2c),
            timeout: Timeout::new(),
            events: Queue::new(),
            state: State::Measuring { cc_pin: CcPin::CC1 },
            message_id: MessageIdCounter::default(),
        }
    }

    fn start_sink(&mut self) {
        // As the interrupt line is also used as SWDIO, the FUSB302B interrupt is
        // not activated until activity on CC1 or CC2 has been detected.
        // Thus, CC1 and CC2 have to be polled manually even though the FUSB302B
        // could do it automatically.

        // BMC threshold: 1.35V with a threshold of 85mV
        self.registers
            .set_slice(Slice::default().with_sdac(0x20).with_sda_hys(0b01));

        self.start_measurement();
    }

    fn start_measurement(&mut self) {
        let State::Measuring { cc_pin } = self.state else {
            panic!();
        };

        let mut switches0 = Switches0::default().with_pdwn1(true).with_pdwn2(true);

        match cc_pin {
            CcPin::CC1 => switches0.set_meas_cc1(true),
            CcPin::CC2 => switches0.set_meas_cc2(true),
        }

        // test CC
        self.registers.set_switches0(switches0);
        self.timeout.start(Duration::millis(10));
    }

    fn check_measurement(&mut self) {
        let State::Measuring { ref mut cc_pin } = self.state else {
            panic!();
        };

        let _ = self.registers.status0();
        if self.registers.status0().bc_lvl() == 0 {
            // No CC activity
            *cc_pin = !*cc_pin;
            self.start_measurement();
            return;
        }

        self.establish_usb_pd_wait();
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
        while !self.registers.status1().rx_empty() {
            let mut header = 0;
            let mut payload = [0; 64];
            self.read_message(&mut header, &mut payload[..]);

            if !self.registers.status0().crc_chk() {
                debug!("Invalid CRC");
            } else if Header(header).message_type()
                == MessageType::Control(ControlMessageType::GoodCRC)
            {
                debug!("Good CRC packet");
            } else {
                if self.state != State::Connected {
                    self.establish_usb_pd();
                }
                let message = Message::parse(Header(header), &payload[..]);
                trace!("{:?}", message);

                self.events
                    .enqueue(DriverEvent::MessageReceived(message))
                    .ok()
                    .unwrap();
            }
        }
    }

    fn establish_retry_wait(&mut self) {
        debug!("Reset");

        // Reset FUSB302
        self.init();
        self.state = State::RetryWait;
        self.timeout.start(Duration::millis(500));
        self.events.enqueue(DriverEvent::StateChanged).ok().unwrap();
    }

    fn establish_usb_20(&mut self) {
        self.start_sink();
    }

    fn establish_usb_pd_wait(&mut self) {
        let State::Measuring { cc_pin } = self.state else {
            panic!();
        };

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
            match cc_pin {
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
            match cc_pin {
                CcPin::CC1 => switches1.with_txcc1(true),
                CcPin::CC2 => switches1.with_txcc2(true),
            }
        };
        self.registers.set_switches1(switches1);

        self.state = State::Ready;
        self.timeout.start(Duration::millis(300u64))
    }

    fn establish_usb_pd(&mut self) {
        self.state = State::Connected;
        self.timeout.cancel();
        debug!("USB PD comm");
        self.events.enqueue(DriverEvent::StateChanged).ok();
    }

    fn read_message(&mut self, header: &mut u16, payload: &mut [u8]) {
        // Read token and header
        let mut buf = [0u8; 3];
        self.registers.read_fifo(&mut buf);

        // Check for SOP token
        if (buf[0] & 0xe0) != 0xe0 {
            // Flush RX FIFO
            self.registers
                .set_control1(Control1::default().with_rx_flush(true));
            warn!("Flushed RX buffer");
            return;
        }

        let header_buf = header as *mut u16 as *mut u8;
        unsafe { header_buf.write_unaligned(buf[1]) };
        unsafe { header_buf.add(1).write_unaligned(buf[2]) };

        // Get payload and CRC length
        let len = Header(*header).num_objects() * 4;
        self.registers.read_fifo(&mut payload[..len + 4]);
    }
}

#[derive(Default)]
struct MessageIdCounter(u8);

impl MessageIdCounter {
    pub fn next(&mut self) -> u8 {
        self.0
    }
    pub fn inc(&mut self) {
        self.0 = (self.0 + 1) % 8;
    }
}
