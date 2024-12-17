#![no_std]

use {
    crate::{
        registers::{
            Control0, Control1, Control3, Mask1, MaskA, MaskB, Power, Register, Registers, Reset,
            Slice, Switches0, Switches1,
        },
        timeout::Timeout,
    },
    defmt::{debug, warn},
    embassy_time::{Duration, Instant},
    embedded_hal_async::i2c::I2c,
    usb_pd::{
        header::{ControlMessageType, Header, MessageType},
        messages::Message,
        sink::{Driver as SinkDriver, DriverState},
        token::Token,
        CcPin,
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

    /// Pending message received by driver
    message: Option<Message>,

    /// Current driver state
    state: State,
}

enum State {
    Measuring {
        /// CC line being measured
        cc_pin: CcPin,
    },
    Ready,
    Connected {
        /// ID for next USB PD message
        message_id: MessageIdCounter,
    },
    RetryWait,
}

impl From<&State> for DriverState {
    fn from(value: &State) -> Self {
        match value {
            State::Measuring { .. } => DriverState::Usb20,
            State::Ready => DriverState::UsbPdWait,
            State::Connected { .. } => DriverState::UsbPd,
            State::RetryWait => DriverState::UsbRetryWait,
        }
    }
}

/// Types of receive errors.
///
/// FIXME: Implement error handling.
pub enum RxError {}

/// Types of transmit errors.
///
/// FIXME: Implement error handling.
pub enum TxError {}

impl<I2C: I2c> SinkDriver for Fusb302b<I2C> {
    type RxError = RxError;
    type TxError = TxError;

    async fn init(&mut self) {
        // full reset
        self.registers
            .set_reset(Reset::default().with_pd_reset(true).with_sw_reset(true))
            .await;

        for _ in 0..1_000_000 {
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        }

        // power up everyting except oscillator
        self.registers
            .set_power(
                Power::default()
                    .with_bandgap_wake(true)
                    .with_measure_block(true)
                    .with_receiver(true),
            )
            .await;

        // Disable all CC monitoring
        self.registers.set_switches0(Switches0::default()).await;

        // Mask all interrupts
        self.registers
            .set_mask1(
                Mask1::default()
                    .with_m_activity(true)
                    .with_m_alert(true)
                    .with_m_bc_lvl(true)
                    .with_m_collision(true)
                    .with_m_comp_chng(true)
                    .with_m_crc_chk(true)
                    .with_m_vbusok(true)
                    .with_m_wake(true),
            )
            .await;

        // Mask all interrupts
        self.registers
            .set_mask_a(
                MaskA::default()
                    .with_m_hardrst(true)
                    .with_m_hardsent(true)
                    .with_m_ocp_temp(true)
                    .with_m_retryfail(true)
                    .with_m_softfail(true)
                    .with_m_softrst(true)
                    .with_m_togdone(true)
                    .with_m_txsent(true),
            )
            .await;

        // Mask all interrupts (incl. good CRC sent)
        self.registers
            .set_mask_b(MaskB::default().with_m_gcrcsent(true))
            .await;

        self.registers
            .set_control0(Control0::default().with_int_mask(false).with_host_cur(0b01))
            .await;
        self.registers
            .set_control3(
                Control3::default()
                    .with_send_hard_reset(true)
                    .with_auto_hardreset(true)
                    .with_auto_softreset(true)
                    .with_auto_retry(true)
                    .with_n_retries(3),
            )
            .await;
        self.state = State::Measuring { cc_pin: CcPin::CC1 };
        self.message = None;

        self.start_sink().await;
    }

    async fn receive_message(&mut self, now: Instant) -> Result<Option<Message>, Self::RxError> {
        self.timeout.update(now);

        self.check_for_interrupts().await;

        match self.state {
            State::Measuring { .. } => {
                if self.timeout.is_expired() {
                    self.check_measurement().await;
                }
            }
            State::Ready => {
                if self.timeout.is_expired() {
                    debug!("ready: timeout expired, establishing retry wait");
                    self.establish_retry_wait().await;
                }
            }
            State::Connected { .. } => (),
            State::RetryWait => {
                if self.timeout.is_expired() {
                    debug!("retry wait: timeout expired, resetting");
                    self.establish_usb_20().await;
                }
            }
        }

        Ok(self.message.take())
    }

    async fn send_message(
        &mut self,
        mut header: Header,
        payload: &[u8],
    ) -> Result<(), Self::TxError> {
        let State::Connected { message_id } = &mut self.state else {
            panic!();
        };

        // Enable internal oscillator
        self.registers
            .set_power(
                Power::default()
                    .with_bandgap_wake(true)
                    .with_internal_oscillator(true)
                    .with_measure_block(true)
                    .with_receiver(true),
            )
            .await;

        assert_eq!(header.num_objects() * 4, payload.len());

        header.set_message_id(message_id.next());

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

        self.registers
            .write_raw(&mut buf[..12 + payload.len()])
            .await;

        message_id.inc();

        Ok(())
    }

    fn state(&mut self) -> DriverState {
        (&self.state).into()
    }
}

impl<I2C: I2c> Fusb302b<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            registers: Registers::new(i2c),
            timeout: Timeout::new(),
            message: None,
            state: State::Measuring { cc_pin: CcPin::CC1 },
        }
    }

    async fn start_sink(&mut self) {
        // As the interrupt line is also used as SWDIO, the FUSB302B interrupt is
        // not activated until activity on CC1 or CC2 has been detected.
        // Thus, CC1 and CC2 have to be polled manually even though the FUSB302B
        // could do it automatically.

        // BMC threshold: 1.35V with a threshold of 85mV
        self.registers
            .set_slice(Slice::default().with_sdac(0x20).with_sda_hys(0b01))
            .await;

        self.start_measurement().await;
    }

    async fn start_measurement(&mut self) {
        let State::Measuring { cc_pin } = self.state else {
            panic!();
        };

        let mut switches0 = Switches0::default().with_pdwn1(true).with_pdwn2(true);

        match cc_pin {
            CcPin::CC1 => switches0.set_meas_cc1(true),
            CcPin::CC2 => switches0.set_meas_cc2(true),
        }

        // test CC
        self.registers.set_switches0(switches0).await;
        self.timeout.start(Duration::from_millis(10));
    }

    async fn check_measurement(&mut self) {
        let State::Measuring { ref mut cc_pin } = self.state else {
            panic!();
        };

        if self.registers.status0().await.bc_lvl() == 0 {
            // No CC activity
            *cc_pin = !*cc_pin;
            self.start_measurement().await;
            return;
        }

        self.establish_usb_pd_wait().await;
    }

    async fn check_for_interrupts(&mut self) {
        let mut may_have_message = false;

        let interrupt = self.registers.interrupt().await;
        let interrupta = self.registers.interrupta().await;
        let interruptb = self.registers.interruptb().await;

        if interrupta.i_hardrst() {
            debug!("Hard reset");
            self.establish_retry_wait().await;
            return;
        }
        if interrupta.i_retryfail() {
            debug!("Retry failed");
        }
        if interrupta.i_txsent() {
            debug!("TX ack");
            // turn off internal oscillator if TX FIFO is empty
            if self.registers.status1().await.tx_empty() {
                let power = self.registers.power().await.with_internal_oscillator(false);
                self.registers.set_power(power).await;
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
            self.check_for_msg().await;
        }
    }

    async fn check_for_msg(&mut self) {
        while !self.registers.status1().await.rx_empty() {
            let mut header = 0;
            let mut payload = [0; 64];
            self.read_message(&mut header, &mut payload[..]).await;

            if !self.registers.status0().await.crc_chk() {
                debug!("Invalid CRC");
            } else if Header(header).message_type()
                == MessageType::Control(ControlMessageType::GoodCRC)
            {
                debug!("Good CRC packet");
            } else {
                if let State::Connected { .. } = self.state {
                } else {
                    self.establish_usb_pd();
                }

                let message = Message::parse(Header(header), &payload[..]);

                debug!("{:?}, {:x}:{:x}", message, header, payload);

                if self.message.replace(message).is_some() {
                    panic!("pending message already set");
                }
            }
        }
    }

    async fn establish_retry_wait(&mut self) {
        debug!("Reset");

        // Reset FUSB302
        self.init().await;
        self.state = State::RetryWait;
        self.timeout.start(Duration::from_millis(500));
    }

    async fn establish_usb_20(&mut self) {
        // debug!("EU20");
        // Timeouts crash without this...
        // self.state = State::Measuring { cc_pin: CcPin::CC1 };
        // self.init();
        // self.timeout.start(Duration::millis(500));
        // self.events.enqueue(DriverEvent::StateChanged).ok().unwrap();

        self.start_sink().await;
    }

    async fn establish_usb_pd_wait(&mut self) {
        let State::Measuring { cc_pin } = self.state else {
            panic!();
        };

        // Enable automatic retries
        self.registers
            .set_control3(
                Control3::default()
                    .with_send_hard_reset(true)
                    .with_auto_hardreset(true)
                    .with_auto_softreset(true)
                    .with_auto_retry(true)
                    .with_n_retries(3),
            )
            .await;

        // Enable interrupts for CC activity and CRC_CHK
        self.registers
            .set_mask1(
                Mask1::default()
                    .with_m_activity(false)
                    .with_m_alert(true)
                    .with_m_bc_lvl(true)
                    .with_m_collision(true)
                    .with_m_comp_chng(true)
                    .with_m_crc_chk(false)
                    .with_m_vbusok(true)
                    .with_m_wake(true),
            )
            .await;

        // Unmask all interrupts (toggle done, hard reset, tx sent etc.)
        self.registers.set_mask_a(MaskA::default()).await;

        // Enable good CRC sent interrupt
        self.registers.set_mask_b(MaskB::default()).await;

        // Enable pull down and CC monitoring
        let switches0 = {
            let switches0 = Switches0(0).with_pdwn1(true).with_pdwn2(true);
            match cc_pin {
                CcPin::CC1 => switches0.with_meas_cc1(true),
                CcPin::CC2 => switches0.with_meas_cc2(true),
            }
        };

        self.registers.set_switches0(switches0).await;

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
        self.registers.set_switches1(switches1).await;

        self.state = State::Ready;
        self.timeout.start(Duration::from_millis(300u64))
    }

    fn establish_usb_pd(&mut self) {
        self.state = State::Connected {
            message_id: MessageIdCounter::default(),
        };
        self.timeout.cancel();
        debug!("USB PD comm");
    }

    async fn read_message(&mut self, header: &mut u16, payload: &mut [u8]) {
        // Read token and header
        let mut buf = [0u8; 3];
        self.registers.read_fifo(&mut buf).await;

        // Check for SOP token
        if (buf[0] & 0xe0) != 0xe0 {
            // Flush RX FIFO
            self.registers
                .set_control1(Control1::default().with_rx_flush(true))
                .await;
            warn!("Flushed RX buffer");
            return;
        }

        let header_buf = header as *mut u16 as *mut u8;
        unsafe { header_buf.write_unaligned(buf[1]) };
        unsafe { header_buf.add(1).write_unaligned(buf[2]) };

        // Get payload and CRC length
        let len = Header(*header).num_objects() * 4;
        self.registers.read_fifo(&mut payload[..len + 4]).await;
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
