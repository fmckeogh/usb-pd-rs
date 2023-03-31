#![no_std]

use {
    crate::{
        header::{ControlMessageType, Header, MessageType},
        registers::{
            Control1, Control3, Mask1, MaskA, MaskB, Power, Reset, Revision, Slice, Switches0,
            Switches1,
        },
    },
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    fugit::ExtU64,
    log::{error, trace},
};

pub mod header;
pub mod registers;

type Instant = fugit::Instant<u64, 1, 1000>;
type Duration = fugit::Duration<u64, 1, 1000>;

/// I2C address of FUSB302BMPX
const DEVICE_ADDRESS: u8 = 0b0100010;

pub struct Fusb302b<I2C> {
    i2c: I2C,
    state: State,
}

enum State {
    /// Measuring CC1/CC2 for activity
    MeasuringCc { cc: CcPin, timeout: Timeout },
    /// Activity detected, ready for first message
    Ready { timeout: Timeout },
    /// Connected
    Connected,
    /// Wait period after a failure
    RetryWait { timeout: Timeout },
}

impl State {
    pub fn new() -> Self {
        Self::MeasuringCc {
            cc: CcPin::CC1,
            timeout: Timeout::new(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum CcPin {
    CC1,
    CC2,
}

impl<I2C: Read + Write + WriteRead> Fusb302b<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            state: State::new(),
        }
    }

    pub fn init(&mut self, now: Instant) {
        self.set_reset(Reset::default().with_pd_reset(true).with_sw_reset(true));

        // power up everyting except oscillator
        self.set_power(
            Power::default()
                .with_bandgap_wake(true)
                .with_measure_block(true)
                .with_receiver(true),
        );

        // disable CC monitoring
        self.set_switches0(Switches0(0));

        // mask all interrupts
        self.set_mask1(Mask1(0xFF));
        self.set_mask_a(MaskA(0xFF));
        self.set_mask_b(MaskB::default().with_m_gcrcsent(true));

        // As the interrupt line is also used as SWDIO, the FUSB302B interrupt is
        // not activated until activity on CC1 or CC2 has been detected.
        // Thus, CC1 and CC2 have to be polled manually even though the FUSB302B
        // could do it automatically.

        // BMC threshold: 1.35V with a threshold of 85mV
        self.set_slice(Slice::default().with_sda_hys(0b01).with_sdac(0x20));

        self.start_measurement(now);
    }

    pub fn poll(&mut self, now: Instant) {
        self.check_interrupts(now);

        match &mut self.state {
            State::MeasuringCc { timeout, .. } => {
                if timeout.is_expired(now) && self.is_cc_active(now).is_some() {
                    trace!("cc active");
                    self.ready(now);
                }
            }
            State::Ready { timeout } => {
                if timeout.is_expired(now) {
                    trace!("no PD activity, resetting");
                    self.retrywait(now);
                }
            }
            State::Connected => (),
            State::RetryWait { timeout } => {
                if timeout.is_expired(now) {
                    trace!("retry wait expired");
                    self.state = State::new();
                    self.init(now);
                }
            }
        }
    }

    fn start_measurement(&mut self, now: Instant) {
        let State::MeasuringCc { cc, timeout } = &mut self.state else {
            return;
        };

        // toggle current CC being measured
        *cc = match cc {
            CcPin::CC1 => CcPin::CC2,
            CcPin::CC2 => CcPin::CC1,
        };

        let mut switches0 = Switches0::default()
            .with_meas_cc1(false)
            .with_meas_cc2(false)
            .with_pdwn1(true)
            .with_pdwn2(true);

        match cc {
            CcPin::CC1 => switches0.set_meas_cc1(true),
            CcPin::CC2 => switches0.set_meas_cc2(true),
        }

        timeout.start(now, 10.millis());

        self.set_switches0(switches0);
    }

    fn is_cc_active(&mut self, now: Instant) -> Option<CcPin> {
        let State::MeasuringCc { cc,.. } = self.state else {
            return None;
        };

        if self.status0().bc_lvl() == 0 {
            self.start_measurement(now);
            return None;
        }

        Some(cc)
    }

    fn ready(&mut self, now: Instant) {
        // Enable automatic retries
        self.set_control3(
            Control3::default()
                .with_auto_retry(true)
                .with_n_retries(0b11),
        );

        // // Enable interrupts for CC activity and CRC_CHK
        // write_register(reg_mask, mask_m_all & ~(mask_m_activity | mask_m_crc_chk));

        // // Unmask all interrupts (toggle done, hard reset, tx sent etc.)
        // write_register(reg_maska, maska_m_none);

        // // Enable good CRC sent interrupt
        // write_register(reg_maskb, maskb_m_none);

        // Enable pull down and CC monitoring

        // write_register(reg_switches0,
        //                switches0_pdwn1 | switches0_pdwn2 | (cc == 1 ? switches0_meas_cc1 : switches0_meas_cc2));

        // Configure: auto CRC and BMC transmit on CC pin
        let mut switches1 = Switches1::default()
            .with_specrev(Revision::R2_0)
            .with_auto_src(true)
            .with_txcc1(false)
            .with_txcc2(false);

        let State::MeasuringCc { cc, .. } = self.state else {
            error!("attempted to switch to ready when current state was not measuring");
            panic!();
        };
        match cc {
            CcPin::CC1 => switches1.set_txcc1(true),
            CcPin::CC2 => switches1.set_txcc2(true),
        }

        self.set_switches1(switches1);

        // // Enable interrupt
        // write_register(reg_control0, control0_none);

        let mut timeout = Timeout::new();
        timeout.start(now, 500.millis());
        self.state = State::Ready { timeout };
    }

    fn retrywait(&mut self, now: Instant) {
        let mut timeout = Timeout::new();
        timeout.start(now, 500.millis());
        self.state = State::RetryWait { timeout };
        // add event state changed
    }

    fn connected(&mut self) {
        self.state = State::Connected;
        // add event state changed
    }

    fn check_interrupts(&mut self, now: Instant) {
        let interrupt = self.interrupt();
        let interrupt_a = self.interrupta();
        let interrupt_b = self.interruptb();

        let mut message_pending = false;

        if interrupt_a.i_hardrst() {
            error!("hard reset");
            self.retrywait(now);
            return;
        }

        if interrupt_a.i_retryfail() {
            error!("retry failed");
        }

        if interrupt_a.i_txsent() {
            trace!("tx ack");

            // turn off internal oscillator if TX FIFO is empty
            if self.status1().tx_empty() {
                let power = self.power().with_internal_oscillator(false);
                self.set_power(power);
            }
        }

        if interrupt.i_activity() {
            message_pending = true;
        }

        if interrupt.i_crc_chk() {
            trace!("crc ok");
            message_pending = true;
        }

        if interrupt_b.i_gcrcsent() {
            trace!("good crc sent");
            message_pending = true;
        }

        if message_pending {
            self.check_message();
        }
    }

    fn check_message(&mut self) {
        trace!("checking message");

        while !self.status1().rx_empty() {
            // read message
            let mut buf = [0u8; 3];
            self.read_fifo(&mut buf);

            if (buf[0] & 0xe0) != 0xe0 {
                trace!("not SOP packet");
                self.set_control1(Control1::default().with_rx_flush(true));
            }

            trace!("was SOP!");

            let header = Header::from_bytes(&buf[1..]);
            trace!("{:X?}", header);
            trace!("{:X?}", header.message_type());

            let mut payload = [0; 64];
            self.read_fifo(&mut payload[..header.num_objects() as usize * 4 + 4]);

            // check crc
            if !self.status0().crc_chk() {
                trace!("bad crc");
            } else if header.message_type() == MessageType::Control(ControlMessageType::GoodCRC) {
                trace!("goodcrc packet")
            } else {
                // connect if not connected
                match self.state {
                    State::Connected => (),
                    _ => self.connected(),
                }
                // add event (header, payload)
            }
        }
    }

    fn write_register_raw(&mut self, register: u8, value: u8) {
        self.i2c.write(DEVICE_ADDRESS, &[register, value]).ok();
    }

    fn read_register_raw(&mut self, register: u8) -> u8 {
        let mut buffer = [0u8];
        self.i2c
            .write_read(DEVICE_ADDRESS, &[register], &mut buffer)
            .ok();
        buffer[0]
    }
}

/// Timeout wrapper
#[derive(Debug, Default)]
struct Timeout(Option<Instant>);

impl Timeout {
    /// Create a new empty timeout
    fn new() -> Self {
        Self::default()
    }

    /// Start a timeout some duration in the future
    fn start(&mut self, now: Instant, duration: Duration) {
        self.0 = Some(now + duration);
    }

    /// Cancel a timeout
    fn cancel(&mut self) {
        self.0 = None;
    }

    /// Test whether a timeout has expired
    fn is_expired(&mut self, now: Instant) -> bool {
        let Some(timeout) = self.0 else {
            return false;
        };

        // is "now" after the timeout?
        let expired = timeout <= now;

        if expired {
            self.cancel();
        }

        expired
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PowerRole {
    Source,
    Sink,
}

impl From<bool> for PowerRole {
    fn from(value: bool) -> Self {
        match value {
            false => Self::Sink,
            true => Self::Source,
        }
    }
}

impl From<PowerRole> for bool {
    fn from(role: PowerRole) -> bool {
        match role {
            PowerRole::Sink => false,
            PowerRole::Source => true,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum DataRole {
    Ufp,
    Dfp,
}

impl From<bool> for DataRole {
    fn from(value: bool) -> Self {
        match value {
            false => Self::Ufp,
            true => Self::Dfp,
        }
    }
}

impl From<DataRole> for bool {
    fn from(role: DataRole) -> bool {
        match role {
            DataRole::Ufp => false,
            DataRole::Dfp => true,
        }
    }
}
