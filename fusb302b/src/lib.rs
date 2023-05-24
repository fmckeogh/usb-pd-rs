#![no_std]

use usb_pd::token::Token;

use {
    crate::{
        registers::{
            Control1, Control3, Mask1, MaskA, MaskB, Power, Registers, Reset, Revision, Slice,
            Switches0, Switches1,
        },
        timeout::Timeout,
    },
    byteorder::{BigEndian, ByteOrder, LittleEndian},
    defmt::{error, trace, Format},
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    fixed_queue::VecDeque,
    fugit::ExtU64,
    usb_pd::{
        header::{ControlMessageType, DataMessageType, Header, MessageType},
        message::Message,
        pdo::FixedVariableRequestDataObject,
        CcPin,
    },
};

pub mod callback;
pub mod registers;
pub mod timeout;

type Instant = fugit::Instant<u64, 1, 1000>;
type Duration = fugit::Duration<u64, 1, 1000>;

/// I2C address of FUSB302BMPX
const DEVICE_ADDRESS: u8 = 0b0100010;

/// FUSB302B Programmable USB Type‐C Controller w/PD
pub struct Fusb302b<I2C> {
    registers: Registers<I2C>,
    state: State,
    callback: callback::Function,
    message_id_counter: MessageIdCounter,
}

/// Current state of the FUSB302B
enum State {
    /// Measuring CC1/CC2 for activity
    MeasuringCc { cc: CcPin, timeout: Timeout },
    /// Activity detected, ready for first message
    Ready { timeout: Timeout },
    /// Connected
    Connected { events: VecDeque<Event, 5> },
    /// Wait period after a failure
    RetryWait { timeout: Timeout },
}

impl State {
    pub fn initial() -> Self {
        Self::MeasuringCc {
            cc: CcPin::CC1,
            timeout: Timeout::new(),
        }
    }
}

/// Internal event to be processed after higher priority work is completed
#[derive(Clone, Copy, Format)]
enum Event {
    MessageReceived(Message),
}

impl<I2C: Read + Write + WriteRead> Fusb302b<I2C> {
    pub fn new(i2c: I2C, callback: callback::Function) -> Self {
        Self {
            registers: Registers::new(i2c),
            state: State::initial(),
            callback,
            message_id_counter: MessageIdCounter::default(),
        }
    }

    pub fn init(&mut self, now: Instant) {
        self.registers
            .set_reset(Reset::default().with_pd_reset(true).with_sw_reset(true));

        // power up everyting except oscillator
        self.registers.set_power(
            Power::default()
                .with_bandgap_wake(true)
                .with_measure_block(true)
                .with_receiver(true)
                .with_internal_oscillator(true),
        );

        // disable CC monitoring
        self.registers.set_switches0(Switches0(0));

        // mask all interrupts
        self.registers.set_mask1(Mask1(0xFF));
        self.registers.set_mask_a(MaskA(0xFF));
        self.registers
            .set_mask_b(MaskB::default().with_m_gcrcsent(true));

        // As the interrupt line is also used as SWDIO, the FUSB302B interrupt is
        // not activated until activity on CC1 or CC2 has been detected.
        // Thus, CC1 and CC2 have to be polled manually even though the FUSB302B
        // could do it automatically.

        // BMC threshold: 1.35V with a threshold of 85mV
        self.registers
            .set_slice(Slice::default().with_sda_hys(0b01).with_sdac(0x20));

        self.start_measure_cc(now);
    }

    pub fn poll(&mut self, now: Instant) {
        loop {
            self.check_interrupts(now);
            self.check_timeouts(now);

            if let State::Connected { events } = &mut self.state {
                if let Some(event) = events.pop_front() {
                    let callback_event = match event {
                        Event::MessageReceived(Message::SourceCapabilities(
                            source_capabilities,
                        )) => callback::Event::SourceCapabilities {
                            source_capabilities,
                        },
                        _ => todo!(),
                    };
                    self.process_callback(callback_event);
                }
            }
        }
    }

    /// Starts the measurement of a CC pin, alternating each time this method is called
    fn start_measure_cc(&mut self, now: Instant) {
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

        self.registers.set_switches0(switches0);
    }

    /// Tests whether the current CC pin being measured is active
    fn is_cc_active(&mut self, now: Instant) -> Option<CcPin> {
        let State::MeasuringCc { cc,.. } = self.state else {
            return None;
        };

        if self.registers.status0().bc_lvl() == 0 {
            self.start_measure_cc(now);
            return None;
        }

        Some(cc)
    }

    fn set_state_ready(&mut self, now: Instant) {
        // Enable automatic retries
        self.registers.set_control3(
            Control3::default()
                .with_auto_retry(true)
                .with_n_retries(0b11),
        );

        // // Enable interrupts for CC activity and CRC_CHK
        //write_register(reg_mask, mask_m_all & ~(mask_m_activity | mask_m_crc_chk));
        self.registers.set_mask1(Mask1(255 & !(64 | 16)));

        // // Unmask all interrupts (toggle done, hard reset, tx sent etc.)
        // write_register(reg_maska, maska_m_none);
        self.registers.set_mask_a(MaskA(0));

        // // Enable good CRC sent interrupt
        // write_register(reg_maskb, maskb_m_none);
        self.registers.set_mask_b(MaskB(0));

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

        self.registers.set_switches1(switches1);

        // // Enable interrupt
        // write_register(reg_control0, control0_none);

        self.state = State::Ready {
            timeout: Timeout::new_start(now, 500.millis()),
        };
    }

    fn set_state_retrywait(&mut self, now: Instant) {
        self.state = State::RetryWait {
            timeout: Timeout::new_start(now, 500.millis()),
        };
        self.process_callback(callback::Event::ProtocolChanged {
            protocol: callback::Protocol::_20,
        });
    }

    fn set_state_connected(&mut self) {
        self.state = State::Connected {
            events: VecDeque::new(),
        };
        self.process_callback(callback::Event::ProtocolChanged {
            protocol: callback::Protocol::PD,
        });
    }

    fn check_interrupts(&mut self, now: Instant) {
        let interrupt = self.registers.interrupt();
        let interrupt_a = self.registers.interrupta();
        let interrupt_b = self.registers.interruptb();

        let mut message_pending = false;

        if interrupt_a.i_hardrst() {
            error!("hard reset");
            self.set_state_retrywait(now);
            return;
        }

        if interrupt_a.i_retryfail() {
            error!("retry failed");
        }

        if interrupt_a.i_txsent() {
            trace!("tx ack");

            // turn off internal oscillator if TX FIFO is empty
            // if self.registers.status1().tx_empty() {
            //     let power = self.registers.power().with_internal_oscillator(false);
            //     self.registers.set_power(power);
            // }
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
            self.read_messages();
        }
    }

    fn read_messages(&mut self) {
        while !self.registers.status1().rx_empty() {
            trace!("rx not empty, reading messages");

            // read message
            let mut buf = [0u8; 3];
            self.registers.read_fifo(&mut buf);

            if (buf[0] & 0xe0) != 0xe0 {
                self.registers
                    .set_control1(Control1::default().with_rx_flush(true));
            }

            let header = Header::from_bytes(&buf[1..]);

            let mut payload = [0; 64];
            self.registers
                .read_fifo(&mut payload[..header.num_objects() as usize * 4 + 4]);

            // check crc
            if !self.registers.status0().crc_chk() {
                trace!("bad crc");
            } else if header.message_type() == MessageType::Control(ControlMessageType::GoodCRC) {
                trace!("goodcrc packet")
            } else {
                // connect if not connected
                match self.state {
                    State::Connected { .. } => (),
                    _ => self.set_state_connected(),
                }

                let State::Connected { events } = &mut self.state else { panic!() };

                let msg = Message::parse(header, &payload);

                trace!("got msg {:?}", msg);
                events
                    .push_back(Event::MessageReceived(msg))
                    .ok()
                    .expect("failed to push event");
            }
        }
    }

    fn check_timeouts(&mut self, now: Instant) {
        match &mut self.state {
            State::MeasuringCc { timeout, .. } => {
                if timeout.is_expired(now) && self.is_cc_active(now).is_some() {
                    trace!("cc active");
                    self.set_state_ready(now);
                }
            }
            State::Ready { timeout } => {
                if timeout.is_expired(now) {
                    trace!("no PD activity, resetting");
                    self.set_state_retrywait(now);
                }
            }
            State::Connected { .. } => {}
            State::RetryWait { timeout } => {
                if timeout.is_expired(now) {
                    trace!("retry wait expired");
                    self.state = State::initial();
                    self.init(now);
                }
            }
        }
    }

    fn process_callback(&mut self, event: callback::Event) {
        let resp = (self.callback)(event);
        trace!("resp: {:?}", resp);

        match resp {
            Some(callback::Response::Request { index, current }) => {
                self.request_power(index, current);
            }
            None => return,
        }
    }

    fn request_power(&mut self, index: usize, mut current: u16) {
        let payload = {
            let mut payload = [0u8; 4];

            let no_usb_suspend = 1;
            let usb_comm_capable = 2;

            if current > 0x3ff {
                current = 0x3ff;
            }
            payload[0] = u8::try_from(current & 0xff).unwrap();
            payload[1] = u8::try_from(((current >> 8) & 0x03) | ((current << 2) & 0xfc)).unwrap();
            payload[2] = u8::try_from((current >> 6) & 0x0f).unwrap();
            payload[3] =
                u8::try_from((index & 0x07) << 4 | no_usb_suspend | usb_comm_capable).unwrap();

            payload
        };

        let header = ((1 & 0x07) << 12) | (130 & 0x1f) | 0x40 | ((2 - 1) << 6);
        // let header = usb_pd::header::Header(0)
        //     .with_message_type_raw(DataMessageType::Request as u8)
        //     .with_num_objects(1)
        //     .with_spec_revision(usb_pd::header::SpecificationRevision::R3_0)
        //     .with_port_power_role(usb_pd::PowerRole::Sink)
        //     .with_message_id(self.message_id_counter.next());

        self.send_message(header, payload.as_slice());
    }

    fn send_message(&mut self, header: u16, payload: &[u8]) {
        let payload_length = 1 * 4;

        trace!(
            "{} {}",
            self.registers.status1().tx_empty(),
            self.registers.status1().tx_full()
        );

        self.registers.write_fifo(&mut [
            0x43, 18, 18, 18, 19, 134, 194, 16, 44, 177, 4, 19, 255, 20, 254, 161,
        ]);

        trace!(
            "{} {}",
            self.registers.status1().tx_empty(),
            self.registers.status1().tx_full()
        );

        // let control0 = self.registers.control0().with_tx_start(true);
        // self.registers.set_control0(control0);

        // self.registers.write_fifo(&mut [
        //     0x43,
        //     Token::Sop1 as u8,
        //     Token::Sop1 as u8,
        //     Token::Sop1 as u8,
        //     Token::Sop2 as u8,
        //     Token::PackSym as u8 | (payload_length + 2),
        // ]);

        // self.registers.write_fifo(&mut [
        //     0x43,
        //     u8::try_from(header & 0xff).unwrap(),
        //     u8::try_from(header >> 8).unwrap(),
        //     payload[0],
        //     payload[1],
        //     payload[2],
        //     payload[3],
        // ]);

        // trace!(
        //     "{} {}",
        //     self.registers.status1().tx_empty(),
        //     self.registers.status1().tx_full()
        // );

        // self.registers.write_fifo(&mut [
        //     0x43,
        //     Token::JamCrc as u8,
        //     Token::Eop as u8,
        //     Token::TxOff as u8,
        //     Token::TxOn as u8,
        // ]);

        let control0 = self.registers.control0().with_tx_start(true);
        self.registers.set_control0(control0);

        trace!(
            "{} {}",
            self.registers.status1().tx_empty(),
            self.registers.status1().tx_full()
        );

        trace!("sent message");

        self.message_id_counter.inc();
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
