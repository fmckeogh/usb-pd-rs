//! FUSB302B registers
//!
//! Setters/getters/clearers generated using macros, `Default` for each register is its reset value.

use {
    crate::{callback::Event, Fusb302b, DEVICE_ADDRESS},
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    proc_bitfield::bitfield,
    usb_pd::{DataRole, PowerRole},
};

macro_rules! generate_register_read {
    ($reg:ident, $fn:ident) => {
        pub fn $fn(&mut self) -> $reg {
            self.read_register_raw(Register::$reg as u8).into()
        }
    };
}

macro_rules! generate_register_write {
    ($reg:ident, $fn:ident) => {
        paste::item! {
            pub fn [<set_ $fn>](&mut self, value: $reg) {
                self.write_register_raw(Register::$reg as u8, value.0);
            }
        }
    };
}

macro_rules! generate_register_clear {
    ($reg:ident, $fn:ident) => {
        paste::item! {
            pub fn [<clear_ $fn>](&mut self) {
                self.write_register_raw(Register::$reg as u8, $reg::default().0);
            }
        }
    };
}

macro_rules! generate_register_accessors {
    () => {};

    (($reg:ident, $fn:ident, r), $($tail:tt)*) => {
        generate_register_read!($reg, $fn);

        generate_register_accessors!($($tail)*);
    };

    (($reg:ident, $fn:ident, rw), $($tail:tt)*) => {
        generate_register_read!($reg, $fn);
        generate_register_write!($reg, $fn);

        generate_register_accessors!($($tail)*);
    };

    (($reg:ident, $fn:ident, wc), $($tail:tt)*) => {
        generate_register_write!($reg, $fn);
        generate_register_clear!($reg, $fn);

        generate_register_accessors!($($tail)*);
    };

    (($reg:ident, $fn:ident, rc), $($tail:tt)*) => {
        generate_register_read!($reg, $fn);
        generate_register_clear!($reg, $fn);

        generate_register_accessors!($($tail)*);
    };

    (($reg:ident, $fn:ident, rwc), $($tail:tt)*) => {
        generate_register_read!($reg, $fn);
        generate_register_write!($reg, $fn);
        generate_register_clear!($reg, $fn);

        generate_register_accessors!($($tail)*);
    };
}

impl<I2C: Read + Write + WriteRead, F: FnMut(Event) -> ()> Fusb302b<I2C, F> {
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

    pub fn read_fifo(&mut self, buf: &mut [u8]) {
        self.i2c
            .write_read(DEVICE_ADDRESS, &[Register::Fifo as u8], buf)
            .ok();
    }

    pub fn write_fifo(&mut self, buf: &mut [u8]) {
        buf[0] = Register::Fifo as u8;
        self.i2c.write(DEVICE_ADDRESS, buf).ok();
    }

    generate_register_accessors!(
        (DeviceId, device_id, r),
        (Switches0, switches0, rw),
        (Switches1, switches1, rw),
        (Measure, measure, rw),
        (Slice, slice, rw),
        (Control0, control0, rwc),
        (Control1, control1, rwc),
        (Control2, control2, rw),
        (Control3, control3, rw),
        (Mask1, mask1, rw),
        (Power, power, rw),
        (Reset, reset, wc),
        (OcPreg, ocpreg, rw),
        (MaskA, mask_a, rw),
        (MaskB, mask_b, rw),
        (Control4, control4, rw),
        (Status0A, status0a, r),
        (Status1A, status1a, r),
        (InterruptA, interrupta, rc),
        (InterruptB, interruptb, rc),
        (Status0, status0, r),
        (Status1, status1, r),
        (Interrupt, interrupt, rc),
    );
}

enum Register {
    DeviceId = 0x01,
    Switches0 = 0x02,
    Switches1 = 0x03,
    Measure = 0x04,
    Slice = 0x05,
    Control0 = 0x06,
    Control1 = 0x07,
    Control2 = 0x08,
    Control3 = 0x09,
    Mask1 = 0x0A,
    Power = 0x0B,
    Reset = 0x0C,
    OcPreg = 0x0D,
    MaskA = 0x0E,
    MaskB = 0x0F,
    Control4 = 0x10,
    Status0A = 0x3C,
    Status1A = 0x3D,
    InterruptA = 0x3E,
    InterruptB = 0x3F,
    Status0 = 0x40,
    Status1 = 0x41,
    Interrupt = 0x42,
    Fifo = 0x43,
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct DeviceId(pub u8): Debug, FromRaw, IntoRaw {
        /// Device version ID by Trim or etc
        pub version_id: u8 [read_only] @ 4..=7,
        pub product_id: u8 [read_only] @ 2..=3,
        /// Revision History of each version
        pub revison_id: u8 [read_only] @ 0..=1,
    }
}

impl Default for DeviceId {
    fn default() -> Self {
        Self(0b1001_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Switches0(pub u8): Debug, FromRaw, IntoRaw {
        /// Apply host pull up current to CC2 pin
        pub pu_en2: bool @ 7,
        /// Apply host pull up current to CC1 pin
        pub pu_en1: bool @ 6,
        /// Turn on the VCONN current to CC2 pin
        pub vconn_cc2: bool @ 5,
        /// Turn on the VCONN current to CC1 pin
        pub vconn_cc1: bool @ 4,
        /// Use the measure block to monitor or measure the voltage on CC2
        pub meas_cc2: bool @ 3,
        /// Use the measure block to monitor or measure the voltage on CC1
        pub meas_cc1: bool @ 2,
        /// Device pull down on CC2
        pub pdwn2: bool @ 1,
        /// Device pull down on CC1
        pub pdwn1: bool @ 0,
    }
}

impl Default for Switches0 {
    fn default() -> Self {
        Self(0b0000_0011)
    }
}

/// Bit used for constructing the GoodCRC acknowledge packet
#[derive(Debug, Clone, Copy)]
pub enum Revision {
    R1_0,
    R2_0,
}

impl From<bool> for Revision {
    fn from(value: bool) -> Self {
        match value {
            false => Self::R1_0,
            true => Self::R2_0,
        }
    }
}

impl From<Revision> for bool {
    fn from(revision: Revision) -> bool {
        match revision {
            Revision::R1_0 => false,
            Revision::R2_0 => true,
        }
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Switches1(pub u8): Debug, FromRaw, IntoRaw {
        /// Bit used for constructing the GoodCRC acknowledge packet. This bit corresponds to the
        /// Port Power Role bit in the message header if an SOP packet is received.
        pub powerrole: bool [set PowerRole, get PowerRole] @ 7,
        /// Bit used for constructing the GoodCRC acknowledge packet. These bits correspond to the
        /// Specification Revision bits in the message header.
        pub specrev: bool [set Revision, get Revision] @ 5,
        /// Bit used for constructing the GoodCRC acknowledge packet. This bit corresponds to the
        /// Port Data Role bit in the message header.
        pub datarole: bool [set DataRole, get DataRole] @ 4,
        /// Starts the transmitter automatically when a message with a good CRC is received and
        /// automatically sends a GoodCRC acknowledge packet back to the relevant SOP*
        pub auto_src: bool @ 2,
        /// Enable BMC transmit driver on CC2 pin
        pub txcc2: bool @ 1,
        /// Enable BMC transmit driver on CC1 pin
        pub txcc1: bool @ 0,
    }
}
impl Default for Switches1 {
    fn default() -> Self {
        Self(0b0010_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Measure(pub u8): Debug, FromRaw, IntoRaw {
        /// false: MDAC/comparator measurement is controlled by MEAS_CC* bits
        /// true: Measure VBUS with the MDAC/comparator. This requires MEAS_CC* bits to be 0
        pub meas_vbus: bool @ 6,
        /// Measure Block DAC data input. LSB is equivalent to 42 mV of voltage which is compared
        /// to the measured CC voltage. The measured CC is selected by MEAS_CC2, or MEAS_CC1 bits:
        ///
        /// | `MDAC[5:0]` | `MEAS_VBUS = 0` | `MEAS_VBUS = 1` | Unit |
        /// |-------------|-----------------|-----------------|------|
        /// | `00_0000`   | 0.042           | 0.420           | V    |
        /// | `00_0001`   | 0.084           | 0.840           | V    |
        /// | `11_0000`   | 2.058           | 20.58           | V    |
        /// | `11_0011`   | 2.184           | 21.84           | V    |
        /// | `11_1110`   | 2.646           | 26.46           | V    |
        /// | `11_1111`   | >2.688          | 26.88           | V    |
        /// | `11_1111`   | >2.688          | 26.88           | V    |
        pub mdac: u8 @ 0..=5,
    }
}

impl Default for Measure {
    fn default() -> Self {
        Self(0b0011_0001)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Slice(pub u8): Debug, FromRaw, IntoRaw {
        /// Adds hysteresis where there are now two thresholds, the lower threshold which is always
        /// the value programmed by SDAC\[5:0\] and the higher threshold that is:
        /// * `11`: 255 mV hysteresis: higher threshold = (SDAC value + 20hex)
        /// * `10`: 170 mV hysteresis: higher threshold = (SDAC value + Ahex)
        /// * `01`: 85 mV hysteresis: higher threshold = (SDAC value + 5)
        /// * `00`: No hysteresis: higher threshold = SDAC value
        pub sda_hys: u8 @ 6..=7,
        /// BMC Slicer DAC data input. Allows for a programmable threshold so as to meet the BMC
        /// receive mask under all noise conditions.
        pub sdac: u8 @ 0..=5,
    }
}

impl Default for Slice {
    fn default() -> Self {
        Self(0b0110_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control0(pub u8): Debug, FromRaw, IntoRaw {
        /// Self clearing bit to flush the content of the transmit FIFO
        pub tx_flush: bool [write_only] @ 6,
        /// Masks all interrupts, when false interrupts to host are enabled
        pub int_mask: bool @ 5,
        /// Controls the host pull up current enabled by PU_EN
        ///
        /// * `00`: No current
        /// * `01`: 80 mA – Default USB power
        /// * `10`: 180 mA – Medium Current Mode: 1.5 A
        /// * `11`: 330 mA – High Current Mode: 3 A
        pub host_cur: u8 @ 2..=3,

        /// Starts the transmitter automatically when a message with a good CRC is received. This
        /// allows the software to take as much as 300 mS to respond after the I_CRC_CHK interrupt
        /// is received. Before starting the transmitter, an internal timer waits for approximately
        /// 170 mS before executing the transmit start and preamble
        pub auto_pre: bool @ 1,

        /// Start transmitter using the data in the transmit FIFO. Preamble is started first.
        /// During the preamble period the transmit data can start to be written to the transmit
        /// FIFO. Self clearing.
        pub tx_start: bool @ 0,
    }
}

impl Default for Control0 {
    fn default() -> Self {
        Self(0b0010_0100)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control1(pub u8): Debug, FromRaw, IntoRaw {
        /// Enable SOP''_DEBUG (SOP double prime debug) packets, false for ignore
        pub ensop2db: bool @ 6,
        /// Enable SOP'_DEBUG (SOP prime debug) packets, false for ignore
        pub ensop1db: bool @ 5,
        /// Sent BIST Mode 01s pattern for testing
        pub bist_mode2: bool @ 4,
        /// Self clearing bit to flush the content of the receive FIFO
        pub rx_flush: bool [write_only] @ 2,
        /// Enable SOP'' (SOP double prime) packets, false for ignore
        pub ensop2: bool @ 1,
        /// Enable SOP' (SOP prime) packets, false for ignore
        pub ensop1: bool @ 1,
    }
}

impl Default for Control1 {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control2(pub u8): Debug, FromRaw, IntoRaw {
        /// * `00`: Don’t go into the DISABLE state after one cycle of toggle
        /// * `01`: Wait between toggle cycles for tDIS time of 40 ms
        /// * `10`: Wait between toggle cycles for tDIS time of 80 ms
        /// * `11`: Wait between toggle cycles for tDIS time of 160 ms
        pub tog_save_pwr: u8 @ 6..=7,

        /// * `true`: When TOGGLE=1 only Rd values will cause the TOGGLE state machine to stop toggling and trigger the I_TOGGLE interrupt
        /// * `false`: When TOGGLE=1, Rd and Ra values will cause the TOGGLE state machine to stop toggling
        pub rog_rd_only: bool @ 5,

        /// Enable Wake Detection functionality if the power state is correct
        pub wake_end: bool @ 3,

        /// * `11`: Enable SRC polling functionality if TOGGLE=1
        /// * `10`: Enable SNK polling functionality if TOGGLE=1
        /// * `01`: Enable DRP polling functionality if TOGGLE=1
        /// * `00`: Do Not Use
        pub mode: u8 @ 1..=2,

        /// Enable DRP, SNK or SRC Toggle autonomous functionality
        pub toggle: bool @ 0,
    }
}

impl Default for Control2 {
    fn default() -> Self {
        Self(0b0000_0010)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control3(pub u8): Debug, FromRaw, IntoRaw {
        /// Send a hard reset packet (highest priority)
        pub send_hard_reset: bool [write_only] @ 6,

        /// BIST mode when enabled. Receive FIFO is cleared immediately after sending GoodCRC response.
        ///
        /// Normal operation when disabled; all packets are treated as usual.
        pub bist_tmode: bool @ 5,

        /// Enable automatic hard reset packet if soft reset fail
        pub auto_hardreset: bool @ 4,

        /// Enable automatic soft reset packet if retries fail
        pub auto_softreset: bool @ 3,

        /// * `11`: Three retries of packet (four total packets sent)
        /// * `10`: Two retries of packet (three total packets sent)
        /// * `01`: One retry of packet (two total packets sent)
        /// * `00`: No retries (similar to disabling auto retry)
        pub n_retries: u8 @ 1..=2,

        /// Enable automatic packet retries if GoodCRC is not received
        pub auto_retry: bool @ 0,
    }
}

impl Default for Control3 {
    fn default() -> Self {
        Self(0b0000_0110)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Mask1(pub u8): Debug, FromRaw, IntoRaw {
        /// Mask I_VBUSOK interrupt bit
        pub m_vbusok: bool @ 7,
        /// Mask interrupt for a transition in CC bus activity
        pub m_activity: bool @ 6,
        /// Mask I_COMP_CHNG interrupt for change is the value of COMP, the measure comparator
        pub m_comp_chng: bool @ 5,
        /// Mask interrupt from CRC_CHK bit
        pub m_crc_chk: bool @ 4,
        /// Mask the I_ALERT interrupt bit
        pub m_alert: bool @ 3,
        /// Mask the I_WAKE interrupt bit
        pub m_wake: bool @ 2,
        /// Mask the I_COLLISION interrupt bit
        pub m_collision: bool @ 1,
        /// Mask a change in host requested current level
        pub m_bc_lvl: bool @ 0,
    }
}

impl Default for Mask1 {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Power(pub u8): Debug, FromRaw, IntoRaw {
        /// Enable internal oscillator
        pub internal_oscillator: bool @ 3,
        /// Measure block powered
        pub measure_block: bool @ 2,
        /// Receiver powered and current references for Measure block
        pub receiver: bool @ 1,
        /// Band gap and wake circuit
        pub bandgap_wake: bool @ 0,
    }
}

impl Default for Power {
    fn default() -> Self {
        Self(0b0000_0001)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Reset(pub u8): Debug, FromRaw, IntoRaw {
        /// Reset just the PD logic for both the PD transmitter and receiver
        pub pd_reset: bool @ 1,

        /// Reset the FUSB302B including the I2C registers to their default values
        pub sw_reset: bool @ 0,
    }
}

impl Default for Reset {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct OcPreg(pub u8): Debug, FromRaw, IntoRaw {
        /// * `true`: OCP range between 100−800 mA (max_range = 800 mA)
        /// * `false`: OCP range between 10−80 mA (max_range = 80 mA)
        pub ocp_range: bool @ 3,

        /// * `111`: max_range (see bit definition above for OCP_RANGE)
        /// * `110`: 7 * max_range / 8
        /// * `101`: 6 * max_range / 8
        /// * `100`: 5 * max_range / 8
        /// * `011`: 4 * max_range / 8
        /// * `010`: 3 * max_range / 8
        /// * `001`: 2 * max_range / 8
        /// * `000`: max_range / 8
        pub ocp_cur: u8 @ 0..=2,
    }
}

impl Default for OcPreg {
    fn default() -> Self {
        Self(0b0000_1111)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct MaskA(pub u8): Debug, FromRaw, IntoRaw {
        /// Mask the I_OCP_TEMP interrupt
        pub m_ocp_temp: bool @ 7,
        /// Mask the I_TOGDONE interrupt
        pub m_togdone: bool @ 6,
        /// Mask the I_SOFTFAIL interrupt
        pub m_softfail: bool @ 5,
        /// Mask the I_RETRYFAIL interrupt
        pub m_retryfail: bool @ 4,
        /// Mask the I_HARDSENT interrupt
        pub m_hardsent: bool @ 3,
        /// Mask the I_TXSENT interrupt
        pub m_txsent: bool @ 2,
        /// Mask the I_SOFTRST interrupt
        pub m_softrst: bool @ 1,
        /// Mask the I_HARDRST interrupt
        pub m_hardrst: bool @ 0,
    }
}

impl Default for MaskA {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct MaskB(pub u8): Debug, FromRaw, IntoRaw {
        /// Mask the I_GCRCSENT interrupt
        pub m_gcrcsent: bool @ 0,
    }
}

impl Default for MaskB {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control4(pub u8): Debug, FromRaw, IntoRaw {
        /// In auto Rd only Toggle mode, stop Toggle at Audio accessory (Ra on both CC)
        pub tog_exit_aud: bool @ 0,
    }
}

impl Default for Control4 {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status0A(pub u8): Debug, FromRaw, IntoRaw {
        /// All soft reset packets with retries have failed to get a GoodCRC acknowledge. This
        /// status is cleared when a START_TX, TXON or SEND_HARD_RESET is executed
        pub softfail: bool [read_only] @ 5,

        /// All packet retries have failed to get a GoodCRC acknowledge. This status is cleared
        /// when a START_TX, TXON or SEND_HARD_RESET is executed
        pub retryfail: bool [read_only] @ 4,

        /// Internal power state when logic internals needs to control the power state. POWER3
        /// corresponds to PWR3 bit and POWER2 corresponds to PWR2 bit. The power state is the
        /// higher of both PWR\[3:0\] and {POWER3, POWER2, PWR\[1:0\]} so that if one is 03 and the
        /// other is F then the internal power state is F.
        pub power: u8 [read_only] @ 2..=3,

        /// One of the packets received was a soft reset packet
        pub softrst: bool [read_only] @ 1,

        /// Hard Reset PD ordered set has been received
        pub hardrst: bool [read_only] @ 0,
    }
}

impl Default for Status0A {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status1A(pub u8): Debug, FromRaw, IntoRaw {
        /// * `000`: Toggle logic running (processor has previously written TOGGLE=1)
        /// * `001`: Toggle functionality has settled to SRCon CC1 (STOP_SRC1 state)
        /// * `010`: Toggle functionality has settled to SRCon CC2 (STOP_SRC2 state)
        /// * `101`: Toggle functionality has settled to SNKon CC1 (STOP_SNK1 state)
        /// * `110`: Toggle functionality has settled to SNKon CC2 (STOP_SNK2 state)
        /// * `111`: Toggle functionality has detected AudioAccessory with vRa on both CC1 and CC2
        /// (settles to STOP_SRC1 state)
        ///
        /// Otherwise: Not defined (do not interpret)
        pub togss: u8 [read_only] @ 3..=5,

        /// Indicates the last packet placed in the RxFIFO is type SOP''_DEBUG (SOP double prime
        /// debug)
        pub rxsop2db: bool [read_only] @ 2,

        /// Indicates the last packet placed in the RxFIFO is type SOP'_DEBUG (SOP prime debug)
        pub rxsop1db: bool [read_only] @ 1,

        /// Indicates the last packet placed in the RxFIFO is type SOP
        pub rxsop: bool [read_only] @ 0,
    }
}

impl Default for Status1A {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct InterruptA(pub u8): Debug, FromRaw, IntoRaw {
        /// Interrupt from either a OCP event on one of the VCONN switches or an over-temperature
        /// event
        pub i_ocp_temp: bool @ 7,
        /// Interrupt indicating the TOGGLE functionality was terminated because a device was
        /// detected
        pub i_togdone: bool @ 6,
        /// Interrupt from automatic soft reset packets with retries have failed
        pub i_softfail: bool @ 5,
        /// Interrupt from automatic packet retries have failed
        pub i_retryfail: bool @ 4,
        /// Interrupt from successfully sending a hard reset ordered set
        pub i_hardsent: bool @ 3,
        /// Interrupt to alert that we sent a packet that was acknowledged with a GoodCRC response
        /// packet
        pub i_txsent: bool @ 2,
        /// Received a soft reset packet
        pub i_softrst: bool @ 1,
        /// Received a hard reset ordered set
        pub i_hardrst: bool @ 0,
    }
}

impl Default for InterruptA {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct InterruptB(pub u8): Debug, FromRaw, IntoRaw {
        /// Sent a GoodCRC acknowledge packet in response to an incoming packet that has the
        /// correct CRC value
        pub i_gcrcsent: bool @ 0,
    }
}

impl Default for InterruptB {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status0(pub u8): Debug, FromRaw, IntoRaw {
        /// Interrupt occurs when VBUS transitions through vVBUSthr. This bit typically is used to
        /// recognize port partner during startup
        pub vbusok: bool [read_only] @ 7,

        /// Transitions are detected on the active CC* line. This bit goes high after a minimum of
        /// 3 CC transitions, and goes low with no Transitions
        pub activity: bool [read_only] @ 6,

        /// Measured CC* input is higher than reference level driven from the MDAC
        pub comp: bool [read_only] @ 5,

        /// Indicates the last received packet had the correct CRC. This bit remains set until the
        /// SOP of the next packet
        ///
        /// If false, packet received for an enabled SOP* and CRC for the enabled packet received
        /// was incorrect
        pub crc_chk: bool [read_only] @ 4,

        /// Alert software an error condition has occurred. An alert is caused by:
        /// * TX_FULL: the transmit FIFO is full
        /// * RX_FULL: the receive FIFO is full
        ///
        /// See Status1 bits
        pub alert: bool [read_only] @ 3,

        /// * `true`: Voltage on CC indicated a device attempting to attach
        /// * `false`: WAKE either not enabled (WAKE_EN=0) or no device attached
        pub wake: bool [read_only] @ 2,

        /// Current voltage status of the measured CC pin interpreted as host current levels as
        /// follows:
        ///
        /// * `00`: < 200 mV
        /// * `01`: >200mV,<660mV
        /// * `10`: >660mV,<1.23V
        /// * `11`: > 1.23 V
        ///
        /// Note the software must measure these at an appropriate time, while there is no signaling
        /// activity on the selected CC line. BC_LVL is only defined when Measure block is on which
        /// is when register bits PWR[2]=1 and either MEAS_CC1=1 or MEAS_CC2=1
        pub bc_lvl: u8 [read_only] @ 0..=1,
    }
}

impl Default for Status0 {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status1(pub u8): Debug, FromRaw, IntoRaw {
        /// Indicates the last packet placed in the RxFIFO is type SOP'' (SOP double prime)
        pub rxsop2: bool [read_only] @ 7,
        /// Indicates the last packet placed in the RxFIFO is type SOP' (SOP prime)
        pub rxsop1: bool [read_only] @ 6,
        /// The receive FIFO is empty
        pub rx_empty: bool [read_only] @ 5,
        /// The receive FIFO is full
        pub rx_full: bool [read_only] @ 4,
        /// The transmit FIFO is empty
        pub tx_empty: bool [read_only] @ 3,
        /// The transmit FIFO is full
        pub tx_full: bool [read_only] @ 2,
        /// Temperature of the device is too high
        pub overtemp: bool [read_only] @ 1,
        /// Indicates an over-current or short condition has occurred on the VCONN switch
        pub ocp: bool [read_only] @ 0,
    }
}

impl Default for Status1 {
    fn default() -> Self {
        Self(0b0010_1000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Interrupt(pub u8): Debug, FromRaw, IntoRaw {
        /// Interrupt occurs when VBUS transitions through 4.5 V. This bit typically is used to recognize port partner during startup
        pub i_vbusok: bool @ 7,

        /// A change in the value of ACTIVITY of the CC bus has oc- curred
        pub i_activity: bool @ 6,

        /// A change in the value of COMP has occurred. Indicates se- lected CC line has tripped a threshold programmed into the MDAC
        pub i_comp_chng: bool @ 5,

        /// The value of CRC_CHK newly valid. I.e. The validity of the incoming packet has been checked
        pub i_crc_chk: bool @ 4,

        /// Alert software an error condition has occurred. An alert is caused by:
        ///
        /// * TX_FULL: the transmit FIFO is full
        /// * RX_FULL: the receive FIFO is full See Status1 bits
        pub i_alert: bool @ 3,

        /// Voltage on CC indicated a device attempting to attach. Software must then power up the clock and receiver blocks
        pub i_wake: bool @ 2,

        /// When a transmit was attempted, activity was detected on the active CC line. Transmit is not done. The packet is received normally
        pub i_collision: bool @ 1,

        /// A change in host requested current level has occurred
        pub i_bc_lvl: bool @ 0,
    }
}

impl Default for Interrupt {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Fifo(pub u8): Debug, FromRaw, IntoRaw {
        /// Writing to this register writes a byte into the transmit FIFO. Reading from this register reads from the receive FIFO.
        /// Each byte is a coded token. Or a token followed by a fixed number of packed data byte (see token coding in Table 41)
        pub token: u8 @ 0..7,
    }
}

impl Default for Fifo {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}
