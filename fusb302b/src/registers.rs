//! FUSB302B registers
//!
//! Setters/getters/clearers generated using macros, `Default` for each register is its reset value.

use {
    crate::Fusb302b,
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    proc_bitfield::bitfield,
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

impl<I2C: Read + Write + WriteRead> Fusb302b<I2C> {
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
        (Fifo, fifo, rw),
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

#[derive(Debug, Clone, Copy)]
pub enum Role {
    Source,
    Sink,
}

impl From<bool> for Role {
    fn from(value: bool) -> Self {
        match value {
            false => Self::Sink,
            true => Self::Source,
        }
    }
}

impl Into<bool> for Role {
    fn into(self) -> bool {
        match self {
            Self::Sink => false,
            Self::Source => true,
        }
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

impl Into<bool> for Revision {
    fn into(self) -> bool {
        match self {
            Self::R1_0 => false,
            Self::R2_0 => true,
        }
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Switches1(pub u8): Debug, FromRaw, IntoRaw {
        /// Bit used for constructing the GoodCRC acknowledge packet. This bit corresponds to the
        /// Port Power Role bit in the message header if an SOP packet is received.
        pub powerrole: bool [set Role, get Role] @ 7,
        /// Bit used for constructing the GoodCRC acknowledge packet. These bits correspond to the
        /// Specification Revision bits in the message header.
        pub specrev: bool [set Revision, get Revision] @ 5,
        /// Bit used for constructing the GoodCRC acknowledge packet. This bit corresponds to the
        /// Port Data Role bit in the message header.
        pub datarole: bool [set Role, get Role] @ 4,
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
        /// the value programmed by SDAC[5:0] and the higher threshold that is:
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

    }
}

impl Default for Fifo {
    fn default() -> Self {
        Self(0b0000_0000)
    }
}
