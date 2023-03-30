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
                self.write_register_raw(Register::$reg as u8, 0);
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
        /// Bit used for constructing the GoodCRC acknowledge packet. This bit corresponds to the Port Power Role bit in the message header if an SOP packet is received.
        pub powerrole: bool [set Role, get Role] @ 7,
        /// Bit used for constructing the GoodCRC acknowledge packet. These bits correspond to the Specification Revision bits in the message header:
        /// false: Revision 1.0
        /// true: Revision 2.0
        pub specrev: bool [set Revision, get Revision] @ 5,
        /// Bit used for constructing the GoodCRC acknowledge packet. This bit corresponds to the Port Data Role bit in the message header.
        pub datarole: bool [set Role, get Role] @ 4,
        /// Starts the transmitter automatically when a message with a good CRC is received and automatically sends a GoodCRC acknowledge packet back to the relevant SOP*
        pub auto_src: bool @ 2,
        /// Enable BMC transmit driver on CC2 pin
        pub txcc2: bool @ 1,
        /// Enable BMC transmit driver on CC1 pin
        pub txcc1: bool @ 0,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Measure(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Slice(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control0(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control1(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control2(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control3(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Mask1(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Power(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Reset(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct OcPreg(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct MaskA(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct MaskB(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Control4(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status0A(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status1A(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct InterruptA(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct InterruptB(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status0(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Status1(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Interrupt(pub u8): Debug, FromRaw, IntoRaw {

    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Fifo(pub u8): Debug, FromRaw, IntoRaw {

    }
}
