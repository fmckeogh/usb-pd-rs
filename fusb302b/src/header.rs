use {
    crate::{DataRole, PowerRole},
    byteorder::{ByteOrder, LittleEndian},
    proc_bitfield::bitfield,
};

#[derive(Debug)]
pub enum SpecificationRevision {
    R1_0,
    R2_0,
    R3_0,
}

impl From<u8> for SpecificationRevision {
    fn from(value: u8) -> Self {
        match value {
            0b00 => Self::R1_0,
            0b01 => Self::R2_0,
            0b10 => Self::R3_0,
            _ => unimplemented!(),
        }
    }
}

impl From<SpecificationRevision> for u8 {
    fn from(value: SpecificationRevision) -> Self {
        match value {
            SpecificationRevision::R1_0 => 0b00,
            SpecificationRevision::R2_0 => 0b01,
            SpecificationRevision::R3_0 => 0b10,
        }
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Header(pub u16): Debug, FromRaw, IntoRaw {
        pub extended: bool @ 15,
        pub num_objects: u8 @ 12..=14,
        pub message_id: u8 @ 9..=11,
        pub port_power_role: bool [get PowerRole] @ 8,
        pub spec_revision: u8 [get SpecificationRevision] @ 6..=7,
        pub port_data_role: bool [get DataRole] @ 5,
        pub message_type_raw: u8 @ 0..4,
    }
}

impl Header {
    pub fn from_bytes(buf: &[u8]) -> Self {
        assert!(buf.len() == 2);

        Header(LittleEndian::read_u16(buf))
    }

    pub fn message_type(&self) -> MessageType {
        if self.num_objects() == 0 {
            MessageType::Control(self.message_type_raw().into())
        } else {
            MessageType::Data(self.message_type_raw().into())
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    Control(ControlMessageType),
    Data(DataMessageType),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMessageType {
    GoodCRC = 0b0_0001,
    GotoMin = 0b0_0010,
    Accept = 0b0_0011,
    Reject = 0b0_0100,
    Ping = 0b0_0101,
    PsRdy = 0b0_0110,
    GetSourceCap = 0b0_0111,
    GetSinkCap = 0b0_1000,
    DrSwap = 0b0_1001,
    PrSwap = 0b0_1010,
    VconnSwap = 0b0_1011,
    Wait = 0b0_1100,
    SoftReset = 0b0_1101,
    DataReset = 0b0_1110,
    DataResetComplete = 0b0_1111,
    NotSupported = 0b1_0000,
    GetSourceCapExtended = 0b1_0001,
    GetStatus = 0b1_0010,
    FrSwap = 0b1_0011,
    GetPpsStatus = 0b1_0100,
    GetCountryCodes = 0b1_0101,
    GetSinkCapExtended = 0b1_0110,
    GetSourceInfo = 0b1_0111,
    GetRevision = 0b1_1000,
    Reserved,
}

impl From<u8> for ControlMessageType {
    fn from(value: u8) -> Self {
        match value {
            0b0_0001 => Self::GoodCRC,
            0b0_0010 => Self::GotoMin,
            0b0_0011 => Self::Accept,
            0b0_0100 => Self::Reject,
            0b0_0101 => Self::Ping,
            0b0_0110 => Self::PsRdy,
            0b0_0111 => Self::GetSourceCap,
            0b0_1000 => Self::GetSinkCap,
            0b0_1001 => Self::DrSwap,
            0b0_1010 => Self::PrSwap,
            0b0_1011 => Self::VconnSwap,
            0b0_1100 => Self::Wait,
            0b0_1101 => Self::SoftReset,
            0b0_1110 => Self::DataReset,
            0b0_1111 => Self::DataResetComplete,
            0b1_0000 => Self::NotSupported,
            0b1_0001 => Self::GetSourceCapExtended,
            0b1_0010 => Self::GetStatus,
            0b1_0011 => Self::FrSwap,
            0b1_0100 => Self::GetPpsStatus,
            0b1_0101 => Self::GetCountryCodes,
            0b1_0110 => Self::GetSinkCapExtended,
            0b1_0111 => Self::GetSourceInfo,
            0b1_1000 => Self::GetRevision,
            _ => Self::Reserved,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataMessageType {
    SourceCapabilities = 0b0_0001,
    Request = 0b0_0010,
    Bist = 0b0_0011,
    SinkCapabilities = 0b0_0100,
    BatteryStatus = 0b0_0101,
    Alert = 0b0_0110,
    GetCountryInfo = 0b0_0111,
    EnterUsb = 0b0_1000,
    EprRequest = 0b0_1001,
    EprMode = 0b0_1010,
    SourceInfo = 0b0_1011,
    Revision = 0b0_1100,
    VendorDefined = 0b0_1111,
    Reserved,
}

impl From<u8> for DataMessageType {
    fn from(value: u8) -> Self {
        match value {
            0b0_0001 => Self::SourceCapabilities,
            0b0_0010 => Self::Request,
            0b0_0011 => Self::Bist,
            0b0_0100 => Self::SinkCapabilities,
            0b0_0101 => Self::BatteryStatus,
            0b0_0110 => Self::Alert,
            0b0_0111 => Self::GetCountryInfo,
            0b0_1000 => Self::EnterUsb,
            0b0_1001 => Self::EprRequest,
            0b0_1010 => Self::EprMode,
            0b0_1011 => Self::SourceInfo,
            0b0_1100 => Self::Revision,
            0b0_1111 => Self::VendorDefined,
            _ => Self::Reserved,
        }
    }
}
