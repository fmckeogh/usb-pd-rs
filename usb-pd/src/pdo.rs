use {
    byteorder::{ByteOrder, LittleEndian},
    defmt::Format,
    proc_bitfield::bitfield,
};

#[derive(Clone, Copy, Format)]
pub enum PowerDataObject {
    FixedSupply(FixedSupply),
    Battery(Battery),
    VariableSupply(VariableSupply),
    AugmentedPowerDataObject(AugmentedPowerDataObject),
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PowerDataObjectRaw(pub u32): FromRaw, IntoRaw {
        pub kind: u8 @ 30..=31,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct FixedSupply(pub u32): FromRaw, IntoRaw {
        /// Fixed supply
        pub kind: u8 @ 30..=31,
        /// Dual-role power
        pub dual_role_power: bool @ 29,
        /// USB suspend supported
        pub usb_suspend_supported: bool @ 28,
        /// Unconstrained power
        pub unconstrained_power: bool @ 27,
        /// USB communications capable
        pub usb_communications_capable: bool @ 26,
        /// Dual-role data
        pub dual_role_data: bool @ 25,
        /// Unchunked extended messages supported
        pub unchunked_extended_messages_supported: bool @ 24,
        /// EPR mode capable
        pub epr_mode_capable: bool @ 23,
        /// Peak current
        pub peak_current: u8 @ 20..=21,
        /// Voltage in 50mV units
        pub voltage: u16 @ 10..=19,
        /// Maximum current in 10mA units
        pub max_current: u16 @ 0..=9,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct Battery(pub u32): FromRaw, IntoRaw {
        /// Battery
        pub kind: u8 @ 30..=31,
        /// Maximum Voltage in 50mV units
        pub max_voltage: u16 @ 20..=29,
        /// Minimum Voltage in 50mV units
        pub min_voltage: u16 @ 10..=19,
        /// Maximum Allowable Power in 250mW units
        pub max_power: u16 @ 0..=9,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct VariableSupply(pub u32): FromRaw, IntoRaw {
        /// Variable supply (non-battery)
        pub kind: u8 @ 30..=31,
        /// Maximum Voltage in 50mV units
        pub max_voltage: u16 @ 20..=29,
        /// Minimum Voltage in 50mV units
        pub min_voltage: u16 @ 10..=19,
        /// Maximum current in 10mA units
        pub max_current: u16 @ 0..=9,
    }
}

#[derive(Clone, Copy, Format)]
pub enum AugmentedPowerDataObject {
    SPR(SPRProgrammablePowerSupply),
    EPR(EPRAdjustableVoltageSupply),
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct AugmentedPowerDataObjectRaw(pub u32): FromRaw, IntoRaw {
        /// Augmented power data object
        pub kind: u8 @ 30..=31,
        pub supply: u8 @ 28..=29,
        pub power_capabilities: u32 @ 0..=27,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct SPRProgrammablePowerSupply(pub u32): FromRaw, IntoRaw {
        /// Augmented power data object
        pub kind: u8 @ 30..=31,
        /// SPR programmable power supply
        pub supply: u8 @ 28..=29,
        pub pps_power_limited: bool @ 27,
        /// Maximum voltage in 100mV increments
        pub max_voltage: u8 @ 17..=24,
        /// Minimum Voltage in 100mV increments
        pub min_voltage: u8 @ 8..=15,
        /// Maximum Current in 50mA increments
        pub maximum_current: u8 @ 0..=6,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct EPRAdjustableVoltageSupply(pub u32): FromRaw, IntoRaw {
        /// Augmented power data object
        pub kind: u8 @ 30..=31,
        /// EPR adjustable voltage supply
        pub supply: u8 @ 28..=29,
        pub peak_current: u8 @ 26..=27,
        /// Maximum voltage in 100mV increments
        pub max_voltage: u16 @ 17..=25,
        /// Minimum Voltage in 100mV increments
        pub min_voltage: u8 @ 8..=15,
        /// PDP in 1W increments
        pub maximum_current: u8 @ 0..=7,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct FixedVariableRequestDataObject(pub u32): FromRaw, IntoRaw {
        /// Valid range 1..=14
        pub object_position: u8 @ 28..=31,
        pub giveback_flag: bool @ 27,
        pub capability_mismatch: bool @ 26,
        pub usb_communications_capable: bool @ 25,
        pub no_usb_suspend: bool @ 24,
        pub unchunked_extended_messages_supported: bool @ 23,
        pub epr_mode_capable: bool @ 22,
        pub operating_current: u16 @ 10..=19,
        pub maximum_operating_current: u16 @ 0..=9,
    }
}

impl FixedVariableRequestDataObject {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

#[derive(Clone, Copy, Format)]
pub enum VDMCommandType {
    InitiatorREQ,
    ResponderACK,
    ResponderNAK,
    ResponderBSY,
}

impl From<VDMCommandType> for u8 {
    fn from(value: VDMCommandType) -> Self {
        match value {
            VDMCommandType::InitiatorREQ => 0,
            VDMCommandType::ResponderACK => 1,
            VDMCommandType::ResponderNAK => 2,
            VDMCommandType::ResponderBSY => 3,
        }
    }
}

impl From<u8> for VDMCommandType {
    fn from(value: u8) -> Self {
        match value {
            0 => VDMCommandType::InitiatorREQ,
            1 => VDMCommandType::ResponderACK,
            2 => VDMCommandType::ResponderNAK,
            3 => VDMCommandType::ResponderBSY,
            _ => panic!("Cannot convert {:} to VDMCommandType", value), /* Illegal values shall
                                                                         * panic. */
        }
    }
}

#[derive(Clone, Copy, Format)]
pub enum VDMCommand {
    DiscoverIdentity,
    DiscoverSVIDS,
    DiscoverModes,
    EnterMode,
    ExitMode,
    Attention,
    DisplayPortStatus,
    DisplayPortConfig,
}
impl From<VDMCommand> for u8 {
    fn from(value: VDMCommand) -> Self {
        match value {
            VDMCommand::DiscoverIdentity => 0x1,
            VDMCommand::DiscoverSVIDS => 0x2,
            VDMCommand::DiscoverModes => 0x3,
            VDMCommand::EnterMode => 0x4,
            VDMCommand::ExitMode => 0x5,
            VDMCommand::Attention => 0x6,
            VDMCommand::DisplayPortStatus => 0x10,
            VDMCommand::DisplayPortConfig => 0x11,
        }
    }
}

impl From<u8> for VDMCommand {
    fn from(value: u8) -> Self {
        match value {
            0x01 => VDMCommand::DiscoverIdentity,
            0x02 => VDMCommand::DiscoverSVIDS,
            0x03 => VDMCommand::DiscoverModes,
            0x04 => VDMCommand::EnterMode,
            0x05 => VDMCommand::ExitMode,
            0x06 => VDMCommand::Attention,
            0x10 => VDMCommand::DisplayPortStatus,
            0x11 => VDMCommand::DisplayPortConfig,
            // TODO: Find document that explains what 0x12-0x1f are (DP_SID??)
            _ => panic!("Cannot convert {:} to VDMCommand", value), // Illegal values shall panic.
        }
    }
}

#[derive(Clone, Copy, Format)]
pub enum VDMType {
    Unstructured,
    Structured,
}

impl From<VDMType> for bool {
    fn from(value: VDMType) -> Self {
        match value {
            VDMType::Unstructured => false,
            VDMType::Structured => true,
        }
    }
}

impl From<bool> for VDMType {
    fn from(value: bool) -> Self {
        match value {
            true => VDMType::Structured,
            false => VDMType::Unstructured,
        }
    }
}

#[derive(Clone, Copy, Format)]
pub enum VDMHeader {
    Structured(VDMHeaderStructured),
    Unstructured(VDMHeaderUnstructured),
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct VDMHeaderRaw(pub u32): FromRaw, IntoRaw {
        /// VDM Standard or Vendor ID
        pub standard_or_vid: u16 @ 16..=31,
        /// VDM Type (Unstructured/Structured)
        pub vdm_type: bool [VDMType] @ 15,
    }
}

impl VDMHeaderRaw {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct VDMHeaderStructured(pub u32): FromRaw, IntoRaw {
        /// VDM Standard or Vendor ID
        pub standard_or_vid: u16 @ 16..=31,
        /// VDM Type (Unstructured/Structured)
        pub vdm_type: bool [VDMType] @ 15,
        /// Structured VDM Version
        pub vdm_version: u8 @ 13..=14,
        /// Object Position
        pub object_position: u8 @ 8..=10,
        /// Command Type
        pub command_type: u8 [VDMCommandType] @ 6..=7,
        /// Command
        pub command: u8 [VDMCommand] @ 0..=4,
    }
}

impl VDMHeaderStructured {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct VDMHeaderUnstructured(pub u32): FromRaw, IntoRaw {
        /// VDM Standard or Vendor ID
        pub standard_or_vid: u16 @ 16..=31,
        /// VDM Type (Unstructured/Structured)
        pub vdm_type: bool [VDMType] @ 15,
        /// Message defined
        pub data: u16 @ 0..=14
    }
}

impl VDMHeaderUnstructured {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct VDMIdentityHeader(pub u32): FromRaw, IntoRaw {
        /// Host data capable
        pub host_data: bool @ 31,
        /// Device data capable
        pub device_data: bool @ 30,
        /// Product type UFP
        pub product_type_ufp: u8 @ 27..=29,
        /// Modal Operation Supported
        pub modal_supported: bool @ 26,
        /// Product type DFP
        pub product_type_dfp: u8 @ 23..=25,
        /// Connector type
        pub connector_type: u8 @ 21..=22,
        /// VID
        pub vid: u16 @ 0..=15,
    }
}

impl VDMIdentityHeader {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct DisplayPortCapabilities(pub u32): FromRaw, IntoRaw {
        /// UFP_D Pin Assignments Supported
        pub ufp_d_pin_assignments: u8 @ 16..=23,
        /// DFP_D Pin Assignments Supported
        pub dfp_d_pin_assignments: u8 @ 8..=15,
        /// USB r2.0 Signalling Not Used
        pub usb20_signalling_not_used: bool @ 7,
        /// Receptacle Indication
        pub receptacle_indication: bool @ 6,
        /// Signalling for Transport of DisplayPort Protocol
        pub signaling_rate: u8 @ 2..=5,
        /// Port Capability
        pub capability: u8 @ 0..=1,
    }
}

impl DisplayPortCapabilities {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}
