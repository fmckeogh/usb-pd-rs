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

