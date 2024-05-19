use {
    byteorder::{ByteOrder, LittleEndian},
    defmt::Format,
    heapless::Vec,
    proc_bitfield::bitfield,
};

#[derive(Clone, Copy, Debug, Format)]
pub enum PowerDataObject {
    FixedSupply(FixedSupply),
    Battery(Battery),
    VariableSupply(VariableSupply),
    AugmentedPowerDataObject(AugmentedPowerDataObject),
    Unknown(PowerDataObjectRaw),
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct PowerDataObjectRaw(pub u32): Debug, FromRaw, IntoRaw {
        pub kind: u8 @ 30..=31,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct FixedSupply(pub u32): Debug, FromRaw, IntoRaw {
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
    pub struct Battery(pub u32): Debug, FromRaw, IntoRaw {
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
    pub struct VariableSupply(pub u32): Debug, FromRaw, IntoRaw {
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

#[derive(Clone, Copy, Debug, Format)]
pub enum AugmentedPowerDataObject {
    SPR(SPRProgrammablePowerSupply),
    EPR(EPRAdjustableVoltageSupply),
    Unknown(u32),
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct AugmentedPowerDataObjectRaw(pub u32): Debug, FromRaw, IntoRaw {
        /// Augmented power data object
        pub kind: u8 @ 30..=31,
        pub supply: u8 @ 28..=29,
        pub power_capabilities: u32 @ 0..=27,
    }
}

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct SPRProgrammablePowerSupply(pub u32): Debug, FromRaw, IntoRaw {
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
    pub struct EPRAdjustableVoltageSupply(pub u32): Debug, FromRaw, IntoRaw {
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
    pub struct FixedVariableRequestDataObject(pub u32): Debug, FromRaw, IntoRaw {
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

bitfield! {
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct BatteryRequestDataObject(pub u32): FromRaw, IntoRaw {
        /// Object position (0000b and 1110b…1111b are Reserved and Shall Not be used)
        pub object_position: u8 @ 28..=31,
        /// GiveBackFlag = 0
        pub giveback_flag: bool @ 27,
        /// Capability mismatch
        pub capability_mismatch: bool @ 26,
        /// USB communications capable
        pub usb_communications_capable: bool @ 25,
        /// No USB Suspend
        pub no_usb_suspend: bool @ 24,
        /// Unchunked extended messages supported
        pub unchunked_extended_messages_supported: bool @ 23,
        /// EPR mode capable
        pub epr_mode_capable: bool @ 22,
        /// Operating power in 250mW units
        pub operating_power: u16 @ 10..=19,
        /// Maximum operating power in 250mW units
        pub maximum_operating_power: u16 @ 0..=9,
    }
}

impl BatteryRequestDataObject {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

bitfield!(
    #[derive(Clone, Copy, PartialEq, Eq, Format)]
    pub struct PPSRequestDataObject(pub u32): FromRaw, IntoRaw {
        /// Object position (0000b and 1110b…1111b are Reserved and Shall Not be used)
        pub object_position: u8 @ 28..=31,
        /// Capability mismatch
        pub capability_mismatch: bool @ 26,
        /// USB communications capable
        pub usb_communications_capable: bool @ 25,
        /// No USB Suspend
        pub no_usb_suspend: bool @ 24,
        /// Unchunked extended messages supported
        pub unchunked_extended_messages_supported: bool @ 23,
        /// EPR mode capable
        pub epr_mode_capable: bool @ 22,
        /// Output voltage in 20mV units
        pub output_voltage: u16 @ 9..=20,
        /// Operating current in 50mA units
        pub operating_current: u16 @ 0..=6,
    }
);

impl PPSRequestDataObject {
    pub fn to_bytes(&self, buf: &mut [u8]) {
        LittleEndian::write_u32(buf, self.0);
    }
}

#[derive(Debug, Clone, Format)]
pub struct SourceCapabilities(pub(crate) Vec<PowerDataObject, 8>);

impl SourceCapabilities {
    pub fn vsafe_5v(&self) -> Option<&FixedSupply> {
        self.0.first().and_then(|supply| {
            if let PowerDataObject::FixedSupply(supply) = supply {
                Some(supply)
            } else {
                None
            }
        })
    }

    pub fn dual_role_power(&self) -> bool {
        self.vsafe_5v()
            .map(FixedSupply::dual_role_power)
            .unwrap_or_default()
    }

    pub fn usb_suspend_supported(&self) -> bool {
        self.vsafe_5v()
            .map(FixedSupply::usb_suspend_supported)
            .unwrap_or_default()
    }

    pub fn unconstrained_power(&self) -> bool {
        self.vsafe_5v()
            .map(FixedSupply::unconstrained_power)
            .unwrap_or_default()
    }

    pub fn dual_role_data(&self) -> bool {
        self.vsafe_5v()
            .map(FixedSupply::dual_role_data)
            .unwrap_or_default()
    }

    pub fn unchunked_extended_messages_supported(&self) -> bool {
        self.vsafe_5v()
            .map(FixedSupply::unchunked_extended_messages_supported)
            .unwrap_or_default()
    }

    pub fn epr_mode_capable(&self) -> bool {
        self.vsafe_5v()
            .map(FixedSupply::epr_mode_capable)
            .unwrap_or_default()
    }

    pub fn pdos(&self) -> &[PowerDataObject] {
        &self.0
    }
}
