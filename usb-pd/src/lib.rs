#![no_std]

pub mod header;
pub mod message;
pub mod pdo;
pub mod vdo;
pub mod sink;
pub mod source;
pub mod token;

pub type Instant = fugit::Instant<u64, 1, 1000>;
pub type Duration = fugit::Duration<u64, 1, 1000>;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CcPin {
    CC1,
    CC2,
}

impl core::ops::Not for CcPin {
    type Output = CcPin;

    fn not(self) -> Self::Output {
        match self {
            CcPin::CC1 => CcPin::CC2,
            CcPin::CC2 => CcPin::CC1,
        }
    }
}

#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
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
