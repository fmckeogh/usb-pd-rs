#![no_std]

pub mod callback;
pub mod header;
pub mod message;
pub mod pdo;
pub mod sink;
pub mod source;
pub mod token;

pub type Instant = fugit::Instant<u64, 1, 1000>;
pub type Duration = fugit::Duration<u64, 1, 1000>;

#[derive(Clone, Copy)]
pub enum CcPin {
    CC1,
    CC2,
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
