#![no_std]

pub mod header;
pub mod message;
pub mod pdo;
pub mod token;

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

pub struct Event;
pub struct Message;

pub trait SinkDriver {
    fn init() -> Self;

    fn poll(&mut self);

    fn get_event(&mut self) -> Option<Event>;

    fn send_message(&mut self, message: Message);
}

pub trait SourceDriver {}
