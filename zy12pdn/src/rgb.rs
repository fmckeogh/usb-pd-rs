use embedded_hal::digital::{OutputPin, PinState};

#[derive(Clone, Copy)]
pub enum Color {
    White = 0b111,
    Black = 0b000,
    Red = 0b100,
    Green = 0b010,
    Blue = 0b001,
    Yellow = 0b110,
    Magenta = 0b101,
    Cyan = 0b011,
}

impl From<u8> for Color {
    fn from(value: u8) -> Self {
        match value & 0b111 {
            0b000 => Self::Black,
            0b100 => Self::Red,
            0b010 => Self::Green,
            0b001 => Self::Blue,
            0b110 => Self::Yellow,
            0b101 => Self::Magenta,
            0b011 => Self::Cyan,
            _ => Self::White,
        }
    }
}

pub struct Rgb<R, G, B> {
    red: R,
    green: G,
    blue: B,
    color: Color,
}

impl<R: OutputPin, G: OutputPin, B: OutputPin> Rgb<R, G, B> {
    pub fn new(red: R, green: G, blue: B) -> Self {
        Self {
            red,
            green,
            blue,
            color: Color::Black,
        }
    }

    pub fn set(&mut self, color: Color) {
        self.color = color;
        self.set_inner(color as u8)
    }

    pub fn get(&self) -> Color {
        self.color
    }

    fn set_inner(&mut self, value: u8) {
        self.red.set_state(get_bit(value, 2)).ok();
        self.green.set_state(get_bit(value, 1)).ok();
        self.blue.set_state(get_bit(value, 0)).ok();
    }
}

fn get_bit(n: u8, k: u8) -> PinState {
    ((n & (1 << k)) >> k == 0).into()
}
