use {
    embedded_hal::digital::v2::PinState,
    stm32f0xx_hal::{
        gpio::{
            gpioa::{Parts, PA5, PA6, PA7},
            Output, PushPull,
        },
        prelude::*,
    },
};

#[derive(Debug, Clone, Copy)]
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

pub struct Rgb {
    red: PA5<Output<PushPull>>,
    green: PA6<Output<PushPull>>,
    blue: PA7<Output<PushPull>>,
}

impl Rgb {
    pub fn new(gpioa: Parts) -> Self {
        cortex_m::interrupt::free(move |cs| Self {
            red: gpioa.pa5.into_push_pull_output(cs),
            green: gpioa.pa6.into_push_pull_output(cs),
            blue: gpioa.pa7.into_push_pull_output(cs),
        })
    }

    pub fn set(&mut self, color: Color) {
        self.set_inner(color as u8)
    }

    fn set_inner(&mut self, value: u8) {
        self.red.set_state(get_bit(value, 2)).unwrap();
        self.green.set_state(get_bit(value, 1)).unwrap();
        self.blue.set_state(get_bit(value, 0)).unwrap();
    }
}

fn get_bit(n: u8, k: u8) -> PinState {
    ((n & (1 << k)) >> k == 0).into()
}
