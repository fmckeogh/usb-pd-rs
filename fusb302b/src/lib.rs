#![no_std]

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

pub mod registers;

/// I2C address of FUSB302BMPX
const DEVICE_ADDRESS: u8 = 0b0100010;

pub struct Fusb302b<I2C> {
    i2c: I2C,
}

impl<I2C: Read + Write + WriteRead> Fusb302b<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    fn write_register_raw(&mut self, register: u8, value: u8) {
        self.i2c.write(DEVICE_ADDRESS, &[register, value]).ok();
    }

    fn read_register_raw(&mut self, register: u8) -> u8 {
        let mut buffer = [0u8];
        self.i2c
            .write_read(DEVICE_ADDRESS, &[register], &mut buffer)
            .ok();
        buffer[0]
    }
}
