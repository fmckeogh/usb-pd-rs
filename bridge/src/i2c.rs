use stm32f0xx_hal::{
    gpio::{
        gpioa::{PA10, PA9},
        OpenDrain, Output, PushPull,
    },
    prelude::*,
};

pub struct BbI2c {
    scl: PA10<Output<PushPull>>,
    sda: PA9<Output<OpenDrain>>,
    is_started: bool,
}

impl BbI2c {
    pub fn new(scl: PA10<Output<PushPull>>, sda: PA9<Output<OpenDrain>>) -> Self {
        Self {
            scl,
            sda,
            is_started: false,
        }
    }

    pub fn write_data(&mut self, addr: u8, reg: u8, data: &mut [u8], end_with_stop: bool) -> bool {
        self.write_start_cond();
        let mut ack = self.write_byte(addr << 1);
        ack = ack && self.write_byte(reg);
        for byte in data {
            ack = ack && self.write_byte(*byte);
        }
        if end_with_stop && !ack {
            self.write_stop_cond();
        }
        return ack;
    }

    pub fn read_data(&mut self, addr: u8, reg: u8, data: &mut [u8]) -> bool {
        let mut ack = self.write_data(addr, reg, &mut [], false);

        if ack {
            self.write_start_cond();
        }

        ack = ack && self.write_byte((addr << 1) | 1);

        for i in 0..data.len() {
            data[i] = self.read_byte(i == data.len() - 1);
        }
        self.write_stop_cond();
        return ack;
    }

    fn write_start_cond(&mut self) {
        if self.is_started {
            self.sda.set_high();
            delay();
            self.scl.set_high();
            delay();
        }

        self.sda.set_low();
        delay();
        self.scl.set_low();
        self.is_started = true;
    }

    fn write_stop_cond(&mut self) {
        self.sda.set_low();
        delay();
        self.scl.set_high();
        delay();
        self.sda.set_high();
        delay();
        self.is_started = false;
    }

    fn write_byte(&mut self, mut value: u8) -> bool {
        for _ in 0..8 {
            self.write_bit((value & 0x80) != 0);
            value <<= 1;
        }

        return !self.read_bit();
    }

    fn read_byte(&mut self, nack: bool) -> u8 {
        let mut value = 0;

        for _ in 0..8 {
            value <<= 1;
            value |= self.read_bit() as u8;
        }

        self.write_bit(nack);

        return value;
    }

    fn write_bit(&mut self, bit: bool) {
        if bit {
            self.sda.set_high().ok();
        } else {
            self.sda.set_low().ok();
        }
        delay();
        self.scl.set_high().ok();
        delay();
        self.scl.set_low().ok();
    }

    fn read_bit(&mut self) -> bool {
        self.sda.set_high().ok();
        delay();
        self.scl.set_high().ok();
        delay();
        let bit = self.sda.is_high().unwrap();
        self.scl.set_low().ok();
        return bit;
    }
}

fn delay() {
    for _ in 0..10 {
        cortex_m::asm::nop();
    }
}
