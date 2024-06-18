use {
    bitbang_hal::i2c::I2cBB,
    stm32f0xx_hal::{
        gpio::{
            gpioa::{PA10, PA9},
            OpenDrain, Output, PushPull,
        },
        pac::TIM3,
        prelude::*,
        timers::Timer,
    },
};

pub struct BbI2c {
    inner: I2cBB<PA10<Output<PushPull>>, PA9<Output<OpenDrain>>, Timer<TIM3>>,
}

impl BbI2c {
    pub fn new(scl: PA10<Output<PushPull>>, sda: PA9<Output<OpenDrain>>, clk: Timer<TIM3>) -> Self {
        Self {
            inner: I2cBB::new(scl, sda, clk),
        }
    }

    pub fn pd_ctrl_read(&mut self, reg: u8, data: &mut [u8]) {
        self.inner.write_read(0x22, &[reg], data).unwrap()
    }

    pub fn pd_ctrl_write(&mut self, data: &mut [u8]) {
        self.inner.write(0x22, data).unwrap()
    }
}
