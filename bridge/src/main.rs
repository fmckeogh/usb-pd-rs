#![no_main]
#![no_std]

use {
    crate::{fsusb302::Fsusb302, pd_sink::PdSink},
    core::sync::atomic::{AtomicU32, Ordering},
    cortex_m::{peripheral::syst::SystClkSource::Core, Peripherals},
    cortex_m_rt::{entry, exception},
    defmt::{error, info, trace},
    defmt_rtt as _,
    i2c::BbI2c,
    stm32f0xx_hal::{pac, prelude::*},
};

mod fsusb302;
mod i2c;
mod pd_sink;
mod rgb;

static MILLIS_COUNT: MillisCount = MillisCount::new();

#[entry]
fn main() -> ! {
    let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) else {
        panic!();
    };

    let mut power_sink = cortex_m::interrupt::free(move |cs| {
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split(&mut rcc);

        // (Re-)configure PA1 as output
        let led = gpioa.pa5.into_push_pull_output(cs);

        let scl = gpioa.pa10.into_push_pull_output_hs(cs);
        let sda = gpioa.pa9.into_open_drain_output(cs);

        let bbi2c = BbI2c::new(scl, sda);

        let fsusb302 = Fsusb302::new(bbi2c);

        let power_sink = PdSink::new(fsusb302);

        let mut syst = cp.SYST;

        // Initialise SysTick counter with a defined value
        unsafe { syst.cvr.write(1) };

        // Set source for SysTick counter, here full operating frequency (== 48MHz)
        syst.set_clock_source(Core);

        // Set reload value, i.e. timer delay 48 MHz/4 Mcounts == 12Hz or 83ms
        syst.set_reload(48_000_000 / 1000);

        // Start counting
        syst.enable_counter();

        // Enable interrupt generation
        syst.enable_interrupt();

        power_sink
    });

    power_sink.init();

    // Work in regular loop
    loop {
        power_sink.poll();
    }
}

// Define an exception handler, i.e. function to call when exception occurs. Here, if our SysTick
// timer generates an exception the following handler will be called
#[exception]
fn SysTick() {
    MILLIS_COUNT.inc();
}

struct MillisCount {
    inner: AtomicU32,
}

impl MillisCount {
    pub const fn new() -> Self {
        Self {
            inner: AtomicU32::new(0),
        }
    }

    pub fn get(&self) -> u32 {
        self.inner.load(Ordering::SeqCst)
    }

    pub fn inc(&self) {
        self.inner.store(self.get() + 1, Ordering::SeqCst);
    }
}

fn delay(ms: u32) {
    let now = MILLIS_COUNT.get();

    while MILLIS_COUNT.get() < now + ms {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    error!("panic!");
    loop {}
}
