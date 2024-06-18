#![no_main]
#![no_std]

use {panic_rtt_target as _, rtic::app};

mod rgb;

#[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use {
        crate::rgb::{Color, Rgb},
        rtt_target::{rprintln, rtt_init_print},
        stm32f0xx_hal::prelude::*,
        systick_monotonic::{fugit::Duration, Systick},
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Rgb,
        state: u8,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH;
        let mut rcc = cx.device.RCC.configure().sysclk(8.mhz()).freeze(&mut flash);

        rtt_init_print!();

        let mono = Systick::new(cx.core.SYST, rcc.clocks.sysclk().0);

        rprintln!("init");

        let gpioa = cx.device.GPIOA.split(&mut rcc);

        let led = Rgb::new(gpioa);

        cycle::spawn().unwrap();

        (Shared {}, Local { led, state: 0 }, init::Monotonics(mono))
    }

    #[task(local = [led, state])]
    fn cycle(cx: cycle::Context) {
        cycle::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        let color = Color::from(*cx.local.state);
        rprintln!("cycle: {:?}", color);
        cx.local.led.set(color);

        *cx.local.state += 1;
    }
}
