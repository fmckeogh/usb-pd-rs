#![no_main]
#![no_std]

use {panic_rtt_target as _, rtic::app};

mod rgb;

#[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use {
        crate::rgb::{Color, Rgb},
        bitbang_hal::i2c::I2cBB,
        fusb302b::Fusb302b,
        rtt_target::{rprintln, rtt_init_print},
        stm32f0xx_hal::{
            gpio::{
                gpioa::{self, PA5, PA6, PA7},
                Output, PushPull,
            },
            prelude::*,
            timers::Timer,
        },
        systick_monotonic::Systick,
    };

    #[shared]
    struct Shared {
        _led: Rgb<PA5<Output<PushPull>>, PA6<Output<PushPull>>, PA7<Output<PushPull>>>,
    }

    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH;
        let mut rcc = cx.device.RCC.configure().freeze(&mut flash);

        rtt_init_print!();

        let mono = Systick::new(cx.core.SYST, rcc.clocks.sysclk().0);

        rprintln!("init");

        let gpioa::Parts {
            pa5,
            pa6,
            pa7,
            pa9,
            pa10,
            ..
        } = cx.device.GPIOA.split(&mut rcc);

        let (pa5, pa6, pa7, sda, scl) = cortex_m::interrupt::free(move |cs| {
            (
                pa5.into_push_pull_output(cs),
                pa6.into_push_pull_output(cs),
                pa7.into_push_pull_output(cs),
                pa9.into_open_drain_output(cs),
                pa10.into_push_pull_output(cs),
            )
        });

        let mut led = Rgb::new(pa5, pa6, pa7);
        led.set(Color::Black);

        let mut fusb302b = {
            let clk = Timer::tim3(cx.device.TIM3, 400.khz(), &mut rcc);
            let i2c = I2cBB::new(scl, sda, clk);

            Fusb302b::new(i2c)
        };

        rprintln!("{:?}", fusb302b.device_id());
        rprintln!("{:?}", fusb302b.switches1());

        // initialise

        (Shared { _led: led }, Local {}, init::Monotonics(mono))
    }
}
