#![no_main]
#![no_std]

use {core::panic::PanicInfo, rtic::app};

mod log;
mod rgb;

#[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use {
        crate::{
            log::init_log,
            rgb::{Color, Rgb},
        },
        bitbang_hal::i2c::I2cBB,
        fusb302b::Fusb302b,
        stm32f0xx_hal::{
            gpio::{
                gpioa::{self, PA10, PA5, PA6, PA7, PA9},
                OpenDrain, Output, PushPull,
            },
            pac::TIM3,
            prelude::*,
            timers::Timer,
        },
        systick_monotonic::Systick,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Rgb<PA5<Output<PushPull>>, PA6<Output<PushPull>>, PA7<Output<PushPull>>>,
        pd: Fusb302b<I2cBB<PA10<Output<PushPull>>, PA9<Output<OpenDrain>>, Timer<TIM3>>>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH;
        let mut rcc = cx.device.RCC.configure().freeze(&mut flash);

        let mono = Systick::new(cx.core.SYST, rcc.clocks.sysclk().0);

        init_log();

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
        led.set(Color::White);

        let mut pd = {
            let clk = Timer::tim3(cx.device.TIM3, 400.khz(), &mut rcc);
            let i2c = I2cBB::new(scl, sda, clk);

            Fusb302b::new(i2c)
        };

        pd.init(monotonics::now());

        log::warn!("init done");

        (Shared {}, Local { led, pd }, init::Monotonics(mono))
    }

    #[idle(local = [pd, led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.local.pd.poll(monotonics::now());
        }
    }
}

#[panic_handler]
fn panic_handler(_: &PanicInfo) -> ! {
    loop {}
}
