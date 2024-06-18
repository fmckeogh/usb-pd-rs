#![no_main]
#![no_std]

use {
    core::cmp::max,
    defmt::{error, trace},
    defmt_rtt as _,
    fusb302b::callback::{Event, Response},
    rtic::app,
    usb_pd::pdo::PowerDataObject,
};

mod rgb;

#[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use {
        crate::{
            callback,
            rgb::{Color, Rgb},
        },
        bitbang_hal::i2c::I2cBB,
        defmt::info,
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

            Fusb302b::new(i2c, &callback)
        };

        pd.init(monotonics::now());

        info!("init done");

        (Shared {}, Local { led, pd }, init::Monotonics(mono))
    }

    #[idle(local = [pd, led])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.local.pd.poll(monotonics::now());
        }
    }
}

fn callback(event: Event) -> Option<Response> {
    match event {
        Event::ProtocolChanged { .. } => trace!("protocol changed"),
        Event::PowerAccepted => trace!("power accepted"),
        Event::PowerRejected => trace!("power rejected"),
        Event::PowerReady { active_voltage_mv } => trace!("power ready {}mV", active_voltage_mv),

        Event::SourceCapabilities {
            source_capabilities,
        } => {
            let mut voltage = 0;
            let mut current = 0;

            for cap in source_capabilities.into_iter().filter(Option::is_some) {
                match cap.unwrap() {
                    PowerDataObject::Battery(battery) => {
                        trace!("battery: {}", battery.max_voltage())
                    }
                    PowerDataObject::FixedSupply(fixed) => {
                        trace!("fixed: {}", fixed.voltage());
                        voltage = max(voltage, fixed.voltage());
                        current = max(current, fixed.max_current());
                    }
                    PowerDataObject::VariableSupply(variable) => {
                        trace!("variable: {}", variable.max_voltage())
                    }
                    PowerDataObject::AugmentedPowerDataObject(_) => {
                        trace!("aug")
                    }
                }
            }

            return Some(Response::Request { voltage, current });
        }
    }

    None
}

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    error!("panic!");
    loop {}
}
