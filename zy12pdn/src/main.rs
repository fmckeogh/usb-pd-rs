#![no_main]
#![no_std]

use {
    crate::rgb::Rgb,
    bitbang_hal::i2c::I2cBB,
    defmt::{debug, error, info},
    defmt_rtt as _,
    fusb302b::Fusb302b,
    rtic::app,
    stm32f0xx_hal::{
        gpio::{
            gpioa::{PA10, PA5, PA6, PA7, PA9},
            OpenDrain, Output, PushPull,
        },
        pac::TIM3,
        timers::Timer,
    },
    usb_pd::{
        callback::{Event, Response},
        pdo::PowerDataObject,
        sink::Sink,
    },
};

mod rgb;

type Led = Rgb<PA5<Output<PushPull>>, PA6<Output<PushPull>>, PA7<Output<PushPull>>>;
type PdSink = Sink<Fusb302b<I2cBB<PA10<Output<PushPull>>, PA9<Output<OpenDrain>>, Timer<TIM3>>>>;

#[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use {
        crate::{
            callback,
            rgb::{Color, Rgb},
            Led, PdSink,
        },
        bitbang_hal::i2c::I2cBB,
        defmt::info,
        fusb302b::Fusb302b,
        stm32f0xx_hal::{gpio::gpioa, prelude::*, timers::Timer},
        systick_monotonic::Systick,
        usb_pd::sink::Sink,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Led,
        pd: PdSink,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH;
        let mut rcc = cx
            .device
            .RCC
            .configure()
            .sysclk(32u32.mhz())
            .freeze(&mut flash);

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
            Sink::new(Fusb302b::new(i2c), &callback)
        };

        pd.init();

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
        Event::SourceCapabilitiesChanged(caps) => {
            info!("Capabilities changed: {}", caps.len());

            // Take maximum voltage
            let (index, supply) = caps
                .iter()
                .enumerate()
                .filter_map(|(i, cap)| {
                    if let PowerDataObject::FixedSupply(supply) = cap {
                        debug!(
                            "supply @ {}: {}mV {}mA",
                            i,
                            supply.voltage() * 50,
                            supply.max_current() * 10
                        );
                        Some((i, supply))
                    } else {
                        None
                    }
                })
                .max_by(|(_, x), (_, y)| x.voltage().cmp(&y.voltage()))
                .unwrap();

            info!("requesting supply {:?}@{}", supply, index);

            return Some(Response::RequestPower {
                index,
                current: supply.max_current() * 10,
            });
        }
        Event::PowerReady => info!("power ready"),
        Event::ProtocolChanged => info!("protocol changed"),
        Event::PowerAccepted => info!("power accepted"),
        Event::PowerRejected => info!("power rejected"),
    }

    None
}

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    error!("panic!");
    loop {}
}
