#![no_main]
#![no_std]

use {
    crate::rgb::Rgb,
    bitbang_hal::i2c::I2cBB,
    defmt::{debug, info},
    defmt_rtt as _,
    fusb302b::Fusb302b,
    panic_probe as _,
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
        pdo::PowerDataObject,
        sink::{Event, Request, Sink},
    },
};

mod rgb;

type Led = Rgb<PA5<Output<PushPull>>, PA6<Output<PushPull>>, PA7<Output<PushPull>>>;
type PdSink = Sink<Fusb302b<I2cBB<PA10<Output<PushPull>>, PA9<Output<OpenDrain>>, Timer<TIM3>>>>;

#[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use crate::handle_event;

    use {
        crate::{
            rgb::{Color, Rgb},
            Led, PdSink,
        },
        bitbang_hal::i2c::I2cBB,
        defmt::info,
        fusb302b::Fusb302b,
        stm32f0xx_hal::{
            gpio::{
                gpioa,
                gpiof::{self, PF1},
                Input, PullUp,
            },
            pac::EXTI,
            prelude::*,
            timers::Timer,
        },
        systick_monotonic::Systick,
        usb_pd::sink::Sink,
    };

    #[shared]
    struct Shared {
        led: Led,
    }

    #[local]
    struct Local {
        pd: PdSink,
        button: PF1<Input<PullUp>>,
        exti: EXTI,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = cx.device.RCC;
        let syscfg = cx.device.SYSCFG;
        let exti = cx.device.EXTI;

        // Enable clock for SYSCFG
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

        let mut flash = cx.device.FLASH;
        let mut rcc = rcc.configure().sysclk(32u32.mhz()).freeze(&mut flash);

        let mono = Systick::new(cx.core.SYST, rcc.clocks.sysclk().0);

        let gpiof::Parts { pf1, .. } = cx.device.GPIOF.split(&mut rcc);

        let button = cortex_m::interrupt::free(move |cs| pf1.into_pull_up_input(cs));

        // Enable external interrupt for PF1
        syscfg.exticr1.modify(|_, w| w.exti1().pf1());

        // Set interrupt request mask for line 1
        exti.imr.modify(|_, w| w.mr1().set_bit());

        // Set interrupt rising trigger for line 1
        exti.ftsr.modify(|_, w| w.tr1().set_bit());

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
            Sink::new(Fusb302b::new(i2c))
        };

        pd.init();

        info!("init done");

        (
            Shared { led },
            Local { pd, button, exti },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [pd])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            cx.local
                .pd
                // poll PD driver
                .poll(monotonics::now())
                // handle event if one was returned from sink
                .and_then(handle_event)
                // make request if one was returned from handler
                .map(|req| cx.local.pd.request(req));
        }
    }

    #[task(binds = EXTI0_1, local = [button, exti], shared = [led])]
    fn button(mut cx: button::Context) {
        info!("button rising edge");

        cx.shared
            .led
            .lock(|led| led.set(Color::from((led.get() as u8 + 1) % 8)));

        cx.local.exti.pr.write(|w| w.pr1().clear());
    }
}

fn handle_event(event: Event) -> Option<Request> {
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

            return Some(Request::RequestPower {
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
