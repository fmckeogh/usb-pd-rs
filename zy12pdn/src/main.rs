#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use {
    crate::{i2c::I2cBB, rgb::Rgb},
    cortex_m_rt::entry,
    defmt::info,
    defmt_rtt as _,
    embassy_executor::Spawner,
    embassy_stm32::{
        exti::ExtiInput,
        gpio::{Input, Level, Output, OutputOpenDrain, Pin, Pull, Speed},
        interrupt,
        peripherals::PF1,
        Config,
    },
    embassy_time::Instant,
    fusb302b::Fusb302b,
    panic_halt as _,
    usb_pd::{
        pdo::PowerDataObject,
        sink::{Event, Request, Sink},
    },
};

mod i2c;
mod rgb;

// main is itself an async function.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    let mut led = Rgb::new(
        Output::new(p.PA5, Level::High, Speed::Low),
        Output::new(p.PA6, Level::High, Speed::Low),
        Output::new(p.PA7, Level::High, Speed::Low),
    );
    led.set(rgb::Color::Magenta);

    let sda = OutputOpenDrain::new(p.PA9, Level::Low, Speed::VeryHigh, Pull::None);
    let scl = Output::new(p.PA10, Level::Low, Speed::VeryHigh);

    let i2c = I2cBB::new(scl, sda, 100_000);

    defmt::error!("started!");

    // Configure the button pin and obtain handler.
    // let button = ExtiInput::new(Input::new(p.PF1, Pull::Up), p.EXTI1);
    // spawner.spawn(button_handler(button)).ok();

    let mut pd = Sink::new(Fusb302b::new(i2c));

    pd.init().await;

    loop {
        pd.poll(Instant::now())
            .await // handle event if one was returned from sink
            .and_then(handle_event)
            // make request if one was returned from handler
            .map(|req| pd.request(req));
    }
}

// #[embassy_executor::task]
// async fn button_handler(mut button: ExtiInput<'static, PF1>) {
//     loop {
//         button.wait_for_falling_edge().await;
//         info!("Pressed!");
//         // led.set(rgb::Color::Cyan);
//         button.wait_for_rising_edge().await;
//         info!("Released!");
//         //led.set(rgb::Color::Magenta);
//     }
// }

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
                        // debug!(
                        //     "supply @ {}: {}mV {}mA",
                        //     i,
                        //     supply.voltage() * 50,
                        //     supply.max_current() * 10
                        // );
                        Some((i, supply))
                    } else {
                        None
                    }
                })
                .max_by(|(_, x), (_, y)| x.voltage().cmp(&y.voltage()))
                .unwrap();

            //  info!("requesting supply {:?}@{}", supply, index);

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
