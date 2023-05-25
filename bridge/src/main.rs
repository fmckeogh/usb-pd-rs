#![no_main]
#![no_std]

use {
    core::mem::MaybeUninit,
    cortex_m_rt::entry,
    defmt::{error, info, trace},
    defmt_rtt as _, stm32f0xx_hal as _,
    zy12pdn_wrapper::root::{
        sink_callback,
        usb_pd::{i2c_bit_bang, mcu_hal, pd_sink},
    },
};

mod rgb;

#[entry]
fn main() -> ! {
    let mut hal = unsafe { MaybeUninit::<mcu_hal>::uninit().assume_init() };
    unsafe { hal.init() };
    info!("hal init");

    let mut i2c = unsafe { MaybeUninit::<i2c_bit_bang>::uninit().assume_init() };
    unsafe { i2c.init() };
    trace!("i2c init");

    let mut power_sink = unsafe { MaybeUninit::<pd_sink>::uninit().assume_init() };
    unsafe { power_sink.set_event_callback(Some(sink_callback)) }
    trace!("sink callback");
    unsafe { power_sink.init() };
    trace!("sink init");

    loop {
        unsafe { power_sink.poll() };
    }

    panic!();
}

// #[app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
// mod app {
//     use core::mem::MaybeUninit;

//     use {
//         crate::rgb::{Color, Rgb},
//         defmt::{info, trace},
//         stm32f0xx_hal::{
//             gpio::{
//                 gpioa::{self, PA5, PA6, PA7},
//                 Output, PushPull,
//             },
//             prelude::*,
//         },
//         systick_monotonic::Systick,
//         zy12pdn_wrapper::root::{
//             sink_callback,
//             usb_pd::{i2c_bit_bang, pd_sink},
//         },
//     };

//     #[shared]
//     struct Shared {}

//     #[local]
//     struct Local {
//         led: Rgb<PA5<Output<PushPull>>, PA6<Output<PushPull>>, PA7<Output<PushPull>>>,
//     }

//     #[monotonic(binds = SysTick, default = true)]
//     type MonoTimer = Systick<1000>;

//     #[init]
//     fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
//         let mut flash = cx.device.FLASH;
//         let mut rcc = cx
//             .device
//             .RCC
//             .configure()
//             .sysclk(32u32.mhz())
//             .freeze(&mut flash);

//         let mono = Systick::new(cx.core.SYST, rcc.clocks.sysclk().0);

//         let gpioa::Parts {
//             pa5,
//             pa6,
//             pa7,
//             pa9,
//             pa10,
//             ..
//         } = cx.device.GPIOA.split(&mut rcc);

//         let (pa5, pa6, pa7, sda, scl) = cortex_m::interrupt::free(move |cs| {
//             (
//                 pa5.into_push_pull_output(cs),
//                 pa6.into_push_pull_output(cs),
//                 pa7.into_push_pull_output(cs),
//                 pa9.into_open_drain_output(cs),
//                 pa10.into_push_pull_output(cs),
//             )
//         });

//         let mut led = Rgb::new(pa5, pa6, pa7);
//         led.set(Color::White);

//         info!("init done");

//         let mut i2c = unsafe { MaybeUninit::<i2c_bit_bang>::uninit().assume_init() };
//         unsafe { i2c.init() };

//         trace!("i2c init");

//         let mut power_sink = unsafe { MaybeUninit::<pd_sink>::uninit().assume_init() };
//         unsafe { power_sink.set_event_callback(Some(sink_callback)) }
//         trace!("sink calback");
//         unsafe { power_sink.init() };
//         trace!("i2c init");

//         loop {
//             unsafe { power_sink.poll() };
//         }

//         (Shared {}, Local { led }, init::Monotonics(mono))
//     }

//     #[idle(local = [led])]
//     fn idle(cx: idle::Context) -> ! {
//         loop {
//             //cx.local.pd.poll(monotonics::now());
//         }
//     }
// }

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    error!("panic!");
    loop {}
}
