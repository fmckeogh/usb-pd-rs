use {
    log::LevelFilter,
    rtt_target::{rprintln, rtt_init_print},
};

static LOGGER: Logger = Logger;

struct Logger;

impl log::Log for Logger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        rprintln!(
            "{:<5} [{}] {}",
            record.level(),
            record.target(),
            record.args()
        );
    }

    fn flush(&self) {}
}

pub fn init_log() {
    rtt_init_print!();
    unsafe { log::set_logger_racy(&LOGGER) }.unwrap();
    log::set_max_level(LevelFilter::Trace);
}
