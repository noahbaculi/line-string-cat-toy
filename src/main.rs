#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut pin1 = Output::new(io.pins.gpio19, Level::Low);
    let mut pin2 = Output::new(io.pins.gpio18, Level::Low);

    loop {
        pin2.set_low();
        pin1.set_high();
        delay.delay_millis(2000);

        pin1.set_low();
        pin2.set_high();
        delay.delay_millis(2000);

        log::info!("Stepping...");
    }
}
