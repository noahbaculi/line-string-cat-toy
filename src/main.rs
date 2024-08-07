#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed},
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

    // Instantiate PWM infra
    let mut ledc_pwm_controller = Ledc::new(peripherals.LEDC, &clocks);
    ledc_pwm_controller.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut pwm_timer = ledc_pwm_controller.get_timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 2.kHz(),
        })
        .unwrap();

    let motor_pwm_pin_forward = io.pins.gpio18;
    let motor_pwm_pin_reverse = io.pins.gpio19;

    // Instantiate PWM channels
    let mut motor_pwm_channel_forward =
        ledc_pwm_controller.get_channel(channel::Number::Channel0, motor_pwm_pin_forward);
    motor_pwm_channel_forward
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut motor_pwm_channel_reverse =
        ledc_pwm_controller.get_channel(channel::Number::Channel1, motor_pwm_pin_reverse);
    motor_pwm_channel_reverse
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    loop {
        motor_pwm_channel_reverse.set_duty(0).unwrap();
        motor_pwm_channel_forward.set_duty(25).unwrap();
        delay.delay_millis(2000);

        motor_pwm_channel_reverse.set_duty(0).unwrap();
        motor_pwm_channel_forward.set_duty(100).unwrap();
        delay.delay_millis(2000);

        motor_pwm_channel_forward.set_duty(0).unwrap();
        motor_pwm_channel_reverse.set_duty(25).unwrap();
        delay.delay_millis(2000);

        motor_pwm_channel_forward.set_duty(0).unwrap();
        motor_pwm_channel_reverse.set_duty(100).unwrap();
        delay.delay_millis(2000);

        log::info!("Stepping...");
    }
}
