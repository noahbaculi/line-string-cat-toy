#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, OutputPin},
    ledc::{
        channel,
        timer::{self, TimerSpeed},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    peripheral::Peripheral,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::println;

enum MotorDirection {
    Forward,
    Reverse,
}

struct Motor<'a, S, O1, O2>
where
    S: TimerSpeed,
    O1: OutputPin,
    O2: OutputPin,
{
    pwm_channel_forward: channel::Channel<'a, S, O1>,
    pwm_channel_reverse: channel::Channel<'a, S, O2>,
}

impl<'a, S, O1, O2> Motor<'a, S, O1, O2>
where
    S: TimerSpeed,
    O1: OutputPin,
    O2: OutputPin,
{
    fn new(
        ledc_pwm_controller: &'a Ledc,
        pwm_timer: &'a dyn TimerIFace<S>,
        pwm_channel_forward_number: channel::Number,
        pwm_channel_reverse_number: channel::Number,
        pwm_pin_forward: impl Peripheral<P = O1> + 'a,
        pwm_pin_reverse: impl Peripheral<P = O2> + 'a,
    ) -> Self {
        // Instantiate PWM channels
        let mut pwm_channel_forward =
            ledc_pwm_controller.get_channel(pwm_channel_forward_number, pwm_pin_forward);
        pwm_channel_forward
            .configure(channel::config::Config {
                timer: pwm_timer,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();

        let mut pwm_channel_reverse =
            ledc_pwm_controller.get_channel(pwm_channel_reverse_number, pwm_pin_reverse);
        pwm_channel_reverse
            .configure(channel::config::Config {
                timer: pwm_timer,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();

        Self {
            pwm_channel_forward,
            pwm_channel_reverse,
        }
    }

    fn start_movement(&mut self, direction: MotorDirection, duty_percent: u8) {
        match direction {
            MotorDirection::Forward => {
                self.pwm_channel_forward.set_duty(duty_percent).unwrap();
                self.pwm_channel_reverse.set_duty(0).unwrap();
            }
            MotorDirection::Reverse => {
                self.pwm_channel_forward.set_duty(0).unwrap();
                self.pwm_channel_reverse.set_duty(duty_percent).unwrap();
            }
        }
    }

    fn stop(&mut self) {
        self.pwm_channel_forward.set_duty(0).unwrap();
        self.pwm_channel_reverse.set_duty(0).unwrap();
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let motor_pwm_pin_forward = io.pins.gpio18;
    let motor_pwm_pin_reverse = io.pins.gpio19;

    esp_println::logger::init_logger_from_env();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

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

    let mut motor = Motor::new(
        &ledc_pwm_controller,
        &pwm_timer,
        channel::Number::Channel0,
        channel::Number::Channel1,
        motor_pwm_pin_forward,
        motor_pwm_pin_reverse,
    );

    loop {
        motor.start_movement(MotorDirection::Forward, 25);
        delay.delay_millis(2000);

        motor.start_movement(MotorDirection::Forward, 100);
        delay.delay_millis(2000);

        motor.start_movement(MotorDirection::Reverse, 25);
        delay.delay_millis(2000);

        motor.start_movement(MotorDirection::Reverse, 100);
        delay.delay_millis(2000);

        motor.stop();
        delay.delay_millis(2000);

        println!("Stepping...");
    }
}
