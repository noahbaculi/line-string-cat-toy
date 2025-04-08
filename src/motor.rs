use esp_hal::gpio::Output;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::{
    gpio::OutputPin,
    ledc::{channel, timer::TimerSpeed, Ledc},
    peripheral::Peripheral,
    prelude::*,
};

#[derive(Debug)]
pub enum MotorDirection {
    Forward,
    Reverse,
}

/// Abstraction for the TB6612FNG motor driver
pub struct Motor<'a, S, O1, O2, O3, O4>
where
    S: TimerSpeed,
    O1: OutputPin,
    O2: OutputPin,
    O3: OutputPin,
    O4: OutputPin,
{
    pin_standby: Output<'a, O2>,
    pwm_channel_a: channel::Channel<'a, S, O1>,
    pin_a_in_1: Output<'a, O3>,
    pin_a_in_2: Output<'a, O4>,
}

impl<'a, S, O1, O2, O3, O4> Motor<'a, S, O1, O2, O3, O4>
where
    S: TimerSpeed,
    O1: OutputPin,
    O2: OutputPin,
    O3: OutputPin,
    O4: OutputPin,
{
    pub fn new(
        ledc_pwm_controller: &'a Ledc,
        pwm_timer: &'a dyn TimerIFace<S>,
        pwm_channel_a: channel::Number,
        pwm_pin: impl Peripheral<P = O1> + 'a,
        pin_standby: Output<'a, O2>,
        pin_a_in_1: Output<'a, O3>,
        pin_a_in_2: Output<'a, O4>,
    ) -> Self {
        // Instantiate PWM channels
        let mut pwm_channel_forward = ledc_pwm_controller.get_channel(pwm_channel_a, pwm_pin);
        pwm_channel_forward
            .configure(channel::config::Config {
                timer: pwm_timer,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();

        Self {
            pwm_channel_a: pwm_channel_forward,
            pin_standby,
            pin_a_in_1,
            pin_a_in_2,
        }
    }

    pub fn start_movement(&mut self, direction: &MotorDirection, duty_percent: u8) {
        self.pin_standby.set_high();
        match direction {
            MotorDirection::Forward => {
                self.pin_a_in_1.set_high();
                self.pin_a_in_2.set_low();
            }
            MotorDirection::Reverse => {
                self.pin_a_in_1.set_low();
                self.pin_a_in_2.set_high();
            }
        }
        self.pwm_channel_a
            .set_duty(duty_percent)
            .expect("Failed to set duty cycle");
    }

    pub fn stop(&mut self) {
        self.pin_a_in_1.set_low();
        self.pin_a_in_2.set_low();
        self.pwm_channel_a
            .set_duty(0)
            .expect("Failed to set duty cycle");
        self.pin_standby.set_low();
    }
}
