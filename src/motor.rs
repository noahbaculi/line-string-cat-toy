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

pub struct Motor<'a, S, O1, O2>
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
    pub fn new(
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

    pub fn start_movement(&mut self, direction: &MotorDirection, duty_percent: u8) {
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

    // fn stop(&mut self) {
    //     self.pwm_channel_forward.set_duty(0).unwrap();
    //     self.pwm_channel_reverse.set_duty(0).unwrap();
    // }
}
