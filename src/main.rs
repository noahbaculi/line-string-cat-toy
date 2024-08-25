#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::timer::{ErasedTimer, OneShotTimer};
use esp_hal::{
    clock::ClockControl,
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
use log::info;
use static_cell::StaticCell;

const MAX_ACTIVE_SEC: u64 = 10 * 60; // Number of seconds the device will be active before going to deep sleep

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

type RtcMutex = Mutex<CriticalSectionRawMutex, Rtc<'static>>;

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock80MHz).freeze();
    esp_println::logger::init_logger(log::LevelFilter::Debug);
    let timer_grp = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    esp_hal_embassy::init(
        &clocks,
        mk_static!(
            [OneShotTimer<ErasedTimer>; 1],
            [OneShotTimer::new(timer_grp.timer0.into())]
        ),
    );
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    static RTC: StaticCell<RtcMutex> = StaticCell::new();
    let rtc = RTC.init(Mutex::new(Rtc::new(peripherals.LPWR, None)));

    let motor_pwm_pin_forward = io.pins.gpio18;
    let motor_pwm_pin_reverse = io.pins.gpio19;

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

    motor.start_movement(MotorDirection::Forward, 25);
    motor.start_movement(MotorDirection::Reverse, 25);
    motor.stop();

    spawner.must_spawn(deep_sleep_countdown(rtc));
}

#[embassy_executor::task]
async fn deep_sleep_countdown(rtc: &'static RtcMutex) {
    Timer::after(Duration::from_secs(MAX_ACTIVE_SEC)).await;
    info!("{} seconds passed, going to deep sleep", MAX_ACTIVE_SEC);
    rtc.lock().await.sleep_deep(&[]);
}

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
