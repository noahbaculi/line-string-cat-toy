#![crate_type = "dylib"]
#![no_std]
#![no_main]

#[macro_use]
extern crate alloc;

mod map_range;
mod motor;

use crate::map_range::map_range;
use crate::motor::{Motor, MotorDirection};
use alloc::string::ToString;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};
use core::{mem::MaybeUninit, str::from_utf8};
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::analog::adc::{Adc, AdcCalLine, AdcConfig, AdcPin, Attenuation};
use esp_hal::gpio::GpioPin;
use esp_hal::peripherals::ADC1;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::timer::systimer::{SystemTimer, Target};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    clock::ClockControl,
    gpio::Io,
    ledc::{
        channel,
        timer::{self},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_wifi::{
    initialize,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiInitFor,
};
use log::{debug, error, info};
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use static_cell::StaticCell;

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASSWORD");
static TEST: AtomicBool = AtomicBool::new(false);
const BUFFER_SIZE: usize = 4096; // Number of bytes allocated for buffers

const NUM_ADC_SAMPLES: usize = 100; // Number of ADC samples to average
const MAX_ACTIVE_SEC: u16 = 10 * 60; // Number of seconds the device will be active before going to deep sleep
const MIN_MOTOR_DUTY_PERCENT: u8 = 20;
const MAX_MOTOR_DUTY_PERCENT: u8 = 100;
const MIN_MOVEMENT_DURATION: u16 = 200; // ms
const MAX_MOVEMENT_DURATION: u16 = 2_000; // ms
const POTENTIOMETER_READ_INTERVAL: u16 = 200; // ms

const MIN_ADC_VOLTAGE: u16 = 0; // mV
const MAX_ADC_VOLTAGE: u16 = 3000; // mV

static CURRENT_MAX_MOTOR_PERCENT: AtomicU8 = AtomicU8::new(MIN_MOTOR_DUTY_PERCENT);
static CURRENT_MAX_MOVEMENT_DURATION: AtomicU16 = AtomicU16::new(MIN_MOVEMENT_DURATION);
static DRASTIC_PARAMETER_CHANGE: AtomicBool = AtomicBool::new(false);

type RtcMutex = Mutex<CriticalSectionRawMutex, Rtc<'static>>;

type Adc1Calibration = AdcCalLine<ADC1>;
type Adc1Mutex = Mutex<CriticalSectionRawMutex, Adc<'static, ADC1>>;
type AdcPin0MutexForSpeed =
    Mutex<CriticalSectionRawMutex, AdcPin<GpioPin<0>, ADC1, Adc1Calibration>>;
type AdcPin1MutexForDuration =
    Mutex<CriticalSectionRawMutex, AdcPin<GpioPin<1>, ADC1, Adc1Calibration>>;
// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    info!("Started");
    init_heap();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(&clocks, systimer.alarm0);

    let init = initialize(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    // Init network stack
    let config = Config::dhcpv4(Default::default());
    let seed = 1234; // very random, very secure seed
    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

    spawner.spawn(connection(controller)).ok();
    spawner.must_spawn(net_task(stack));
    spawner.must_spawn(start_web_server(stack));

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    static RTC: StaticCell<RtcMutex> = StaticCell::new();
    let rtc = RTC.init(Mutex::new(Rtc::new(peripherals.LPWR)));

    let motor_pwm_pin_forward = io.pins.gpio2;
    let motor_pwm_pin_reverse = io.pins.gpio3;

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

    // Instantiate ADC and mutexes
    let mut adc1_config = AdcConfig::new();
    static SPEED_STATIC_CELL: StaticCell<AdcPin0MutexForSpeed> = StaticCell::new();
    let speed_pot_pin = SPEED_STATIC_CELL.init(Mutex::new(
        adc1_config
            .enable_pin_with_cal::<_, Adc1Calibration>(io.pins.gpio0, Attenuation::Attenuation11dB),
    ));
    static DURATION_STATIC_CELL: StaticCell<AdcPin1MutexForDuration> = StaticCell::new();
    let duration_pot_pin = DURATION_STATIC_CELL.init(Mutex::new(
        adc1_config
            .enable_pin_with_cal::<_, Adc1Calibration>(io.pins.gpio1, Attenuation::Attenuation11dB),
    ));
    let adc1 = Adc::new(peripherals.ADC1, adc1_config);
    static ADC1_MUTEX: StaticCell<Adc1Mutex> = StaticCell::new();
    let adc1 = ADC1_MUTEX.init(Mutex::new(adc1));

    spawner.must_spawn(deep_sleep_countdown(rtc));
    spawner.must_spawn(monitor_speed_pot(adc1, speed_pot_pin));
    spawner.must_spawn(monitor_duration_pot(adc1, duration_pot_pin));

    // Main loop
    let mut small_rng = SmallRng::seed_from_u64(1); // Seed is irrelevant for random number generation
    let mut ticker = Ticker::every(Duration::from_millis(POTENTIOMETER_READ_INTERVAL.into()));
    for direction in [MotorDirection::Forward, MotorDirection::Reverse]
        .iter()
        .cycle()
    {
        let start_time = Instant::now();
        let max_motor_percent = CURRENT_MAX_MOTOR_PERCENT.load(Ordering::Relaxed);
        let duty_percent = small_rng.gen_range(MIN_MOTOR_DUTY_PERCENT..=max_motor_percent);

        let max_movement_duration = CURRENT_MAX_MOVEMENT_DURATION.load(Ordering::Relaxed);
        let movement_duration_ms =
            small_rng.gen_range(MIN_MOVEMENT_DURATION..=max_movement_duration);
        let movement_duration = Duration::from_millis(movement_duration_ms.into());

        motor.start_movement(direction, duty_percent);
        debug!(
            "Movement started: {:?} @ {}% for {} ms",
            direction, duty_percent, movement_duration_ms
        );
        DRASTIC_PARAMETER_CHANGE.store(false, Ordering::Relaxed);

        while Instant::now().duration_since(start_time) <= movement_duration {
            ticker.next().await;
            if DRASTIC_PARAMETER_CHANGE.load(Ordering::Relaxed) {
                debug!("Drastic parameter change detected, breaking loop");
                break; // Break the loop if there is a drastic parameter change
            }
        }
    }
}

#[embassy_executor::task]
async fn deep_sleep_countdown(rtc: &'static RtcMutex) {
    Timer::after(Duration::from_secs(MAX_ACTIVE_SEC.into())).await;
    info!("{} seconds passed, going to deep sleep", MAX_ACTIVE_SEC);
    rtc.lock().await.sleep_deep(&[]);
}

#[embassy_executor::task]
async fn monitor_speed_pot(
    adc1_mutex: &'static Adc1Mutex,
    speed_pot_pin_mutex: &'static AdcPin0MutexForSpeed,
) {
    let mut ticker = Ticker::every(Duration::from_millis(POTENTIOMETER_READ_INTERVAL.into()));
    let mut prev_max_duty_percent = MIN_MOTOR_DUTY_PERCENT;
    loop {
        debug!("Checking speed pot pin (#0)");
        {
            let mut adc1 = adc1_mutex.lock().await;
            let mut speed_pot_pin = speed_pot_pin_mutex.lock().await;
            let avg_pin_value: u16 = ((0..NUM_ADC_SAMPLES)
                .map(|_| nb::block!(adc1.read_oneshot(&mut speed_pot_pin)).unwrap() as u32)
                .sum::<u32>()
                / NUM_ADC_SAMPLES as u32)
                .try_into()
                .expect("Average of ADC readings is too large to fit into u16");
            debug!("Average speed pot pin value: {}", avg_pin_value);
            let max_duty_percent = map_range(
                avg_pin_value as u32,
                MIN_ADC_VOLTAGE.into(),
                MAX_ADC_VOLTAGE.into(),
                MIN_MOTOR_DUTY_PERCENT.into(),
                MAX_MOTOR_DUTY_PERCENT.into(),
            )
            .unwrap()
            .try_into()
            .expect("Max duty percent is too large to fit into u8");
            debug!("Max duty percent: {}", max_duty_percent);
            CURRENT_MAX_MOTOR_PERCENT.store(max_duty_percent, Ordering::Relaxed);

            if prev_max_duty_percent.abs_diff(max_duty_percent) > 10 {
                DRASTIC_PARAMETER_CHANGE.store(true, Ordering::Relaxed);
            }

            prev_max_duty_percent = max_duty_percent;
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn monitor_duration_pot(
    adc1_mutex: &'static Adc1Mutex,
    duration_pot_pin_mutex: &'static AdcPin1MutexForDuration,
) {
    let mut ticker = Ticker::every(Duration::from_millis(POTENTIOMETER_READ_INTERVAL.into()));
    let mut prev_max_duration = MIN_MOVEMENT_DURATION;
    loop {
        debug!("Checking duration pot pin (#1)");
        {
            let mut adc1 = adc1_mutex.lock().await;
            let mut duration_pot_pin = duration_pot_pin_mutex.lock().await;
            let avg_pin_value: u16 = ((0..NUM_ADC_SAMPLES)
                .map(|_| nb::block!(adc1.read_oneshot(&mut duration_pot_pin)).unwrap() as u32)
                .sum::<u32>()
                / NUM_ADC_SAMPLES as u32)
                .try_into()
                .expect("Average of ADC readings is too large to fit into u16");
            debug!("Average duration pot pin value: {}", avg_pin_value);
            let max_duration = map_range(
                avg_pin_value as u32,
                MIN_ADC_VOLTAGE.into(),
                MAX_ADC_VOLTAGE.into(),
                MIN_MOVEMENT_DURATION.into(),
                MAX_MOVEMENT_DURATION.into(),
            )
            .unwrap()
            .try_into()
            .expect("Max duration is too large to fit into u16");
            debug!("Max duration: {}", max_duration);
            CURRENT_MAX_MOVEMENT_DURATION.store(max_duration, Ordering::Relaxed);

            if prev_max_duration.abs_diff(max_duration) > 10 {
                DRASTIC_PARAMETER_CHANGE.store(true, Ordering::Relaxed);
            }

            prev_max_duration = max_duration;
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn start_web_server(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    let mut rx_buffer = [0; BUFFER_SIZE];
    let mut tx_buffer = [0; BUFFER_SIZE];
    let mut buf = [0; BUFFER_SIZE];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        Timer::after(Duration::from_millis(1_000)).await;
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        let port_num = 80;

        info!("Listening on TCP:{port_num}...");
        if let Err(e) = socket.accept(port_num).await {
            error!("Accept error: {:?}", e);
            continue;
        }

        let n = match socket.read(&mut buf).await {
            Ok(0) => continue,
            Ok(n) => n,
            Err(e) => {
                error!("Read error: {:?}", e);
                continue;
            }
        };

        let request = from_utf8(&buf[..n]).unwrap_or("");
        debug!("Request: {}", request);

        let mut response = {
            if request.starts_with("GET / ") {
                let state = TEST.load(Ordering::Relaxed);
                let html_template = include_str!("index.html");
                let html_value = html_template.replace("{state}", &state.to_string());
                format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\n\r\n{}",
                    html_value.len(),
                    html_value
                )
            } else if request.starts_with("POST /toggle") {
                let new_state = !TEST.load(Ordering::Relaxed);
                TEST.store(new_state, Ordering::Relaxed);
                info!("Toggle state changed: {}", new_state);
                let body = format!("{{\"toggled\":{}}}", new_state);
                format!(
                    "HTTP/2.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{}",
                    body.len(),
                    body
                )
            } else if request.starts_with("GET /state") {
                let body = format!("{{\"state\":{}}}", TEST.load(Ordering::Relaxed));
                format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{}",
                    body.len(),
                    body
                )
            } else {
                let message = "Not Found";
                format!(
                    "HTTP/1.1 404 Not Found\r\nContent-Length: {}\r\n\r\n{}",
                    message.len(),
                    message
                )
            }
        };

        if response.len() > BUFFER_SIZE {
            error!(
                "HTTP Response is too large. Truncating to {} bytes...",
                BUFFER_SIZE
            );
            response = response[..BUFFER_SIZE].to_string();
        }

        match socket.write_all(response.as_bytes()).await {
            Ok(_) => {
                if let Err(e) = socket.flush().await {
                    error!("Flush error: {:?}", e);
                }
            }
            Err(e) => error!("Write error: {:?}", e),
        }

        socket.close();
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Starting wifi connection task");
    debug!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        if WifiState::StaConnected == esp_wifi::wifi::get_wifi_state() {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start().await.unwrap();
            info!("Wifi started!");
        }

        match controller.connect().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                let retry_duration = Duration::from_millis(5000);
                info!(
                    "Failed to connect to wifi: {e:?}. Retrying in {}ms",
                    retry_duration.as_millis()
                );
                Timer::after(retry_duration).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}
