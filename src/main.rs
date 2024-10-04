#![crate_type = "dylib"]
#![no_std]
#![no_main]
#[macro_use]
extern crate alloc;

use alloc::string::ToString;
use core::{
    mem::MaybeUninit,
    str::from_utf8,
    sync::atomic::{AtomicBool, Ordering},
};
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, peripherals::Peripherals, rng::Rng, system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_hal::{
    peripherals::WIFI,
    timer::systimer::{SystemTimer, Target},
};
use esp_wifi::{
    initialize,
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiInitFor, EspWifiInitialization,
};
use log::{debug, error, info};

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASSWORD");
static TEST: AtomicBool = AtomicBool::new(false);
const BUFFER_SIZE: usize = 4096; // Number of bytes allocated for buffers

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
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    info!("Started");
    init_heap();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    let init = initialize(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(&clocks, systimer.alarm0);

    spawner.must_spawn(start_web_server(init, peripherals.WIFI));

    loop {
        todo!();
    }
}

#[embassy_executor::task]
async fn start_web_server(init: EspWifiInitialization, wifi: WIFI) {
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

    connection(controller).await;
    stack.run().await;

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

        let response = {
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
                info!("Failed to connect to wifi: {e:?}. Retrying in {retry_duration}");
                Timer::after(retry_duration).await
            }
        }
    }
}
