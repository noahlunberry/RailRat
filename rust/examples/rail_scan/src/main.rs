#![no_std]
#![no_main]

// Track scanning run: streams EVERY byte the lidar sends to the computer and
// drives the cart for as long as you want.
//
// - At startup the lidar is asked which scan modes it has, and the one with the
//   most samples per second is used (more points per revolution). The list is
//   printed so you can see what was picked.
// - The lidar's raw scan bytes are sent over the debug probe (defmt/RTT) undecoded,
//   in numbered chunks, so the computer can check that nothing was lost.
//   `record.py` decodes them and saves every point to a CSV file.
// - probe-rs puts the defmt channel in blocking mode, so when the computer falls
//   behind the board waits instead of dropping data. Meanwhile the lidar keeps
//   streaming into a large DMA buffer (the queue), which gets read once the
//   computer catches up.
// - The cart starts driving AUTO_START_SECS after boot. The blue USER button
//   stops it, and pressing again starts it again.
// - Distance along the track is estimated as CART_MM_PER_SEC x driving time.
// - Watchdog: if the board stops running (probe halted it, or the laptop stopped
//   reading for WATCHDOG_SECS), the chip resets, which stops the cart.
//
// Run from `rust/`: `python examples/rail_scan/record.py`

use defmt::{info, warn, Debug2Format};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    dma,
    gpio::{Input, Level, Output, OutputType, Pull, Speed},
    peripherals,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    usart::{self, Config, Uart},
    wdg::IndependentWatchdog,
};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use core::sync::atomic::{AtomicBool, Ordering};

use defmt_rtt as _;
use panic_probe as _;

use railrat::drivers::{MotorDriver, Rplidar};


bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
});


// Cart motor speed in percent. Negative runs it in reverse.
const DRIVE_SPEED: i8 = 90;
// Lower PWM frequency = more torque at the same % on the L298N (less switching loss)
const MOTOR_PWM_HZ: u32 = 1_000;
// Estimated cart speed at DRIVE_SPEED, used for distance along the track.
// Calibrate: drive for 10 s, measure the distance, divide by 10.
const CART_MM_PER_SEC: i32 = 300;
// Seconds after boot before the cart starts on its own. None = wait for the button.
const AUTO_START_SECS: Option<u64> = Some(5);

// Stop-and-go: drive for STEP_MS, stop, wait SETTLE_MS for the cart to stop
// rocking, then scan the same spot for HOLD_MS (a "station"), and repeat.
// Each station stacks ~HOLD_MS/250 lidar revolutions on one cross-section with
// no motion blur, which is what gauge measurement needs.
// false = drive continuously.
const STOP_AND_GO: bool = false;
const STEP_MS: u64 = 400;
const SETTLE_MS: u64 = 300;
const HOLD_MS: u64 = 2000;

// The cart runs until Ctrl+C in record.py, which stops it within ~3-5 s: the
// board stops being read, the watchdog resets it, and it does not auto-start
// after a watchdog reset. The blue button stops it instantly.

// Slower spin = more points per degree, but fewer cross-sections per meter of track.
// 60 gave ~4 rev/s. Aim for 7-9 rev/s (A1 max is 10): twice the cross-sections
// per meter, still ~5 mm between points at the rail lip.
const LIDAR_MOTOR_SPEED: u8 = 85;
// None = use the scan mode with the most samples per second. Some(id) forces one
// (ids are printed at startup).
// Boost (2): on the track, Boost and Stability saw the rail heads in ~99% of
// revolutions vs 6-33% for Sensitivity (3), which ties Boost on samples/s and
// would otherwise be auto-picked as the lidar's "typical" mode.
const SCAN_MODE: Option<u16> = Some(2);
// Mode comparison: run each of these scan modes for SECS_PER_MODE, then stop the
// cart. Every point in points.csv is labeled with its mode. Empty = normal run
// using SCAN_MODE, forever.
const MODE_SEQUENCE: &[u16] = &[];  // e.g. &[2, 4] to compare Boost and Stability
const SECS_PER_MODE: u64 = 90;

const STOP_COMMAND: u8 = 0x25;

// Receive queue for lidar bytes. The lidar sends about 11 KB/s, so this holds
// about 4 s of data if the computer falls behind. DMA limits it to under 64 KB.
const LIDAR_RX_BUFFER_SIZE: usize = 48 * 1024;
// Bytes per message to the computer. Larger chunks mean less overhead per byte.
const CHUNK_SIZE: usize = 250;

const WATCHDOG_SECS: u32 = 3;
const BUTTON_POLL_MS: u64 = 20;

// Set by the lidar task when MODE_SEQUENCE is finished; main then stops the cart
static ALL_MODES_DONE: AtomicBool = AtomicBool::new(false);

static mut LIDAR_RX_BUFFER: [u8; LIDAR_RX_BUFFER_SIZE] = [0; LIDAR_RX_BUFFER_SIZE];


/// True if the last reset was the watchdog. Clears the reset flags.
/// RCC_CSR on STM32F4: IWDGRSTF is bit 29, writing RMVF (bit 24) clears the flags.
fn take_watchdog_reset_flag() -> bool {
    const RCC_CSR: *mut u32 = 0x4002_3874 as *mut u32;
    unsafe {
        let csr = core::ptr::read_volatile(RCC_CSR);
        core::ptr::write_volatile(RCC_CSR, csr | (1 << 24));
        csr & (1 << 29) != 0
    }
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let watchdog_reset = take_watchdog_reset_flag();
    let p = embassy_stm32::init(Default::default());

    if watchdog_reset {
        warn!("Restarted by the watchdog. Cart will NOT start on its own; press the blue button.");
    }

    let mut watchdog = IndependentWatchdog::new(p.IWDG, WATCHDOG_SECS * 1_000_000);
    watchdog.unleash();

    // Cart motor (L298N)
    let motor_pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new(p.PE9, OutputType::PushPull)),  // L298N ENA
        None,
        None,
        None,
        Hertz(MOTOR_PWM_HZ),
        CountingMode::EdgeAlignedUp,
    );
    let in1 = Output::new(p.PG9, Level::Low, Speed::Low);
    let in2 = Output::new(p.PG14, Level::Low, Speed::Low);
    let mut motor = MotorDriver::new(motor_pwm.split().ch1, in1, in2);
    motor.brake();

    // Blue USER button on the Nucleo, reads high while pressed
    let button = Input::new(p.PC13, Pull::Down);

    // Lidar
    let mut config = Config::default();
    config.baudrate = 115_200;

    let uart = Uart::new(
        p.USART2,
        p.PD6,       // RX, from lidar TX (yellow)
        p.PD5,       // TX, to lidar RX (green)
        p.DMA1_CH6,  // USART2 TX DMA resource
        p.DMA1_CH5,  // USART2 RX DMA resource
        Irqs,
        config,
    ).unwrap();

    let lidar_pwm = SimplePwm::new(
        p.TIM4,
        None,
        None,
        None,
        Some(PwmPin::new(p.PD15, OutputType::PushPull)),  // Lidar motor control (blue)
        Hertz(25_000),
        CountingMode::EdgeAlignedUp,
    );

    // DMA writes into this buffer for the whole program. Main runs once, so this
    // is the only reference to it.
    let rx_buffer = unsafe { &mut *core::ptr::addr_of_mut!(LIDAR_RX_BUFFER) };

    let mut lidar = Rplidar::new(lidar_pwm.split().ch4, uart, rx_buffer);

    info!("Spinning up lidar motor");
    lidar.set_motor_speed(LIDAR_MOTOR_SPEED);
    Timer::after_secs(2).await;
    watchdog.pet();

    spawner.spawn(lidar_task(lidar).unwrap());

    let mut auto_start_pending = AUTO_START_SECS.is_some() && !watchdog_reset;

    match AUTO_START_SECS {
        Some(secs) if auto_start_pending => info!("Cart starts in {} s. Blue button stops/starts it.", secs),
        _ => info!("Press the blue button to start/stop the cart."),
    }

    let boot = Instant::now();
    let mut was_pressed = false;

    let mut phase = Phase::Stopped;
    let mut phase_until = Instant::now();
    let mut station: u32 = 0;

    // Estimated distance along the track, in mm
    let mut odometer_mm: f32 = 0.0;
    let mut last_tick = Instant::now();

    loop {
        watchdog.pet();
        Timer::after_millis(BUTTON_POLL_MS).await;

        let now = Instant::now();
        if phase == Phase::Moving {
            let dt_s = (now - last_tick).as_micros() as f32 / 1e6;
            odometer_mm += signed_cart_speed() as f32 * dt_s;
        }
        last_tick = now;

        // Rising edge = button just went down
        let pressed = button.is_high();
        let just_pressed = pressed && !was_pressed;
        was_pressed = pressed;

        let mut start = false;
        let mut stop = false;

        if just_pressed {
            info!("Button pressed");
            auto_start_pending = false;
            if phase == Phase::Stopped { start = true } else { stop = true }
        }

        if let (true, Some(secs)) = (auto_start_pending, AUTO_START_SECS) {
            if boot.elapsed() >= Duration::from_secs(secs) {
                auto_start_pending = false;
                start = true;
            }
        }

        if ALL_MODES_DONE.load(Ordering::Relaxed) {
            auto_start_pending = false;
            if phase != Phase::Stopped {
                stop = true;
                start = false;
            }
        }

        if start {
            motor_on(&mut motor, now);
            phase = Phase::Moving;
            phase_until = now + Duration::from_millis(STEP_MS);
        } else if stop {
            match phase {
                Phase::Moving => motor_off(&mut motor, now),
                Phase::Holding => station_end(station, now),
                _ => {}
            }
            motor.brake();
            phase = Phase::Stopped;
            info!("Cart stopped, about {} m along the track", odometer_mm / 1000.0);
        } else if STOP_AND_GO && now >= phase_until {
            match phase {
                Phase::Moving => {
                    motor_off(&mut motor, now);
                    phase = Phase::Settling;
                    phase_until = now + Duration::from_millis(SETTLE_MS);
                }
                Phase::Settling => {
                    station += 1;
                    // STATION_START <n> <time us> <estimated mm along track> is read by record.py
                    info!("STATION_START {=u32} {=u64} {=i32}", station, now.as_micros(), odometer_mm as i32);
                    phase = Phase::Holding;
                    phase_until = now + Duration::from_millis(HOLD_MS);
                }
                Phase::Holding => {
                    station_end(station, now);
                    motor_on(&mut motor, now);
                    phase = Phase::Moving;
                    phase_until = now + Duration::from_millis(STEP_MS);
                }
                Phase::Stopped => {}
            }
        }

        // Debounce
        if just_pressed {
            Timer::after_millis(200).await;
        }
    }
}


#[derive(PartialEq)]
enum Phase {
    Stopped,
    Moving,
    // Motor off, waiting for the cart to stop rocking
    Settling,
    // Standing still, scanning one station
    Holding,
}


fn motor_on(motor: &mut MotorDriver<peripherals::TIM1>, now: Instant) {
    motor.set_speed(DRIVE_SPEED);
    // DRIVE <time us> <motor %> <estimated mm/s> is read by record.py
    info!("DRIVE {=u64} {=i8} {=i32}", now.as_micros(), DRIVE_SPEED, signed_cart_speed());
}


fn motor_off(motor: &mut MotorDriver<peripherals::TIM1>, now: Instant) {
    motor.brake();
    info!("DRIVE {=u64} {=i8} {=i32}", now.as_micros(), 0i8, 0i32);
}


fn station_end(station: u32, now: Instant) {
    info!("STATION_END {=u32} {=u64}", station, now.as_micros());
}


/// Estimated cart speed in mm/s, negative when DRIVE_SPEED is reverse.
fn signed_cart_speed() -> i32 {
    if DRIVE_SPEED < 0 { -CART_MM_PER_SEC } else { CART_MM_PER_SEC }
}


// GET_LIDAR_CONF types
const CONF_SCAN_MODE_COUNT: u32 = 0x70;
const CONF_SCAN_MODE_US_PER_SAMPLE: u32 = 0x71;
const CONF_SCAN_MODE_MAX_DISTANCE: u32 = 0x74;
const CONF_SCAN_MODE_ANS_TYPE: u32 = 0x75;
const CONF_SCAN_MODE_TYPICAL: u32 = 0x7C;
const CONF_SCAN_MODE_NAME: u32 = 0x7F;

// Stream formats record.py can decode
const ANS_STANDARD: u8 = 0x81;
const ANS_CAPSULED: u8 = 0x82;
const ANS_ULTRA_CAPSULED: u8 = 0x84;
const ANS_DENSE_CAPSULED: u8 = 0x85;

#[derive(Clone, Copy)]
struct ScanMode {
    id: u16,
    answer_type: u8,
}

const STANDARD_MODE: ScanMode = ScanMode { id: 0, answer_type: ANS_STANDARD };


async fn conf(
    lidar: &mut Rplidar<peripherals::TIM4>,
    conf_type: u32,
    mode: Option<u16>,
    out: &mut [u8],
) -> Option<usize> {
    match with_timeout(Duration::from_secs(1), lidar.get_lidar_conf(conf_type, mode, out)).await {
        Ok(Ok(n)) => Some(n),
        _ => None,
    }
}


/// Lists the lidar's scan modes and picks the one with the most samples per second.
async fn choose_scan_mode(lidar: &mut Rplidar<peripherals::TIM4>, forced_mode: Option<u16>) -> ScanMode {
    let mut buf = [0u8; 64];

    let Some(2..) = conf(lidar, CONF_SCAN_MODE_COUNT, None, &mut buf).await else {
        warn!("Lidar can't list scan modes (firmware older than 1.24?), using standard scan");
        return STANDARD_MODE;
    };
    let count = u16::from_le_bytes([buf[0], buf[1]]);

    let typical = match conf(lidar, CONF_SCAN_MODE_TYPICAL, None, &mut buf).await {
        Some(2..) => Some(u16::from_le_bytes([buf[0], buf[1]])),
        _ => None,
    };

    let mut best: Option<(ScanMode, u32)> = None;

    for id in 0..count {
        let us_per_sample_q8 = match conf(lidar, CONF_SCAN_MODE_US_PER_SAMPLE, Some(id), &mut buf).await {
            Some(4..) => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            _ => continue,
        };
        let max_distance_q8 = match conf(lidar, CONF_SCAN_MODE_MAX_DISTANCE, Some(id), &mut buf).await {
            Some(4..) => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            _ => 0,
        };
        let answer_type = match conf(lidar, CONF_SCAN_MODE_ANS_TYPE, Some(id), &mut buf).await {
            Some(1..) => buf[0],
            _ => continue,
        };
        let name_len = conf(lidar, CONF_SCAN_MODE_NAME, Some(id), &mut buf).await.unwrap_or(0);
        let name_len = buf[..name_len].iter().position(|&b| b == 0).unwrap_or(name_len);
        let name = core::str::from_utf8(&buf[..name_len]).unwrap_or("?");

        let us_per_sample = us_per_sample_q8 as f32 / 256.0;
        info!(
            "Scan mode {}: {} - {} samples/s, max {} m, format 0x{:02x}",
            id, name, (1e6 / us_per_sample) as u32, max_distance_q8 as f32 / 256.0, answer_type,
        );

        let mode = ScanMode { id, answer_type };
        let decodable = matches!(answer_type, ANS_STANDARD | ANS_CAPSULED | ANS_ULTRA_CAPSULED | ANS_DENSE_CAPSULED);

        if let Some(forced) = forced_mode {
            if forced == id {
                best = Some((mode, us_per_sample_q8));
            }
            continue;
        }

        if !decodable {
            continue;
        }

        // Fewest microseconds per sample wins; on a tie prefer the lidar's typical mode
        let better = match best {
            None => true,
            Some((_, best_us)) => us_per_sample_q8 < best_us
                || (us_per_sample_q8 == best_us && Some(id) == typical),
        };
        if better {
            best = Some((mode, us_per_sample_q8));
        }
    }

    match best {
        Some((mode, _)) => {
            info!("Using scan mode {}", mode.id);
            mode
        }
        None => {
            warn!("No usable scan mode found, using standard scan");
            STANDARD_MODE
        }
    }
}


async fn stop_scan(lidar: &mut Rplidar<peripherals::TIM4>) {
    // Stop any running scan without stopping the lidar motor
    if let Err(error) = lidar.send_command(STOP_COMMAND).await {
        warn!("Stop command failed: {}", Debug2Format(&error));
    }
    Timer::after_millis(10).await;
    lidar.clear_rx().await;
}


/// Starts a scan in `mode`. Returns the stream format, or None on failure.
async fn start_scan(lidar: &mut Rplidar<peripherals::TIM4>, mode: ScanMode) -> Option<u8> {
    let result = if mode.answer_type == ANS_STANDARD && mode.id == 0 {
        with_timeout(Duration::from_secs(2), lidar.start()).await.map(|r| r.map(|_| ANS_STANDARD))
    } else {
        with_timeout(Duration::from_secs(2), lidar.start_express(mode.id)).await
    };

    match result {
        Ok(Ok(answer_type)) if answer_type == mode.answer_type => Some(answer_type),
        Ok(Ok(answer_type)) => {
            warn!("Expected format 0x{:02x}, lidar sent 0x{:02x}", mode.answer_type, answer_type);
            None
        }
        Ok(Err(error)) => {
            warn!("Start failed: {}", Debug2Format(&error));
            None
        }
        Err(_) => {
            warn!("No response from lidar, check green/yellow wiring and power");
            None
        }
    }
}


#[embassy_executor::task]
async fn lidar_task(mut lidar: Rplidar<peripherals::TIM4>) {
    // Chunk counter, never reset, so the computer can spot any missing chunk
    let mut seq: u32 = 0;

    if MODE_SEQUENCE.is_empty() {
        stop_scan(&mut lidar).await;
        let mode = choose_scan_mode(&mut lidar, SCAN_MODE).await;
        scan_until(&mut lidar, mode, None, &mut seq).await;
    }

    for (i, &mode_id) in MODE_SEQUENCE.iter().enumerate() {
        stop_scan(&mut lidar).await;
        let mode = choose_scan_mode(&mut lidar, Some(mode_id)).await;
        info!("MODE TEST {} of {}: scan mode {} for {} s", i + 1, MODE_SEQUENCE.len(), mode.id, SECS_PER_MODE);
        let deadline = Instant::now() + Duration::from_secs(SECS_PER_MODE);
        scan_until(&mut lidar, mode, Some(deadline), &mut seq).await;
    }

    stop_scan(&mut lidar).await;
    lidar.set_motor_speed(0);
    ALL_MODES_DONE.store(true, Ordering::Relaxed);
    info!("ALL MODES DONE, cart stopped. Press Ctrl+C to save.");

    loop {
        Timer::after_secs(3600).await;
    }
}


/// Scans in `mode` until `deadline` (or forever), restarting the scan after errors.
async fn scan_until(
    lidar: &mut Rplidar<peripherals::TIM4>,
    mode: ScanMode,
    deadline: Option<Instant>,
    seq: &mut u32,
) {
    let mut chunk = [0u8; CHUNK_SIZE];
    let expired = || deadline.is_some_and(|d| Instant::now() >= d);

    while !expired() {
        stop_scan(lidar).await;

        let Some(answer_type) = start_scan(lidar, mode).await else {
            Timer::after_secs(1).await;
            continue;
        };

        // Bytes after this are a fresh stream in this format; record.py restarts decoding here.
        // SCAN_START <time us> <format> <mode id>
        info!("SCAN_START {=u64} {=u8} {=u16}", Instant::now().as_micros(), answer_type, mode.id);

        let mut scanning = true;

        while scanning && !expired() {
            let mut filled = 0;

            while filled < CHUNK_SIZE {
                match with_timeout(Duration::from_secs(2), lidar.read_bytes(&mut chunk[filled..])).await {
                    Ok(Ok(n)) => filled += n,
                    Ok(Err(error)) => {
                        // Usually an overrun: the queue filled up and bytes were lost
                        warn!("LOST lidar read failed at chunk {}, restarting scan: {}", *seq, Debug2Format(&error));
                        scanning = false;
                        break;
                    }
                    Err(_) => {
                        warn!("Scan data stopped, restarting scan");
                        scanning = false;
                        break;
                    }
                }
            }

            // Send what we have, even a partial chunk before a restart.
            // Time is when the chunk's last byte was read.
            if filled > 0 {
                info!("D {=u32} {=u64} {=[u8]}", *seq, Instant::now().as_micros(), &chunk[..filled]);
                *seq = seq.wrapping_add(1);
            }
        }
    }
}
