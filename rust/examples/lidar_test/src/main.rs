#![no_std]
#![no_main]

use defmt::{info, warn, Debug2Format};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    dma,
    gpio::OutputType,
    peripherals,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    usart::{self, Config, Uart},
};
use embassy_time::{with_timeout, Duration, Timer};

use defmt_rtt as _;
use panic_probe as _;

// Reuse the main project's driver so there is only one copy of it
#[path = "../../../RAILRAT_MAIN_PROJECT/src/shared/mod.rs"]
mod shared;
#[path = "../../../RAILRAT_MAIN_PROJECT/src/drivers/lidar.rs"]
mod lidar;

use lidar::Rplidar;


// When code does `.await` on this UART, the task sleeps until one of these
// interrupts wakes it up, so other tasks can run meanwhile.
bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_STREAM5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
});


const STOP_COMMAND: u8 = 0x25;

// Logging is slow. Printing every point lets the lidar outrun the receive
// buffer, which shows up as Uart(Overrun), so only every Nth point is printed.
const LOG_EVERY_N_POINTS: u32 = 8;


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 115_200;

    // The two DMA channels move bytes to and from memory without using the CPU.
    let uart = Uart::new(
        p.USART2,
        p.PD6,       // RX, from lidar TX (yellow)
        p.PD5,       // TX, to lidar RX (green)
        p.DMA1_CH6,  // USART2 TX DMA resource
        p.DMA1_CH5,  // USART2 RX DMA resource
        Irqs,
        config,
    ).unwrap();

    // The four arguments after the timer are channels 1 to 4; only channel 4 is used.
    let pwm = SimplePwm::new(
        p.TIM4,
        None,
        None,
        None,
        Some(PwmPin::new(p.PD15, OutputType::PushPull)),  // Motor control (blue)
        Hertz(25_000),
        CountingMode::EdgeAlignedUp,
    );

    // DMA writes into this buffer for the whole program, so it must live forever.
    // singleton! creates it once and hands back a `&'static mut` reference.
    let rx_buffer = cortex_m::singleton!(: [u8; 1024] = [0; 1024]).unwrap();

    let mut lidar = Rplidar::new(pwm.split().ch4, uart, rx_buffer);

    info!("Spinning up lidar motor");
    lidar.set_motor_speed(60);
    Timer::after_secs(3).await;

    loop {
        // Stop any running scan without stopping the motor
        if let Err(error) = lidar.send_command(STOP_COMMAND).await {
            warn!("Stop command failed: {}", Debug2Format(&error));
        }
        Timer::after_millis(10).await;
        lidar.clear_rx().await;

        info!("Starting scan");

        // with_timeout gives two layers of Result:
        // Ok(Ok(_)) finished, Ok(Err(_)) the lidar call failed, Err(_) time ran out.
        match with_timeout(Duration::from_secs(2), lidar.start()).await {
            Ok(Ok(())) => info!("Lidar responded, scan started"),
            Ok(Err(error)) => {
                warn!("Start failed: {}", Debug2Format(&error));
                Timer::after_secs(1).await;
                continue;
            }
            Err(_) => {
                warn!("No response from lidar, check green/yellow wiring and power");
                Timer::after_secs(1).await;
                continue;
            }
        }

        let mut count: u32 = 0;

        loop {
            // The lidar sends nothing until its motor is stable, so allow a long wait
            match with_timeout(Duration::from_secs(5), lidar.read_point()).await {
                Ok(Ok(point)) => {
                    // Distance 0 marks an invalid measurement
                    if count % LOG_EVERY_N_POINTS == 0
                        && point.distance_mm > 0.0
                        && point.quality > 0
                    {
                        info!(
                            "distance={} mm, angle={} deg, quality={}",
                            point.distance_mm,
                            point.angle_deg,
                            point.quality,
                        );
                    }

                    count = count.wrapping_add(1);
                }
                Ok(Err(error)) => {
                    warn!("Read failed after {} points, restarting scan: {}", count, Debug2Format(&error));
                    break;
                }
                Err(_) => {
                    warn!("Scan data stopped, restarting scan");
                    break;
                }
            }
        }
    }
}
