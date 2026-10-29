#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_stm32::{
    bind_interrupts,
    dma,
    gpio::{Level, Output, OutputType, Speed},
    mode::Async,
    peripherals,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    usart::{self, Config, DataBits, Parity, StopBits, Uart, UartTx, UartRx},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};

use defmt_rtt as _;
use panic_probe as _;

mod drivers;
use crate::drivers::motor_driver::MotorDriver;
// use crate::drivers::{MotorDriver, GPS, LiDAR};



bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;

    DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_STREAM1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
});



//  GLOBAL SIGNALS

static EXAMPLE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();



#[embassy_executor::main]
async fn main(spawner: Spawner) {

    // STM32 peripherals
    let p = embassy_stm32::init(Default::default());

    // GPIO
    let green_led = Output::new(
        p.PB0,
        Level::Low,
        Speed::Low,
    );
    let blue_led = Output::new(
        p.PB7,
        Level::Low,
        Speed::Low,
    );
    let red_led = Output::new(
        p.PB14,
        Level::Low,
        Speed::Low,
    );

    let motor_driver_in1 = Output::new(
        p.PG2,
        Level::Low,
        Speed::Low,
    );
    let motor_driver_in2 = Output::new(
        p.PG3,
        Level::Low,
        Speed::Low,
    );



    // UART

    let mut config = Config::default();
    config.baudrate = 115_200;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityNone;
    config.stop_bits = StopBits::STOP1;

    // DMA-driven UART

    let uart3 = Uart::new(
        p.USART3,
        p.PD9,       // RX
        p.PD8,       // TX
        p.DMA1_CH3,   // USART3 TX DMA resource
        p.DMA1_CH1,      // USART3 RX DMA resource
        Irqs,
        config,
    ).unwrap();
    let (mut uart3_tx, mut uart3_rx) = uart3.split();



    // PMW

    let mut pwm = SimplePwm::new(
        p.TIM4,
        None,
        None,
        Some(PwmPin::new(p.PD14, OutputType::PushPull)),
        Some(PwmPin::new(p.PD15, OutputType::PushPull)),
        Hertz(10_000),
        CountingMode::EdgeAlignedUp,
    );
    
    let pwm_channels = pwm.split();

    let mut motor_pwm = pwm_channels.ch3;




    //  INITIALIZE RAILRAT MODULES

    let mut motor_driver = MotorDriver::new(
        motor_pwm,
        motor_driver_in1,
        motor_driver_in2,
    );




    //  SPAWNING TASKS



    

    //  MAIN LOOP

    let speeds: [i8; 5] = [100, 50, 0, -50, -100];
    let mut buffer = [0u8; 1];
    
    
    loop {

        let mut iteration = 1;

        for speed in speeds.iter() {
            motor_driver.set_speed(*speed);
            uart3_tx.write(&[b'0' + iteration]).await.unwrap();
            uart3_tx.write(b"\r\n\n").await.unwrap();

            uart3_rx.read(&mut buffer).await.unwrap();
            iteration += 1;
        }
        
    }

}