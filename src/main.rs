#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;

use embassy_stm32::{
    bind_interrupts,
    dma,
    gpio::{Level, Output, Speed},
    mode::Async,
    peripherals,
    usart::{self, Config, DataBits, Parity, StopBits, Uart, UartTx, UartRx},
};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};

use defmt_rtt as _;
use panic_probe as _;


bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;

    DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_STREAM1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
});


/*
 *  GLOBAL SIGNALS
 */

static RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new(); // Global signal to be triggered when UART receive



/*
 *  TASKS
 */

// LED blink task
#[embassy_executor::task(pool_size = 3)]
async fn blink_led(
    mut led : Output<'static>,
    num_ms : u64,
) {
    loop {
        led.toggle();
        Timer::after_millis(num_ms).await;
    }
}

#[embassy_executor::task]
async fn flash_led_task(
    mut led: Output<'static>,
) {
    loop {
        // Wait until a character is received
        RX_SIGNAL.wait().await;

        // Flash LED
        led.set_high();
        Timer::after_millis(50).await;
        led.set_low();
    }
}


/*
 *  FUNCTIONS
 */

// UART transmit function
async fn uart_transmit(
    uart: &mut UartTx<'_, Async>,
    data: &[u8],
) {
    uart.write(data).await.unwrap();
}


// UART receive function
async fn uart_receive(
    uart: &mut UartRx<'_, Async>,
    buffer: &mut [u8],
) {
    uart.read(buffer).await.unwrap();
}


// Embassy main function
#[embassy_executor::main]
async fn main(spawner : Spawner) {

    // Initialize STM32 peripherals
    let p = embassy_stm32::init(Default::default());

    // Configure UART
    let mut config = Config::default();
    config.baudrate = 115_200;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityNone;
    config.stop_bits = StopBits::STOP1;

    // Configure PB0 (green LED)
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

    // Initialize DMA-driven UART
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

    // UART receive buffer
    let mut buffer = [0u8; 1];

    // Send initial message
    let initial_msg = b"UART echo program >:D\r\nNote: enter key does not work lol\r\n";
    uart_transmit(&mut uart3_tx, initial_msg).await;

    // Spawn task(s)
    spawner.spawn(blink_led(green_led, 500).unwrap());
    spawner.spawn(flash_led_task(blue_led).unwrap());
    
    loop {
        uart_receive(&mut uart3_rx, &mut buffer).await;

        RX_SIGNAL.signal(()); // Trigger signal

        uart_transmit(&mut uart3_tx, &buffer).await;
    }

}