use embassy_stm32::{
    usart::{
        Uart, UartTx, RingBufferedUartRx
    },
    mode::Async,

}



use crate::shared::uart::{uart_read, uart_write, UartError};

#[derive(Debug)]
pub enum GpsError {
    Uart(UartError),
    InvalidSentence,
    BufferOverflow,
}

impl From<UartError> for GpsError {
    fn from(error: UartError) -> Self {
        Self::Uart(error)
    }
}



// The SAM-M10Q's default UART configurations are:
// BAUD = 9600
// 8N1
// I/O in NMEA
const DEFAULT_BAUD = 9600;



pub struct Gps {
    tx: UartTx<'static, Async>,
    rx: RingBufferedUartRx<'static>,
}


impl Gps {
    
    pub fn new(
        uart: Uart<'static, Async>,
    ) -> Self {

        let (tx, rx) = uart.split();

        Self { tx, rx }
    }

    
    async fn read(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), GpsError> {


    }
}