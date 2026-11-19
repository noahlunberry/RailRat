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
// NMEA sentences in RMC
const DEFAULT_BAUD = 9600;



#[derive(Debug, Clone, Copy, Default)]
pub struct GpsData {
    pub latitude_deg: Option<f64>,
    pub longitude_deg: Option<f64>,
    pub altitude_m: Option<f32>,
    pub speed_mps: Option<f32>,
    pub satellites: Option<u8>,
    pub fix_valid: bool,
}

#[derive(Debug)]
pub enum GpsError {
    Uart(usart::Error),
    InvalidSentence,
    InvalidChecksum,
    BufferOverflow,
}


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


    async fn parse_nmea(
        input: [u8],
    ) {

    }
}