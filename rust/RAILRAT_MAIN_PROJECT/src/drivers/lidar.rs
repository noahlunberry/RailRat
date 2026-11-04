use embedded_hal::serial::{Read, Write};
use embedded_hal::digital::v2::OutputPin;
use nb::block;

#[derive(Debug)]
pub enum Error<E, PinE> {
    SerialError(E),
    PinError(PinE),
    InvalidSync,
    InvalidCheckBit,
}

#[derive(Debug)]
pub struct ScanPoint {
    pub distance_mm: f32,
    pub angle_deg: f32,
    pub quality: u8,
    pub start_flag: bool, 
}

pub struct Rplidar<UART, MOTOCTL> {
    uart: UART,
    motor_pin: MOTOCTL,
}

impl<UART, MOTOCTL, E, PinE> Rplidar<UART, MOTOCTL>
where
    UART: Read<u8, Error = E> + Write<u8, Error = E>,
    MOTOCTL: OutputPin<Error = PinE>,
{
    pub fn new(uart: UART, motor_pin: MOTOCTL) -> Self {
        Self { uart, motor_pin }
    }

    pub fn start_motor(&mut self) -> Result<(), Error<E, PinE>> {
        self.motor_pin.set_high().map_err(Error::PinError)
    }

    pub fn stop_motor(&mut self) -> Result<(), Error<E, PinE>> {
        self.motor_pin.set_low().map_err(Error::PinError)
    }

    pub fn start_scan(&mut self) -> Result<(), Error<E, PinE>> {
        block!(self.uart.write(0xA5)).map_err(Error::SerialError)?;
        block!(self.uart.write(0x20)).map_err(Error::SerialError)?;


        for _ in 0..7 {
            let _ = block!(self.uart.read()).map_err(Error::SerialError)?;
        }
        Ok(())
    }   

}
