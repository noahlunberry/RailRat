use embedded_hal::serial::{Read, Write};
use embedded_hal::digital::v2::OutputPin;
use nb::block;

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

    pub fn start_motor(&mut self) {
        let _ = self.motor_pin.set_high();
    }

    pub fn stop_motor(&mut self) {
        let _ = self.motor_pin.set_low();
    }

    pub fn start_scan(&mut self) {
        let _ = block!(self.uart.write(0xA5));
        let _ = block!(self.uart.write(0x20));

        for _ in 0..7 {
            let _ = block!(self.uart.read());
        }
    }

    pub fn read_point(&mut self) -> ScanPoint {
        let mut buf = [0u8; 5];
        
        for i in 0..5 {
            buf[i] = block!(self.uart.read()).unwrap_or(0);
        }

        let s = buf[0] & 0x01;
        
        let quality = buf[0] >> 2;
        let start_flag = s == 1;

        let angle_q6 = ((buf[1] >> 1) as u16) | ((buf[2] as u16) << 7);
        let angle_deg = angle_q6 as f32 / 64.0;

        let dist_q2 = (buf[3] as u16) | ((buf[4] as u16) << 8);
        let distance_mm = dist_q2 as f32 / 4.0;

        ScanPoint {
            distance_mm,
            angle_deg,
            quality,
            start_flag,
        }
    }
}
