use embassy_stm32::{
    usart::Uart,
    mode::Async,
    timer::{
        GeneralInstance4Channel,
        simple_pwm::SimplePwmChannel,
    },
};


pub struct ScanPoint {
    pub distance_mm: f32,
    pub angle_deg: f32,
    pub quality: u8,
    pub start_flag: bool,
}


pub struct Rplidar<Timer> 
where
    Timer: GeneralInstance4Channel,
{
    uart: Uart<'static, Async>,
    pwm: SimplePwmChannel<'static, Timer>, 
}


impl<Timer> Rplidar<Timer>
where
    Timer: GeneralInstance4Channel,
{
    pub fn new(
        mut pwm: SimplePwmChannel<'static, Timer>,
        uart: Uart<'static, Async>,
    ) -> Self {
        pwm.set_duty_cycle_fully_off();
        pwm.enable();

        Self { uart, pwm }
    }

    pub fn start_motor(&mut self) {
        self.pwm.set_duty_cycle_fully_on();
    }

    pub fn stop_motor(&mut self) {
        self.pwm.set_duty_cycle_fully_off();
    }

    pub async fn start_scan(
        &mut self,
    ) -> Result<(), embassy_stm32::usart::Error> {
        self.uart.write(&[0xA5, 0x20]).await?;

        let mut descriptor = [0u8; 7];
        self.uart.read(&mut descriptor).await?;

        Ok(())
    }

    pub async fn read_point(
        &mut self,
    ) -> Result<ScanPoint, embassy_stm32::usart::Error> {
        let mut buf = [0u8; 5];

        // Read the scan of 5 bytes.
        self.uart.read(&mut buf).await?;

        // Check packet validity.
        let start = buf[0] & 0x01;
        let inverse_start = (buf[0] >> 1) & 0x01;
        let check_bit = buf[1] & 0x01;

        if (start == inverse_start) || (check_bit != 1) {
            return Err(LidarError::InvalidMeasurement);
        }


        let quality = buf[0] >> 2;
        let start_flag = (buf[0] & 0x01) != 0;

        let angle_q6 =
            ((buf[1] as u16) >> 1) | ((buf[2] as u16) << 7);

        let dist_q2 =
            (buf[3] as u16) | ((buf[4] as u16) << 8);

        Ok(ScanPoint {
            distance_mm: dist_q2 as f32 / 4.0,
            angle_deg: angle_q6 as f32 / 64.0,
            quality,
            start_flag,
        })
    }
}