use embassy_stm32::{
    usart::Uart,
    mode::Async,
    timer::{
        GeneralInstance4Channel,
        simple_pwm::SimplePwmChannel,
    },
};
use embassy_time::{
    with_timeout, 
    Duration
};



#[derive(Debug)]
pub enum LidarError {
    Uart(embassy_stm32::usart::Error),
    InvalidMeasurement,
    InvalidDescriptor,
    Timeout,
}


impl From<embassy_stm32::usart::Error> for LidarError {
    fn from(error: embassy_stm32::usart::Error) -> Self {
        LidarError::Uart(error)
    }
}


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

    pub fn set_motor_speed(
        &mut self, 
        speed: u8,
    ) {
        let speed = speed.clamp(0, 100);

        self.pwm.set_duty_cycle_percent(speed);
    }

    pub async fn reset(
        &mut self,
    ) -> Result<(), LidarError> {
        self.uart.write(&[0xA5, 0x40]).await?;
        Ok(())
    }

    pub async fn start(
        &mut self,
    ) -> Result<(), LidarError> {
        self.uart.write(&[0xA5, 0x20]).await?;

        let mut res = [0u8; 7];
        self.uart.read(&mut res).await?;

        if res != [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81] {
            return Err(LidarError::InvalidDescriptor);
        }

        Ok(())
    }

    pub async fn halt(
        &mut self,
    ) -> Result<(), LidarError> {
        let res = self.uart.write(&[0xA5, 0x25]).await?;
        self.pwm.set_duty_cycle_fully_off();
        
        Ok(())
    }

    pub async fn read_point(
        &mut self,
    ) -> Result<ScanPoint, LidarError> {
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

    pub async fn send_command(
        &mut self,
        command: u8,
    ) -> Result<(), LidarError> {
        self.uart.write(&[0xA5, command]).await?;
        Ok(())
    }
}