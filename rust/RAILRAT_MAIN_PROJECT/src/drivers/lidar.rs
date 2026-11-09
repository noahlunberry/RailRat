use embassy_stm32::{
    usart::{RingBufferedUartRx, Uart, UartTx},
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

use crate::shared::uart::UartError;


#[derive(Debug)]
pub enum LidarError {
    Uart(UartError),
    InvalidMeasurement,
    InvalidDescriptor,
}

impl From<UartError> for LidarError {
    fn from(error: UartError) -> Self {
        Self::Uart(error)
    }
}

// The ring-buffered receiver and UartTx return the raw UART error.
impl From<embassy_stm32::usart::Error> for LidarError {
    fn from(error: embassy_stm32::usart::Error) -> Self {
        Self::Uart(UartError::Hardware(error))
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
    tx: UartTx<'static, Async>,
    rx: RingBufferedUartRx<'static>,
    pwm: SimplePwmChannel<'static, Timer>,
}


impl<Timer> Rplidar<Timer>
where
    Timer: GeneralInstance4Channel,
{
    pub fn new(
        mut pwm: SimplePwmChannel<'static, Timer>,
        uart: Uart<'static, Async>,
        rx_buffer: &'static mut [u8],
    ) -> Self {
        pwm.set_duty_cycle_fully_off();
        pwm.enable();

        // The lidar streams almost as fast as 115200 baud can carry, so a plain
        // uart.read() loses bytes in the gap between calls. In ring-buffered mode,
        // DMA keeps writing into rx_buffer in the background, looping back to the
        // start when it reaches the end, and read() takes bytes out of it.
        // Splitting first is required because only the receive half is ring-buffered.
        let (tx, rx) = uart.split();
        let mut rx = rx.into_ring_buffered(rx_buffer);
        rx.start_uart();

        Self { tx, rx, pwm }
    }

    /// Fills all of `buf`. A ring-buffered read() returns however many bytes are
    /// ready, which can be fewer than requested, so this keeps reading until full.
    async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), LidarError> {
        let mut filled = 0;

        while filled < buf.len() {
            filled += self.rx.read(&mut buf[filled..]).await?;
        }

        Ok(())
    }

    /// Discards any received bytes left over from a previous command.
    /// Reads until the line has been quiet for 20 ms, then the buffer is empty.
    pub async fn clear_rx(
        &mut self,
    ) {
        let mut buf = [0u8; 64];

        for _ in 0..100 {
            if with_timeout(Duration::from_millis(20), self.rx.read(&mut buf)).await.is_err() {
                break;
            }
        }
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
        self.tx.write(&[0xA5, 0x40]).await?;
        Ok(())
    }

    pub async fn start(
        &mut self,
    ) -> Result<(), LidarError> {
        self.tx.write(&[0xA5, 0x20]).await?;

        let mut res = [0u8; 7];
        self.read_exact(&mut res).await?;

        if res != [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81] {
            return Err(LidarError::InvalidDescriptor);
        }

        Ok(())
    }

    pub async fn halt(
        &mut self,
    ) -> Result<(), LidarError> {
        self.tx.write(&[0xA5, 0x25]).await?;
        self.pwm.set_duty_cycle_fully_off();

        Ok(())
    }

    pub async fn read_point(
        &mut self,
    ) -> Result<ScanPoint, LidarError> {
        // How many single-byte shifts to try before giving up on realigning.
        const MAX_RESYNC_BYTES: usize = 64;

        let mut buf = [0u8; 5];

        // Read the scan of 5 bytes.
        self.read_exact(&mut buf).await?;

        // Valid packets have S-inverse opposite to S, and C set to 1 (protocol doc, p.16).
        // If those checks fail we started mid-packet, so slide forward one byte
        // at a time until they pass.
        let mut shifts = 0;

        loop {
            let start = buf[0] & 0x01;
            let inverse_start = (buf[0] >> 1) & 0x01;
            let check_bit = buf[1] & 0x01;

            if (start != inverse_start) && (check_bit == 1) {
                break;
            }

            if shifts == MAX_RESYNC_BYTES {
                return Err(LidarError::InvalidMeasurement);
            }

            buf.copy_within(1.., 0);
            self.read_exact(&mut buf[4..]).await?;
            shifts += 1;
        }


        let quality = buf[0] >> 2;
        let start_flag = (buf[0] & 0x01) != 0;
        let angle_q6 = ((buf[1] as u16) >> 1) | ((buf[2] as u16) << 7);
        let dist_q2 = (buf[3] as u16) | ((buf[4] as u16) << 8);

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
        self.tx.write(&[0xA5, command]).await?;
        Ok(())
    }
}