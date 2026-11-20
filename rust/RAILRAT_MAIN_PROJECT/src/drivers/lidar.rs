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

    /// Copies whatever raw bytes are waiting into `buf`, waiting if there are none.
    /// Returns how many were copied. Used to forward the scan stream undecoded.
    pub async fn read_bytes(
        &mut self,
        buf: &mut [u8],
    ) -> Result<usize, LidarError> {
        Ok(self.rx.read(buf).await?)
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

    /// Reads a response descriptor (A5 5A, 30-bit length, answer type).
    /// Returns (length of the data that follows, answer type).
    async fn read_descriptor(
        &mut self,
    ) -> Result<(usize, u8), LidarError> {
        let mut descriptor = [0u8; 7];
        self.read_exact(&mut descriptor).await?;

        if descriptor[0] != 0xA5 || descriptor[1] != 0x5A {
            return Err(LidarError::InvalidDescriptor);
        }

        let length = u32::from_le_bytes([descriptor[2], descriptor[3], descriptor[4], descriptor[5]]) & 0x3FFF_FFFF;

        Ok((length as usize, descriptor[6]))
    }

    /// GET_LIDAR_CONF (0x84, firmware 1.24+). Used to list the scan modes.
    /// `mode` is required for the per-mode queries. Copies up to `out.len()`
    /// bytes of the answer into `out` and returns how many were copied.
    pub async fn get_lidar_conf(
        &mut self,
        conf_type: u32,
        mode: Option<u16>,
        out: &mut [u8],
    ) -> Result<usize, LidarError> {
        // A5 84 <payload size> <payload> <checksum>, checksum = XOR of every byte before it
        let mut request = [0u8; 10];
        request[0] = 0xA5;
        request[1] = 0x84;
        request[3..7].copy_from_slice(&conf_type.to_le_bytes());

        let mut payload_len = 4;
        if let Some(mode) = mode {
            request[7..9].copy_from_slice(&mode.to_le_bytes());
            payload_len = 6;
        }
        request[2] = payload_len as u8;

        let checksum_at = 3 + payload_len;
        request[checksum_at] = request[..checksum_at].iter().fold(0, |acc, byte| acc ^ byte);
        self.tx.write(&request[..=checksum_at]).await?;

        let (length, answer_type) = self.read_descriptor().await?;
        if answer_type != 0x20 || !(4..=256).contains(&length) {
            return Err(LidarError::InvalidDescriptor);
        }

        // The answer starts by repeating the conf type
        let mut echo = [0u8; 4];
        self.read_exact(&mut echo).await?;
        if u32::from_le_bytes(echo) != conf_type {
            return Err(LidarError::InvalidDescriptor);
        }

        let data_len = length - 4;
        let kept = data_len.min(out.len());
        self.read_exact(&mut out[..kept]).await?;

        // Throw away anything that didn't fit
        let mut scratch = [0u8; 1];
        for _ in kept..data_len {
            self.read_exact(&mut scratch).await?;
        }

        Ok(kept)
    }

    /// Starts an express scan (0x82) in the given scan mode. Returns the answer
    /// type from the descriptor, which says how the data stream is packed
    /// (0x82 capsuled, 0x84 ultra capsuled, 0x85 dense capsuled).
    pub async fn start_express(
        &mut self,
        mode: u16,
    ) -> Result<u8, LidarError> {
        // A5 82 05 <working mode> <flags u16> <param u16> <checksum>
        let mut request = [0xA5, 0x82, 0x05, mode as u8, 0, 0, 0, 0, 0];
        request[8] = request[..8].iter().fold(0, |acc, byte| acc ^ byte);
        self.tx.write(&request).await?;

        let (_, answer_type) = self.read_descriptor().await?;

        Ok(answer_type)
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