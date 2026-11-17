use embassy_stm32::{
    i2c::{Error as HalI2cError, I2c},
    mode::Async,
};
use embassy_time::{Duration, Timer};

pub const I2C_ADDR_PRIMARY: u8 = 0x68;
pub const I2C_ADDR_SECONDARY: u8 = 0x69;

const EXPECTED_WHO_AM_I: u8 = 0xA9;

const REG_SMPLRT_DIV: u8        = 0x19;
const REG_CONFIG: u8            = 0x1A;
const REG_GYRO_CONFIG: u8       = 0x1B;
const REG_ACCEL_CONFIG: u8      = 0x1C;
const REG_ACCEL_CONFIG2: u8     = 0x1D;
const REG_LP_MODE_CFG: u8       = 0x1E;
const REG_ACCEL_XOUT_H: u8      = 0x3B;
const REG_SIGNAL_PATH_RESET: u8 = 0x68;
const REG_USER_CTRL: u8         = 0x6A;
const REG_PWR_MGMT_1: u8        = 0x6B;
const REG_PWR_MGMT_2: u8        = 0x6C;
const REG_WHO_AM_I: u8          = 0x75;


#[derive(Debug)]
pub enum ImuError {
    I2c(HalI2cError),
    InvalidWhoAmI(u8),
    ResetTimeout,
}

impl From<HalI2cError> for ImuError {
    fn from(error: HalI2cError) -> Self {
        Self::I2c(error)
    }
}


#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GyroScale {
    Dps250  = 0b00,
    Dps500  = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

impl GyroScale {
    fn lsb_per_dps(self) -> f32 {
        match self {
            Self::Dps250  => 131.0,
            Self::Dps500  => 65.5,
            Self::Dps1000 => 32.8,
            Self::Dps2000 => 16.4,
        }
    }
}


#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AccelScale {
    G2  = 0b00,
    G4  = 0b01,
    G8  = 0b10,
    G16 = 0b11,
}

impl AccelScale {
    fn lsb_per_g(self) -> f32 {
        match self {
            Self::G2  => 16384.0,
            Self::G4  => 8192.0,
            Self::G8  => 4096.0,
            Self::G16 => 2048.0,
        }
    }
}


pub struct ImuSample {
    pub accel_g: [f32; 3],
    pub gyro_dps: [f32; 3],
    pub temp_c: f32,
}


pub struct Iam20680 {
    i2c: I2c<'static, Async>,
    address: u8,
    accel_scale: AccelScale,
    gyro_scale: GyroScale,
}


impl Iam20680 {
    pub fn new(
        i2c: I2c<'static, Async>,
        address: u8,
    ) -> Self {
        Self {
            i2c,
            address,
            accel_scale: AccelScale::G2,
            gyro_scale: GyroScale::Dps250,
        }
    }

    async fn write_reg(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), ImuError> {
        self.i2c.write(self.address, &[reg, val]).await?;
        Ok(())
    }

    async fn read_reg(
        &mut self,
        reg: u8,
    ) -> Result<u8, ImuError> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    pub async fn init(
        &mut self,
    ) -> Result<(), ImuError> {
        Timer::after(Duration::from_millis(100)).await;

        self.write_reg(REG_PWR_MGMT_1, 0x81).await?;
        Timer::after(Duration::from_millis(20)).await;

        let mut reset_done = false;
        for _ in 0..20 {
            if (self.read_reg(REG_PWR_MGMT_1).await? & 0x80) == 0 {
                reset_done = true;
                break;
            }
            Timer::after(Duration::from_millis(10)).await;
        }

        if !reset_done {
            return Err(ImuError::ResetTimeout);
        }

        self.write_reg(REG_PWR_MGMT_1, 0x01).await?;
        Timer::after(Duration::from_millis(40)).await;

        self.write_reg(REG_SIGNAL_PATH_RESET, 0x03).await?;
        self.write_reg(REG_USER_CTRL, 0x01).await?;
        Timer::after(Duration::from_millis(15)).await;

        let who_am_i = self.read_reg(REG_WHO_AM_I).await?;
        if who_am_i != EXPECTED_WHO_AM_I {
            return Err(ImuError::InvalidWhoAmI(who_am_i));
        }

        self.write_reg(REG_PWR_MGMT_2, 0x00).await?;
        self.write_reg(REG_LP_MODE_CFG, 0x00).await?;
        self.write_reg(REG_CONFIG, 0x05).await?;

        self.set_gyro_scale(self.gyro_scale).await?;
        self.set_accel_scale(self.accel_scale).await?;

        self.write_reg(REG_ACCEL_CONFIG2, 0x05).await?;
        self.set_sample_rate_divider(0x09).await?;

        Timer::after(Duration::from_millis(10)).await;
        Ok(())
    }

pub async fn set_gyro_scale(
        &mut self,
        scale: GyroScale,
    ) -> Result<(), ImuError> {
        todo!()
    }

    pub async fn set_accel_scale(
        &mut self,
        scale: AccelScale,
    ) -> Result<(), ImuError> {
        todo!()
    }

    pub async fn set_sample_rate_divider(
        &mut self,
        div: u8,
    ) -> Result<(), ImuError> {
        todo!()
    }

    pub async fn read_sample(
        &mut self,
    ) -> Result<ImuSample, ImuError> {
        todo!()
    }

    pub async fn sleep(
        &mut self,
    ) -> Result<(), ImuError> {
        todo!()
    }
}
