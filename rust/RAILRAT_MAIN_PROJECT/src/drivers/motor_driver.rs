use embassy_stm32::{
    gpio::Output,
    timer::{
        GeneralInstance4Channel,
        simple_pwm::SimplePwmChannel,
    },
};


// Uses a generic timer
pub struct MotorDriver<Timer> 
where
    Timer: GeneralInstance4Channel,
{
    pwm: SimplePwmChannel<'static, Timer>, // connect enable to PMW channel
    in1: Output<'static>,
    in2: Output<'static>,
}

impl<Timer: GeneralInstance4Channel> MotorDriver<Timer> {

    /// Creates a new MotorDriver instance.
    /// 
    /// # Arguments
    /// 
    /// - `mut pwm` (`SimplePwmChannel<'static, Timer>`) - A PWM-compatible timer channel for L298N's ENA.
    /// - `in1` (`Output<'static>`) - GPIO pin for L298N IN1.
    /// - `in2` (`Output<'static>`) - GPIO pin for L298N IN2.
    /// 
    /// # Returns
    /// 
    /// - `Self` - A MotorDriver instance.
    /// 
    /// # Examples
    /// 
    /// ```
    /// use crate::...;
    /// 
    /// let mut ex = MotorDriver::new(pwm, in1, in2);
    /// ```
    pub fn new(
        mut pwm: SimplePwmChannel<'static, Timer>,
        in1: Output<'static>,
        in2: Output<'static>,
    ) -> Self {
        pwm.set_duty_cycle_fully_off();
        pwm.enable();

        Self { pwm, in1, in2 }
    }

    /// Sends the 'brake' signal to the motor driver: in1 = in2
    /// 
    /// # Arguments
    /// 
    /// - None.
    /// 
    /// # Examples
    /// 
    /// ```
    /// use crate::...;
    /// 
    /// ex.brake();
    /// ```
    pub fn brake(&mut self) {
        self.in1.set_low();
        self.in2.set_low();
        self.pwm.set_duty_cycle_fully_on();
    }


    /// Turns ENA fully off, causing the motor to coast.
    /// 
    /// # Arguments
    /// 
    /// - None.
    /// 
    /// # Examples
    /// 
    /// ```
    /// use crate::...;
    /// 
    /// ex.coast();
    /// ```
    pub fn coast(&mut self) {
        self.pwm.set_duty_cycle_fully_off();
    }


    /// Sets the duty cycle of the motor driver, changing the motor speed.
    /// 
    /// # Arguments
    /// 
    /// - `speed` (`i8`) - Desired speed/PWM duty cycle.
    ///     > Negative values: reverse
    ///     > Positive values: forward
    ///     > Zero: brake
    /// 
    /// # Examples
    /// 
    /// ```
    /// use crate::...;
    /// 
    /// ex.set_speed(50);
    /// ```
    pub fn set_speed(&mut self, speed: i8) {

        let speed = speed.clamp(-100, 100);

        self.pwm.set_duty_cycle_fully_off();

        if speed > 0 { // forwards
            self.in1.set_high();
            self.in2.set_low();
        }
        else if speed < 0 { // reverse
            self.in1.set_low();
            self.in2.set_high();
        }
        else {
            self.brake();
        }

        self.pwm.set_duty_cycle_percent(speed.unsigned_abs());
    }
}