use embassy_stm32::{
    gpio::Output,
    timer::{
        GeneralInstance4Channel,
        simple_pwm::SimplePwmChannel,
    },
};


// Uses a generic timer
pub struct MotorDriver<T: GeneralInstance4Channel> {
    pwm: SimplePwmChannel<'static, T>, // connect enable to PMW channel
    in1: Output<'static>,
    in2: Output<'static>,
}

impl<T: GeneralInstance4Channel> MotorDriver<T> {

    pub fn new(
        mut pwm: SimplePwmChannel<'static, T>,
        in1: Output<'static>,
        in2: Output<'static>,
    ) -> Self {
        pwm.set_duty_cycle_fully_off();
        pwm.enable();

        Self { pwm, in1, in2 }
    }

    fn brake(&mut self) {
        self.in1.set_low();
        self.in2.set_low();
        self.pwm.set_duty_cycle_fully_on();
    }

    fn coast(&mut self) {
        self.pwm.set_duty_cycle_fully_off();
    }

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
            self.in1.set_low();
            self.in2.set_low();
        }

        self.pwm.set_duty_cycle_percent(speed.unsigned_abs());
    }
}