use embassy_stm32::{
    timer::{
        GeneralInstance4Channel,
        // QEI enables encoder mode on a timer. 
        // Every encoder pulse increments a timer counter with no CPU overhead.
        qei::Qei,
    },
};
use embassy_time::Instant;

/// The JGB37-520's encoder uses the Hall Effect to generate discrete pulses. 
/// There are two pulses that are each 90 degrees out of phase, used to determine direction.
/// Call them A and B. Each signal pulses 11 times every motor rotation.
/// 
/// Embassy uses the STM32's QEI x4 quadrature mode by default. This means that it increments
/// the timer counter for every rising and falling edge of A and B. This equals four
/// increments for every period (a period starts with a rising edge of A and ends with 
/// the next falling edge of B).


pub struct MotorEncoder<Timer>
where
    Timer: GeneralInstance4Channel,
{
    qei: Qei<'static, Timer>,
    ticks_per_wheel_rev: f32,
    previous_count: u16,
    previous_time: Instant,
    position_ticks: i64,
}


impl<Timer> MotorEncoder<Timer>
where
    Timer: GeneralInstance4Channel,
{

    pub const TICKS_PER_MOTOR_REV : u8 = 11;
    pub const QEIx4_PULSES_PER_PERIOD : u8 = 4;

    pub fn new(
        qei: Qei<'static, Timer>,
        ticks_per_wheel_rev: f32,
    ) -> Self {
        let previous_count = qei.count();

        Self {
            qei,
            ticks_per_wheel_rev,
            previous_count,
            previous_time: Instant::now(),
            position_ticks: 0,
        }
    }


    pub fn update(&mut self) -> f32 {
        let current_count = self.qei.count();
        let current_time = Instant::now();

        // Handle 16-bit counter wraparound
        let delta_ticks = current_count.wrapping_sub(self.previous_count) as i16;
        let delta_time = ((current_time - self.previous_time).as_micros() as f32) / 1_000_000.0;

        self.position_ticks += delta_ticks as i64;
        self.previous_count = current_count;
        self.previous_time = current_time;

        if delta_time == 0.0 {
            return 0.0;
        }

        // rev / s
        let rps = delta_ticks as f32
            / self.ticks_per_wheel_rev
            / delta_time;

        // rev / min
        rps * 60.0
    }


    pub fn get_position_ticks(&self) -> i64 {
        self.position_ticks
    }


    pub fn get_position_revolutions(&self) -> f32 {
        (self.position_ticks as f32) / self.ticks_per_wheel_rev
    }


    pub fn reset(&mut self) {
        self.position_ticks = 0;
        self.previous_count = self.qei.count();
        self.previous_time = Instant::now();
    }
}