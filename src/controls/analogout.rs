use core::cmp::min;
use embassy_rp::pwm::{Pwm, PwmOutput, SetDutyCycle};
use embassy_time::{Duration, Timer};

const ONE_HUNDRED_PERCENT: u8 = 100;

pub struct AnalogOutput<'a> {
    pwm_out: PwmOutput<'a>,
    duty_cycle_percent: u8,
}

impl<'a> AnalogOutput<'a> {
    pub fn new(mut pwm_out: PwmOutput<'a>, duty_cycle_percent: u8) -> Self {
        let duty_cycle_percent = min(duty_cycle_percent, ONE_HUNDRED_PERCENT);
        let _ = pwm_out.set_duty_cycle_percent(duty_cycle_percent);
        Self {
            pwm_out,
            duty_cycle_percent,
        }
    }

    pub fn set_duty_cycle_percent(&mut self, duty_cycle_percent: u8) {
        let duty_cycle_percent = min(duty_cycle_percent, ONE_HUNDRED_PERCENT);
        if self.duty_cycle_percent != duty_cycle_percent {
            let _ = self
                .pwm_out
                .set_duty_cycle_percent(min(duty_cycle_percent, ONE_HUNDRED_PERCENT));
            self.duty_cycle_percent = duty_cycle_percent;
        }
    }

    pub async fn duty_cycle_percent(&self) -> u8 {
        self.duty_cycle_percent
    }
}
