use core::cmp::min;
//use defmt::*;
use embassy_futures::yield_now;
use embassy_rp::{
    Peri,
    dma::Channel as DmaChannel,
    peripherals::{
        PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIO1, PWM_SLICE0, PWM_SLICE1, PWM_SLICE2,
        PWM_SLICE3, PWM_SLICE4, PWM_SLICE5, PWM_SLICE6,
    },
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin, PioPin,
        ShiftDirection, StateMachine, program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
    pwm::{Config as ConfigPwm, Pwm, SetDutyCycle},
};
use embassy_time::{Duration, Timer};
use portable_atomic::{AtomicU8, Ordering};

use super::{PWM_TX_FIFO_VALUES, PWM_VALUE_CYCLE_MAX, PWM_VALUE_MAX, SM1_CLOCK_DIVIDER_1_MHZ};

// -- ---------------------------------------------------------------------
// -- SM1 - Analog Output
// -- ---------------------------------------------------------------------

pub fn setup_pio_task_sm1<'d>(
    pio: &mut Common<'d, PIO1>,
    sm1: &mut StateMachine<'d, PIO1, 1>,
    pin_out1: Peri<'d, impl PioPin>,
    pin_out2: Peri<'d, impl PioPin>,
    pin_out3: Peri<'d, impl PioPin>,
    pin_out4: Peri<'d, impl PioPin>,
    pin_out5: Peri<'d, impl PioPin>,
    pin_out6: Peri<'d, impl PioPin>,
) {
    let prg = pio_asm!(
        r"
        .wrap_target
            out pins, 6
        .wrap",
    );
    // -- setup sm1
    let mut cfg = PioConfig::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);
    let pio_out1 = pio.make_pio_pin(pin_out1);
    let pio_out2 = pio.make_pio_pin(pin_out2);
    let pio_out3 = pio.make_pio_pin(pin_out3);
    let pio_out4 = pio.make_pio_pin(pin_out4);
    let pio_out5 = pio.make_pio_pin(pin_out5);
    let pio_out6 = pio.make_pio_pin(pin_out6);
    cfg.set_out_pins(&[
        &pio_out1, &pio_out2, &pio_out3, &pio_out4, &pio_out5, &pio_out6,
    ]);
    cfg.clock_divider = calculate_pio_clock_divider(SM1_CLOCK_DIVIDER_1_MHZ);
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.out_sticky = true;
    cfg.shift_out.auto_fill = true;
    cfg.shift_out.direction = ShiftDirection::Left;
    cfg.shift_out.threshold = 30;
    sm1.set_pin_dirs(
        PioPinDirection::Out,
        &[
            &pio_out1, &pio_out2, &pio_out3, &pio_out4, &pio_out5, &pio_out6,
        ],
    );
    sm1.set_config(&cfg);
}

#[inline(always)]
fn calc_fifo_pwm_out_bits(out: &mut [u8; 6]) -> u32 {
    let mut pwm_out_bits: u32 = 0;
    // -- 5 times 6 bits => 30 bit plus two unused ones
    for _ in 0..PWM_TX_FIFO_VALUES {
        for i in 0..out.len() {
            let out_bit = if out[i] > 0 {
                out[i] -= 1;
                1
            } else {
                0
            };
            // -- out direction is shift-left and the order of the
            // -- six bits is out1-out2-out6-out5-out4-out3
            pwm_out_bits = (pwm_out_bits << 1) | out_bit;
        }
    }
    pwm_out_bits << 2
}

#[inline(always)]
fn update_pwm_out_values(
    out_pwm_count_down: &mut [u8; 6],
    analog_out_1: &'static AtomicU8,
    analog_out_2: &'static AtomicU8,
    analog_out_3: &'static AtomicU8,
    analog_out_4: &'static AtomicU8,
    analog_out_5: &'static AtomicU8,
    analog_out_6: &'static AtomicU8,
) {
    out_pwm_count_down[0] = min(analog_out_1.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[1] = min(analog_out_2.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[5] = min(analog_out_3.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[4] = min(analog_out_4.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[2] = min(analog_out_5.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[3] = min(analog_out_6.load(Ordering::Relaxed), PWM_VALUE_MAX);
}

#[embassy_executor::task]
pub async fn pio_task_sm1(
    mut sm1: StateMachine<'static, PIO1, 1>,
    mut dma_ch: Option<DmaChannel<'static>>,
    analog_out_1: &'static AtomicU8,
    analog_out_2: &'static AtomicU8,
    analog_out_3: &'static AtomicU8,
    analog_out_4: &'static AtomicU8,
    analog_out_5: &'static AtomicU8,
    analog_out_6: &'static AtomicU8,
) {
    // -- PWM duty cycle count down values
    let mut out_pwm_count_down: [u8; 6] = [0; 6];
    update_pwm_out_values(
        &mut out_pwm_count_down,
        analog_out_1,
        analog_out_2,
        analog_out_3,
        analog_out_4,
        analog_out_5,
        analog_out_6,
    );
    // -- the duty cycle has to be in the range of 0 to 100
    let mut pwm_duty_cycle_count = 0;
    // -- enable state machine and start loop
    sm1.tx().push(0);
    sm1.set_enable(true);
    let mut pwm_out_bits_last = u32::MAX;
    loop {
        //let start = Instant::now();
        // -- out 1 to 100%
        //let pwm_out_bits = 0b10000010000010000010000010000000;
        // -- out 2 to 100%
        //let pwm_out_bits = 0b01000001000001000001000001000000;
        // -- out 6 to 100%
        //let pwm_out_bits = 0b00100000100000100000100000100000;
        // -- out 5 to 100%
        //let pwm_out_bits = 0b00010000010000010000010000010000;
        // -- out 4 to 100%
        //let pwm_out_bits = 0b00001000001000001000001000001000;
        // -- out 3 to 100%
        //let pwm_out_bits = 0b00000100000100000100000100000100;
        // sm1.tx().push(pwm_out_bits);
        // -- calculate PWM bits
        let pwm_out_bits = calc_fifo_pwm_out_bits(&mut out_pwm_count_down);
        // -- push PWM bits into the TX FIFO if there is a change
        if pwm_out_bits != pwm_out_bits_last {
            pwm_out_bits_last = pwm_out_bits;
            if let Some(dma_ch) = dma_ch.as_mut() {
                sm1.tx().dma_push(dma_ch, &[pwm_out_bits], false).await;
            } else {
                sm1.tx().wait_push(pwm_out_bits).await;
            }
        }
        // -- check if cycle is finished, restart with new values if so
        pwm_duty_cycle_count += 1;
        if pwm_duty_cycle_count == PWM_VALUE_CYCLE_MAX {
            pwm_duty_cycle_count = 0;
            // -- update PWM values for next cycle
            update_pwm_out_values(
                &mut out_pwm_count_down,
                analog_out_1,
                analog_out_2,
                analog_out_3,
                analog_out_4,
                analog_out_5,
                analog_out_6,
            );
        }
        yield_now().await;
        Timer::after(Duration::from_micros(1)).await;
    }
}

#[embassy_executor::task]
pub async fn pwm_analog_out(
    mut slice0: Peri<'static, PWM_SLICE0>,
    mut slice1: Peri<'static, PWM_SLICE1>,
    mut slice2: Peri<'static, PWM_SLICE2>,
    pin_out1: Peri<'static, PIN_21>,
    pin_out2: Peri<'static, PIN_20>,
    pin_out3: Peri<'static, PIN_16>,
    pin_out4: Peri<'static, PIN_17>,
    pin_out5: Peri<'static, PIN_18>,
    pin_out6: Peri<'static, PIN_19>,
    analog_out_1: &'static AtomicU8,
    analog_out_2: &'static AtomicU8,
    analog_out_3: &'static AtomicU8,
    analog_out_4: &'static AtomicU8,
    analog_out_5: &'static AtomicU8,
    analog_out_6: &'static AtomicU8,
) {
    // -- the top value sets the period of the PWM cycle
    // -- a counter goes from 0 to top and then wraps around to 0
    // -- every such wraparound is one PWM cycle
    // -- here is how we get 25KHz:
    let desired_freq_hz = 25_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut c = ConfigPwm::default();
    c.top = period;
    c.divider = divider.into();

    let mut pwm_out_2_1 = Pwm::new_output_ab(slice2, pin_out2, pin_out1, c.clone());
    let (mut pwm_out_2, mut pwm_out_1) = pwm_out_2_1.split();
    let mut pwm_out_3_4 = Pwm::new_output_ab(slice0, pin_out3, pin_out4, c.clone());
    let (mut pwm_out_3, mut pwm_out_4) = pwm_out_3_4.split();
    let mut pwm_out_5_6 = Pwm::new_output_ab(slice1, pin_out5, pin_out6, c.clone());
    let (mut pwm_out_5, mut pwm_out_6) = pwm_out_5_6.split();

    let mut out_1_prev_percentage = 0;
    let mut out_2_prev_percentage = 0;
    let mut out_3_prev_percentage = 0;
    let mut out_4_prev_percentage = 0;
    let mut out_5_prev_percentage = 0;
    let mut out_6_prev_percentage = 0;

    loop {
        let out_1_percentage =
            ((min(analog_out_1.load(Ordering::Relaxed), PWM_VALUE_MAX) as u16 * 10) / 25) as u8;
        let out_2_percentage =
            ((min(analog_out_2.load(Ordering::Relaxed), PWM_VALUE_MAX) as u16 * 10) / 25) as u8;
        let out_3_percentage =
            ((min(analog_out_3.load(Ordering::Relaxed), PWM_VALUE_MAX) as u16 * 10) / 25) as u8;
        let out_4_percentage =
            ((min(analog_out_4.load(Ordering::Relaxed), PWM_VALUE_MAX) as u16 * 10) / 25) as u8;
        let out_5_percentage =
            ((min(analog_out_5.load(Ordering::Relaxed), PWM_VALUE_MAX) as u16 * 10) / 25) as u8;
        let out_6_percentage =
            ((min(analog_out_6.load(Ordering::Relaxed), PWM_VALUE_MAX) as u16 * 10) / 25) as u8;

        if out_1_percentage != out_1_prev_percentage {
            out_1_prev_percentage = out_1_percentage;
            if let Some(pwm_out_1) = pwm_out_1.as_mut() {
                let _ = pwm_out_1.set_duty_cycle_percent(out_1_percentage);
            }
        }
        if out_2_percentage != out_2_prev_percentage {
            out_2_prev_percentage = out_2_percentage;
            if let Some(pwm_out_2) = pwm_out_2.as_mut() {
                let _ = pwm_out_2.set_duty_cycle_percent(out_2_percentage);
            }
        }
        if out_3_percentage != out_3_prev_percentage {
            out_3_prev_percentage = out_3_percentage;
            if let Some(pwm_out_3) = pwm_out_3.as_mut() {
                let _ = pwm_out_3.set_duty_cycle_percent(out_3_percentage);
            }
        }
        if out_4_percentage != out_4_prev_percentage {
            out_4_prev_percentage = out_4_percentage;
            if let Some(pwm_out_4) = pwm_out_4.as_mut() {
                let _ = pwm_out_4.set_duty_cycle_percent(out_4_percentage);
            }
        }
        if out_5_percentage != out_5_prev_percentage {
            out_5_prev_percentage = out_5_percentage;
            if let Some(pwm_out_5) = pwm_out_5.as_mut() {
                let _ = pwm_out_5.set_duty_cycle_percent(out_5_percentage);
            }
        }
        if out_6_percentage != out_6_prev_percentage {
            out_6_prev_percentage = out_6_percentage;
            if let Some(pwm_out_6) = pwm_out_6.as_mut() {
                let _ = pwm_out_6.set_duty_cycle_percent(out_6_percentage);
            }
        }
        yield_now().await;
        Timer::after(Duration::from_millis(100)).await;
    }
}
