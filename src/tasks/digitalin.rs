use defmt::*;
use embassy_rp::{
    Peri,
    peripherals::PIO0,
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, Irq, PioPin, ShiftDirection,
        StateMachine, program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};

use super::SM0_CLOCK_DIVIDER_48_KHZ;

// -- ---------------------------------------------------------------------
// -- SM0 - Digital Input
// -- ---------------------------------------------------------------------

pub fn setup_pio_task_sm0<'d>(
    pio: &mut Common<'d, PIO0>,
    sm0: &mut StateMachine<'d, PIO0, 0>,
    pin: Peri<'d, impl PioPin>,
) {
    // -- read digital input triggers
    let prg = pio_asm!(
        ".wrap_target",
        "wait 0 pin 0",
        "wait 1 pin 0",
        "irq 3",
        ".wrap",
    );
    // -- setup sm0
    let mut cfg = PioConfig::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);
    let in_pin = pio.make_pio_pin(pin);
    cfg.set_in_pins(&[&in_pin]);
    cfg.clock_divider = calculate_pio_clock_divider(SM0_CLOCK_DIVIDER_48_KHZ);
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.direction = ShiftDirection::Right;
    sm0.set_pin_dirs(PioPinDirection::In, &[&in_pin]);
    sm0.set_config(&cfg);
}

#[embassy_executor::task]
pub async fn pio_task_sm0(
    mut irq3: Irq<'static, PIO0, 3>,
    mut sm0: StateMachine<'static, PIO0, 0>,
) {
    sm0.set_enable(true);
    loop {
        irq3.wait().await;
        info!("IRQ trigged");
    }
}
