use embassy_rp::{
    Peri,
    dma::Channel as DmaChannel,
    gpio::{Drive, Level, SlewRate},
    i2c::{Async as I2cAsync, I2c},
    peripherals::{I2C0, PIO0, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin,
        Instance as PioInstance, Irq, PioPin, ShiftDirection, StateMachine, program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};

// -- bus speed 400 kHz with 4 PIO clock cycles for I2C each high/low interval
// pub const SM_CLOCK_DIVIDER_1_6_MHZ: u32 = 1_600_000;
// -- bus speed 1 MHz with 4 PIO clock cycles for I2C each high/low interval
pub const SM_CLOCK_DIVIDER_4_MHZ: u32 = 4_000_000;

// -- ---------------------------------------------------------------------
// -- SM2 - MPC4725 i2c DAC output
// -- ---------------------------------------------------------------------

pub fn setup_i2c_pio<'d>(
    pio0: &mut Common<'d, PIO0>,
    sm0: &mut StateMachine<'d, PIO0, 0>,
    sda_pin: Peri<'d, impl PioPin>,
    scl_pin: Peri<'d, impl PioPin>,
) {
    // -- continouously write data to devices
    // -- side 0 is SCL
    let prg = pio_asm!(
        r"
        .side_set 1                             ; -- SCL is side set
        public entry_point:
            set pindirs, 1      side 1 [3]      ; -- 00 - SDA output
        .wrap_target
            set pins, 0         side 1 [2]      ; -- 01 - START condition SDA to low, SCL high
            set x, 7            side 1          ; -- 02 - write 8 bits, SCL high
            out y, 8            side 0 [1]      ; -- 03 - number of bytes to write from OSR, SCL low
            jmp y-- bit_loop    side 0          ; -- 04 - jump if y > 0 prior to decrement, SCL low
            jmp entry_point     side 0          ; -- 05 - restart when zero bytes to write, SCL low
        bit_loop:
            out pins, 1         side 0          ; -- 06 - read next bit from OSR, SCL low
            nop                 side 1 [3]      ; -- 07 - confirm SDA value with SCL pulse
            jmp x-- bit_loop    side 0 [2]      ; -- 08 - jump if x > 0 prior to decrement
            set pindirs, 0      side 0          ; -- 09 - SDA input
            set x, 7            side 1 [3]      ; -- 10 - confirm SDA value with SCL pulse
            jmp pin do_nack     side 0          ; -- 11 - Check ACK from MPC4725
            set pindirs, 1      side 0          ; -- 12 - SDA output
            jmp y-- bit_loop    side 0          ; -- 13 - jump if y > 0 prior to decrement
        do_stop:
            set pins, 0         side 0          ; -- 14 - SDA low, SCL low
            set pins, 0         side 1 [3]      ; -- 15 - SDA low, SCL high
            set pins, 1         side 1 [3]      ; -- 16 - STOP condition SDA to high, SCL high
        .wrap
        do_nack:
            irq nowait 0        side 0 [2]      ; -- 17 - indicate error, SCL low
            jmp entry_point     side 1          ; -- 18 - continue with start condition
        ",
    );
    // -- setup state machine
    let mut sda_pio_pin = pio0.make_pio_pin(sda_pin);
    sda_pio_pin.set_pull(embassy_rp::gpio::Pull::Up);
    let mut scl_pio_pin = pio0.make_pio_pin(scl_pin);
    scl_pio_pin.set_pull(embassy_rp::gpio::Pull::Up);
    let mut cfg = PioConfig::default();
    let prg = pio0.load_program(&prg.program);
    cfg.use_program(&prg, &[&scl_pio_pin]);
    cfg.set_in_pins(&[&sda_pio_pin, &scl_pio_pin]);
    cfg.set_set_pins(&[&sda_pio_pin]);
    cfg.set_out_pins(&[&sda_pio_pin]);
    cfg.set_jmp_pin(&sda_pio_pin);
    //cfg.clock_divider = calculate_pio_clock_divider(SM2_CLOCK_DIVIDER_1_6_MHZ); // -- bus speed 400 kH
    cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_4_MHZ); // -- bus speed 1 MHz
    cfg.out_sticky = true;
    cfg.shift_out.auto_fill = true;
    cfg.shift_out.direction = ShiftDirection::Left;
    cfg.shift_out.threshold = 8;
    cfg.fifo_join = FifoJoin::TxOnly;
    sm0.set_config(&cfg);
    sm0.set_pin_dirs(PioPinDirection::Out, &[&sda_pio_pin, &scl_pio_pin]);
    sm0.set_pins(Level::High, &[&sda_pio_pin, &scl_pio_pin]);
    sm0.set_enable(true);
}

// pub fn enable_i2c_pio<'d>(sm0: &mut StateMachine<'d, PIO0, 0>) {
//     sm0.set_enable(true);
// }

#[embassy_executor::task]
pub async fn pio_task_sm0_irq0(mut irq0: Irq<'static, PIO0, 0>) {
    loop {
        irq0.wait().await;
        defmt::error!("IRQ 0 trigged - I2C PIO state machine is in trouble...");
    }
}

pub async fn i2c_write<'d, const LEN: usize>(
    sm: &mut StateMachine<'static, PIO0, 0>,
    dma_ch: &mut DmaChannel<'static>,
    dev_addr: u8,
    data: [u8; LEN],
) {
    // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
    let no_of_bytes = (LEN + 1) as u8;
    let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
    let header = [no_of_bytes, dev_addr_write];
    sm.tx().dma_push(dma_ch, &header, false).await;
    sm.tx().dma_push(dma_ch, &data, false).await;
}

pub async fn i2c_write_two_bytes<'d>(
    sm: &mut StateMachine<'static, PIO0, 0>,
    dma_ch: &mut DmaChannel<'static>,
    dev_addr: u8,
    byte1: u8,
    byte2: u8,
) {
    // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
    let no_of_bytes = 3;
    let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
    let data = [no_of_bytes, dev_addr_write, byte1, byte2];
    sm.tx().dma_push(dma_ch, &data, false).await;
}

pub async fn i2c_write_byte_and_data<'d, const LEN: usize>(
    sm: &mut StateMachine<'static, PIO0, 0>,
    dma_ch: &mut DmaChannel<'static>,
    dev_addr: u8,
    byte: u8,
    data: [u8; LEN],
) {
    // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
    let no_of_bytes = (LEN + 1) as u8;
    let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
    let header = [no_of_bytes, dev_addr_write, byte];
    sm.tx().dma_push(dma_ch, &header, false).await;
    sm.tx().dma_push(dma_ch, &data, false).await;
}
