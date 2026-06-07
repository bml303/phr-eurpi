use embassy_futures::yield_now;
use embassy_rp::{
    Peri,
    dma::Channel as DmaChannel,
    gpio::{Drive, Level, SlewRate},
    i2c::{Async as I2cAsync, I2c},
    peripherals::{I2C0, PIO0, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin,
        Instance as PioInstance, Irq, PioPin, ShiftDirection, StateMachine, StateMachineTx,
        program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};
use heapless::vec::Vec;

// -- bus speed 400 kHz with 4 PIO clock cycles for I2C each high/low interval
// pub const SM_CLOCK_DIVIDER_1_6_MHZ: u32 = 1_600_000;
// -- bus speed 1 MHz with 4 PIO clock cycles for I2C each high/low interval
pub const SM_CLOCK_DIVIDER_4_MHZ: u32 = 4_000_000;

// -- ---------------------------------------------------------------------
// -- SM0 - i2c PIO
// -- ---------------------------------------------------------------------

pub struct I2CPIO<'d> {
    sm0: StateMachine<'d, PIO0, 0>,
    dma_ch: Option<DmaChannel<'d>>,
    data_buf: [u32; 600],
}

impl<'d> I2CPIO<'d> {
    pub fn new(sm0: StateMachine<'d, PIO0, 0>, dma_ch: Option<DmaChannel<'d>>) -> Self {
        I2CPIO {
            sm0,
            dma_ch,
            data_buf: [0; 600],
        }
    }

    pub fn setup_i2c_pio(
        &mut self,
        pio0: &mut Common<'d, PIO0>,
        sda_pin: Peri<'d, impl PioPin>,
        scl_pin: Peri<'d, impl PioPin>,
    ) {
        // -- continouously write data to devices
        // -- side 0 is SCL
        let prg = pio_asm!(
            r"
            .side_set 1                             ; -- SCL is side set
            public entry_point:
                set pindirs, 1      side 1 [2]      ; -- 00 - SDA output, SCL high
            .wrap_target
                pull block          side 1          ; -- 01 - load number of bytes to write from TX FIFO, SCL high
                set pins, 0         side 1 [1]      ; -- 02 - START condition SDA to low, SCL high
                set x, 7            side 1          ; -- 03 - write 8 bits, SCL high
                out y, 32           side 1          ; -- 04 - number of bytes to write from OSR, SCL low
                jmp y-- byte_loop   side 0 [2]      ; -- 05 - jump if y > 0 prior to decrement, SCL low
                jmp entry_point     side 0          ; -- 06 - restart when zero bytes to write, SCL low
            byte_loop:
            bit_loop:
                out pins, 1         side 0          ; -- 07 - read next bit from OSR, SCL low
                nop                 side 1 [3]      ; -- 08 - confirm SDA value with SCL pulse
                jmp x-- bit_loop    side 0 [2]      ; -- 09 - jump if x > 0 prior to decrement
                set pindirs, 0      side 0          ; -- 10 - SDA input
                set x, 7            side 1 [3]      ; -- 11 - confirm SDA value with SCL pulse
                jmp pin do_nack     side 0          ; -- 12 - Check ACK
                set pindirs, 1      side 0          ; -- 13 - SDA output
                jmp y-- byte_loop   side 0          ; -- 14 - jump if y > 0 prior to decrement
            do_stop:
                set pins, 0         side 0          ; -- 15 - SDA low, SCL low
                set pins, 0         side 1 [3]      ; -- 16 - SDA low, SCL high
                set pins, 1         side 1 [3]      ; -- 17 - STOP condition SDA to high, SCL high
            .wrap
            do_nack:
                irq nowait 0        side 0 [2]      ; -- 18 - indicate error, SCL low
                jmp entry_point     side 1          ; -- 19 - continue with start condition
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
        cfg.shift_out.threshold = 32;
        cfg.fifo_join = FifoJoin::TxOnly;
        self.sm0.set_config(&cfg);
        self.sm0
            .set_pin_dirs(PioPinDirection::Out, &[&sda_pio_pin, &scl_pio_pin]);
        self.sm0
            .set_pins(Level::High, &[&sda_pio_pin, &scl_pio_pin]);
    }

    pub fn enable(&mut self) {
        self.sm0.set_enable(true);
    }

    pub async fn i2c_write_byte_and_data_v2<const LEN: usize>(
        &mut self,
        dev_addr: u8,
        cmd_byte: u8,
        data: [u8; LEN],
    ) {
        // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
        let no_of_bytes = (LEN + 2) as u32;
        //defmt::debug!("no_of_bytes is {}", no_of_bytes);
        let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
        //let header = [dev_addr_write, byte];
        self.data_buf[0] = no_of_bytes;
        self.data_buf[1] = ((dev_addr_write as u32) << 24) | ((cmd_byte as u32) << 16);
        if data.len() >= 1 {
            self.data_buf[1] |= (data[0] as u32) << 8;
        }
        if data.len() >= 2 {
            self.data_buf[1] |= data[1] as u32;
        }
        yield_now().await;
        let mut j = 2;
        if data.len() >= 3 {
            for i in (2..data.len()).step_by(4) {
                self.data_buf[j] = (data[i] as u32) << 24;
                if i + 1 < data.len() {
                    self.data_buf[j] |= (data[i + 1] as u32) << 16;
                }
                if i + 2 < data.len() {
                    self.data_buf[j] |= (data[i + 2] as u32) << 8;
                }
                if i + 3 < data.len() {
                    self.data_buf[j] |= data[i + 3] as u32;
                }
                j += 1;
                yield_now().await;
            }
        }
        if let Some(dma_ch) = self.dma_ch.as_mut() {
            self.sm0
                .tx()
                .dma_push(dma_ch, &self.data_buf[..j], false)
                .await;
        } else {
            for i in 0..j {
                self.sm0.tx().wait_push(self.data_buf[i]).await;
            }
        }
    }

    pub async fn i2c_write_byte_and_data_v1<const LEN: usize>(
        &mut self,
        dev_addr: u8,
        byte: u8,
        data: [u8; LEN],
    ) {
        // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
        let no_of_bytes = (LEN + 2) as u32;
        defmt::debug!("no_of_bytes is {}", no_of_bytes);
        let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
        //let header = [dev_addr_write, byte];
        if let Some(dma_ch) = self.dma_ch.as_mut() {
            self.sm0.tx().wait_push(no_of_bytes).await;
            self.sm0.tx().wait_push((dev_addr_write as u32) << 24).await;
            self.sm0.tx().wait_push((byte as u32) << 24).await;
            for i in 0..data.len() {
                self.sm0.tx().wait_push((data[i] as u32) << 24).await;
            }
            // self.sm0.tx().dma_push(dma_ch, &[no_of_bytes], false).await;
            // self.sm0.tx().dma_push(dma_ch, &header, false).await;
            // self.sm0.tx().dma_push(dma_ch, &data, false).await;
            // self.sm0.tx().dma_push(dma_ch, &[no_of_bytes], false).await;
            // self.sm0.tx().dma_push(dma_ch, &header, false).await;
            // self.sm0.tx().dma_push(dma_ch, &data, false).await;
        } else {
            // self.sm0.tx().wait_push((no_of_bytes[0] as u32) << 24).await;
            // self.sm0.tx().wait_push((no_of_bytes[1] as u32) << 24).await;
            // self.sm0.tx().wait_push((dev_addr_write as u32) << 24).await;
            // self.sm0.tx().wait_push((byte as u32) << 24).await;
            // for byte in data {
            //     self.sm0.tx().wait_push((byte as u32) << 24).await;
            // }
        }
    }

    pub async fn i2c_write_byte_and_data_len_u8<const LEN: usize>(
        &mut self,
        dev_addr: u8,
        byte: u8,
        data: [u8; LEN],
    ) {
        // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
        let no_of_bytes = (LEN + 2) as u8;
        defmt::debug!("no_of_bytes is {}", no_of_bytes);
        let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
        let header = [no_of_bytes, dev_addr_write, byte];
        if let Some(dma_ch) = self.dma_ch.as_mut() {
            self.sm0.tx().dma_push(dma_ch, &header, false).await;
            self.sm0.tx().dma_push(dma_ch, &data, false).await;
        } else {
            self.sm0.tx().wait_push((no_of_bytes as u32) << 24).await;
            self.sm0.tx().wait_push((dev_addr_write as u32) << 24).await;
            self.sm0.tx().wait_push((byte as u32) << 24).await;
            for byte in data {
                self.sm0.tx().wait_push((byte as u32) << 24).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn pio_task_sm0_irq0(mut irq0: Irq<'static, PIO0, 0>) {
    loop {
        irq0.wait().await;
        defmt::error!("IRQ 0 trigged - I2C PIO state machine is in trouble...");
    }
}

// pub fn setup_i2c_pio<'d>(
//     pio0: &mut Common<'d, PIO0>,
//     sm0: &mut StateMachine<'d, PIO0, 0>,
//     sda_pin: Peri<'d, impl PioPin>,
//     scl_pin: Peri<'d, impl PioPin>,
// ) {
//     // -- continouously write data to devices
//     // -- side 0 is SCL
//     let prg = pio_asm!(
//         r"
//         .side_set 1                             ; -- SCL is side set
//         public entry_point:
//             set pindirs, 1      side 1 [3]      ; -- 00 - SDA output
//         .wrap_target
//             set pins, 0         side 1 [2]      ; -- 01 - START condition SDA to low, SCL high
//             set x, 7            side 1          ; -- 02 - write 8 bits, SCL high
//             out y, 8            side 0 [1]      ; -- 03 - number of bytes to write from OSR, SCL low
//             jmp y-- bit_loop    side 0          ; -- 04 - jump if y > 0 prior to decrement, SCL low
//             jmp entry_point     side 0          ; -- 05 - restart when zero bytes to write, SCL low
//         bit_loop:
//             out pins, 1         side 0          ; -- 06 - read next bit from OSR, SCL low
//             nop                 side 1 [3]      ; -- 07 - confirm SDA value with SCL pulse
//             jmp x-- bit_loop    side 0 [2]      ; -- 08 - jump if x > 0 prior to decrement
//             set pindirs, 0      side 0          ; -- 09 - SDA input
//             set x, 7            side 1 [3]      ; -- 10 - confirm SDA value with SCL pulse
//             jmp pin do_nack     side 0          ; -- 11 - Check ACK from MPC4725
//             set pindirs, 1      side 0          ; -- 12 - SDA output
//             jmp y-- bit_loop    side 0          ; -- 13 - jump if y > 0 prior to decrement
//         do_stop:
//             set pins, 0         side 0          ; -- 14 - SDA low, SCL low
//             set pins, 0         side 1 [3]      ; -- 15 - SDA low, SCL high
//             set pins, 1         side 1 [3]      ; -- 16 - STOP condition SDA to high, SCL high
//         .wrap
//         do_nack:
//             irq nowait 0        side 0 [2]      ; -- 17 - indicate error, SCL low
//             jmp entry_point     side 1          ; -- 18 - continue with start condition
//         ",
//     );
//     // -- setup state machine
//     let mut sda_pio_pin = pio0.make_pio_pin(sda_pin);
//     sda_pio_pin.set_pull(embassy_rp::gpio::Pull::Up);
//     let mut scl_pio_pin = pio0.make_pio_pin(scl_pin);
//     scl_pio_pin.set_pull(embassy_rp::gpio::Pull::Up);
//     let mut cfg = PioConfig::default();
//     let prg = pio0.load_program(&prg.program);
//     cfg.use_program(&prg, &[&scl_pio_pin]);
//     cfg.set_in_pins(&[&sda_pio_pin, &scl_pio_pin]);
//     cfg.set_set_pins(&[&sda_pio_pin]);
//     cfg.set_out_pins(&[&sda_pio_pin]);
//     cfg.set_jmp_pin(&sda_pio_pin);
//     //cfg.clock_divider = calculate_pio_clock_divider(SM2_CLOCK_DIVIDER_1_6_MHZ); // -- bus speed 400 kH
//     cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_4_MHZ); // -- bus speed 1 MHz
//     cfg.out_sticky = true;
//     cfg.shift_out.auto_fill = true;
//     cfg.shift_out.direction = ShiftDirection::Left;
//     cfg.shift_out.threshold = 8;
//     cfg.fifo_join = FifoJoin::TxOnly;
//     sm0.set_config(&cfg);
//     sm0.set_pin_dirs(PioPinDirection::Out, &[&sda_pio_pin, &scl_pio_pin]);
//     sm0.set_pins(Level::High, &[&sda_pio_pin, &scl_pio_pin]);
//     sm0.set_enable(true);
// }

// // pub fn enable_i2c_pio<'d>(sm0: &mut StateMachine<'d, PIO0, 0>) {
// //     sm0.set_enable(true);
// // }

// pub async fn i2c_write<'d, const LEN: usize>(
//     sm: &mut StateMachine<'static, PIO0, 0>,
//     dma_ch: &mut DmaChannel<'static>,
//     dev_addr: u8,
//     data: [u8; LEN],
// ) {
//     // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
//     let no_of_bytes = (LEN + 1) as u8;
//     let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
//     let header = [no_of_bytes, dev_addr_write];
//     sm.tx().dma_push(dma_ch, &header, false).await;
//     sm.tx().dma_push(dma_ch, &data, false).await;
// }

// pub async fn i2c_write_two_bytes<'d>(
//     sm: &mut StateMachine<'static, PIO0, 0>,
//     dma_ch: &mut DmaChannel<'static>,
//     dev_addr: u8,
//     byte1: u8,
//     byte2: u8,
// ) {
//     // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
//     let no_of_bytes = 3;
//     let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
//     let data = [no_of_bytes, dev_addr_write, byte1, byte2];
//     sm.tx().dma_push(dma_ch, &data, false).await;
// }

// pub async fn i2c_write_byte_and_data<'d, const LEN: usize>(
//     sm: &mut StateMachine<'static, PIO0, 0>,
//     dma_ch: &mut Option<DmaChannel<'static>>,
//     dev_addr: u8,
//     byte: u8,
//     data: [u8; LEN],
// ) {
//     // -- prepare for I2C PIO: <no of bytes - device address - data byte 1 - data byte 2 - ...>
//     let no_of_bytes = (LEN + 2) as u8;
//     let dev_addr_write = dev_addr << 1; // -- 7 msb = device addr, 1 lsb 0 for write
//     let header = [no_of_bytes, dev_addr_write, byte];
//     sm.tx().dma_push(dma_ch, &header, false).await;
//     sm.tx().dma_push(dma_ch, &data, false).await;
// }
