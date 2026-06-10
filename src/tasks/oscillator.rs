use defmt::*;
use embassy_futures::yield_now;
use embassy_rp::{
    Peri,
    dma::Channel as DmaChannel,
    gpio::{Drive, Level, SlewRate},
    i2c::{Async as I2cAsync, I2c},
    peripherals::{I2C1, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin, Irq, PioPin,
        ShiftDirection, StateMachine, program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};
use embassy_time::{Duration, Ticker, Timer, WithTimeout};

use crate::audio::oscillator::sine_oscillator::SineOscillator;
use crate::io::i2c::mpc4725::*;

use super::{
    ChannelOscillatorType,
    SAMPLE_BLOCK_SIZE,
    SAMPLE_RATE_24KHZ,
    SAMPLE_RATE_48KHZ,
    //SM_CLOCK_DIVIDER_1_6_MHZ,
    SM_CLOCK_DIVIDER_4_MHZ,
    SM_CLOCK_DIVIDER_48_KHZ,
};

// -- ---------------------------------------------------------------------
// -- SM1 - oscillator trigger
// -- ---------------------------------------------------------------------

pub fn setup_oscillator_clock_pio_task<'d>(
    pio: &mut Common<'d, PIO1>,
    sm1: &mut StateMachine<'d, PIO1, 1>,
) {
    // -- continouously trigger irq 1 -> delivers 48 kHz clock
    let prg = pio_asm!(
        r"
        .wrap_target
            irq nowait 1                        ; -- 00 - trigger irq 0
        .wrap
        ",
    );
    // -- setup sm1
    let mut cfg = PioConfig::default();
    let prg = pio.load_program(&prg.program);
    cfg.use_program(&prg, &[]);
    cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_48_KHZ);
    sm1.set_config(&cfg);
}

// -- ---------------------------------------------------------------------
// -- SM2 - MPC4725 i2c DAC output
// -- ---------------------------------------------------------------------

pub fn setup_oscillator_pio_task<'d>(
    pio: &mut Common<'d, PIO1>,
    sm2: &mut StateMachine<'d, PIO1, 2>,
    sda_pin: Peri<'d, impl PioPin>,
    scl_pin: Peri<'d, impl PioPin>,
) {
    // -- continouously write data to MPC4725 DAC
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
            irq nowait 2        side 0 [2]      ; -- 17 - indicate error, SCL low
            jmp entry_point     side 1          ; -- 18 - continue with start condition
        ",
    );
    // -- setup sm2
    let mut sda_pin = pio.make_pio_pin(sda_pin);
    sda_pin.set_pull(embassy_rp::gpio::Pull::Up);
    let mut scl_pin = pio.make_pio_pin(scl_pin);
    scl_pin.set_pull(embassy_rp::gpio::Pull::Up);
    let mut cfg = PioConfig::default();
    let prg = pio.load_program(&prg.program);
    cfg.use_program(&prg, &[&scl_pin]);
    cfg.set_in_pins(&[&sda_pin, &scl_pin]);
    cfg.set_set_pins(&[&sda_pin]);
    cfg.set_out_pins(&[&sda_pin]);
    cfg.set_jmp_pin(&sda_pin);
    //cfg.clock_divider = calculate_pio_clock_divider(SM2_CLOCK_DIVIDER_1_6_MHZ); // -- bus speed 400 kHz
    // -- bus speed 1 MHz with 4 PIO clock cycles for I2C each high/low interval
    cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_4_MHZ);
    cfg.out_sticky = true;
    cfg.shift_out.auto_fill = true;
    cfg.shift_out.direction = ShiftDirection::Left;
    cfg.shift_out.threshold = 32;
    cfg.fifo_join = FifoJoin::TxOnly;
    sm2.set_config(&cfg);
    sm2.set_pin_dirs(PioPinDirection::Out, &[&sda_pin, &scl_pin]);
    sm2.set_pins(Level::High, &[&sda_pin, &scl_pin]);
}

#[embassy_executor::task]
pub async fn oscillator_irq2_handler(mut irq2: Irq<'static, PIO1, 2>) {
    loop {
        irq2.wait().await;
        error!("IRQ 2 trigged - MPC4725 state machine is in trouble...");
    }
}

#[embassy_executor::task]
pub async fn oscillator_irq1_handler(
    mut irq1: Irq<'static, PIO1, 1>,
    mut sm1: StateMachine<'static, PIO1, 1>,
    mut sm2: StateMachine<'static, PIO1, 2>,
    mut dma_ch: Option<DmaChannel<'static>>,
) {
    info!("oscillator_irq1_handler started");
    // -- setup oscillator
    let sample_rate = SAMPLE_RATE_48KHZ;
    //let frequency = 110.0;
    //let frequency = 440.0;
    let frequency = 880.0;
    //let frequency = 5000.0;
    let f = frequency / sample_rate;
    let mut out = [0.0; SAMPLE_BLOCK_SIZE];
    let mut osc = SineOscillator::new();
    osc.init();
    //let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    //let every_nanos = (400_000_000f32 / sample_rate) as u64;
    //debug!("pio_task_sm2: Every nanos is {}", every_nanos);
    //let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
    sm1.set_enable(true);
    sm2.set_enable(true);
    loop {
        irq1.wait().await;
        // -- prepare sample
        osc.render(f, &mut out);
        let sample = ((out[0] + 1f32) * 4096f32 / 2f32) as u16;
        // -- prepare I2C message for MPC4725 I2C PIO: <no of bytes - addr - data byte 1 - data byte 2>
        // -- this is for cfg.shift_out.threshold = 8;
        // let no_of_bytes = 3u8;
        // let dev_addr_write: u8 = 0b11000100;
        // let data_byte_1 = (sample >> 8) as u8;
        // let data_byte_2 = (sample & 0xff) as u8;
        // if let Some(dma_ch) = dma_ch.as_mut() {
        //     let data_out = [no_of_bytes, dev_addr_write, data_byte_1, data_byte_2];
        //     sm2.tx().dma_push(dma_ch, &data_out, false).await;
        // } else {
        //     sm2.tx().wait_push((no_of_bytes as u32) << 24).await;
        //     sm2.tx().wait_push((dev_addr_write as u32) << 24).await;
        //     sm2.tx().wait_push((data_byte_1 as u32) << 24).await;
        //     sm2.tx().wait_push((data_byte_2 as u32) << 24).await;
        // }
        // -- this is for cfg.shift_out.threshold = 32;
        let no_of_bytes = 3u32;
        let addr: u32 = 0b11000100;
        let data_byte_1 = ((sample >> 8) as u8) as u32;
        let data_byte_2 = ((sample & 0xff) as u8) as u32;
        let data_out = (no_of_bytes << 24) | (addr << 16) | (data_byte_1 << 8) | data_byte_2;
        if let Some(dma_ch) = dma_ch.as_mut() {
            sm2.tx().dma_push(dma_ch, &[data_out], false).await;
        } else {
            sm2.tx().wait_push(data_out).await;
        }
        yield_now().await;
    }
}

// #[embassy_executor::task]
// pub async fn pio_task_sm2_irq1(
//     mut irq1: Irq<'static, PIO0, 1>,
//     mut sm2: StateMachine<'static, PIO0, 2>,
// ) {
//     // -- setup oscillator
//     let sample_rate = SAMPLE_RATE_48KHZ;
//     //let frequency = 110.0;
//     let frequency = 440.0;
//     //let frequency = 880.0;
//     //let frequency = 5000.0;
//     let f = frequency / sample_rate;
//     let mut out = [0.0; SAMPLE_BLOCK_SIZE];
//     let mut osc = SineOscillator::new();
//     osc.init();
//     sm2.set_enable(true);
//     loop {
//         osc.render(f, &mut out);
//         let sample = ((out[0] + 1f32) * 4096f32 / 2f32) as u16;
//         let addr = 0b11000100;
//         let data_byte_1 = (sample >> 8) as u8;
//         let data_byte_2 = (sample & 0xff) as u8;
//         let data_out = (addr as u32) << 24 | (data_byte_1 as u32) << 16 | (data_byte_2 as u32) << 8;
//         if !sm2.tx().full() {
//             sm2.tx().push(data_out);
//             //debug!("Pushed data to SM2 TX FIFO")
//         }
//         //yield_now().await;
//         irq1.wait().await;
//         //info!("IRQ 1 trigged - MPC4725 state machine requests data");
//     }
// }

// #[embassy_executor::task]
// pub async fn osc_task_generate(channel_oscillator: &'static ChannelOscillatorType) {
//     info!("Oscillator generate task started");
//     let sample_rate = SAMPLE_RATE_48KHZ;
//     //let frequency = 110.0;
//     let frequency = 440.0;
//     //let frequency = 880.0;
//     //let frequency = 5000.0;
//     let f = frequency / sample_rate;
//     let mut out = [0.0; SAMPLE_BLOCK_SIZE];
//     let mut samples = [0u16; SAMPLE_BLOCK_SIZE];
//     let mut osc = SineOscillator::new();
//     osc.init();
//     loop {
//         while channel_oscillator.free_capacity() > 0 {
//             //debug!("rendering...");
//             osc.render(f, &mut out);
//             for i in 0..out.len() {
//                 // -- normalize the sample and send it to the DAC
//                 samples[i] = ((out[i] + 1f32) * 4096f32 / 2f32) as u16;
//             }
//             channel_oscillator.send(samples).await;
//             Timer::after_micros(2).await;
//         }
//         //info!("Oscillator generate task: Channel saturated");
//         Timer::after_millis(5).await;
//     }
// }

// #[embassy_executor::task]
// pub async fn osc_task_dac(
//     mut i2c1: I2c<'static, I2C1, I2cAsync>,
//     channel_oscillator: &'static ChannelOscillatorType,
// ) {
//     info!("Oscillator DAC task started");
//     // -- setup MPC47
//     let mut mpc4725 = match Mpc4725::new(&mut i2c1, Mpc4725DeviceAddress::Default).await {
//         Ok(mpc4725) => mpc4725,
//         Err(err) => {
//             error!("Failed to initialize MPC4725: {}", err);
//             return;
//         }
//     };
//     // -- initialize sample playing loop
//     let sample_rate = SAMPLE_RATE_48KHZ;
//     let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
//     debug!("Every nanos is {}", every_nanos);
//     let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
//     loop {
//         let samples = channel_oscillator.receive().await;
//         //debug!("Oscillator DAC task: Samples received");
//         for sample in &samples {
//             // -- normalize the sample and send it to the DAC
//             mpc4725
//                 .write_dac_value_fast(&mut i2c1, *sample)
//                 //.write_dac_value_regular(&mut i2c1, *sample)
//                 .await
//                 .unwrap();
//             ticker.next().await;
//         }
//     }
//     // // -- setup oscillator
//     // let sample_rate = SAMPLE_RATE_48KHZ;
//     // let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
//     // debug!("Every nanos is {}", every_nanos);
//     // //let frequency = 110.0;
//     // //let frequency = 440.0;
//     // let frequency = 880.0;
//     // //let frequency = 5000.0;
//     // let f = frequency / sample_rate;
//     // let mut out = [0.0; 1];
//     // let mut osc = SineOscillator::new();
//     // osc.init();
//     // osc.render(f, &mut out);
//     // // let mut start = Instant::now();
//     // let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
//     // loop {
//     //     // -- this kinda works
//     //     osc.render(f, &mut out);
//     //     let sample = out[0];
//     //     let value = (((sample + 1f32) * 4096f32) / 2f32) as u16;
//     //     // //debug!("sample is {}, value is {}", sample, value);
//     //     mpc4725.write_dac_value(&mut i2c1, value).await.unwrap();
//     //     ticker.next().await;
//     // }
// }

// #[embassy_executor::task]
// pub async fn osc_task_consolidated(mut i2c1: I2c<'static, I2C1, I2cAsync>) {
//     info!("Oscillator task started");
//     //let sample_rate = SAMPLE_RATE_48KHZ;
//     let sample_rate = SAMPLE_RATE_24KHZ;
//     //let frequency = 110.0;
//     //let frequency = 440.0;
//     let frequency = 880.0;
//     //let frequency = 5000.0;
//     let f = frequency / sample_rate;
//     let mut out = [0.0; 1];
//     let mut osc = SineOscillator::new();
//     osc.init();
//     // -- setup MPC47
//     let mut mpc4725 = match Mpc4725::new(&mut i2c1, Mpc4725DeviceAddress::Default).await {
//         Ok(mpc4725) => mpc4725,
//         Err(err) => {
//             error!("Failed to initialize MPC4725: {}", err);
//             return;
//         }
//     };
//     // -- initialize sample playing loop
//     let sample_rate = SAMPLE_RATE_48KHZ;
//     let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
//     debug!("Every nanos is {}", every_nanos);
//     let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
//     loop {
//         osc.render(f, &mut out);
//         let sample = ((out[0] + 1f32) * 4096f32 / 2f32) as u16;
//         // -- normalize the sample and send it to the DAC
//         mpc4725
//             .write_dac_value_fast(&mut i2c1, sample)
//             //.write_dac_value_regular(&mut i2c1, *sample)
//             .await
//             .unwrap();
//         ticker.next().await;
//     }
// }
