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
use embassy_time::{Duration, Instant, Ticker, Timer, WithTimeout};

// use crate::audio::oscillator::{
//     oscillator::{Oscillator, OscillatorShape},
//     sine_oscillator::SineOscillator,
//     string_synth_oscillator::StringSynthOscillator,
// };
use crate::audio::oscillator::{Oscillator, Waveform};
use crate::io::i2c::mpc4725::*;

use super::{
    ChannelFrequencyType, ChannelOscillatorType, SAMPLE_BLOCK_SIZE, SAMPLE_RATE_24KHZ,
    SAMPLE_RATE_48KHZ, SAMPLE_RATE_50KHZ, SAMPLE_RATE_96KHZ, SM_CLOCK_DIVIDER_8_MHZ,
    SM_CLOCK_DIVIDER_11_150_KHZ, SM_CLOCK_DIVIDER_13_600_KHZ, SM_CLOCK_DIVIDER_50_KHZ,
};

const SAMPLE_BUF_SIZE: usize = 50000;

// -- ---------------------------------------------------------------------
// -- SM1 - oscillator trigger
// -- ---------------------------------------------------------------------

pub fn setup_oscillator_clock_pio_task<'d>(
    pio: &mut Common<'d, PIO1>,
    sm1: &mut StateMachine<'d, PIO1, 1>,
) {
    // -- continouously trigger irq 1 -> delivers 50 kHz clock
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
    cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_50_KHZ);
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
    //cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_4_MHZ);
    //cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_8_MHZ);
    //cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_11_150_KHZ);
    // -- bus speed 3.4 MHz with 4 PIO clock cycles for I2C each high/low interval
    cfg.clock_divider = calculate_pio_clock_divider(SM_CLOCK_DIVIDER_13_600_KHZ);
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
    //mut i2c1: I2c<'static, I2C1, I2cAsync>,
    mut irq1: Irq<'static, PIO1, 1>,
    mut sm1: StateMachine<'static, PIO1, 1>,
    mut sm2: StateMachine<'static, PIO1, 2>,
    mut dma_ch: Option<DmaChannel<'static>>,
    frequency_channel: &'static ChannelFrequencyType,
) {
    info!("oscillator_irq1_handler started");
    // -- setup oscillator
    let sample_rate = SAMPLE_RATE_50KHZ;
    //let frequency = 110.0;
    let frequency = 440.0;
    //let frequency = 880.0;
    //let frequency = 1000.0;
    //let frequency = 4400.0;
    //let frequency = 5000.0;
    //let frequency = 10000.0;
    let mut sample_buf = [0.0; SAMPLE_BUF_SIZE];

    let mut osc = match Oscillator::new(Waveform::Sine, frequency, sample_rate) {
        Ok(osc) => osc,
        Err(e) => {
            error!("Failed to create oscillator: {}", e);
            return;
        }
    };
    debug!("Start rendering");
    let render_start = Instant::now();
    osc.fill_buffer(&mut sample_buf);
    debug!(
        "Rendering done in {} ms",
        &render_start.elapsed().as_millis()
    );
    let mut dac_1_is_non_zero = true;
    let mut dac_2_is_non_zero = true;
    sm1.set_enable(true);
    sm2.set_enable(true);
    let mut sample_index = 0;
    //let mut ticker = embassy_time::Ticker::every(Duration::from_nanos(20833));
    loop {
        irq1.wait().await;
        // if frequency_channel.len() > 0 {
        //     if let Ok(frequency) = frequency_channel.try_receive() {
        //         defmt::debug!("Setting frequency {}", frequency);
        //         f = frequency as f32 / sample_rate;
        //         osc.init();
        //         osc.render(
        //             f,
        //             1.0,
        //             None,
        //             &mut sample_buf,
        //             OscillatorShape::SquareTriangle,
        //             false,
        //         );
        //         //osc.render(f, &mut sample_buf);
        //         //osc.render(f, &registration, 1.0, &mut sample_buf);
        //         sample_index = 0;
        //     }
        // }
        // let mut sample_buf = [0.0; 1];
        // osc.render(f, &mut sample_buf);
        // let sample = sample_buf[0];
        // -- get sample from buffer and refill sample buffer if necessary
        // let sample = unsafe { SAMPLE_BUF[sample_index] } * 4096f32;
        let sample = sample_buf[sample_index];
        sample_index = (sample_index + 1) % SAMPLE_BUF_SIZE;
        // sample_index += 1;
        // if sample_index >= SAMPLE_BUF_SIZE {
        //     sample_index = 0;
        // }
        //sample_index = (sample_index + 1) % SAMPLE_BUF_SIZE;
        // if sample_index >= SAMPLE_BUF_SIZE {
        //     sample_index = 0;
        //     //debug!("Sample buf wrap: {}", &start.elapsed().as_millis());
        //     //start = embassy_time::Instant::now();
        // }

        // if sample_index >= SAMPLE_BLOCK_SIZE {
        //     //osc.render(f, &mut sample_buf);
        //     osc.render(
        //         f,
        //         1.0,
        //         None,
        //         &mut sample_buf,
        //         OscillatorShape::SquareTriangle,
        //         false,
        //     );
        //     //osc.render(f, &registration, 1.0, &mut sample_buf);
        //     sample_index = 0;
        // }
        // -- prepare DAC value from sample
        // -- positive values go to first DAC
        // -- negative values go to second DAC
        // -- zero values go to both DACs
        let (dac_val_1, dac_val_2) = if sample > 0.0 {
            ((sample * 4096f32) as u16, 0)
        } else if sample < 0.0 {
            (0, (-sample * 4096f32) as u16)
        } else {
            (0, 0)
        };

        // if sample_index % 2 == 0 {
        //     write_to_dacs(
        //         &mut sm2,
        //         &mut dma_ch,
        //         Mpc4725DeviceAddress::Default,
        //         dac_val_1,
        //         Mpc4725DeviceAddress::Secondary,
        //         dac_val_2,
        //     )
        //     .await;
        // }

        // write_to_dac(
        //     &mut sm2,
        //     &mut dma_ch,
        //     Mpc4725DeviceAddress::Default,
        //     dac_val_1,
        // )
        // .await;
        // write_to_dac(
        //     &mut sm2,
        //     &mut dma_ch,
        //     Mpc4725DeviceAddress::Secondary,
        //     dac_val_2,
        // )
        // .await;
        //ticker.next().await;

        // if dac_val_1 > 0 {
        //     // -- write to DAC 1
        //     write_to_dac(
        //         &mut sm2,
        //         &mut dma_ch,
        //         Mpc4725DeviceAddress::Default,
        //         dac_val_1,
        //     )
        //     .await;
        // } else if dac_val_2 > 0 {
        //     // -- write to DAC 2
        //     write_to_dac(
        //         &mut sm2,
        //         &mut dma_ch,
        //         Mpc4725DeviceAddress::Secondary,
        //         dac_val_2,
        //     )
        //     .await;
        // }
        //

        // -- write to DAC 1 & DAC 2
        // write_values_to_dacs(
        //     &mut sm2,
        //     &mut dma_ch,
        //     Mpc4725DeviceAddress::Default,
        //     dac_val_1,
        //     &mut dac_1_is_non_zero,
        //     Mpc4725DeviceAddress::Secondary,
        //     dac_val_2,
        //     &mut dac_2_is_non_zero,
        // )
        // .await;

        // -- write to DAC 2
        write_value_to_dac(
            &mut sm2,
            &mut dma_ch,
            Mpc4725DeviceAddress::Secondary,
            dac_val_2,
            &mut dac_2_is_non_zero,
        )
        .await;
        // -- write to DAC 1
        write_value_to_dac(
            &mut sm2,
            &mut dma_ch,
            Mpc4725DeviceAddress::Default,
            dac_val_1,
            &mut dac_1_is_non_zero,
        )
        .await;

        // let dev_addr: u8 = Mpc4725DeviceAddress::
        // Default.value() as u8;
        // let data_byte_1 = (dac_val_1 >> 8) as u8;
        // let data_byte_2 = (dac_val_1 & 0xff) as u8;
        // let data_bytes = [data_byte_1, data_byte_2];
        // let _ = i2c1.blocking_write(dev_addr, &data_bytes);

        // let dev_addr: u8 = Mpc4725DeviceAddress::Secondary.value() as u8;
        // let data_byte_1 = (dac_val_2 >> 8) as u8;
        // let data_byte_2 = (dac_val_2 & 0xff) as u8;
        // let data_bytes = [data_byte_1, data_byte_2];
        // let _ = i2c1.blocking_write(dev_addr, &data_bytes);

        // let dev_addr: u8 = 0x62;
        // let data_byte_1 = ((sample >> 8) as u8);
        // let data_byte_2 = ((sample & 0xff) as u8);
        // let data_bytes = [data_byte_1, data_byte_2];
        // let _ = i2c1.blocking_write(dev_addr, &data_bytes);
        //yield_now().await;
    }
}

#[inline(always)]
async fn write_values_to_dacs(
    sm2: &mut StateMachine<'static, PIO1, 2>,
    dma_ch: &mut Option<DmaChannel<'static>>,
    dac_1_addr: Mpc4725DeviceAddress,
    dac_1_val: u16,
    dac_1_is_non_zero: &mut bool,
    dac_2_addr: Mpc4725DeviceAddress,
    dac_2_val: u16,
    dac_2_is_non_zero: &mut bool,
) {
    let dac_1_val = if dac_1_val != 0 {
        *dac_1_is_non_zero = true;
        Some(dac_1_val)
    } else if *dac_1_is_non_zero {
        *dac_1_is_non_zero = false;
        Some(0)
    } else {
        None
    };
    let dac_2_val = if dac_2_val != 0 {
        *dac_2_is_non_zero = true;
        Some(dac_2_val)
    } else if *dac_2_is_non_zero {
        *dac_2_is_non_zero = false;
        Some(0)
    } else {
        None
    };

    if let Some(dac_1_val) = dac_1_val
        && let Some(dac_2_val) = dac_2_val
    {
        write_to_both_dacs(sm2, dma_ch, dac_1_addr, dac_1_val, dac_2_addr, dac_2_val).await;
    } else if let Some(dac_1_val) = dac_1_val {
        write_to_dac(sm2, dma_ch, dac_1_addr, dac_1_val).await;
    } else if let Some(dac_2_val) = dac_2_val {
        write_to_dac(sm2, dma_ch, dac_2_addr, dac_2_val).await;
    }
}

#[inline(always)]
async fn write_value_to_dac(
    sm2: &mut StateMachine<'static, PIO1, 2>,
    dma_ch: &mut Option<DmaChannel<'static>>,
    dac_addr: Mpc4725DeviceAddress,
    dac_val: u16,
    dac_is_non_zero: &mut bool,
) {
    let dac_val = if dac_val != 0 {
        *dac_is_non_zero = true;
        Some(dac_val)
    } else if *dac_is_non_zero {
        *dac_is_non_zero = false;
        Some(0)
    } else {
        None
    };
    if let Some(dac_val) = dac_val {
        let data_out = get_tx_fifo_value(dac_addr, dac_val);
        if let Some(dma_ch) = dma_ch.as_mut() {
            sm2.tx().dma_push(dma_ch, &[data_out], false).await;
        } else {
            sm2.tx().wait_push(data_out).await;
        }
    }
}

#[inline(always)]
async fn write_to_dac(
    sm2: &mut StateMachine<'static, PIO1, 2>,
    dma_ch: &mut Option<DmaChannel<'static>>,
    dac_addr: Mpc4725DeviceAddress,
    dac_val: u16,
) {
    // -- DAC 1
    let data_out = get_tx_fifo_value(dac_addr, dac_val);
    // -- push the data to the TX FIFO
    if let Some(dma_ch) = dma_ch.as_mut() {
        sm2.tx().dma_push(dma_ch, &[data_out], false).await;
    } else {
        sm2.tx().wait_push(data_out).await;
    }
}

#[inline(always)]
async fn write_to_both_dacs(
    sm2: &mut StateMachine<'static, PIO1, 2>,
    dma_ch: &mut Option<DmaChannel<'static>>,
    dac_1_addr: Mpc4725DeviceAddress,
    dac_1_val: u16,
    dac_2_addr: Mpc4725DeviceAddress,
    dac_2_val: u16,
) {
    // -- DAC 1
    let data_out_1 = get_tx_fifo_value(dac_1_addr, dac_1_val);
    // -- DAC 2
    let data_out_2 = get_tx_fifo_value(dac_2_addr, dac_2_val);
    // -- push the data to the TX FIFO
    if let Some(dma_ch) = dma_ch.as_mut() {
        sm2.tx()
            .dma_push(dma_ch, &[data_out_1, data_out_2], false)
            .await;
    } else {
        sm2.tx().wait_push(data_out_1).await;
        sm2.tx().wait_push(data_out_2).await;
    }
}

#[inline(always)]
fn get_tx_fifo_value(dac_addr: Mpc4725DeviceAddress, dac_val: u16) -> u32 {
    let dac_addr = (dac_addr.value() as u32) << 1;
    let data_byte_1 = ((dac_val >> 8) as u8) as u32;
    let data_byte_2 = ((dac_val & 0xff) as u8) as u32;
    (3 << 24) | (dac_addr << 16) | (data_byte_1 << 8) | data_byte_2
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
