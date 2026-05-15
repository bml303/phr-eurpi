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
    ChannelOscillatorType, SAMPLE_BLOCK_SIZE, SAMPLE_RATE_24KHZ, SAMPLE_RATE_48KHZ,
    SM2_CLOCK_DIVIDER_1_6_MHZ, SM2_CLOCK_DIVIDER_4_MHZ,
};

// -- ---------------------------------------------------------------------
// -- SM2 - MPC4725 i2c DAC output
// -- ---------------------------------------------------------------------

pub fn setup_pio_task_sm2<'d>(
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
            set pindirs, 1      side 1 [2]      ; -- 00 - SDA output
        .wrap_target
            pull ifempty block  side 1          ; -- 01 - load 32 bits from TX FIFO into OSR, SCL high
            set pins, 0         side 1 [2]      ; -- 02 - START condition SDA to low, SCL high
            set x, 7            side 1          ; -- 03 - write 8 bits, SCL high
            set y, 2            side 0 [2]      ; -- 04 - write 3 bytes, SCL low
        bit_loop:
            out pins, 1         side 0          ; -- 05 - read next bit from OSR, SCL low
            nop                 side 1 [3]      ; -- 06 - confirm SDA value with SCL pulse
            jmp x-- bit_loop    side 0 [2]      ; -- 07 - jump if x > 0 prior to decrement
            set pindirs, 0      side 0          ; -- 08 - SDA input
            set x, 7            side 1 [3]      ; -- 09 - confirm SDA value with SCL pulse
            jmp pin do_nack     side 0          ; -- 10 - Check ACK from MPC4725
            set pindirs, 1      side 0          ; -- 11 - SDA output
            jmp y-- bit_loop    side 0          ; -- 12 - jump if y > 0 prior to decrement
        do_stop:
            set pins, 0         side 0 [1]      ; -- 13 - SDA low, SCL low
            out null, 32        side 0          ; -- 14 - remainder of OSR is invalid
            set pins, 0         side 1 [3]      ; -- 15 - SDA low, SCL high
            set pins, 1         side 1 [2]      ; -- 16 - STOP condition SDA to high, SCL high
        .wrap
        do_nack:
            irq nowait 2        side 0 [2]      ; -- 17 - indicate error, SCL high
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
    cfg.clock_divider = calculate_pio_clock_divider(SM2_CLOCK_DIVIDER_4_MHZ); // -- bus speed 1 MHz
    cfg.out_sticky = true;
    cfg.shift_out.auto_fill = false;
    cfg.shift_out.direction = ShiftDirection::Left;
    cfg.shift_out.threshold = 32;
    cfg.fifo_join = FifoJoin::TxOnly;
    sm2.set_config(&cfg);
    sm2.set_pin_dirs(PioPinDirection::Out, &[&sda_pin, &scl_pin]);
    sm2.set_pins(Level::High, &[&sda_pin, &scl_pin]);
}

#[embassy_executor::task]
pub async fn pio_task_sm2(
    mut sm2: StateMachine<'static, PIO1, 2>,
    mut dma_out_ch: Option<DmaChannel<'static>>,
) {
    info!("pio_task_sm2 started");
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
    sm2.set_enable(true);
    loop {
        osc.render(f, &mut out);
        let sample = ((out[0] + 1f32) * 4096f32 / 2f32) as u16;
        let addr: u32 = 0b11000100;
        let data_byte_1 = (sample >> 8) as u8;
        let data_byte_2 = (sample & 0xff) as u8;
        let data_out = (addr << 24) | ((data_byte_1 as u32) << 16) | ((data_byte_2 as u32) << 8);
        if let Some(dma_out_ch) = dma_out_ch.as_mut() {
            sm2.tx().dma_push(dma_out_ch, &[data_out], false).await;
        } else {
            sm2.tx().wait_push(data_out).await;
        }
        yield_now().await;
        Timer::after(Duration::from_micros(1)).await;
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

#[embassy_executor::task]
pub async fn pio_task_sm2_irq2(mut irq2: Irq<'static, PIO1, 2>) {
    loop {
        irq2.wait().await;
        error!("IRQ 2 trigged - MPC4725 state machine is in trouble...");
    }
}

#[embassy_executor::task]
pub async fn osc_task_generate(channel_oscillator: &'static ChannelOscillatorType) {
    info!("Oscillator generate task started");
    let sample_rate = SAMPLE_RATE_48KHZ;
    //let frequency = 110.0;
    let frequency = 440.0;
    //let frequency = 880.0;
    //let frequency = 5000.0;
    let f = frequency / sample_rate;
    let mut out = [0.0; SAMPLE_BLOCK_SIZE];
    let mut samples = [0u16; SAMPLE_BLOCK_SIZE];
    let mut osc = SineOscillator::new();
    osc.init();
    loop {
        while channel_oscillator.free_capacity() > 0 {
            //debug!("rendering...");
            osc.render(f, &mut out);
            for i in 0..out.len() {
                // -- normalize the sample and send it to the DAC
                samples[i] = ((out[i] + 1f32) * 4096f32 / 2f32) as u16;
            }
            channel_oscillator.send(samples).await;
            Timer::after_micros(2).await;
        }
        //info!("Oscillator generate task: Channel saturated");
        Timer::after_millis(5).await;
    }
}

#[embassy_executor::task]
pub async fn osc_task_dac(
    mut i2c1: I2c<'static, I2C1, I2cAsync>,
    channel_oscillator: &'static ChannelOscillatorType,
) {
    info!("Oscillator DAC task started");
    // -- setup MPC47
    let mut mpc4725 = match Mpc4725::new(&mut i2c1, Mpc4725DeviceAddress::Default).await {
        Ok(mpc4725) => mpc4725,
        Err(err) => {
            error!("Failed to initialize MPC4725: {}", err);
            return;
        }
    };
    // -- initialize sample playing loop
    let sample_rate = SAMPLE_RATE_48KHZ;
    let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    debug!("Every nanos is {}", every_nanos);
    let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
    loop {
        let samples = channel_oscillator.receive().await;
        //debug!("Oscillator DAC task: Samples received");
        for sample in &samples {
            // -- normalize the sample and send it to the DAC
            mpc4725
                .write_dac_value_fast(&mut i2c1, *sample)
                //.write_dac_value_regular(&mut i2c1, *sample)
                .await
                .unwrap();
            ticker.next().await;
        }
    }
    // // -- setup oscillator
    // let sample_rate = SAMPLE_RATE_48KHZ;
    // let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    // debug!("Every nanos is {}", every_nanos);
    // //let frequency = 110.0;
    // //let frequency = 440.0;
    // let frequency = 880.0;
    // //let frequency = 5000.0;
    // let f = frequency / sample_rate;
    // let mut out = [0.0; 1];
    // let mut osc = SineOscillator::new();
    // osc.init();
    // osc.render(f, &mut out);
    // // let mut start = Instant::now();
    // let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
    // loop {
    //     // -- this kinda works
    //     osc.render(f, &mut out);
    //     let sample = out[0];
    //     let value = (((sample + 1f32) * 4096f32) / 2f32) as u16;
    //     // //debug!("sample is {}, value is {}", sample, value);
    //     mpc4725.write_dac_value(&mut i2c1, value).await.unwrap();
    //     ticker.next().await;
    // }
}

#[embassy_executor::task]
pub async fn osc_task_consolidated(mut i2c1: I2c<'static, I2C1, I2cAsync>) {
    info!("Oscillator task started");
    //let sample_rate = SAMPLE_RATE_48KHZ;
    let sample_rate = SAMPLE_RATE_24KHZ;
    //let frequency = 110.0;
    //let frequency = 440.0;
    let frequency = 880.0;
    //let frequency = 5000.0;
    let f = frequency / sample_rate;
    let mut out = [0.0; 1];
    let mut osc = SineOscillator::new();
    osc.init();
    // -- setup MPC47
    let mut mpc4725 = match Mpc4725::new(&mut i2c1, Mpc4725DeviceAddress::Default).await {
        Ok(mpc4725) => mpc4725,
        Err(err) => {
            error!("Failed to initialize MPC4725: {}", err);
            return;
        }
    };
    // -- initialize sample playing loop
    let sample_rate = SAMPLE_RATE_48KHZ;
    let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    debug!("Every nanos is {}", every_nanos);
    let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
    loop {
        osc.render(f, &mut out);
        let sample = ((out[0] + 1f32) * 4096f32 / 2f32) as u16;
        // -- normalize the sample and send it to the DAC
        mpc4725
            .write_dac_value_fast(&mut i2c1, sample)
            //.write_dac_value_regular(&mut i2c1, *sample)
            .await
            .unwrap();
        ticker.next().await;
    }
}
