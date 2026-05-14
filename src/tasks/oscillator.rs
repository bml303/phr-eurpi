use defmt::*;
use embassy_rp::{
    Peri,
    i2c::{Async as I2cAsync, I2c},
    peripherals::{I2C1, PIO0},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin, PioPin,
        ShiftDirection, StateMachine, program::pio_asm,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};
use embassy_time::{Duration, Ticker, Timer};

use crate::audio::oscillator::sine_oscillator::SineOscillator;
use crate::io::i2c::mpc4725::*;

use super::{
    ChannelOscillatorType, SAMPLE_BLOCK_SIZE, SAMPLE_RATE_48KHZ, SM2_CLOCK_DIVIDER_48_KHZ,
};

// -- ---------------------------------------------------------------------
// -- SM2 - MPC4725 i2c DAC output
// -- ---------------------------------------------------------------------

pub fn setup_pio_task_sm2<'d>(
    pio: &mut Common<'d, PIO0>,
    sm2: &mut StateMachine<'d, PIO0, 2>,
    sda_pin: Peri<'d, impl PioPin>,
    scl_pin: Peri<'d, impl PioPin>,
) {
    // -- reset MPC4725, then continouously write data to DAC
    // -- side 0 is 1SCL
    let prg = pio_asm!(
        // r"
        // .origin 0
        // .side_set 2 opt                     ; -- SDA, SCL are side set
        // do_nack:
        //     irq wait 4      side 0b11       ; -- stop, SDA & SCL high, ask for help
        //     jmp entry_point side 0b11       ; -- restart processing
        // do_0:
        //     jmp x-- nojmp_0 side 0b00       ; -- decrement x with pseudo-jump, set zero value on SDA
        // nojmp_0:
        //     jmp !x last_0   side 0b01 [1]   ; -- confirm value with SCL pulse
        //     out y, 1        side 0b00       ; -- get next bit from OSR, keep SDA low, pull SCL down
        //     jmp y-- into_1  side 0b00       ; -- keep SDA low, pull SCL down
        // into_0:
        //     jmp do_0        side 0b00       ; -- set zero value on SDA
        // last_0:
        //     set pindirs, 0  side 0b00 [1]   ; -- SDA input, SDA low, SCL down
        //     jmp pin do_nack side 0b01 [1]   ; -- confirm SDA value with SCL pulse
        //     set pindirs, 1  side 0b00       ; -- SDA output, confirm SDA value with SCL pulse
        //     jmp entry_point side 0b00       ; -- byte done
        // do_1:
        //     jmp x-- nojmp_1 side 0b10       ; -- decrement x with pseudo-jump, set one value on SDA
        // nojmp_1:
        //     jmp !x last_1   side 0b11 [1]   ; -- confirm value with SCL pulse
        //     out y, 1        side 0b10       ; -- get next bit from OSR, keep SDA high, pull SCL down
        //     jmp !y into_0   side 0b10       ; -- keep SDA high, pull SCL down
        // into_1:
        //     jmp do_1        side 0b10       ; -- set one value on SDA
        // last_1:
        //     set pindirs, 0  side 0b10 [1]   ; -- SDA input, SDA high, SCL down
        //     jmp pin do_nack side 0b11 [1]   ; -- confirm SDA value with SCL pulse
        //     set pindirs, 1  side 0b10 [1]   ; -- SDA output, confirm SDA value with SCL pulse

        // do_byte:
        //     out y, 1                        ; -- read signal bit from OSR
        //     jmp !y do_stop                  ; -- non-zero indicating STOP
        //     out y, 1                        ; -- read next data bit from OSR
        //     set x, 7                        ; -- loop 8 times
        //     jmp y-- do_1                    ; -- jump if y > 0 prior to decrement
        //     jmp do_0
        // do_stop:
        //     out null, 32    side 0b01 [1]   ; -- reminder of OSR is invalid, SDA low, SCL high
        //     nop             side 0b11 [1]   ; -- STOP condition

        // public entry_point:
        // .wrap_target
        //     out y, 1        side 0b11       ; -- read next bit from OSR, SDA high, SCL high (idle)
        //     set x, 7        side 0b01       ; -- loop 8 times, START condition SDA low, SCL high
        //     jmp y-- into_1  side 0b00 [1]   ; -- jump if y > 0 prior to decrement
        //     jmp do_0        side 0b00       ; --
        //     nop
        // .wrap
        // ",
        r"
        .origin 0
        .side_set 1 opt                     ; -- SCL is side set
        do_0s:
            set x, 4        side 0          ; -- 01 - set number of zeros, SCL low
        loop_0s:
            set pins 0      side 0          ; -- 02 - set SDA low, SCL low
            jmp x-- end_0s  side 1 [3]      ; -- 03 - confirm SDA value with SCL pulse
            jmp loop_0s     side 0 [2]      ; -- 04 - jump if x > 0 prior to decrement
        end_0s:
            set x, 4        side 0 [2]      ; -- 05 - set number of bits, SCL low
        do_bits:
            out pins, 1     side 0          ; -- 06 - read next bit from OSR, SCL low
            nop             side 1 [3]      ; -- 07 - confirm SDA value with SCL pulse
            jmp x-- do_bits side 0 [2]      ; -- 08 - jump if x > 0 prior to decrement
            set pindirs, 0  side 0          ; -- 09 - SDA input
            jmp pin do_nack side 1 [2]      ; -- 10 - confirm SDA value with SCL pulse
            mov pc, y       side 1          ; -- 11 - return to caller
        do_nack:
            irq wait 4      side 1          ; -- 12 - stop, SCL high, ask for help
        public entry_point:
            set pindirs, 1  side 1          ; -- 13 - SDA output
            set pins, 1     side 1 [1]      ; -- 14 - SDA high, SCL high (idle)
        .wrap_target
            set pins, 0     side 1 [3]      ; -- 15 - START condition SDA to low, SCL high
            set x, 8        side 0          ; -- 16 - set number of bits, SCL low
            set y, 19       side 0          ; -- 17 - set return address 18, SCL low
            jmp do_bits     side 0          ; -- 18 - write bits, SCL low
            set y, 21       side 0          ; -- 19 - set return address 21, SCL low
            jmp do_0s       side 0          ; -- 20 - write 4 zeros, and four bits, SCL low
            set y, 23       side 0          ; -- 21 - set return address 23, SCL low
            jmp do_bits     side 0          ; -- 22 - write bits, SCL low
            set y, 25       side 0          ; -- 23 - set return address 25, SCL low
            jmp do_0s       side 0          ; -- 24 - write 4 zeros, and four bits, SCL low
            set y, 27       side 0          ; -- 25 - set return address 27, SCL low
            jmp do_bits     side 0          ; -- 26 - write bits, SCL low
            set pins, 0     side 0 [1]      ; -- 27 - SDA low, SCL low
            set pins, 0     side 1 [1]      ; -- 28 - SDA low, SCL high
            set pins, 1     side 1 [1]      ; -- 29 - STOP condition SDA to high, SCL high
        .wrap
        ",
    );
    // -- setup sm0
    let sda_pin = pio.make_pio_pin(sda_pin);
    let scl_pin = pio.make_pio_pin(scl_pin);
    let mut cfg = PioConfig::default();
    cfg.use_program(&pio.load_program(&prg.program), &[&scl_pin]);
    cfg.set_in_pins(&[&sda_pin]);
    cfg.set_set_pins(&[&sda_pin]);
    cfg.set_out_pins(&[&sda_pin]);
    cfg.clock_divider = calculate_pio_clock_divider(SM2_CLOCK_DIVIDER_48_KHZ);
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.direction = ShiftDirection::Left;
    sm2.set_config(&cfg);
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
