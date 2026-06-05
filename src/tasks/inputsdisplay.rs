use core::fmt::{self, Write};
//use defmt::*;
use embassy_futures::yield_now;
use embassy_rp::{
    adc::{Adc, Async as AdcAsync, Channel as AdcChannel},
    dma::Channel as DmaChannel,
    gpio::Level,
    i2c::{Async as I2cAsync, Blocking as I2cBlocking, I2c},
    peripherals::I2C0,
};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable},
    text::{Baseline, Text},
};
use heapless::HistoryBuf;
use portable_atomic::{AtomicU8, Ordering};
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};

use crate::controls::{AnalogOutput, Debouncer};

use super::{ChannelInputsType, PWM_VALUE_MAX};

// struct FormatBuf {
//     buf: [u8; 64],
//     len: usize,
// }

// impl Write for FormatBuf {
//     fn write_str(&mut self, s: &str) -> fmt::Result {
//         let bytes = s.as_bytes();
//         if self.len + bytes.len() > self.buf.len() {
//             return Err(fmt::Error);
//         }
//         self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
//         self.len += bytes.len();
//         Ok(())
//     }
// }

// fn format_values(
//     ain: u16,
//     btn1_lvl: bool,
//     kn1: u16,
//     btn2_lvl: bool,
//     kn2: u16,
// ) -> Result<FormatBuf, fmt::Error> {
//     let mut b = FormatBuf {
//         buf: [0; 64],
//         len: 0,
//     };
//     //write!(&mut b, "{}", ain)?;
//     // //write!(&mut b, "{} {} {} {} {}", ain, btn1_lvl, kn1, btn2_lvl, kn2)?;
//     // if btn1_lvl {
//     //     if btn2_lvl {
//     //         write!(&mut b, "{} + {} + {}", ain, kn1, kn2)?;
//     //     } else {
//     //         write!(&mut b, "{} + {} - {}", ain, kn1, kn2)?;
//     //     }
//     // } else {
//     //     if btn2_lvl {
//     //         write!(&mut b, "{} - {} + {}", ain, kn1, kn2)?;
//     //     } else {
//     //         write!(&mut b, "{} - {} - {}", ain, kn1, kn2)?;
//     //     }
//     // }
//     Ok(b)
// }

// fn format_values(ain: u16, btn1_lvl: bool, kn1: u16, btn2_lvl: bool, kn2: u16) -> &'static str {
//     if btn1_lvl {
//         if btn2_lvl {
//             "{} + {} + {}"
//         } else {
//             "{} + {} - {}"
//         }
//     } else {
//         if btn2_lvl {
//             "{} - {} + {}"
//         } else {
//             "{} - {} - {}"
//         }
//     }
// }

#[embassy_executor::task]
pub async fn inputs_display_task(
    mut adc: Adc<'static, AdcAsync>,
    mut adc_ch_ain: AdcChannel<'static>,
    mut adc_ch_kn1: AdcChannel<'static>,
    mut adc_ch_kn2: AdcChannel<'static>,
    mut dma_ch: DmaChannel<'static>,
    mut btn1: Debouncer<'static>,
    mut btn2: Debouncer<'static>,
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, I2cAsync>>,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >,
    text_style: MonoTextStyle<'static, BinaryColor>,
    mut analog_out_1: AnalogOutput<'static>,
    mut analog_out_2: AnalogOutput<'static>,
    mut analog_out_3: AnalogOutput<'static>,
    mut analog_out_4: AnalogOutput<'static>,
    mut analog_out_5: AnalogOutput<'static>,
    mut analog_out_6: AnalogOutput<'static>,
) {
    // // -- clear screen
    let p0 = Point::zero();
    let p1 = Point::new(0, 16);
    let p2 = Point::new(128, 32);
    // display.clear(BinaryColor::Off).unwrap();
    // display.flush().unwrap();
    // -- prepare analog out values
    analog_out_1.set_duty_cycle_percent(0);
    analog_out_2.set_duty_cycle_percent(0);
    analog_out_3.set_duty_cycle_percent(100);
    analog_out_4.set_duty_cycle_percent(50);
    analog_out_5.set_duty_cycle_percent(25);
    analog_out_6.set_duty_cycle_percent(0);
    // -- handle inputs / updates
    let mut format_buf = [0u8; 64];
    //let level_text = "Too slow to happen";
    let mut ain_prev = 0u16;
    let mut kn1_prev = 0u16;
    let mut kn2_prev = 0u16;
    let mut btn1_lvl_prev = Level::Low;
    let mut btn2_lvl_prev = Level::Low;
    let mut adc_channels = [adc_ch_ain, adc_ch_kn1, adc_ch_kn2];
    const ADC_BLOCK_SIZE: usize = 500;
    const ADC_NUM_CHANNELS: usize = 3;
    loop {
        let mut adc_buf = [0_u16; { ADC_BLOCK_SIZE * ADC_NUM_CHANNELS }];
        let adc_div = 3199u16; // 1000Hz sample rate (48Mhz / (5000Hz * 3ch) - 1)
        adc.read_many_multichannel(&mut adc_channels, &mut adc_buf, adc_div, &mut dma_ch)
            .await
            .unwrap();
        let mut ain_avg = ain_prev;
        let mut kn1_avg = kn1_prev;
        let mut kn2_avg = kn2_prev;
        for i in (0..adc_buf.len()).step_by(ADC_NUM_CHANNELS) {
            ain_avg = (ain_avg + adc_buf[i]) / 2;
            kn1_avg = (kn1_avg + adc_buf[i + 1]) / 2;
            kn2_avg = (kn2_avg + adc_buf[i + 2]) / 2;
        }
        let btn1_lvl = btn1.level().await;
        let btn2_lvl = btn2.level().await;
        // -- normalize kn1 and kn2 to percent values 0 - 100
        let out1_value: u8 = PWM_VALUE_MAX - (kn1_avg as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
        let out2_value: u8 = PWM_VALUE_MAX - (kn2_avg as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
        // -- update out1 and out2
        analog_out_1.set_duty_cycle_percent(out1_value);
        analog_out_2.set_duty_cycle_percent(out2_value);
        // // -- update display
        let do_update = ain_avg != ain_prev
            || kn1_avg != kn1_prev
            || kn2_avg != kn2_prev
            || btn1_lvl != btn1_lvl_prev
            || btn2_lvl != btn2_lvl_prev;
        if do_update {
            ain_prev = ain_avg;
            kn1_prev = kn1_avg;
            kn2_prev = kn2_avg;
            btn1_lvl_prev = btn1_lvl;
            btn2_lvl_prev = btn2_lvl;
            let kn1_avg = 4096 - kn1_avg;
            let kn2_avg = 4096 - kn2_avg;
            //f = frequency / kn1 as f32;
            // -- update display
            let btn1_lvl = match btn1_lvl {
                Level::High => "+",
                Level::Low => "-",
            };
            let btn2_lvl = match btn2_lvl {
                Level::High => "+",
                Level::Low => "-",
            };
            //let level_text = format_values(ain, btn1_lvl, kn1, btn2_lvl, kn2);
            // if let Ok(level_text) = format_values(ain, btn1_lvl, kn1, btn2_lvl, kn2) {
            //     // Text::with_baseline(level_text.buf, Point::new(0, 16), text_style, Baseline::Top)
            //     //     .draw(&mut display)
            // }

            // -- FIXME: This disrupts the other tasks, formatting and drawing both use too long and block everything else (why?)
            if let Ok(level_text) = format_no_std::show(
                &mut format_buf,
                format_args!(
                    "{} {} {} {} {}",
                    ain_avg, btn1_lvl, kn1_avg, btn2_lvl, kn2_avg
                ),
            ) {
                //yield_now().await;
                display.clear(BinaryColor::Off).unwrap();
                Text::with_baseline(level_text, Point::new(0, 16), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();
                display.flush().unwrap();
            }
        }
        yield_now().await;
        //Timer::after_millis(100).await;
    }
}
