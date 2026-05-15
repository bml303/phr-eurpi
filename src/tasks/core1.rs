use core::fmt::{self, Write};
//use defmt::*;
use embassy_futures::yield_now;
use embassy_rp::{
    adc::{Adc, Async as AdcAsync, Channel as AdcChannel},
    gpio::Level,
    i2c::{Async as I2cAsync, Blocking as I2cBlocking, I2c},
    peripherals::I2C0,
};
use embassy_time::Timer;
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

use crate::utils::Debouncer;

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
pub async fn core1_task(
    mut adc: Adc<'static, AdcAsync>,
    mut p26: AdcChannel<'static>,
    mut p27: AdcChannel<'static>,
    mut p28: AdcChannel<'static>,
    mut btn1: Debouncer<'static>,
    mut btn2: Debouncer<'static>,
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, I2cBlocking>>,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >,
    text_style: MonoTextStyle<'static, BinaryColor>,
    analog_out_1: &'static AtomicU8,
    analog_out_2: &'static AtomicU8,
    analog_out_3: &'static AtomicU8,
    analog_out_4: &'static AtomicU8,
    analog_out_5: &'static AtomicU8,
    analog_out_6: &'static AtomicU8,
    channel_inputs: &'static ChannelInputsType,
) {
    //info!("Hello from core 1");
    // -- clear screen
    let p0 = Point::zero();
    let p1 = Point::new(0, 16);
    let p2 = Point::new(128, 32);
    let style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .build();
    Rectangle::with_corners(p0, p2)
        .draw_styled(&style, &mut display)
        .unwrap();
    display.flush().unwrap();
    // -- prepare analog out values
    analog_out_1.store(0, Ordering::Relaxed);
    analog_out_2.store(0, Ordering::Relaxed);
    analog_out_3.store(PWM_VALUE_MAX, Ordering::Relaxed);
    analog_out_4.store(PWM_VALUE_MAX / 2, Ordering::Relaxed);
    analog_out_5.store(PWM_VALUE_MAX, Ordering::Relaxed);
    analog_out_6.store(0, Ordering::Relaxed);
    // -- handle inputs / updates
    let mut format_buf = [0u8; 64];
    //let level_text = "Too slow to happen";
    let mut ain_prev = 0u16;
    let mut kn1_avg_prev = 0u16;
    let mut kn2_avg_prev = 0u16;
    let mut btn1_lvl_prev = Level::Low;
    let mut btn2_lvl_prev = Level::Low;
    let mut kn1_buffer: HistoryBuf<u16, 10> = Default::default();
    let mut kn2_buffer: HistoryBuf<u16, 10> = Default::default();
    loop {
        let ain = adc.read(&mut p26).await.unwrap();
        let kn1 = adc.read(&mut p27).await.unwrap();
        let kn2 = adc.read(&mut p28).await.unwrap();
        let btn1_lvl = btn1.level().await;
        let btn2_lvl = btn2.level().await;
        // -- unjitter knobs
        kn1_buffer.write(kn1);
        let kn1_avg =
            (kn1_buffer.iter().map(|x| *x as u32).sum::<u32>() / kn1_buffer.len() as u32) as u16;
        kn2_buffer.write(kn2);
        let kn2_avg =
            (kn2_buffer.iter().map(|x| *x as u32).sum::<u32>() / kn2_buffer.len() as u32) as u16;
        // -- normalize kn1 and kn2 to percent values 0 - 100
        let out1_value: u8 = PWM_VALUE_MAX - (kn1 as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
        let out2_value: u8 = PWM_VALUE_MAX - (kn2 as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
        // -- update out1 and out2
        analog_out_1.store(out1_value, Ordering::Relaxed);
        analog_out_2.store(out2_value, Ordering::Relaxed);
        // -- update display
        let do_update = ain != ain_prev
            || kn1_avg != kn1_avg_prev
            || kn2_avg != kn2_avg_prev
            || btn1_lvl != btn1_lvl_prev
            || btn2_lvl != btn2_lvl_prev;
        if do_update {
            ain_prev = ain;
            kn1_avg_prev = kn1_avg;
            kn2_avg_prev = kn2_avg;
            btn1_lvl_prev = btn1_lvl;
            btn2_lvl_prev = btn2_lvl;
            //let (ain, kn1, kn2, btn1_lvl, btn2_lvl) = channel_inputs.receive().await;
            //if let Ok((ain, kn1, kn2, btn1_lvl, btn2_lvl)) = channel_inputs.try_receive() {
            let kn1 = 4096 - kn1;
            let kn2 = 4096 - kn2;
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
            // if let Ok(level_text) = format_no_std::show(
            //     &mut format_buf,
            //     format_args!("{} {} {} {} {}", ain, btn1_lvl, kn1, btn2_lvl, kn2),
            // ) {
            //     if let Ok(_) = Rectangle::with_corners(p1, p2).draw_styled(&style, &mut display) {
            //         if let Ok(_) = Text::with_baseline(
            //             level_text,
            //             Point::new(0, 16),
            //             text_style,
            //             Baseline::Top,
            //         )
            //         .draw(&mut display)
            //         {
            //             if let Ok(_) = display.flush() {}
            //         }
            //     }
            // }
            // yield_now().await;
            // continue;
        }
        //}
        //yield_now().await;
        Timer::after_millis(50).await;
    }
}
