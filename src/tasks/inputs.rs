//use defmt::*;
use embassy_futures::yield_now;
use embassy_rp::{
    adc::{Adc, Async as AdcAsync, Channel as AdcChannel},
    dma::Channel as DmaChannel,
    gpio::Level,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::{
    controls::{AnalogOutput, Debouncer},
    utils,
};

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
pub async fn inputs_task(
    mut adc: Adc<'static, AdcAsync>,
    mut adc_ch_ain: AdcChannel<'static>,
    mut adc_ch_kn1: AdcChannel<'static>,
    mut adc_ch_kn2: AdcChannel<'static>,
    //mut dma_ch: DmaChannel<'static>,
    mut btn1: Debouncer<'static>,
    mut btn2: Debouncer<'static>,
    mut analog_out_1: AnalogOutput<'static>,
    mut analog_out_2: AnalogOutput<'static>,
    mut analog_out_3: AnalogOutput<'static>,
    mut analog_out_4: AnalogOutput<'static>,
    mut analog_out_5: AnalogOutput<'static>,
    mut analog_out_6: AnalogOutput<'static>,
    display_channel: &'static Channel<CriticalSectionRawMutex, String<32>, 10>,
) {
    // -- prepare analog out values
    analog_out_1.set_duty_cycle_percent(0);
    analog_out_2.set_duty_cycle_percent(0);
    analog_out_3.set_duty_cycle_percent(100);
    analog_out_4.set_duty_cycle_percent(50);
    analog_out_5.set_duty_cycle_percent(25);
    analog_out_6.set_duty_cycle_percent(0);
    // -- handle inputs / updates
    // let mut adc_channels = [adc_ch_ain, adc_ch_kn1, adc_ch_kn2];
    // const ADC_NUM_CHANNELS: usize = 3;
    loop {
        // let mut adc_buf = [0_u16; ADC_NUM_CHANNELS];
        // let adc_div = 3199u16; // 1000Hz sample rate (48Mhz / (5000Hz * 3ch) - 1)
        // adc.read_many_multichannel(&mut adc_channels, &mut adc_buf, adc_div, &mut dma_ch)
        //     .await
        //     .unwrap();
        // let ain_val = adc_buf[0];
        // let kn1_val = adc_buf[1];
        // let kn2_val = adc_buf[2];

        let ain_val = adc.blocking_read(&mut adc_ch_ain).unwrap();
        let kn1_val = adc.blocking_read(&mut adc_ch_kn1).unwrap();
        let kn2_val = adc.blocking_read(&mut adc_ch_kn2).unwrap();
        // -- normalize kn1 and kn2 to percent values 0 - 100
        let out1_percent: u8 = PWM_VALUE_MAX - (kn1_val as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
        let out2_percent: u8 = PWM_VALUE_MAX - (kn2_val as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
        // -- update out1 and out2
        analog_out_1.set_duty_cycle_percent(out1_percent);
        analog_out_2.set_duty_cycle_percent(out2_percent);
        // -- read button status
        let btn1_lvl = btn1.level();
        let btn2_lvl = btn2.level();
        // -- update display
        let kn1_val = 4096 - kn1_val;
        let kn2_val = 4096 - kn2_val;
        // -- update display
        let mut status_string: String<32> = String::new();
        let ain_hexstr = utils::u16_to_hexstring(ain_val);
        let _ = status_string.push_str(&ain_hexstr);
        let _ = match btn1_lvl {
            Level::High => status_string.push_str(" + "),
            Level::Low => status_string.push_str(" - "),
        };
        let kn1_hexstr = utils::u16_to_hexstring(kn1_val);
        let _ = status_string.push_str(&kn1_hexstr);
        let _ = match btn2_lvl {
            Level::High => status_string.push_str(" + "),
            Level::Low => status_string.push_str(" - "),
        };
        let kn2_hexstr = utils::u16_to_hexstring(kn2_val);
        let _ = status_string.push_str(&kn2_hexstr);

        let _ = display_channel.try_send(status_string);
        //display_channel.send(status_string).await;

        // let _ = display.clear(BinaryColor::Off);
        // let _ = Text::with_baseline(&status_string, Point::new(0, 16), text_style, Baseline::Top)
        //     .draw(&mut display);
        // let _ = display.flush();

        //let level_text = format_values(ain, btn1_lvl, kn1, btn2_lvl, kn2);
        // if let Ok(level_text) = format_values(ain_avg, btn1_lvl, kn1_avg, btn2_lvl, kn2_avg) {
        //     // Text::with_baseline(level_text.buf, Point::new(0, 16), text_style, Baseline::Top)
        //     //     .draw(&mut display)
        // }

        // -- FIXME: This disrupts the other tasks, formatting and drawing both use too long and block everything else (why?)
        // if let Ok(level_text) = format_no_std::show(
        //     &mut format_buf,
        //     format_args!(
        //         "{} {} {} {} {}",
        //         ain_avg, btn1_lvl, kn1_avg, btn2_lvl, kn2_avg
        //     ),
        // ) {
        //     //yield_now().await;
        //     display.clear(BinaryColor::Off).unwrap();
        //     Text::with_baseline(level_text, Point::new(0, 16), text_style, Baseline::Top)
        //         .draw(&mut display)
        //         .unwrap();
        //     display.flush().unwrap();
        // }

        yield_now().await;
        Timer::after_millis(100).await;
    }
}
