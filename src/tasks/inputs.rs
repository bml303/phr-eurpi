use core::fmt::Write;

//use defmt::*;
use embassy_futures::yield_now;
use embassy_rp::{
    adc::{Adc, Async as AdcAsync, Channel as AdcChannel},
    dma::Channel as DmaChannel,
    gpio::Level,
    i2c::{self, Async as I2cAsync, Config as I2cConfig, I2c},
    peripherals::I2C0,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use heapless::String;
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::TerminalMode, prelude::*};

use crate::{
    controls::{AnalogOutput, Debouncer},
    io::i2c::{i2cpio::I2CPIO, ssd1306::*},
    utils,
};

use super::{ChannelFrequencyType, ChannelInputsType, PWM_VALUE_MAX};

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
//
pub async fn init_display(
    i2cpio: I2CPIO<'static>,
    //i2c0: I2c<'static, I2C0, I2cAsync>,
) -> SSD1306<'static> {
    let mut ssd1306 = SSD1306::new(i2cpio, SSD1306Addr::Default);
    ssd1306.init().await;
    ssd1306.set_display_off().await; // -- switch display off
    ssd1306.enable_entire_on_all().await; // -- set all pixels on
    // ssd1306.set_display_on().await;
    let delay_dur = Duration::from_millis(500);
    for _i in 0..3 {
        ssd1306.set_display_on().await; // -- switch display off
        Timer::after(delay_dur).await;
        ssd1306.set_display_off().await; // -- switch display on
        Timer::after(delay_dur).await;
    }
    ssd1306
        .cmd_set_mem_addr_mode(SSD1306MemoryAddressMode::Page)
        .await;
    ssd1306.set_column_addr(0, 127).await;
    ssd1306.set_page_addr(0, 3).await;
    ssd1306.set_column_start_addr_for_page_mode(0).await;
    ssd1306.set_page_addr_start_for_page_mode(0).await;
    ssd1306.disable_scrolling().await;
    ssd1306.disable_entire_on().await; // -- go back to following RAM for pixel state
    ssd1306.set_display_on().await; // -- switch display on
    ssd1306
}

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
    // mut display: Ssd1306<
    //     I2CInterface<I2c<'static, I2C0, I2cAsync>>,
    //     DisplaySize128x32,
    //     TerminalMode,
    // >,
    //i2c0: I2c<'static, I2C0, I2cAsync>,
    i2cpio: I2CPIO<'static>,
    //display_channel: &'static Channel<CriticalSectionRawMutex, String<14>, 10>,
    frequency_channel: &'static ChannelFrequencyType,
) {
    // -- perpare display
    let mut display_buf: [u8; SSD1306_BUF_LEN] = [0; SSD1306_BUF_LEN];
    let mut ssd1306 = init_display(i2cpio).await;
    ssd1306.clear_display(&display_buf).await;
    let mut frame_area = SSD1306RenderArea::new();
    // frame_area.set_columns(0, 127);
    //SSD1306::write_string(&mut display_buf, 0, 0, "01234567890");
    //SSD1306::set_pixel(&mut display_buf, 0, 0, true);
    // display_buf[0] = 0xff;
    // display_buf[128 + 3] = 0xff;
    // display_buf[256 + 5] = 0xff;
    // display_buf[384 + 7] = 0xff;
    SSD1306::draw_line(&mut display_buf, 0, 0, 127, 0, true);
    SSD1306::draw_line(&mut display_buf, 0, 0, 0, 31, true);
    SSD1306::draw_line(&mut display_buf, 0, 31, 127, 31, true);
    SSD1306::draw_line(&mut display_buf, 127, 0, 127, 31, true);
    SSD1306::write_string(&mut display_buf, 8, 8, "This gugus");
    SSD1306::write_string(&mut display_buf, 8, 16, "  is happening");
    ssd1306.render(&display_buf, &frame_area).await;

    //let _ = display.write_char('A');
    //let _ = display.clear();
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

        if btn1_lvl == Level::Low {
            let kn1_val = 4096 - kn1_val;
            let _ = frequency_channel.try_send(kn1_val);
        }

        // // -- update display
        // let kn1_val = 4096 - kn1_val;
        // let kn2_val = 4096 - kn2_val;
        // // -- update display
        // let mut status_string: String<14> = String::new();
        // let ain_hexstr = utils::u16_to_hexstring(ain_val);
        // let _ = status_string.push_str(&ain_hexstr);
        // let _ = match btn1_lvl {
        //     Level::High => status_string.push_str("+"),
        //     Level::Low => status_string.push_str("-"),
        // };
        // let kn1_hexstr = utils::u16_to_hexstring(kn1_val);
        // let _ = status_string.push_str(&kn1_hexstr);
        // let _ = match btn2_lvl {
        //     Level::High => status_string.push_str("+"),
        //     Level::Low => status_string.push_str("-"),
        // };
        // let kn2_hexstr = utils::u16_to_hexstring(kn2_val);
        // let _ = status_string.push_str(&kn2_hexstr);

        // -- update display
        // let _ = display.set_position(0, 0);
        // for ch in status_string.chars() {
        //     //let _ = display.write_char(ch);
        //     let _ = display.print_char(ch);
        // }
        // frame_area.set_pages(1, 2);
        // SSD1306::write_string(&mut display_buf, 8, 8, "01234567890");
        // SSD1306::write_string(&mut display_buf, 8, 16, &status_string);
        // ssd1306.render(&display_buf, &frame_area).await;

        //let _ = display_channel.try_send(status_string);
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
