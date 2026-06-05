use core::fmt::Write;

use embassy_futures::yield_now;
use embassy_rp::{
    i2c::{Async as I2cAsync, I2c},
    peripherals::I2C0,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::TerminalMode, prelude::*};

#[embassy_executor::task]
pub async fn display_task(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, I2cAsync>>,
        DisplaySize128x32,
        TerminalMode,
    >,
    text_style: MonoTextStyle<'static, BinaryColor>,
    display_channel: &'static Channel<CriticalSectionRawMutex, String<14>, 10>,
) {
    let _ = display.init();
    let _ = display.clear();
    loop {
        //let status_string = display_channel.receive().await;
        if let Ok(mut status_string) = display_channel.try_receive() {
            // let _ = display.clear();
            let _ = display.set_position(0, 0);
            for c in status_string.drain(..) {
                let _ = display.print_char(c);
                yield_now().await;
            }
            //let _ = display.write_str(status_string.as_str());
            // let _ = display.clear(BinaryColor::Off);
            // let _ =
            //     Text::with_baseline(&status_string, Point::new(0, 16), text_style, Baseline::Top)
            //         .draw(&mut display);
            // let _ = display.flush().await;
        }
        yield_now().await;
    }
}
