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
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};

#[embassy_executor::task]
pub async fn display_task(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, I2cAsync>>,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >,
    text_style: MonoTextStyle<'static, BinaryColor>,
    display_channel: &'static Channel<CriticalSectionRawMutex, String<32>, 10>,
) {
    loop {
        //let status_string = display_channel.receive().await;
        if let Ok(status_string) = display_channel.try_receive() {
            // let _ = display.clear(BinaryColor::Off);
            // let _ =
            //     Text::with_baseline(&status_string, Point::new(0, 16), text_style, Baseline::Top)
            //         .draw(&mut display);
            // let _ = display.flush();
        }
        yield_now().await;
    }
}
