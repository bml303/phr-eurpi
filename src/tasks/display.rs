use core::fmt::Write;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_futures::yield_now;
use embassy_rp::{
    dma::Channel as DmaChannel,
    i2c::{Async as I2cAsync, I2c},
    peripherals::{I2C0, PIO0},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin,
        Instance as PioInstance, Irq, PioPin, ShiftDirection, StateMachine, program::pio_asm,
    },
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use heapless::String;
//use ssd1306::{I2CDisplayInterface, Ssd1306, mode::TerminalMode, prelude::*};

use crate::io::i2c::{i2cpio::I2CPIO, ssd1306::*};

#[embassy_executor::task]
pub async fn display_task(
    // mut display: Ssd1306<
    //     I2CInterface<I2c<'static, I2C0, I2cAsync>>,
    //     DisplaySize128x32,
    //     TerminalMode,
    // >,
    // text_style: MonoTextStyle<'static, BinaryColor>,
    i2cpio: I2CPIO<'static>,
    display_channel: &'static Channel<CriticalSectionRawMutex, String<14>, 10>,
) {
    yield_now().await;
    //defmt::debug!("SSD1306 initialization");
    let mut ssd1306 = SSD1306::new(i2cpio, SSD1306Addr::Default);
    ssd1306.init().await;
    //defmt::debug!("SSD1306 init completed");
    // -- intro sequence: flash the screen 3 times
    let delay_dur = Duration::from_millis(500);
    yield_now().await;
    Timer::after(delay_dur).await;
    //defmt::debug!("SSD1306 display on all");
    ssd1306.enable_entire_on_all().await; // -- set all pixels on
    yield_now().await;
    Timer::after(delay_dur).await;
    //loop {
    for _i in 0..3 {
        //defmt::debug!("SSD1306 display off");
        ssd1306.set_display_off().await; // -- switch display off
        yield_now().await;
        Timer::after(delay_dur).await;
        //defmt::debug!("SSD1306 display on");
        ssd1306.set_display_on().await; // -- switch display on
        yield_now().await;
        Timer::after(delay_dur).await;
    }
    //defmt::debug!("SSD1306 display off");
    ssd1306.set_display_off().await; // -- switch display off
    yield_now().await;
    //defmt::debug!("SSD1306 display on ram");
    ssd1306.disable_entire_on().await; // -- go back to following RAM for pixel state
    yield_now().await;
    ssd1306.set_display_on().await; // -- switch display on
    yield_now().await;
    // -- zero the entire display
    defmt::debug!("SSD1306 clearing display");
    let mut display_buf: [u8; SSD1306_BUF_LEN] = [0; SSD1306_BUF_LEN];
    let frame_area = SSD1306RenderArea::new();
    ssd1306.render(display_buf, &frame_area).await;
    defmt::debug!("SSD1306 display cleared");
    return;

    // let _ = display.init();
    // let _ = display.clear();
    loop {
        //let status_string = display_channel.receive().await;
        if let Ok(status_string) = display_channel.try_receive() {
            //defmt::debug!("SSD1306 status string {}", status_string);
            //SSD1306::draw_line(&mut display_buf, 0, 0, 32, 32, true);
            //SSD1306::write_string(&mut display_buf, 0, 0, &status_string);
            //ssd1306.render(display_buf, &frame_area).await;
            // let _ = display.set_position(0, 0);
            // for c in status_string.drain(..) {
            //     let _ = display.print_char(c);
            //     yield_now().await;
            // }
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
