use defmt::*;
use embassy_rp::{
    gpio::Level,
    i2c::{Async as I2cAsync, I2c},
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
use portable_atomic::{AtomicU8, Ordering};
use ssd1306::{Ssd1306, mode::BufferedGraphicsMode, prelude::*};

use crate::audio::oscillator::sine_oscillator::SineOscillator;

use super::{
    ChannelInputsType, ChannelOscillatorType, PWM_VALUE_MAX, SAMPLE_BLOCK_SIZE, SAMPLE_RATE_48KHZ,
};

#[embassy_executor::task]
pub async fn core1_task(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, I2cAsync>>,
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
    channel_oscillator: &'static ChannelOscillatorType,
) {
    info!("Hello from core 1");
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

    // CHANNEL_OUT.send((CHANNEL_OUT_1, 0)).await;
    // CHANNEL_OUT.send((CHANNEL_OUT_2, 0)).await;
    // CHANNEL_OUT.send((CHANNEL_OUT_3, PWM_VALUE_MAX)).await;
    // CHANNEL_OUT.send((CHANNEL_OUT_4, PWM_VALUE_MAX / 2)).await;
    // CHANNEL_OUT.send((CHANNEL_OUT_5, PWM_VALUE_MAX)).await;
    // CHANNEL_OUT.send((CHANNEL_OUT_6, 0)).await;
    analog_out_1.store(0, Ordering::Relaxed);
    analog_out_2.store(0, Ordering::Relaxed);
    analog_out_3.store(PWM_VALUE_MAX, Ordering::Relaxed);
    analog_out_4.store(PWM_VALUE_MAX / 2, Ordering::Relaxed);
    analog_out_5.store(PWM_VALUE_MAX, Ordering::Relaxed);
    analog_out_6.store(0, Ordering::Relaxed);

    // -- setup oscillator
    let sample_rate = SAMPLE_RATE_48KHZ;
    let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    debug!("Every nanos is {}", every_nanos);
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
        if let Ok((ain, kn1, kn2, btn1_lvl, btn2_lvl)) = channel_inputs.try_receive() {
            // let (ain, kn1, kn2, btn1_lvl, btn2_lvl) = CHANNEL_CORES.receive().await;
            // {
            // -- normalize kn1 and kn2 to percent values 0 - 100
            let out1_value: u8 = PWM_VALUE_MAX - (kn1 as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
            let out2_value: u8 = PWM_VALUE_MAX - (kn2 as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
            // -- update out1 and out2
            analog_out_1.store(out1_value, Ordering::Relaxed);
            analog_out_2.store(out2_value, Ordering::Relaxed);
            let kn1 = 4096 - kn1;
            let kn2 = 4096 - kn2;
            // -- update display
            let btn1_lvl = match btn1_lvl {
                Level::High => "+",
                Level::Low => "-",
            };
            let btn2_lvl = match btn2_lvl {
                Level::High => "+",
                Level::Low => "-",
            };
            let mut format_buf = [0u8; 64];
            let level_text = format_no_std::show(
                &mut format_buf,
                format_args!("{} {} {} {} {}", ain, btn1_lvl, kn1, btn2_lvl, kn2),
            )
            .unwrap();
            Rectangle::with_corners(p1, p2)
                .draw_styled(&style, &mut display)
                //.into_styled(text_style)
                //.draw(&mut display)
                .unwrap();
            Text::with_baseline(level_text, Point::new(0, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            display.flush().unwrap();
        }
        Timer::after_millis(5).await;
    }
}
