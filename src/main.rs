#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use cortex_m_rt::entry;
//use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::Executor;
use embassy_rp::{
    // adc::{
    //     Adc, Channel as AdcChannel,
    //     Config as AdcConfig,
    //     InterruptHandler as AdcInterruptHandler
    // },
    bind_interrupts,
    dma,
    flash::Flash,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, Config},
    multicore::{Stack, spawn_core1},
    peripherals::{DMA_CH0, DMA_CH1, DMA_CH11, I2C0, I2C1, PIO0},
    pio::{InterruptHandler, Pio},
    rtc::Rtc,
    spi::{self, Spi},
    watchdog::*,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Delay, Timer};
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use fixed_dsp::basic::sin_i16;
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode, prelude::*};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// #[allow(dead_code)]
mod io;
// mod task;
mod utils;

use io::flash::FLASH_SIZE;

// use task::{
//     input_task, output_task, LogData, Ping, INIT_CHANNEL_CAPACITY, OUTPUT_CHANNEL_CAPACITY,
//     PING_CHANNEL_CAPACITY,
// };

const CARGO_PKG_NAME: &str = env!("CARGO_PKG_NAME");
const CARGO_PKG_VERSION: &str = env!("CARGO_PKG_VERSION");

// Bind the RTC interrupt to the handler
bind_interrupts!(struct IrqsRtc {
    RTC_IRQ => embassy_rp::rtc::InterruptHandler;
});

bind_interrupts!(struct IrqsPioSpiAndFlash {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH11>;
});

bind_interrupts!(struct IrqsI2c0 {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

bind_interrupts!(struct IrqsI2c1 {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

// bind_interrupts!(struct IrqsSpiAndFlash {
//     DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH11>;
// });

// bind_interrupts!(struct IrqsAdc {
//     ADC_IRQ_FIFO => AdcInterruptHandler;
// });

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, LedState, 1> = Channel::new();

enum LedState {
    On,
    Off,
}

//#[embassy_executor::main]
//async fn main(spawner: Spawner) {
#[entry]
fn main() -> ! {
    // -- init pico and get peripherals
    let p = embassy_rp::init(Default::default());
    info!("Starting up {} {}", CARGO_PKG_NAME, CARGO_PKG_VERSION);

    // -- ---------------------------------------------------------------------
    // -- RTC & watchdog
    // -- ---------------------------------------------------------------------

    let rtc = Rtc::new(p.RTC, IrqsRtc);
    let watchdog = Watchdog::new(p.WATCHDOG);

    // -- ---------------------------------------------------------------------
    // -- Display resources
    // -- ---------------------------------------------------------------------

    // -- init cs, dc and reset I/O pins
    let cs = Output::new(p.PIN_9, Level::High);
    let dc = Output::new(p.PIN_8, Level::High);
    let reset = Output::new(p.PIN_12, Level::High);

    // -- i2c bus 0 is used for display
    let sda_0 = p.PIN_0;
    let scl_0 = p.PIN_1;
    info!("Setting up i2c bus 0");
    let i2c0 = i2c::I2c::new_async(p.I2C0, scl_0, sda_0, IrqsI2c0, Config::default());

    // -- display config
    let interface = I2CDisplayInterface::new(i2c0);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    // -- user keys
    let key1 = Input::new(p.PIN_15, Pull::None);
    let key2 = Input::new(p.PIN_17, Pull::None);

    // -- i2c bus 1 is used for I2C peripherals
    let sda_1 = p.PIN_2;
    let scl_1 = p.PIN_3;
    info!("Setting up i2c bus 1");
    let i2c1 = i2c::I2c::new_async(p.I2C1, scl_1, sda_1, IrqsI2c1, Config::default());

    // -- ---------------------------------------------------------------------
    // -- Flash
    // -- ---------------------------------------------------------------------

    let mut flash = Flash::<_, embassy_rp::flash::Async, FLASH_SIZE>::new(
        p.FLASH,
        p.DMA_CH11,
        IrqsPioSpiAndFlash,
    );

    let (flash_uid_val, _flash_uid) = io::flash::check_flash(&mut flash);
    let board_id = utils::u64_to_hexstring(flash_uid_val);

    Text::with_baseline(board_id.as_str(), Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    // // -- ---------------------------------------------------------------------
    // // -- ADC / Temperature resources
    // // -- ---------------------------------------------------------------------

    // let adc = Adc::new(p.ADC, IrqsAdc, AdcConfig::default());
    // let p26 = AdcChannel::new_pin(p.PIN_26, Pull::None);
    // let p27 = AdcChannel::new_pin(p.PIN_27, Pull::None);
    // let p28 = AdcChannel::new_pin(p.PIN_28, Pull::None);
    // let ts = AdcChannel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // -- ---------------------------------------------------------------------
    // -- LED
    // -- ---------------------------------------------------------------------
    //
    let led = Output::new(p.PIN_25, Level::Low);

    // -- ---------------------------------------------------------------------
    // -- Core 1 task
    // -- ---------------------------------------------------------------------

    // -- spawn i2c sensoring task on core 1
    info!("Spawning Task running on core 0");
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.spawn(unwrap!(core1_task(led))));
        },
    );

    // -- ---------------------------------------------------------------------
    // -- Core 0 task
    // -- ---------------------------------------------------------------------

    // -- run output task on core 0
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.spawn(unwrap!(core0_task())));
}

#[embassy_executor::task]
async fn core0_task() {
    let x: i16 = 0x4000;
    let y = sin_i16(x);
    info!("Hello from core 0: sin_q15 = {}", y);
    loop {
        CHANNEL.send(LedState::On).await;
        Timer::after_millis(100).await;
        CHANNEL.send(LedState::Off).await;
        Timer::after_millis(400).await;
    }
}

#[embassy_executor::task]
async fn core1_task(mut led: Output<'static>) {
    info!("Hello from core 1");
    loop {
        match CHANNEL.receive().await {
            LedState::On => led.set_high(),
            LedState::Off => led.set_low(),
        }
    }
}
