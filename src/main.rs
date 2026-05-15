#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use cortex_m_rt::entry;
//use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_rp::{
    Peri,
    adc::{
        Adc, Async as AdcAsync, Channel as AdcChannel, Config as AdcConfig,
        InterruptHandler as AdcInterruptHandler,
    },
    bind_interrupts,
    clocks::{ClockConfig, clk_sys_freq, core_voltage},
    config::Config,
    dma,
    flash::Flash,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, Async as I2cAsync, Config as I2cConfig, I2c},
    interrupt,
    interrupt::{InterruptExt, Priority},
    multicore::{Stack, spawn_core1},
    peripherals::{DMA_CH0, DMA_CH1, DMA_CH11, I2C0, I2C1, PIO0, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin,
        InterruptHandler as PioInterruptHandler, Irq, PinConfig, Pio, PioPin, ShiftDirection,
        StateMachine, program::pio_asm,
    },
    pio_programs::{
        clock_divider::calculate_pio_clock_divider,
        pwm::{PioPwm, PioPwmProgram},
    },
    rtc::Rtc,
    spi::{self, Spi},
    watchdog::*,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, WithTimeout};
use embedded_graphics::{
    mono_font::{MonoTextStyle, MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable},
    text::{Baseline, Text},
};
use heapless::Vec;
use portable_atomic::{AtomicU8, Ordering};
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode, prelude::*};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// #[allow(dead_code)]
mod audio;
mod io;
mod tasks;
mod utils;

use io::flash::FLASH_SIZE;
use tasks::{
    ChannelInputsType, ChannelOscillatorType, I2C1_BUS_FREQUENCY_1_MBIT, core0_task, core1_task,
    osc_task_consolidated, osc_task_dac, osc_task_generate, pio_task_sm0, pio_task_sm1,
    pio_task_sm2, pio_task_sm2_irq1, pio_task_sm2_irq2, setup_pio_task_sm0, setup_pio_task_sm1,
    setup_pio_task_sm2,
};
use utils::Debouncer;

const CARGO_PKG_NAME: &str = env!("CARGO_PKG_NAME");
const CARGO_PKG_VERSION: &str = env!("CARGO_PKG_VERSION");

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR_CORE0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_CORE1: StaticCell<Executor> = StaticCell::new();
// static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
// static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

static ANALOG_OUT_1: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_2: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_3: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_4: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_5: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_6: AtomicU8 = AtomicU8::new(0);
static CHANNEL_INPUTS: ChannelInputsType = Channel::new();
static CHANNEL_OSCILLATOR: ChannelOscillatorType = Channel::new();

// Bind the RTC interrupt to the handler
bind_interrupts!(struct IrqsRtc {
    RTC_IRQ => embassy_rp::rtc::InterruptHandler;
});

bind_interrupts!(struct IrqsPioSpiAndFlash {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH11>;
});

bind_interrupts!(struct IrqsPio1 {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

bind_interrupts!(struct IrqsI2c0 {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

bind_interrupts!(struct IrqsI2c1 {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

bind_interrupts!(struct IrqsAdc {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

// #[interrupt]
// unsafe fn SWI_IRQ_1() {
//     unsafe { EXECUTOR_HIGH.on_interrupt() }
// }

// #[interrupt]
// unsafe fn SWI_IRQ_2() {
//     unsafe { EXECUTOR_MEDIUM.on_interrupt() }
// }

//#[embassy_executor::main]
//async fn main(spawner: Spawner) {
#[entry]
fn main() -> ! {
    // Set up for clock frequency of 200 MHz, setting all necessary defaults.
    let config = Config::new(ClockConfig::system_freq(200_000_000).unwrap());

    // -- init pico and get peripherals
    //let p = embassy_rp::init(Default::default());
    let p = embassy_rp::init(config);
    info!("Starting up {} {}", CARGO_PKG_NAME, CARGO_PKG_VERSION);

    // -- ---------------------------------------------------------------------
    // -- RTC & watchdog
    // -- ---------------------------------------------------------------------

    // let rtc = Rtc::new(p.RTC, IrqsRtc);
    // let watchdog = Watchdog::new(p.WATCHDOG);

    // -- ---------------------------------------------------------------------
    // -- Display resources
    // -- ---------------------------------------------------------------------

    // -- init cs, dc and reset I/O pins
    // let cs = Output::new(p.PIN_9, Level::High);
    // let dc = Output::new(p.PIN_8, Level::High);
    // let reset = Output::new(p.PIN_12, Level::High);

    // -- i2c bus 0 is used for display
    let sda_0 = p.PIN_0;
    let scl_0 = p.PIN_1;
    info!("Setting up i2c bus 0");
    let i2c0 = i2c::I2c::new_async(p.I2C0, scl_0, sda_0, IrqsI2c0, I2cConfig::default());

    // -- display config
    let interface = I2CDisplayInterface::new(i2c0);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // -- ---------------------------------------------------------------------
    // -- I2C bus for peripherals
    // -- ---------------------------------------------------------------------

    // -- i2c bus 1 is used for I2C peripherals
    // let sda_1 = p.PIN_2;
    // let scl_1 = p.PIN_3;
    // info!("Setting up i2c bus 1");
    // let i2c1_config = {
    //     let mut i2c_config = I2cConfig::default();
    //     //i2c_config.frequency = I2C1_BUS_FREQUENCY_400_KBIT;
    //     i2c_config.frequency = I2C1_BUS_FREQUENCY_1_MBIT;
    //     i2c_config.scl_pullup = true;
    //     i2c_config.sda_pullup = true;
    //     i2c_config
    // };
    // //let i2c1_config = I2cConfig::default();
    // let i2c1 = i2c::I2c::new_async(p.I2C1, scl_1, sda_1, IrqsI2c1, i2c1_config);

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

    // -- ---------------------------------------------------------------------
    // -- ADC / Temperature resources
    // -- ---------------------------------------------------------------------

    let adc = Adc::new(p.ADC, IrqsAdc, AdcConfig::default());
    let p26 = AdcChannel::new_pin(p.PIN_26, Pull::None);
    let p27 = AdcChannel::new_pin(p.PIN_27, Pull::None);
    let p28 = AdcChannel::new_pin(p.PIN_28, Pull::None);
    //let ts = AdcChannel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    info!("ADC channels ready");

    // -- ---------------------------------------------------------------------
    // -- Buttons
    // -- ---------------------------------------------------------------------
    //
    let btn1 = Debouncer::new(Input::new(p.PIN_4, Pull::Up), Duration::from_millis(20));
    let btn2 = Debouncer::new(Input::new(p.PIN_5, Pull::Up), Duration::from_millis(20));
    info!("Buttons ready");

    // -- ---------------------------------------------------------------------
    // -- PIO task(s) for digital input & analog output
    // -- ---------------------------------------------------------------------

    let Pio {
        mut common,
        irq1,
        irq2,
        irq3,
        mut sm0,
        mut sm1,
        mut sm2,
        ..
    } = Pio::new(p.PIO0, IrqsPioSpiAndFlash);
    setup_pio_task_sm0(&mut common, &mut sm0, p.PIN_22);
    info!("pio_task_sm0 is setup");
    setup_pio_task_sm1(
        &mut common,
        &mut sm1,
        p.PIN_16, // -- out 3
        p.PIN_17, // -- out 4
        p.PIN_18, // -- out 5
        p.PIN_19, // -- out 6
        p.PIN_20, // -- out 2
        p.PIN_21, // -- out 1
    );
    info!("pio_task_sm1 is setup");
    let sda_1 = p.PIN_2;
    let scl_1 = p.PIN_3;
    setup_pio_task_sm2(&mut common, &mut sm2, sda_1, scl_1);
    info!("pio_task_sm2 is setup");
    //let dma_out_ref = dma::Channel::new(p.DMA_CH0, IrqsPioSpiAndFlash);
    //info!("DMA channel for pio_task_sm2 ready");
    //let mut dma_in_ref = dma::Channel::new(p.DMA_CH1, IrqsPioSpiAndFlash);

    // -- Medium-priority executor: SWI_IRQ_2, priority level 3
    // interrupt::SWI_IRQ_2.set_priority(Priority::P3);
    // let spawner = EXECUTOR_MEDIUM.start(interrupt::SWI_IRQ_2);
    // spawner.spawn(unwrap!(osc_task_consolidated(i2c1)));
    //spawner.spawn(unwrap!(osc_task_generate(&CHANNEL_OSCILLATOR)));
    //spawner.spawn(unwrap!(osc_task_dac(i2c1, &CHANNEL_OSCILLATOR)));

    // -- High-priority executor: SWI_IRQ_1, priority level 2
    // interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    // let spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    // spawner.spawn(unwrap!(osc_task_consolidated(i2c1)));
    //spawner.spawn(unwrap!(osc_task_generate(&CHANNEL_OSCILLATOR)));
    //spawner.spawn(unwrap!(osc_task_dac(i2c1, &CHANNEL_OSCILLATOR)));

    // -- ---------------------------------------------------------------------
    // -- Core 1 task
    // -- ---------------------------------------------------------------------

    // -- spawn i2c sensoring task on core 1
    info!("Spawning Task running on core 1");
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR_CORE1.init(Executor::new());
            executor1.run(|spawner| {
                // spawner.spawn(unwrap!(core1_task(
                //     display,
                //     text_style,
                //     &ANALOG_OUT_1,
                //     &ANALOG_OUT_2,
                //     &ANALOG_OUT_3,
                //     &ANALOG_OUT_4,
                //     &ANALOG_OUT_5,
                //     &ANALOG_OUT_6,
                //     &CHANNEL_INPUTS,
                // )));
                spawner.spawn(unwrap!(pio_task_sm2_irq2(irq2)));
                //spawner.spawn(unwrap!(pio_task_sm2(sm2)));
                //spawner.spawn(unwrap!(osc_task_generate(&CHANNEL_OSCILLATOR)));
                //spawner.spawn(unwrap!(osc_task_dac(i2c1, &CHANNEL_OSCILLATOR)));
            });
        },
    );

    // -- ---------------------------------------------------------------------
    // -- Core 0 task
    // -- ---------------------------------------------------------------------

    // -- run output task on core 0
    let executor0 = EXECUTOR_CORE0.init(Executor::new());
    executor0.run(|spawner| {
        // spawner.spawn(unwrap!(core0_task(
        //     adc,
        //     p26,
        //     p27,
        //     p28,
        //     btn1,
        //     btn2,
        //     &CHANNEL_INPUTS
        // )));
        // spawner.spawn(unwrap!(pio_task_sm0(irq3, sm0)));
        // spawner.spawn(unwrap!(pio_task_sm1(
        //     sm1,
        //     &ANALOG_OUT_1,
        //     &ANALOG_OUT_2,
        //     &ANALOG_OUT_3,
        //     &ANALOG_OUT_4,
        //     &ANALOG_OUT_5,
        //     &ANALOG_OUT_6,
        // )));
        //spawner.spawn(unwrap!(pio_task_sm2_irq1(irq1, sm2)));
        spawner.spawn(unwrap!(pio_task_sm2(sm2)));
        //spawner.spawn(unwrap!(osc_task_consolidated(i2c1)));
        //spawner.spawn(unwrap!(osc_task_generate(&CHANNEL_OSCILLATOR)));
        //spawner.spawn(unwrap!(osc_task_dac(i2c1, &CHANNEL_OSCILLATOR)));
    });
}
