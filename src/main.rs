#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::mem::ManuallyDrop;
use core::sync::atomic::{AtomicBool, compiler_fence};
use cortex_m_rt::entry;
//use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_futures::yield_now;
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
    gpio::{self, Input, Level, Output, Pull},
    i2c::{self, Async as I2cAsync, Config as I2cConfig, I2c},
    interrupt,
    interrupt::{InterruptExt, Priority},
    multicore::Stack, //spawn_core1},
    pac,
    peripherals::{CORE1, DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH10, DMA_CH11, I2C0, I2C1, PIO0, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, FifoJoin,
        InterruptHandler as PioInterruptHandler, Irq, PinConfig, Pio, PioPin, ShiftDirection,
        StateMachine, program::pio_asm,
    },
    pio_programs::{
        clock_divider::calculate_pio_clock_divider,
        pwm::{PioPwm, PioPwmProgram},
    },
    pwm::{Config as ConfigPwm, Pwm, SetDutyCycle},
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
use heapless::String;
use portable_atomic::{AtomicU8, Ordering};
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::TerminalMode, prelude::*};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// #[allow(dead_code)]
mod audio;
mod controls;
mod io;
mod tasks;
mod utils;

use controls::{AnalogOutput, Debouncer};
use io::{
    flash::FLASH_SIZE,
    i2c::i2cpio::{I2CPIO, pio_task_sm0_irq0},
};
use tasks::{
    ChannelFrequencyType, ChannelInputsType, ChannelOscillatorType, I2C_BUS_FREQUENCY_1_MBIT,
    I2C_BUS_FREQUENCY_100_KBIT, I2C_BUS_FREQUENCY_400_KBIT, inputs_task, oscillator_irq1_handler,
    oscillator_irq2_handler, pio_task_sm3, setup_oscillator_clock_pio_task,
    setup_oscillator_pio_task, setup_pio_task_sm3,
};

const CARGO_PKG_NAME: &str = env!("CARGO_PKG_NAME");
const CARGO_PKG_VERSION: &str = env!("CARGO_PKG_VERSION");

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR_CORE0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_CORE1: StaticCell<Executor> = StaticCell::new();
// static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
// static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();
// static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

// static ANALOG_OUT_1: AtomicU8 = AtomicU8::new(0);
// static ANALOG_OUT_2: AtomicU8 = AtomicU8::new(0);
// static ANALOG_OUT_3: AtomicU8 = AtomicU8::new(0);
// static ANALOG_OUT_4: AtomicU8 = AtomicU8::new(0);
// static ANALOG_OUT_5: AtomicU8 = AtomicU8::new(0);
// static ANALOG_OUT_6: AtomicU8 = AtomicU8::new(0);
//
//static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, String<14>, 10> = Channel::new();

static FREQ_CHANNEL: ChannelFrequencyType = Channel::new();

bind_interrupts!(
    struct IrqsAdcPioDma {
        ADC_IRQ_FIFO => AdcInterruptHandler;
        PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
        PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
        DMA_IRQ_0 =>  dma::InterruptHandler<DMA_CH0>,
                      dma::InterruptHandler<DMA_CH1>,
                      dma::InterruptHandler<DMA_CH2>,
                      dma::InterruptHandler<DMA_CH10>,
                      dma::InterruptHandler<DMA_CH11>;
    }
);

bind_interrupts!(struct IrqsI2c0 {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

bind_interrupts!(struct IrqsI2c1 {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

// #[interrupt]
// unsafe fn SWI_IRQ_4() {
//     unsafe { EXECUTOR_HIGH.on_interrupt() }
// }

// #[interrupt]
// unsafe fn SWI_IRQ_5() {
//     unsafe { EXECUTOR_MEDIUM.on_interrupt() }
// }

// #[embassy_executor::task]
// async fn run_low() {
//     loop {
//         yield_now().await;
//     }
// }

//#[embassy_executor::main]
//async fn main(spawner: Spawner) {
#[entry]
fn main() -> ! {
    // -- set up for clock frequency of 200 MHz
    let config = Config::new(ClockConfig::system_freq(200_000_000).unwrap());
    // -- set up for default clock frequency of 125 MHz
    //let config = Config::default();

    // -- init pico and get peripherals
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

    // -- i2c bus 0 is used for display
    // let sda_0 = p.PIN_0;
    // let scl_0 = p.PIN_1;
    // info!("Setting up i2c bus 0");
    // let i2c0_config = {
    //     let mut i2c_config = I2cConfig::default();
    //     i2c_config.frequency = I2C_BUS_FREQUENCY_1_MBIT;
    //     i2c_config.scl_pullup = true;
    //     i2c_config.sda_pullup = true;
    //     i2c_config
    // };
    // let i2c0 = i2c::I2c::new_async(p.I2C0, scl_0, sda_0, IrqsI2c0, i2c0_config);

    // -- display config
    // let interface = I2CDisplayInterface::new(i2c0);
    // let mut display =
    //     Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0).into_terminal_mode();
    // display.init().unwrap();

    // let text_style = MonoTextStyleBuilder::new()
    //     .font(&FONT_6X10)
    //     .text_color(BinaryColor::On)
    //     .build();

    // -- ---------------------------------------------------------------------
    // -- I2C bus for peripherals
    // -- ---------------------------------------------------------------------

    // // -- i2c bus 1 is used for I2C peripherals
    // let sda_1 = p.PIN_2;
    // let scl_1 = p.PIN_3;
    // info!("Setting up i2c bus 1");
    // let i2c1_config = {
    //     let mut i2c_config = I2cConfig::default();
    //     //i2c_config.frequency = I2C_BUS_FREQUENCY_400_KBIT;
    //     i2c_config.frequency = I2C_BUS_FREQUENCY_1_MBIT;
    //     i2c_config.scl_pullup = true;
    //     i2c_config.sda_pullup = true;
    //     i2c_config
    // };
    // //let i2c1_config = I2cConfig::default();
    // let i2c1 = i2c::I2c::new_async(p.I2C1, scl_1, sda_1, IrqsI2c1, i2c1_config);

    // -- ---------------------------------------------------------------------
    // -- Flash
    // -- ---------------------------------------------------------------------

    let mut flash =
        Flash::<_, embassy_rp::flash::Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH11, IrqsAdcPioDma);
    let (board_id, _flash_uid) = io::flash::check_flash(&mut flash);
    let board_id = utils::u64_to_hexstring(board_id);
    info!("Board ID is {}", board_id);
    // Text::with_baseline(board_id.as_str(), Point::zero(), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();
    // display.flush().unwrap();

    // -- ---------------------------------------------------------------------
    // -- ADC / Temperature resources
    // -- ---------------------------------------------------------------------

    let adc = Adc::new(p.ADC, IrqsAdcPioDma, AdcConfig::default());
    //let mut dma_ch10 = dma::Channel::new(p.DMA_CH10, IrqsAdcPioDma);
    let adc_ch_ain = AdcChannel::new_pin(p.PIN_26, Pull::None);
    let adc_ch_kn1 = AdcChannel::new_pin(p.PIN_27, Pull::None);
    let adc_ch_kn2 = AdcChannel::new_pin(p.PIN_28, Pull::None);
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
    // -- Analog output based on PWM IO
    // -- ---------------------------------------------------------------------

    let desired_freq_hz = 100_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut c = ConfigPwm::default();
    c.top = period;
    c.divider = divider.into();

    // -- PIN_20 is analog out 2, PIN_21 is analog out 1
    let pwm_out_2_1 = Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_20, p.PIN_21, c.clone());
    let (pwm_out_1, pwm_out_2) = {
        let pwm_out_2_1 = pwm_out_2_1.split(); // A is out 2, B is out 1
        (pwm_out_2_1.1.unwrap(), pwm_out_2_1.0.unwrap()) // get rid of the options
    };
    // -- PIN_16 is analog out 3, PIN_17 is analog out 4
    let pwm_out_3_4 = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_16, p.PIN_17, c.clone());
    let (pwm_out_3, pwm_out_4) = {
        let pwm_out_3_4 = pwm_out_3_4.split(); // A is out 3, B is out 4
        (pwm_out_3_4.0.unwrap(), pwm_out_3_4.1.unwrap()) // get rid of the options
    };
    // -- PIN_18 is analog out 5, PIN_19 is analog out 6
    let pwm_out_5_6 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_18, p.PIN_19, c.clone());
    let (pwm_out_5, pwm_out_6) = {
        let pwm_out_5_6 = pwm_out_5_6.split(); // A is out 5, B is out 6
        (pwm_out_5_6.0.unwrap(), pwm_out_5_6.1.unwrap()) // get rid of the options
    };

    let analog_out_1 = AnalogOutput::new(pwm_out_1, 0);
    let analog_out_2 = AnalogOutput::new(pwm_out_2, 0);
    let analog_out_3 = AnalogOutput::new(pwm_out_3, 0);
    let analog_out_4 = AnalogOutput::new(pwm_out_4, 0);
    let analog_out_5 = AnalogOutput::new(pwm_out_5, 0);
    let analog_out_6 = AnalogOutput::new(pwm_out_6, 0);
    info!("Analog outputs ready");

    // -- ---------------------------------------------------------------------
    // -- PIO task(s) for display, digital input & oscillator
    // -- ---------------------------------------------------------------------

    let Pio {
        mut common,
        irq0,
        sm0,
        ..
    } = Pio::new(p.PIO0, IrqsAdcPioDma);

    let sda_pin = p.PIN_0;
    let scl_pin = p.PIN_1;
    let i2cpio = I2CPIO::new(
        &mut common,
        sda_pin,
        scl_pin,
        sm0,
        Some(dma::Channel::new(p.DMA_CH0, IrqsAdcPioDma)),
    );
    //let mut i2cpio = I2CPIO::new(&mut common, sda_pin, scl_pin, sm0, None);

    let Pio {
        mut common,
        irq1,
        irq2,
        irq3,
        mut sm1,
        mut sm2,
        mut sm3,
        ..
    } = Pio::new(p.PIO1, IrqsAdcPioDma);
    // -- PIO for oscillator clock
    setup_oscillator_clock_pio_task(&mut common, &mut sm1);
    info!("pio_task_sm1 for oscillator clock is setup");
    // -- PIO for MPC4725 DAC oscillator
    let sda_1 = p.PIN_2;
    let scl_1 = p.PIN_3;
    setup_oscillator_pio_task(&mut common, &mut sm2, sda_1, scl_1);
    let dma_ch2 = dma::Channel::new(p.DMA_CH2, IrqsAdcPioDma);
    info!("pio_task_sm2 for MPC4725 DAC oscillator is setup");
    // -- PIO for digital in (triggers)
    setup_pio_task_sm3(&mut common, &mut sm3, p.PIN_22);
    info!("pio_task_sm3 is setup");

    // -- ---------------------------------------------------------------------
    // -- Core 1 task
    // -- ---------------------------------------------------------------------

    // -- spawn PIO tasks on core 1
    info!("Spawning tasks running on core 1");
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR_CORE1.init(Executor::new());
            executor1.run(|spawner_c1| {
                spawner_c1.spawn(unwrap!(inputs_task(
                    adc,
                    adc_ch_ain,
                    adc_ch_kn1,
                    adc_ch_kn2,
                    //dma_ch10,
                    btn1,
                    btn2,
                    analog_out_1,
                    analog_out_2,
                    analog_out_3,
                    analog_out_4,
                    analog_out_5,
                    analog_out_6,
                    //display,
                    //i2c0,
                    i2cpio,
                    //&DISPLAY_CHANNEL,
                    &FREQ_CHANNEL,
                )));
                // -- display
                spawner_c1.spawn(unwrap!(pio_task_sm0_irq0(irq0)));
                // spawner_c1.spawn(unwrap!(display_task(i2cpio, &DISPLAY_CHANNEL)));
                info!("All running on core1");
            });
        },
    );

    // -- ---------------------------------------------------------------------
    // -- Medium-priority executor: SWI_IRQ_5, priority level 3
    // -- ---------------------------------------------------------------------
    // interrupt::SWI_IRQ_5.set_priority(Priority::P3);
    // let spawner_m = EXECUTOR_MEDIUM.start(interrupt::SWI_IRQ_5);
    // // -- digital input
    // spawner_m.spawn(unwrap!(pio_task_sm3(irq3, sm3)));
    // // -- oscillator
    // spawner_m.spawn(unwrap!(oscillator_irq2_handler(irq2)));
    // spawner_m.spawn(unwrap!(oscillator_irq1_handler(
    //     //i2c1,
    //     irq1,
    //     sm1,
    //     sm2,
    //     Some(dma_ch2)
    // )));

    // -- ---------------------------------------------------------------------
    // -- Core 0 task
    // -- ---------------------------------------------------------------------

    // // -- High-priority executor: SWI_IRQ_4, priority level 2
    // interrupt::SWI_IRQ_4.set_priority(Priority::P2);
    // let spawner_h = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_4);
    // // -- display
    // // spawner_h.spawn(unwrap!(pio_task_sm0_irq0(irq0)));
    // // spawner_h.spawn(unwrap!(display_task(i2cpio, &DISPLAY_CHANNEL)));
    // // // -- oscillator
    // // spawner_hi.spawn(unwrap!(oscillator_irq2_handler(irq2)));
    // // spawner_hi.spawn(unwrap!(oscillator_irq1_handler(
    // //     irq1,
    // //     sm1,
    // //     sm2,
    // //     Some(dma_ch2)
    // // )));
    // // // -- digital input
    // // spawner_hi.spawn(unwrap!(pio_task_sm3(irq3, sm3)));

    // // -- oscillator
    // spawner_m.spawn(unwrap!(oscillator_irq2_handler(irq2)));
    // spawner_m.spawn(unwrap!(oscillator_irq1_handler(
    //     irq1,
    //     sm1,
    //     sm2,
    //     Some(dma_ch2)
    // )));
    // // -- digital input
    // spawner_m.spawn(unwrap!(pio_task_sm3(irq3, sm3)));

    // // Low priority executor: runs in thread mode, using WFE/SEV
    // let executor = EXECUTOR_LOW.init(Executor::new());
    // executor.run(|spawner_l| {
    //     spawner_l.spawn(unwrap!(inputs_task(
    //         adc,
    //         adc_ch_ain,
    //         adc_ch_kn1,
    //         adc_ch_kn2,
    //         //dma_ch10,
    //         btn1,
    //         btn2,
    //         analog_out_1,
    //         analog_out_2,
    //         analog_out_3,
    //         analog_out_4,
    //         analog_out_5,
    //         analog_out_6,
    //         &DISPLAY_CHANNEL,
    //     )));
    //     // // -- display
    //     // spawner.spawn(unwrap!(pio_task_sm0_irq0(irq0)));
    //     // spawner.spawn(unwrap!(display_task(i2cpio, &DISPLAY_CHANNEL)));
    // });

    // -- spawn other tasks on core 0
    let executor0 = EXECUTOR_CORE0.init(Executor::new());
    executor0.run(|spawner_c0| {
        // -- digital input
        spawner_c0.spawn(unwrap!(pio_task_sm3(irq3, sm3)));
        // -- oscillator
        spawner_c0.spawn(unwrap!(oscillator_irq2_handler(irq2)));
        spawner_c0.spawn(unwrap!(oscillator_irq1_handler(
            //i2c1,
            irq1,
            sm1,
            sm2,
            Some(dma_ch2),
            &FREQ_CHANNEL,
        )));
    });
}
