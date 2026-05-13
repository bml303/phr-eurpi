#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::cmp::min;
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
mod utils;

use audio::oscillator::sine_oscillator::SineOscillator;
use io::flash::FLASH_SIZE;
use io::i2c::mpc4725::{Mpc4725, Mpc4725DeviceAddress};
use utils::Debouncer;

// use task::{
//     input_task, output_task, LogData, Ping, INIT_CHANNEL_CAPACITY, OUTPUT_CHANNEL_CAPACITY,
//     PING_CHANNEL_CAPACITY,
// };

const CARGO_PKG_NAME: &str = env!("CARGO_PKG_NAME");
const CARGO_PKG_VERSION: &str = env!("CARGO_PKG_VERSION");
const CHANNEL_OUT_1: u8 = 0;
const CHANNEL_OUT_2: u8 = 1;
const CHANNEL_OUT_3: u8 = 5;
const CHANNEL_OUT_4: u8 = 4;
const CHANNEL_OUT_5: u8 = 3;
const CHANNEL_OUT_6: u8 = 2;
const CHANNEL_INDEX_TO_NR: [usize; 6] = [1, 2, 6, 5, 4, 3];
const I2C1_BUS_FREQUENCY_100_KBIT: u32 = 100_000;
const I2C1_BUS_FREQUENCY_400_KBIT: u32 = 400_000;
const I2C1_BUS_FREQUENCY_1_MBIT: u32 = 1_000_000;
const SM0_CLOCK_DIVIDER_48_KHZ: u32 = 48_000;
const SM1_CLOCK_DIVIDER_1_MHZ: u32 = 1_000_000;
const TICKER_EVERY_50_MICROS: u64 = 50; // -- 200'000 Hz = 200 kHz
const TICKER_EVERY_500_MICROS: u64 = 500; // -- 20'000 Hz = 20 kHz
const PWM_VALUE_MIN: u8 = 0;
const PWM_VALUE_MAX: u8 = 250;
const PWM_TX_FIFO_VALUES: u8 = 5;
const PWM_VALUE_CYCLE_MAX: u8 = PWM_VALUE_MAX / PWM_TX_FIFO_VALUES;

const SAMPLE_RATE_48KHZ: f32 = 48000.0;
const SAMPLE_RATE_44KHZ: f32 = 44000.0;
const SAMPLE_RATE_25KHZ: f32 = 25000.0;
const SAMPLE_RATE_10KHZ: f32 = 10000.0;
const SAMPLE_RATE_5KHZ: f32 = 5000.0;
//const SAMPLE_BLOCK_SIZE: usize = 24;
const SAMPLE_BLOCK_SIZE: usize = 48;

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

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

static CHANNEL_CORES: Channel<CriticalSectionRawMutex, (u16, u16, u16, Level, Level), 10> =
    Channel::new();

static ANALOG_OUT_1: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_2: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_3: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_4: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_5: AtomicU8 = AtomicU8::new(0);
static ANALOG_OUT_6: AtomicU8 = AtomicU8::new(0);

static CHANNEL_OSCILLATOR: Channel<
    CriticalSectionRawMutex,
    [u16; SAMPLE_BLOCK_SIZE],
    SAMPLE_BLOCK_SIZE,
> = Channel::new();

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

    let rtc = Rtc::new(p.RTC, IrqsRtc);
    let watchdog = Watchdog::new(p.WATCHDOG);

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

    // // -- i2c bus 1 is used for I2C peripherals
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

    // -- ---------------------------------------------------------------------
    // -- Buttons
    // -- ---------------------------------------------------------------------
    //
    let btn1 = Debouncer::new(Input::new(p.PIN_4, Pull::Up), Duration::from_millis(20));
    let btn2 = Debouncer::new(Input::new(p.PIN_5, Pull::Up), Duration::from_millis(20));

    // -- ---------------------------------------------------------------------
    // -- PIO task(s) for digital input & analog output
    // -- ---------------------------------------------------------------------

    let Pio {
        mut common,
        irq3,
        mut sm0,
        mut sm1,
        mut sm2,
        ..
    } = Pio::new(p.PIO0, IrqsPioSpiAndFlash);
    setup_pio_task_sm0(&mut common, &mut sm0, p.PIN_22);
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
    setup_pio_task_sm2(&mut common, &mut sm2, p.PIN_2, p.PIN_3);
    let mut dma_out_ref = dma::Channel::new(p.DMA_CH0, IrqsPioSpiAndFlash);
    let mut dma_in_ref = dma::Channel::new(p.DMA_CH1, IrqsPioSpiAndFlash);

    // -- ---------------------------------------------------------------------
    // -- Core 1 task
    // -- ---------------------------------------------------------------------

    // -- spawn i2c sensoring task on core 1
    info!("Spawning Task running on core 1");
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(unwrap!(core1_task(display, text_style)));
            });
        },
    );

    // -- Medium-priority executor: SWI_IRQ_2, priority level 2
    // interrupt::SWI_IRQ_2.set_priority(Priority::P2);
    // let spawner = EXECUTOR_MED.start(interrupt::SWI_IRQ_2);
    // spawner.spawn(unwrap!(osc_task(i2c1)));

    // -- High-priority executor: SWI_IRQ_1, priority level 1
    interrupt::SWI_IRQ_1.set_priority(Priority::P1);
    let spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    spawner.spawn(unwrap!(pio_task_sm1(sm1)));
    //spawner.spawn(unwrap!(osc_task(i2c1)));

    // -- ---------------------------------------------------------------------
    // -- Core 0 task
    // -- ---------------------------------------------------------------------

    // -- run output task on core 0
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(core0_task(adc, p26, p27, p28, btn1, btn2)));
        //spawner.spawn(unwrap!(pio_task_sm0(irq3, sm0)));
    });
}

#[interrupt]
unsafe fn SWI_IRQ_0() {
    unsafe { EXECUTOR_MED.on_interrupt() }
}

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

// -- ---------------------------------------------------------------------
// -- SM0 - Digital Input
// -- ---------------------------------------------------------------------

fn setup_pio_task_sm0<'d>(
    pio: &mut Common<'d, PIO0>,
    sm0: &mut StateMachine<'d, PIO0, 0>,
    pin: Peri<'d, impl PioPin>,
) {
    // -- read digital input triggers
    let prg = pio_asm!(
        ".origin 0",
        ".wrap_target",
        "wait 0 pin 0",
        "wait 1 pin 0",
        "irq 3",
        ".wrap",
    );
    // -- setup sm0
    let mut cfg = PioConfig::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);
    let in_pin = pio.make_pio_pin(pin);
    cfg.set_in_pins(&[&in_pin]);
    cfg.clock_divider = calculate_pio_clock_divider(SM0_CLOCK_DIVIDER_48_KHZ);
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.direction = ShiftDirection::Right;
    sm0.set_pin_dirs(PioPinDirection::In, &[&in_pin]);
    sm0.set_config(&cfg);
}

#[embassy_executor::task]
async fn pio_task_sm0(mut irq3: Irq<'static, PIO0, 3>, mut sm0: StateMachine<'static, PIO0, 0>) {
    sm0.set_enable(true);
    loop {
        irq3.wait().await;
        info!("IRQ trigged");
    }
}

// -- ---------------------------------------------------------------------
// -- SM1 - Analog Output
// -- ---------------------------------------------------------------------

fn setup_pio_task_sm1<'d>(
    pio: &mut Common<'d, PIO0>,
    sm1: &mut StateMachine<'d, PIO0, 1>,
    pin_out1: Peri<'d, impl PioPin>,
    pin_out2: Peri<'d, impl PioPin>,
    pin_out3: Peri<'d, impl PioPin>,
    pin_out4: Peri<'d, impl PioPin>,
    pin_out5: Peri<'d, impl PioPin>,
    pin_out6: Peri<'d, impl PioPin>,
) {
    let prg = pio_asm!(
        "set pins, 0"
        "pull block"
        ".wrap_target",
        "out pins, 6",
        "out pins, 6",
        "out pins, 6",
        "out pins, 6",
        "out pins, 6",
        ".wrap",
    );
    // -- setup sm1
    let mut cfg = PioConfig::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);
    let pio_out1 = pio.make_pio_pin(pin_out1);
    let pio_out2 = pio.make_pio_pin(pin_out2);
    let pio_out3 = pio.make_pio_pin(pin_out3);
    let pio_out4 = pio.make_pio_pin(pin_out4);
    let pio_out5 = pio.make_pio_pin(pin_out5);
    let pio_out6 = pio.make_pio_pin(pin_out6);
    cfg.set_out_pins(&[
        &pio_out1, &pio_out2, &pio_out3, &pio_out4, &pio_out5, &pio_out6,
    ]);
    cfg.clock_divider = calculate_pio_clock_divider(SM1_CLOCK_DIVIDER_1_MHZ);
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.out_sticky = false;
    cfg.shift_out.auto_fill = true;
    cfg.shift_out.direction = ShiftDirection::Left;
    cfg.shift_out.threshold = 30;
    sm1.set_pin_dirs(
        PioPinDirection::Out,
        &[
            &pio_out1, &pio_out2, &pio_out3, &pio_out4, &pio_out5, &pio_out6,
        ],
    );
    sm1.set_config(&cfg);
}

#[inline(always)]
fn calc_fifo_pwm_out_bits(out: &mut [u8; 6]) -> u32 {
    let mut pwm_out_bits: u32 = 0;
    // -- 5 times 6 bits => 30 bit plus two unused ones
    for _ in 0..PWM_TX_FIFO_VALUES {
        for i in 0..out.len() {
            let out_bit = if out[i] > 0 {
                out[i] -= 1;
                1
            } else {
                0
            };
            // -- out direction is shift-left and the order of the
            // -- six bits is out1-out2-out6-out5-out4-out3
            pwm_out_bits = (pwm_out_bits << 1) | out_bit;
        }
    }
    pwm_out_bits << 2
}

#[inline(always)]
fn update_pwm_out_values(out_pwm_count_down: &mut [u8; 6]) {
    out_pwm_count_down[0] = min(ANALOG_OUT_1.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[1] = min(ANALOG_OUT_2.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[5] = min(ANALOG_OUT_3.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[4] = min(ANALOG_OUT_4.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[2] = min(ANALOG_OUT_5.load(Ordering::Relaxed), PWM_VALUE_MAX);
    out_pwm_count_down[3] = min(ANALOG_OUT_6.load(Ordering::Relaxed), PWM_VALUE_MAX);
}

#[embassy_executor::task]
async fn pio_task_sm1(mut sm1: StateMachine<'static, PIO0, 1>) {
    // -- PWM duty cycle count down values
    let mut out_pwm_count_down: [u8; 6] = [0; 6];
    update_pwm_out_values(&mut out_pwm_count_down);
    // -- the duty cycle has to be in the range of 0 to 100
    let mut pwm_duty_cycle_count = 0;
    // -- enable state machine and start loop
    sm1.set_enable(true);
    sm1.tx().push(0);
    let mut pwm_out_bits_last = u32::MAX;
    loop {
        let start = Instant::now();
        // -- out 1 to 100%
        //let pwm_out_bits = 0b10000010000010000010000010000000;
        // -- out 2 to 100%
        //let pwm_out_bits = 0b01000001000001000001000001000000;
        // -- out 6 to 100%
        //let pwm_out_bits = 0b00100000100000100000100000100000;
        // -- out 5 to 100%
        //let pwm_out_bits = 0b00010000010000010000010000010000;
        // -- out 4 to 100%
        //let pwm_out_bits = 0b00001000001000001000001000001000;
        // -- out 3 to 100%
        //let pwm_out_bits = 0b00000100000100000100000100000100;
        // sm1.tx().push(pwm_out_bits);
        // -- calculate PWM bits
        let pwm_out_bits = calc_fifo_pwm_out_bits(&mut out_pwm_count_down);
        // -- push PWM bits into the TX FIFO if there is a change
        if pwm_out_bits != pwm_out_bits_last {
            pwm_out_bits_last = pwm_out_bits;
            if !sm1.tx().full() {
                sm1.tx().push(pwm_out_bits);
            }
        }
        // -- check if cycle is finished, restart with new values if so
        pwm_duty_cycle_count += 1;
        if pwm_duty_cycle_count == PWM_VALUE_CYCLE_MAX {
            pwm_duty_cycle_count = 0;
            // -- update PWM values for next cycle
            update_pwm_out_values(&mut out_pwm_count_down);
        }
        let elapsed_microsecs = start.elapsed().as_micros();
        if elapsed_microsecs > TICKER_EVERY_50_MICROS {
            warn!(
                "SM1 loop exceeded cycle time: {} micro seconds",
                elapsed_microsecs
            )
        } else if elapsed_microsecs < TICKER_EVERY_50_MICROS {
            // -- wait for the next tick
            Timer::after(Duration::from_micros(
                TICKER_EVERY_50_MICROS - elapsed_microsecs,
            ))
            .await;
        }
    }
}

// -- ---------------------------------------------------------------------
// -- SM2 - MPC4725 i2c DAC output
// -- ---------------------------------------------------------------------

fn setup_pio_task_sm2<'d>(
    pio: &mut Common<'d, PIO0>,
    sm2: &mut StateMachine<'d, PIO0, 2>,
    sda_pin: Peri<'d, impl PioPin>,
    scl_pin: Peri<'d, impl PioPin>,
) {
    // -- reset MPC4725, then continouously write data to DAC
    // -- side 0 is SCL
    let prg = pio_asm!(
        // r"
        // .origin 0
        // .side_set 2 opt                     ; -- SDA, SCL are side set
        // do_nack:
        //     irq wait 4      side 0b11       ; -- stop, SDA & SCL high, ask for help
        //     jmp entry_point side 0b11       ; -- restart processing
        // do_0:
        //     jmp x-- nojmp_0 side 0b00       ; -- decrement x with pseudo-jump, set zero value on SDA
        // nojmp_0:
        //     jmp !x last_0   side 0b01 [1]   ; -- confirm value with SCL pulse
        //     out y, 1        side 0b00       ; -- get next bit from OSR, keep SDA low, pull SCL down
        //     jmp y-- into_1  side 0b00       ; -- keep SDA low, pull SCL down
        // into_0:
        //     jmp do_0        side 0b00       ; -- set zero value on SDA
        // last_0:
        //     set pindirs, 0  side 0b00 [1]   ; -- SDA input, SDA low, SCL down
        //     jmp pin do_nack side 0b01 [1]   ; -- confirm SDA value with SCL pulse
        //     set pindirs, 1  side 0b00       ; -- SDA output, confirm SDA value with SCL pulse
        //     jmp entry_point side 0b00       ; -- byte done
        // do_1:
        //     jmp x-- nojmp_1 side 0b10       ; -- decrement x with pseudo-jump, set one value on SDA
        // nojmp_1:
        //     jmp !x last_1   side 0b11 [1]   ; -- confirm value with SCL pulse
        //     out y, 1        side 0b10       ; -- get next bit from OSR, keep SDA high, pull SCL down
        //     jmp !y into_0   side 0b10       ; -- keep SDA high, pull SCL down
        // into_1:
        //     jmp do_1        side 0b10       ; -- set one value on SDA
        // last_1:
        //     set pindirs, 0  side 0b10 [1]   ; -- SDA input, SDA high, SCL down
        //     jmp pin do_nack side 0b11 [1]   ; -- confirm SDA value with SCL pulse
        //     set pindirs, 1  side 0b10 [1]   ; -- SDA output, confirm SDA value with SCL pulse

        // do_byte:
        //     out y, 1                        ; -- read signal bit from OSR
        //     jmp !y do_stop                  ; -- non-zero indicating STOP
        //     out y, 1                        ; -- read next data bit from OSR
        //     set x, 7                        ; -- loop 8 times
        //     jmp y-- do_1                    ; -- jump if y > 0 prior to decrement
        //     jmp do_0
        // do_stop:
        //     out null, 32    side 0b01 [1]   ; -- reminder of OSR is invalid, SDA low, SCL high
        //     nop             side 0b11 [1]   ; -- STOP condition

        // public entry_point:
        // .wrap_target
        //     out y, 1        side 0b11       ; -- read next bit from OSR, SDA high, SCL high (idle)
        //     set x, 7        side 0b01       ; -- loop 8 times, START condition SDA low, SCL high
        //     jmp y-- into_1  side 0b00 [1]   ; -- jump if y > 0 prior to decrement
        //     jmp do_0        side 0b00       ; --
        //     nop
        // .wrap
        // ",
        r"
        .origin 0
        .side_set 1 opt                     ; -- SCL is side set
        do_0s:
            set x, 4        side 0          ; -- 01 - set number of zeros, SCL low
        loop_0s:
            set pins 0      side 0          ; -- 02 - set SDA low, SCL low
            jmp x-- end_0s  side 1 [3]      ; -- 03 - confirm SDA value with SCL pulse
            jmp loop_0s     side 0 [2]      ; -- 04 - jump if x > 0 prior to decrement
        end_0s:
            set x, 4        side 0 [2]      ; -- 05 - set number of bits, SCL low
        do_bits:
            out pins, 1     side 0          ; -- 06 - read next bit from OSR, SCL low
            nop             side 1 [3]      ; -- 07 - confirm SDA value with SCL pulse
            jmp x-- do_bits side 0 [2]      ; -- 08 - jump if x > 0 prior to decrement
            set pindirs, 0  side 0          ; -- 09 - SDA input
            jmp pin do_nack side 1 [2]      ; -- 10 - confirm SDA value with SCL pulse
            mov pc, y       side 1          ; -- 11 - return to caller
        do_nack:
            irq wait 4      side 1          ; -- 12 - stop, SCL high, ask for help
        public entry_point:
            set pindirs, 1  side 1          ; -- 13 - SDA output
            set pins, 1     side 1 [1]      ; -- 14 - SDA high, SCL high (idle)
        .wrap_target
            set pins, 0     side 1 [3]      ; -- 15 - START condition SDA to low, SCL high
            set x, 8        side 0          ; -- 16 - set number of bits, SCL low
            set y, 19       side 0          ; -- 17 - set return address 18, SCL low
            jmp do_bits     side 0          ; -- 18 - write bits, SCL low
            set y, 21       side 0          ; -- 19 - set return address 21, SCL low
            jmp do_0s       side 0          ; -- 20 - write 4 zeros, and four bits, SCL low
            set y, 23       side 0          ; -- 21 - set return address 23, SCL low
            jmp do_bits     side 0          ; -- 22 - write bits, SCL low
            set y, 25       side 0          ; -- 23 - set return address 25, SCL low
            jmp do_0s       side 0          ; -- 24 - write 4 zeros, and four bits, SCL low
            set y, 27       side 0          ; -- 25 - set return address 27, SCL low
            jmp do_bits     side 0          ; -- 26 - write bits, SCL low
            set pins, 0     side 0 [1]      ; -- 27 - SDA low, SCL low
            set pins, 0     side 1 [1]      ; -- 28 - SDA low, SCL high
            set pins, 1     side 1 [1]      ; -- 29 - STOP condition SDA to high, SCL high
        .wrap
        ",
    );
    // -- setup sm0
    let sda_pin = pio.make_pio_pin(sda_pin);
    let scl_pin = pio.make_pio_pin(scl_pin);
    let mut cfg = PioConfig::default();
    cfg.use_program(&pio.load_program(&prg.program), &[&scl_pin]);
    cfg.set_in_pins(&[&sda_pin]);
    cfg.set_set_pins(&[&sda_pin]);
    cfg.set_out_pins(&[&sda_pin]);
    cfg.clock_divider = calculate_pio_clock_divider(SM0_CLOCK_DIVIDER_48_KHZ);
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.direction = ShiftDirection::Left;
    sm2.set_config(&cfg);
}

#[embassy_executor::task]
async fn osc_task(mut i2c1: I2c<'static, I2C1, I2cAsync>) {
    info!("Oscillator task started");
    // -- setup MPC47
    let mut mpc4725 = match Mpc4725::new(&mut i2c1, Mpc4725DeviceAddress::Default).await {
        Ok(mpc4725) => mpc4725,
        Err(err) => {
            error!("Failed to initialize MPC4725: {}", err);
            return;
        }
    };
    // -- initialize sample playing loop
    let sample_rate = SAMPLE_RATE_48KHZ;
    let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    debug!("Every nanos is {}", every_nanos);
    let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
    loop {
        let samples = CHANNEL_OSCILLATOR.receive().await;
        //debug!("Writing to DAC...");
        for sample in &samples {
            // -- normalize the sample and send it to the DAC
            mpc4725.write_dac_value(&mut i2c1, *sample).await.unwrap();
            ticker.next().await;
        }
    }
    // // -- setup oscillator
    // let sample_rate = SAMPLE_RATE_48KHZ;
    // let every_nanos = (1_000_000_000f32 / sample_rate) as u64;
    // debug!("Every nanos is {}", every_nanos);
    // //let frequency = 110.0;
    // //let frequency = 440.0;
    // let frequency = 880.0;
    // //let frequency = 5000.0;
    // let f = frequency / sample_rate;
    // let mut out = [0.0; 1];
    // let mut osc = SineOscillator::new();
    // osc.init();
    // osc.render(f, &mut out);
    // // let mut start = Instant::now();
    // let mut ticker = Ticker::every(Duration::from_nanos(every_nanos));
    // loop {
    //     // -- this kinda works
    //     osc.render(f, &mut out);
    //     let sample = out[0];
    //     let value = (((sample + 1f32) * 4096f32) / 2f32) as u16;
    //     // //debug!("sample is {}, value is {}", sample, value);
    //     mpc4725.write_dac_value(&mut i2c1, value).await.unwrap();
    //     ticker.next().await;
    // }
}

#[embassy_executor::task]
async fn core0_task(
    mut adc: Adc<'static, AdcAsync>,
    mut p26: AdcChannel<'static>,
    mut p27: AdcChannel<'static>,
    mut p28: AdcChannel<'static>,
    mut btn1: Debouncer<'static>,
    mut btn2: Debouncer<'static>,
) {
    info!("Hello from core 0");
    loop {
        // -- do this every 100 milliseconds
        let ain = adc.read(&mut p26).await.unwrap();
        let kn1 = adc.read(&mut p27).await.unwrap();
        let kn2 = adc.read(&mut p28).await.unwrap();
        let btn1_lvl = btn1.level().await;
        let btn2_lvl = btn2.level().await;
        CHANNEL_CORES
            .send((ain, kn1, kn2, btn1_lvl, btn2_lvl))
            .await;
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn core1_task(
    mut display: Ssd1306<
        I2CInterface<I2c<'static, I2C0, I2cAsync>>,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >,
    text_style: MonoTextStyle<'static, BinaryColor>,
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
    ANALOG_OUT_1.store(0, Ordering::Relaxed);
    ANALOG_OUT_2.store(0, Ordering::Relaxed);
    ANALOG_OUT_3.store(PWM_VALUE_MAX, Ordering::Relaxed);
    ANALOG_OUT_4.store(PWM_VALUE_MAX / 2, Ordering::Relaxed);
    ANALOG_OUT_5.store(PWM_VALUE_MAX, Ordering::Relaxed);
    ANALOG_OUT_6.store(0, Ordering::Relaxed);

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
        while CHANNEL_OSCILLATOR.free_capacity() > 0 {
            //debug!("rendering...");
            osc.render(f, &mut out);
            for i in 0..out.len() {
                // -- normalize the sample and send it to the DAC
                samples[i] = ((out[i] + 1f32) * 4096f32 / 2f32) as u16;
            }
            CHANNEL_OSCILLATOR.send(samples).await;
            Timer::after_micros(2).await;
        }
        if let Ok((ain, kn1, kn2, btn1_lvl, btn2_lvl)) = CHANNEL_CORES.try_receive() {
            // let (ain, kn1, kn2, btn1_lvl, btn2_lvl) = CHANNEL_CORES.receive().await;
            // {
            // -- normalize kn1 and kn2 to percent values 0 - 100
            let out1_value: u8 = PWM_VALUE_MAX - (kn1 as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
            let out2_value: u8 = PWM_VALUE_MAX - (kn2 as u64 * PWM_VALUE_MAX as u64 / 4096) as u8;
            // -- update out1 and out2
            ANALOG_OUT_1.store(out1_value, Ordering::Relaxed);
            ANALOG_OUT_2.store(out2_value, Ordering::Relaxed);
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
