#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

use core::cmp::min;
use cortex_m_rt::entry;
//use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::*;
use embassy_executor::Executor;
use embassy_rp::{
    Peri,
    adc::{
        Adc, Async as AdcAsync, Channel as AdcChannel, Config as AdcConfig,
        InterruptHandler as AdcInterruptHandler,
    },
    bind_interrupts, dma,
    flash::Flash,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, Config},
    multicore::{Stack, spawn_core1},
    peripherals::{DMA_CH0, DMA_CH1, DMA_CH11, I2C0, I2C1, PIO0, PIO1},
    pio::{
        Common, Config as PioConfig, Direction as PioPinDirection, InterruptHandler,
        InterruptHandler as PioInterruptHandler, Irq, Pio, PioPin, ShiftDirection, StateMachine,
        program::pio_asm,
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
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_graphics::{
    mono_font::{MonoTextStyle, MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable},
    text::{Baseline, Text},
};
use ssd1306::{I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode, prelude::*};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// #[allow(dead_code)]
mod io;
// mod task;
mod utils;

use io::flash::FLASH_SIZE;
use utils::Debouncer;

// use task::{
//     input_task, output_task, LogData, Ping, INIT_CHANNEL_CAPACITY, OUTPUT_CHANNEL_CAPACITY,
//     PING_CHANNEL_CAPACITY,
// };

const CARGO_PKG_NAME: &str = env!("CARGO_PKG_NAME");
const CARGO_PKG_VERSION: &str = env!("CARGO_PKG_VERSION");
const CHANNEL_OUT_1: usize = 5;
const CHANNEL_OUT_2: usize = 4;
const CHANNEL_OUT_6: usize = 3;
const CHANNEL_OUT_5: usize = 2;
const CHANNEL_OUT_4: usize = 1;
const CHANNEL_OUT_3: usize = 0;
const CHANNEL_INDEX_TO_NR: [usize; 6] = [3, 4, 5, 6, 2, 1];
const CLOCK_DIVIDER_48_KHZ: u32 = 48_000;
const CLOCK_DIVIDER_100_KHZ: u32 = 100_000;
const PWM_DUTY_CYCLE_MAX: u8 = 100;
const TICKER_EVERY_50_MICROS: u64 = 50;

// Bind the RTC interrupt to the handler
bind_interrupts!(struct IrqsRtc {
    RTC_IRQ => embassy_rp::rtc::InterruptHandler;
});

bind_interrupts!(struct IrqsPioSpiAndFlash {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH11>;
});

bind_interrupts!(struct IrqsPio1 {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
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

static CHANNEL_CORES: Channel<CriticalSectionRawMutex, (u16, u16, u16, Level, Level), 10> =
    Channel::new();

static CHANNEL_OUT: [Channel<CriticalSectionRawMutex, u8, 10>; 6] = [
    Channel::new(),
    Channel::new(),
    Channel::new(),
    Channel::new(),
    Channel::new(),
    Channel::new(),
];

fn setup_pio_task_sm0<'d>(
    pio: &mut Common<'d, PIO0>,
    sm: &mut StateMachine<'d, PIO0, 0>,
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
    cfg.clock_divider = calculate_pio_clock_divider(CLOCK_DIVIDER_48_KHZ);
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.direction = ShiftDirection::Right;
    sm.set_pin_dirs(PioPinDirection::In, &[&in_pin]);
    sm.set_config(&cfg);
}

fn setup_pio_task_sm1<'d>(
    pio: &mut Common<'d, PIO1>,
    sm: &mut StateMachine<'d, PIO1, 1>,
    pin_out1: Peri<'d, impl PioPin>,
    pin_out2: Peri<'d, impl PioPin>,
    pin_out3: Peri<'d, impl PioPin>,
    pin_out4: Peri<'d, impl PioPin>,
    pin_out5: Peri<'d, impl PioPin>,
    pin_out6: Peri<'d, impl PioPin>,
) {
    // -- This uses 10 steps / ticks to write the PWM values 5 times to the six pins.
    // -- A full duty cycle writes the values 100 times and do this 1000 times every second.
    // -- 1000 full duty cycles of 100 writes in the worst case means 100'000 ticks.
    // -- This PIO prog has to run at 100kHz.
    let prg = pio_asm!(
        //"set pindirs, 1",
        "set pins, 0; Drive pins low",
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
    // -- the sequence 3-4-5-6-1-2 is deliberate,
    // -- assuming GP16 = out3, GP17 = out4, GP18 = out5, GP19 = out6, GP20 = out1, GP21 = out2
    // -- and that the range of pins has to be contiguous (not sure if that is really necessary)
    cfg.set_out_pins(&[
        &pio_out1, &pio_out2, &pio_out3, &pio_out4, &pio_out5, &pio_out6,
    ]);
    // cfg.set_set_pins(&[
    //     &pio_out1, &pio_out2, &pio_out3, //&pio_out4, &pio_out5, &pio_out6,
    // ]);
    cfg.clock_divider = calculate_pio_clock_divider(CLOCK_DIVIDER_100_KHZ);
    cfg.shift_out.auto_fill = true;
    cfg.shift_out.direction = ShiftDirection::Right;
    cfg.shift_out.threshold = 30;
    sm.set_config(&cfg);
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

    // -- user keys
    // let key1 = Input::new(p.PIN_15, Pull::None);
    // let key2 = Input::new(p.PIN_17, Pull::None);

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
    // -- PIO task(s) for digital input
    // -- ---------------------------------------------------------------------

    let Pio {
        mut common,
        irq3,
        mut sm0,
        ..
    } = Pio::new(p.PIO0, IrqsPioSpiAndFlash);
    setup_pio_task_sm0(&mut common, &mut sm0, p.PIN_22);

    // -- ---------------------------------------------------------------------
    // -- PIO task(s) for digital output
    // -- ---------------------------------------------------------------------

    let Pio {
        mut common,
        mut sm1,
        ..
    } = Pio::new(p.PIO1, IrqsPio1);
    setup_pio_task_sm1(
        &mut common,
        &mut sm1,
        p.PIN_16,
        p.PIN_17,
        p.PIN_18,
        p.PIN_19,
        p.PIN_20,
        p.PIN_21,
    );

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
            executor1.run(|spawner| {
                spawner.spawn(unwrap!(core1_task(display, text_style)));
                spawner.spawn(unwrap!(pio_task_sm1(sm1)));
            });
        },
    );

    // -- ---------------------------------------------------------------------
    // -- Core 0 task
    // -- ---------------------------------------------------------------------

    // -- run output task on core 0
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(core0_task(adc, p26, p27, p28, btn1, btn2)));
        spawner.spawn(unwrap!(pio_task_sm0(irq3, sm0)));
    });
}

#[embassy_executor::task]
async fn pio_task_sm0(mut irq3: Irq<'static, PIO0, 3>, mut sm0: StateMachine<'static, PIO0, 0>) {
    sm0.set_enable(true);
    loop {
        irq3.wait().await;
        info!("IRQ trigged");
    }
}

fn calc_fifo_pwm_out_bits(out: &mut [u8; 6]) -> u32 {
    let mut pwm_out_bits: u32 = 0;
    // -- 5 times 6 bits => 30 bit plus two unused ones
    for _ in 0..5 {
        for i in 0..out.len() {
            let out_bit = if out[i] > 0 {
                out[i] -= 1;
                1
            } else {
                0
            };
            // -- out direction is shift-left so move the eralier bits
            // -- to higher positions and add the lower bits at the end
            pwm_out_bits = (pwm_out_bits << 1) + out_bit;
        }
    }
    pwm_out_bits
}

async fn update_pwm_out_values(out_pwm_duty_cycle: &mut [u8; 6]) {
    for i in 0..CHANNEL_OUT.len() {
        if !CHANNEL_OUT[i].is_empty() {
            let out_value = min(CHANNEL_OUT[i].receive().await, PWM_DUTY_CYCLE_MAX);
            info!(
                "Out value for channel {} changed from {} to {}",
                CHANNEL_INDEX_TO_NR[i], out_pwm_duty_cycle[i], out_value
            );
            out_pwm_duty_cycle[i] = out_value;
        };
    }
}

#[embassy_executor::task]
async fn pio_task_sm1(mut sm1: StateMachine<'static, PIO1, 1>) {
    // -- PWM duty cycle start values
    let mut out_pwm_duty_cycle: [u8; 6] = [0; 6];
    // -- PWM duty cycle count down values
    let mut out_pwm_count_down: [u8; 6] = [0; 6];
    // -- the duty cycle has to be in the range of 0 to 100
    let mut pwm_duty_cycle_count = PWM_DUTY_CYCLE_MAX;
    // -- The full duty cycle has to be completed 1000 times per second = 100kHz.
    // -- The PWM prog uses 1 tick to write 6 out bits so it has to run at 100kHz.
    // -- This task pushes 5 * 6 bits + 2 unused ones (32 bit) every cycle.
    // -- So it has to run at 1/5th of the speed of the PWM prog = 20kHz => every loop 50 microsecs
    let mut ticker_20000_hz = Ticker::every(Duration::from_micros(TICKER_EVERY_50_MICROS));
    // -- enable state machine and start cycle loop (one cycle = 1 second)
    sm1.set_enable(true);
    //let mut pwm_out_bits_last = u32::MAX;
    loop {
        // -- calculate PWM bits
        let _pwm_out_bits = calc_fifo_pwm_out_bits(&mut out_pwm_count_down);
        //if pwm_out_bits != pwm_out_bits_last {
        // -- push PWM bits into the TX FIFO if there is a change
        //pwm_out_bits_last = pwm_out_bits;
        //sm1.tx().push(pwm_out_bits);
        let pwm_out_bits = 0xffff;
        sm1.tx().push(pwm_out_bits);
        //}
        // -- update PWM values for next cycle
        update_pwm_out_values(&mut out_pwm_duty_cycle).await;
        // -- check if cycle is finished, restart with new values if so
        pwm_duty_cycle_count -= 1;
        if pwm_duty_cycle_count == 0 {
            pwm_duty_cycle_count = PWM_DUTY_CYCLE_MAX;
            out_pwm_count_down = out_pwm_duty_cycle;
            //pwm_out_bits_last = u32::MAX;
        }
        // -- wait for the next tick
        ticker_20000_hz.next().await;
    }
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
        I2CInterface<i2c::I2c<'static, I2C0, i2c::Async>>,
        DisplaySize128x32,
        BufferedGraphicsMode<DisplaySize128x32>,
    >,
    text_style: MonoTextStyle<'static, BinaryColor>,
) {
    info!("Hello from core 1");
    CHANNEL_OUT[CHANNEL_OUT_1].send(100).await;
    CHANNEL_OUT[CHANNEL_OUT_2].send(100).await;
    CHANNEL_OUT[CHANNEL_OUT_3].send(100).await;
    CHANNEL_OUT[CHANNEL_OUT_4].send(100).await;
    CHANNEL_OUT[CHANNEL_OUT_5].send(100).await;
    CHANNEL_OUT[CHANNEL_OUT_6].send(100).await;
    let p1 = Point::new(0, 16);
    let p2 = Point::new(128, 32);
    let style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .build();
    loop {
        match CHANNEL_CORES.receive().await {
            (ain, kn1, kn2, btn1_lvl, btn2_lvl) => {
                // // -- normalize kn1 and kn2 to percent values 0 - 100
                // let out1_value: u8 = (kn1 as u64 * 100 / 4096) as u8;
                // let out2_value: u8 = (kn2 as u64 * 100 / 4096) as u8;
                // // -- update out1 and out2
                // CHANNEL_OUT[0].send(out1_value).await;
                // CHANNEL_OUT[1].send(out2_value).await;
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
        }
    }
}
