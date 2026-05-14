use embassy_rp::gpio::Level;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

mod analogout;
mod core0;
mod core1;
mod digitalin;
mod oscillator;

pub use analogout::{pio_task_sm1, setup_pio_task_sm1};
pub use core0::core0_task;
pub use core1::core1_task;
pub use digitalin::{pio_task_sm0, setup_pio_task_sm0};
pub use oscillator::{osc_task_dac, osc_task_generate, setup_pio_task_sm2};

// pub const CHANNEL_OUT_1: u8 = 0;
// pub const CHANNEL_OUT_2: u8 = 1;
// pub const CHANNEL_OUT_3: u8 = 5;
// pub const CHANNEL_OUT_4: u8 = 4;
// pub const CHANNEL_OUT_5: u8 = 3;
// pub const CHANNEL_OUT_6: u8 = 2;
// pub const CHANNEL_INDEX_TO_NR: [usize; 6] = [1, 2, 6, 5, 4, 3];
pub const I2C1_BUS_FREQUENCY_100_KBIT: u32 = 100_000;
pub const I2C1_BUS_FREQUENCY_400_KBIT: u32 = 400_000;
pub const I2C1_BUS_FREQUENCY_1_MBIT: u32 = 1_000_000;

pub const PWM_TX_FIFO_VALUES: u8 = 5;
pub const PWM_VALUE_MAX: u8 = 250;
pub const PWM_VALUE_MIN: u8 = 0;
pub const PWM_VALUE_CYCLE_MAX: u8 = PWM_VALUE_MAX / PWM_TX_FIFO_VALUES;

pub const SAMPLE_BLOCK_SIZE: usize = 48;
//const SAMPLE_BLOCK_SIZE: usize = 24;
// const SAMPLE_RATE_44KHZ: f32 = 44000.0;
// const SAMPLE_RATE_25KHZ: f32 = 25000.0;
// const SAMPLE_RATE_10KHZ: f32 = 10000.0;
// const SAMPLE_RATE_5KHZ: f32 = 5000.0;
pub const SAMPLE_RATE_48KHZ: f32 = 48000.0;

pub const SM0_CLOCK_DIVIDER_48_KHZ: u32 = 48_000;
pub const SM1_CLOCK_DIVIDER_1_MHZ: u32 = 1_000_000;
pub const SM2_CLOCK_DIVIDER_48_KHZ: u32 = 48_000;

pub const TICKER_EVERY_50_MICROS: u64 = 50; // -- 200'000 Hz = 200 kHz
pub const TICKER_EVERY_500_MICROS: u64 = 500; // -- 20'000 Hz = 20 kHz

pub type ChannelOscillatorType =
    Channel<CriticalSectionRawMutex, [u16; SAMPLE_BLOCK_SIZE], SAMPLE_BLOCK_SIZE>;

pub type ChannelInputsType = Channel<CriticalSectionRawMutex, (u16, u16, u16, Level, Level), 10>;
