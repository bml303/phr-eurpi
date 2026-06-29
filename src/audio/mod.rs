// -- mostly lifted from https://github.com/MacCracken/naad
use defmt::*;

pub mod noise;
pub mod oscillator;

pub const TAU: f32 = 6.28318530717958647692528676655900577_f32; // 6.28318548f32

#[derive(Debug, Format)]
pub enum AudioErrorType {
    InvalidFrequency,
    InvalidSampleRate,
}

// -- validate that a frequency is within the valid range for a given sample rate.
#[must_use]
pub(crate) fn validate_frequency(frequency: f32, sample_rate: f32) -> Option<AudioErrorType> {
    let nyquist = sample_rate / 2.0;
    if frequency <= 0.0 || frequency >= nyquist || !frequency.is_finite() {
        warn!("Invalid frequency: {}, {}", frequency, nyquist,);
        Some(AudioErrorType::InvalidFrequency)
    } else {
        None
    }
}

// -- validate that a sample rate is positive and finite.
#[must_use]
pub(crate) fn validate_sample_rate(sample_rate: f32) -> Option<AudioErrorType> {
    if sample_rate <= 0.0 || !sample_rate.is_finite() {
        warn!("Invalid sample rate: {}", sample_rate);
        Some(AudioErrorType::InvalidSampleRate)
    } else {
        None
    }
}
