//! Noise generators: white, pink (Voss-McCartney), and brown noise.

use serde::{Deserialize, Serialize};

/// Type of noise to generate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum NoiseType {
    /// Uniform random noise, flat spectrum.
    White,
    /// Pink noise (-3 dB/octave), Voss-McCartney algorithm.
    Pink,
    /// Brown noise (-6 dB/octave), integrated white noise.
    Brown,
}

/// Simple xorshift32 PRNG for deterministic noise.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct Xorshift32 {
    state: u32,
}

impl Xorshift32 {
    fn new(seed: u32) -> Self {
        // Avoid zero state
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    /// Generate a random f32 in [-1.0, 1.0).
    #[inline]
    fn next_f32(&mut self) -> f32 {
        xorshift32_signed_f32(&mut self.state)
    }
}

/// Number of octaves for Voss-McCartney pink noise.
const PINK_OCTAVES: usize = 16;

/// Noise generator with state for different noise types.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NoiseGenerator {
    /// The type of noise being generated.
    noise_type: NoiseType,
    /// PRNG state.
    rng: Xorshift32,
    /// Voss-McCartney octave values for pink noise.
    pink_octaves: [f32; PINK_OCTAVES],
    /// Counter for Voss-McCartney pink noise.
    pink_counter: u32,
    /// Running sum for pink noise.
    pink_running_sum: f32,
    /// Previous value for brown noise.
    brown_prev: f32,
}

impl NoiseGenerator {
    /// Create a new noise generator.
    ///
    /// # Arguments
    ///
    /// * `noise_type` - Type of noise to generate
    /// * `seed` - Random seed for deterministic output
    #[must_use]
    pub fn new(noise_type: NoiseType, seed: u32) -> Self {
        let mut rng = Xorshift32::new(seed);

        // Initialize pink noise octave values
        let mut pink_octaves = [0.0f32; PINK_OCTAVES];
        let mut pink_running_sum = 0.0f32;
        if noise_type == NoiseType::Pink {
            for octave in &mut pink_octaves {
                let val = rng.next_f32();
                *octave = val;
                pink_running_sum += val;
            }
        }

        Self {
            noise_type,
            rng,
            pink_octaves,
            pink_counter: 0,
            pink_running_sum,
            brown_prev: 0.0,
        }
    }

    /// Returns the type of noise being generated.
    #[inline]
    #[must_use]
    pub fn noise_type(&self) -> NoiseType {
        self.noise_type
    }

    /// Generate the next noise sample.
    #[inline]
    #[must_use]
    pub fn next_sample(&mut self) -> f32 {
        match self.noise_type {
            NoiseType::White => self.white_noise(),
            NoiseType::Pink => self.pink_noise(),
            NoiseType::Brown => self.brown_noise(),
        }
    }

    /// Generate white noise sample.
    #[inline]
    fn white_noise(&mut self) -> f32 {
        self.rng.next_f32()
    }

    /// Generate pink noise using Voss-McCartney algorithm.
    ///
    /// Uses a tree structure where each octave updates at half the rate
    /// of the previous, producing approximately -3 dB/octave rolloff.
    #[inline]
    fn pink_noise(&mut self) -> f32 {
        self.pink_counter = self.pink_counter.wrapping_add(1);

        // Determine which octaves to update based on trailing zeros
        let changed_bits = self.pink_counter ^ self.pink_counter.wrapping_sub(1);

        for i in 0..PINK_OCTAVES {
            if changed_bits & (1 << i) != 0 {
                self.pink_running_sum -= self.pink_octaves[i];
                let new_val = self.rng.next_f32();
                self.pink_octaves[i] = new_val;
                self.pink_running_sum += new_val;
            }
        }

        // Add white noise component and normalize
        let white = self.rng.next_f32();
        (self.pink_running_sum + white) / (PINK_OCTAVES as f32 + 1.0)
    }

    /// Generate brown noise (integrated white noise).
    #[inline]
    fn brown_noise(&mut self) -> f32 {
        let white = self.rng.next_f32();
        self.brown_prev += white * 0.02;
        self.brown_prev = self.brown_prev.clamp(-1.0, 1.0);
        // Apply leaky integrator to prevent DC drift
        self.brown_prev *= 0.999;
        self.brown_prev
    }

    /// Fill a buffer with noise samples.
    #[inline]
    pub fn fill_buffer(&mut self, buffer: &mut [f32]) {
        for sample in buffer.iter_mut() {
            *sample = self.next_sample();
        }
    }
}

/// Generate a single white noise sample from a seed.
///
/// For convenience when you don't need persistent state.
#[must_use]
pub fn white_noise_sample(seed: &mut u32) -> f32 {
    xorshift32_signed_f32(seed)
}

/// One step of [`xorshift32`] mapped to a signed `f32` in `[-1.0, 1.0)`.
#[inline]
#[must_use]
pub fn xorshift32_signed_f32(state: &mut u32) -> f32 {
    (xorshift32(state) as f32 / u32::MAX as f32) * 2.0 - 1.0
}

/// One step of the Marsaglia xorshift32 PRNG.
///
/// Updates `state` in place and returns the new value. Includes a
/// zero-state guard — `xorshift32(0) = 0` would otherwise loop forever,
/// so a zero state is silently reset to `1` before stepping.
///
/// This is the canonical xorshift implementation used by every PRNG
/// site in the crate (white/pink/brown noise, granular spray jitter,
/// drum click transients, physical-modeling exciters). Use it directly
/// instead of inlining the three-XOR sequence.
#[inline]
pub fn xorshift32(state: &mut u32) -> u32 {
    if *state == 0 {
        *state = 1;
    }
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    x
}
