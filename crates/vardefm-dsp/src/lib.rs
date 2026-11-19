//! Signal processing for the VardeFM modem: OFDM, constellations and filters.

pub mod filter;
pub mod ofdm;
pub mod params;
pub mod prng;
pub mod qam;

pub use num_complex::Complex32;

/// A received sample beyond this is a glitch: full scale is 1. Far enough beyond it,
/// a finite sample overflows the high-Q sections of an IIR filter and leaves its state
/// NaN, so the filters do not take one.
pub const MAX_SAMPLE: f32 = 4.0;
pub use ofdm::{Frame, Ofdm};
pub use params::OfdmParams;
pub use qam::Modulation;
