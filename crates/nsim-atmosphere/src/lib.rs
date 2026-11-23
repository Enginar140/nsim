//! # nsim-atmosphere
//!
//! The air that NSIM vehicles fly through:
//! - [`isa`]: U.S. Standard Atmosphere 1976 (temperature, pressure, density,
//!   speed of sound), with ISA±ΔT days;
//! - [`wind`]: mean wind with the MIL-F-8785C logarithmic shear profile;
//! - [`dryden`]: Dryden turbulence (MIL-F-8785C) with an exact discrete-time
//!   implementation whose statistics do not depend on the step size.
//!
//! All transcendental math goes through `nsim_core::fmath`, so results are
//! bit-identical on every platform.

pub mod dryden;
pub mod isa;
pub mod wind;

pub use dryden::{DrydenTurbulence, Intensity};
pub use isa::{AirState, Isa};
pub use wind::MeanWind;
