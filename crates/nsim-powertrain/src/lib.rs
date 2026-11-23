//! # nsim-powertrain
//!
//! Electric power train models, at two levels of detail:
//! - **simple**: [`FirstOrderMotor`] — throttle × max speed with a lag; no
//!   electrical data needed;
//! - **electric**: [`BldcMotor`] driven through an ESC duty cycle from a
//!   [`Battery`] bus (OCV(SoC), R0, optional RC pair), so battery sag reduces
//!   motor speed and thrust.
//!
//! These are pure models (no ECS); `nsim-rotor` wires them to propellers.

pub mod battery;
pub mod motor;

pub use battery::{Battery, Provenance, RcPair};
pub use motor::{BldcMotor, FirstOrderMotor, MotorStep, kv_from_rpm_per_volt};
