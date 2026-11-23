//! # nsim-rotor
//!
//! Propellers and rotors for multirotors, fixed-wing tractors/pushers and
//! (with water density) thrusters:
//! - [`propeller`]: thrust/torque/power from CT(J) and CQ(J) tables, so thrust
//!   depends on airspeed and air density; Cheeseman–Bennett ground effect.
//! - [`system`]: [`Rotor`]s mounted on bodies and the [`RotorSystem`] that
//!   turns them into forces and reaction torques each step.
//!
//! Each rotor has a [`Drive`]: speed set directly, a simple first-order motor,
//! or an electric BLDC motor fed by the body's [`PowerBus`] battery.

pub mod propeller;
pub mod system;

pub use propeller::{Propeller, PropellerOutput, ground_effect_factor};
pub use system::{Drive, PowerBus, Rotor, RotorSystem, Rotors, Spin};
