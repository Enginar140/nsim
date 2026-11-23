//! # nsim-rotor
//!
//! Propellers and rotors for multirotors, fixed-wing tractors/pushers and
//! (with water density) thrusters:
//! - [`propeller`]: thrust/torque/power from CT(J) and CQ(J) tables, so thrust
//!   depends on airspeed and air density; Cheeseman–Bennett ground effect.
//! - [`system`]: [`Rotor`]s mounted on bodies and the [`RotorSystem`] that
//!   turns them into forces and reaction torques each step.
//!
//! Rotor speed is an input (`Rotor::omega`); the electric power train (ESC,
//! motor, battery) that produces it is a separate model.

pub mod propeller;
pub mod system;

pub use propeller::{Propeller, PropellerOutput, ground_effect_factor};
pub use system::{Rotor, RotorSystem, Rotors, Spin};
