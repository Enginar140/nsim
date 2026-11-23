//! # nsim-world
//!
//! Entities, components, the phased system scheduler and the simulation loop.
//!
//! Each [`Sim::step`] runs, in this order:
//! 1. **PreUpdate** systems — controllers, flight-controller bridges: read
//!    sensors, write commands.
//! 2. **Update** systems — force models (aerodynamics, rotors, hydrodynamics):
//!    read [`Pose`]/[`Velocity`], add to [`ExternalWrench`].
//! 3. **Physics** — wrenches are handed to the backend, the backend steps,
//!    [`Pose`] and [`Velocity`] are written back and wrenches are cleared.
//! 4. **PostUpdate** systems — sensors, loggers, publishers. They receive the
//!    world **read-only** (`&World`), so the type system enforces Gazebo's
//!    otherwise unenforced "PostUpdate must not write" rule.
//!
//! Within a phase, systems run by ascending [`System::priority`], ties in
//! registration order. Systems never touch the physics engine directly; that
//! keeps force models backend-independent.

mod components;
mod sim;

pub use components::{Body, ExternalWrench, Name, Pose, Velocity};
pub use hecs::{Entity, World};
pub use sim::{Sim, SimError, Snapshot, StepContext, System};
