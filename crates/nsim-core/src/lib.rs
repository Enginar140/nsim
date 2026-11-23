//! # nsim-core
//!
//! Foundation types shared by every NSIM crate. Simulation time is an integer
//! number of nanoseconds so that stepping is exact and reproducible on every
//! platform (no floating-point drift between runs or machines).

pub mod fmath;
pub mod frames;
pub mod math;
pub mod rng;
pub mod time;

pub use math::{Iso3, Mat3, Quat, Twist, Vec3, Wrench};
pub use rng::RngStream;
pub use time::{SimDuration, SimTime};
