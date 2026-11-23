//! # nsim-core
//!
//! Foundation types shared by every NSIM crate. Simulation time is an integer
//! number of nanoseconds so that stepping is exact and reproducible on every
//! platform (no floating-point drift between runs or machines).

pub mod time;

pub use time::{SimDuration, SimTime};
