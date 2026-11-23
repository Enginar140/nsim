//! Simulation time.
//!
//! [`SimTime`] is an instant since simulation start and [`SimDuration`] a span,
//! both stored as signed 64-bit nanoseconds (range ≈ ±292 years). Arithmetic is
//! checked: overflow panics in debug and release alike, because silently
//! wrapping simulation time would corrupt every downstream model.

use std::fmt;
use std::ops::{Add, AddAssign, Sub};

const NANOS_PER_SEC: i64 = 1_000_000_000;

/// A span of simulation time in integer nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimDuration(i64);

impl SimDuration {
    /// Zero-length duration.
    pub const ZERO: Self = Self(0);

    /// Creates a duration from nanoseconds.
    pub const fn from_nanos(nanos: i64) -> Self {
        Self(nanos)
    }

    /// Creates a duration from microseconds. Panics on overflow.
    pub const fn from_micros(micros: i64) -> Self {
        Self(checked(micros.checked_mul(1_000)))
    }

    /// Creates a duration from milliseconds. Panics on overflow.
    pub const fn from_millis(millis: i64) -> Self {
        Self(checked(millis.checked_mul(1_000_000)))
    }

    /// Creates a duration from whole seconds. Panics on overflow.
    pub const fn from_secs(secs: i64) -> Self {
        Self(checked(secs.checked_mul(NANOS_PER_SEC)))
    }

    /// The period of a fixed update rate, e.g. `from_hz(400)` = 2.5 ms.
    ///
    /// Returns `None` if `hz` is zero or the period is not a whole number of
    /// nanoseconds; a rounded period would make the rate drift over long runs.
    pub const fn from_hz(hz: u32) -> Option<Self> {
        if hz == 0 || NANOS_PER_SEC % hz as i64 != 0 {
            return None;
        }
        Some(Self(NANOS_PER_SEC / hz as i64))
    }

    /// Nanoseconds in this duration.
    pub const fn as_nanos(self) -> i64 {
        self.0
    }

    /// This duration in seconds as `f64`, for use in physics models.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / NANOS_PER_SEC as f64
    }
}

/// An instant of simulation time, measured from the start of the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SimTime(i64);

impl SimTime {
    /// The start of the simulation.
    pub const ZERO: Self = Self(0);

    /// Creates an instant from nanoseconds since simulation start.
    pub const fn from_nanos(nanos: i64) -> Self {
        Self(nanos)
    }

    /// Nanoseconds since simulation start.
    pub const fn as_nanos(self) -> i64 {
        self.0
    }

    /// Seconds since simulation start as `f64`.
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / NANOS_PER_SEC as f64
    }

    /// Time elapsed since `earlier`.
    pub fn since(self, earlier: SimTime) -> SimDuration {
        self - earlier
    }
}

const fn checked(value: Option<i64>) -> i64 {
    match value {
        Some(v) => v,
        None => panic!("simulation time overflow"),
    }
}

impl Add<SimDuration> for SimTime {
    type Output = SimTime;
    fn add(self, rhs: SimDuration) -> SimTime {
        SimTime(checked(self.0.checked_add(rhs.0)))
    }
}

impl AddAssign<SimDuration> for SimTime {
    fn add_assign(&mut self, rhs: SimDuration) {
        *self = *self + rhs;
    }
}

impl Sub for SimTime {
    type Output = SimDuration;
    fn sub(self, rhs: SimTime) -> SimDuration {
        SimDuration(checked(self.0.checked_sub(rhs.0)))
    }
}

impl Add for SimDuration {
    type Output = SimDuration;
    fn add(self, rhs: SimDuration) -> SimDuration {
        SimDuration(checked(self.0.checked_add(rhs.0)))
    }
}

impl fmt::Display for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sign = if self.0 < 0 { "-" } else { "" };
        let abs = self.0.unsigned_abs();
        let per_sec = NANOS_PER_SEC as u64;
        write!(f, "{sign}{}.{:09}s", abs / per_sec, abs % per_sec)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_constructors_agree() {
        assert_eq!(SimDuration::from_secs(1), SimDuration::from_millis(1_000));
        assert_eq!(SimDuration::from_millis(1), SimDuration::from_micros(1_000));
        assert_eq!(SimDuration::from_micros(1), SimDuration::from_nanos(1_000));
    }

    #[test]
    fn from_hz_gives_exact_periods() {
        assert_eq!(
            SimDuration::from_hz(400),
            Some(SimDuration::from_micros(2_500))
        );
        assert_eq!(
            SimDuration::from_hz(1_000),
            Some(SimDuration::from_millis(1))
        );
    }

    #[test]
    fn from_hz_rejects_zero_and_inexact_rates() {
        assert_eq!(SimDuration::from_hz(0), None);
        // 1e9 / 3 is not an integer number of nanoseconds.
        assert_eq!(SimDuration::from_hz(3), None);
    }

    #[test]
    fn stepping_is_exact_over_long_runs() {
        // One hour at 1 kHz: 3.6 million steps must land exactly on 3600 s,
        // which accumulating an f64 dt of 0.001 does not.
        let dt = SimDuration::from_hz(1_000).unwrap();
        let mut t = SimTime::ZERO;
        for _ in 0..3_600_000 {
            t += dt;
        }
        assert_eq!(t.since(SimTime::ZERO), SimDuration::from_secs(3_600));
    }

    #[test]
    fn time_difference_is_a_duration() {
        let a = SimTime::from_nanos(5_000);
        let b = SimTime::from_nanos(2_000);
        assert_eq!(a - b, SimDuration::from_nanos(3_000));
        assert_eq!(b - a, SimDuration::from_nanos(-3_000));
    }

    #[test]
    fn display_formats_seconds_with_nanoseconds() {
        assert_eq!(
            SimTime::from_nanos(1_500_000_000).to_string(),
            "1.500000000s"
        );
        assert_eq!(SimTime::from_nanos(-1).to_string(), "-0.000000001s");
        assert_eq!(
            SimTime::from_nanos(-2_500_000_000).to_string(),
            "-2.500000000s"
        );
    }

    #[test]
    #[should_panic(expected = "simulation time overflow")]
    fn overflow_panics_instead_of_wrapping() {
        let _ = SimTime::from_nanos(i64::MAX) + SimDuration::from_nanos(1);
    }
}
