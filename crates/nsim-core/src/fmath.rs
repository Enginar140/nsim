//! Cross-platform deterministic math functions.
//!
//! `f64::exp`, `ln`, `powf`, `sin`, … in std call the platform's C math
//! library, whose last-bit results differ between operating systems and
//! versions. That would silently break NSIM's guarantee that the same seed
//! gives bit-identical results everywhere. **Simulation models must use these
//! functions** (pure-Rust `libm`, the same choice Rapier makes for its
//! `enhanced-determinism` mode). `sqrt`, `abs`, `+ − × ÷` are exactly rounded
//! by IEEE 754 and safe to use from std.

/// e^x
#[inline]
pub fn exp(x: f64) -> f64 {
    libm::exp(x)
}

/// Natural logarithm.
#[inline]
pub fn ln(x: f64) -> f64 {
    libm::log(x)
}

/// x^y
#[inline]
pub fn pow(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}

/// Sine (radians).
#[inline]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

/// Cosine (radians).
#[inline]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// Tangent (radians).
#[inline]
pub fn tan(x: f64) -> f64 {
    libm::tan(x)
}

/// Four-quadrant arctangent of y/x (radians).
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// Arcsine (radians).
#[inline]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

/// Arccosine (radians).
#[inline]
pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

/// e^x − 1, accurate for small x.
#[inline]
pub fn exp_m1(x: f64) -> f64 {
    libm::expm1(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agrees_with_std_to_within_an_ulp_or_two() {
        for &x in &[-3.7, -0.5, 0.0, 1e-8, 0.3, 1.0, 2.5, 10.0] {
            assert!((exp(x) - x.exp()).abs() <= 2.0 * f64::EPSILON * x.exp().abs().max(1.0));
            assert!((sin(x) - x.sin()).abs() <= 2.0 * f64::EPSILON);
            assert!((cos(x) - x.cos()).abs() <= 2.0 * f64::EPSILON);
        }
        assert!((pow(2.0, 0.5) - std::f64::consts::SQRT_2).abs() <= f64::EPSILON);
        assert!((ln(std::f64::consts::E) - 1.0).abs() <= f64::EPSILON);
    }
}
