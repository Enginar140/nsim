//! Dryden turbulence, MIL-F-8785C, with an exact discrete-time
//! implementation.
//!
//! Each component is white noise shaped by the Dryden transfer function:
//! - longitudinal: `H_u(s) ∝ 1 / (1 + s/a)` with `a = V / L_u`
//! - lateral and vertical: `H(s) ∝ (1 + √3·s/a) / (1 + s/a)²`
//!
//! Instead of the usual forward-Euler filter (whose statistics drift with the
//! step size), the shaping filters are discretized **exactly**: state
//! transition `e^{A·dt}` and process-noise covariance are derived in closed
//! form, so variance and autocorrelation are correct for any `dt`. States are
//! normalized to unit stationary variance, which keeps the process
//! continuous when airspeed or altitude (and so `a`) change.
//!
//! Altitude model: the low-altitude formulas of MIL-F-8785C apply up to
//! 1000 ft. Between 1000 and 2000 ft the scale lengths blend linearly to the
//! medium/high-altitude value of 1750 ft. **Approximation:** above 1000 ft the
//! intensities are held at their 1000 ft values instead of following the
//! specification's probability-of-exceedance table.

use nsim_core::{RngStream, Vec3, fmath};

const FT: f64 = 0.3048;
/// Minimum airspeed used for the filter time constants [m/s]. Below it
/// (hover) Taylor's frozen-turbulence hypothesis breaks down; holding a
/// floor keeps the gusts varying in time at a hover-like rate.
pub const MIN_AIRSPEED: f64 = 1.0;

/// Turbulence severity, defined by the wind speed at 20 ft (W20).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Intensity {
    /// W20 = 15 kn.
    Light,
    /// W20 = 30 kn.
    Moderate,
    /// W20 = 45 kn.
    Severe,
    /// Explicit wind speed at 20 ft [m/s].
    W20(f64),
}

impl Intensity {
    /// Wind speed at 20 ft [m/s].
    pub fn w20(self) -> f64 {
        const KNOT: f64 = 1852.0 / 3600.0;
        match self {
            Self::Light => 15.0 * KNOT,
            Self::Moderate => 30.0 * KNOT,
            Self::Severe => 45.0 * KNOT,
            Self::W20(v) => v,
        }
    }
}

/// Standard deviations [m/s] and scale lengths [m] at a given altitude.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrydenParams {
    /// Longitudinal, lateral, vertical intensities [m/s].
    pub sigma: Vec3,
    /// Longitudinal, lateral, vertical scale lengths [m].
    pub length: Vec3,
}

/// MIL-F-8785C parameters at `altitude_agl` [m] for wind `w20` [m/s].
pub fn params(altitude_agl: f64, w20: f64) -> DrydenParams {
    let h_ft = (altitude_agl / FT).clamp(10.0, 2000.0);
    let low_ft = h_ft.min(1000.0);
    let k = 0.177 + 0.000_823 * low_ft;
    let sigma_w = 0.1 * w20;
    let sigma_uv = sigma_w / fmath::pow(k, 0.4);
    let l_uv_low = low_ft / fmath::pow(k, 1.2);
    let l_w_low = low_ft;
    // Blend scale lengths to 1750 ft between 1000 and 2000 ft.
    let blend = ((h_ft - 1000.0) / 1000.0).clamp(0.0, 1.0);
    let l_uv = (l_uv_low + blend * (1750.0 - l_uv_low)) * FT;
    let l_w = (l_w_low + blend * (1750.0 - l_w_low)) * FT;
    DrydenParams {
        sigma: Vec3::new(sigma_uv, sigma_uv, sigma_w),
        length: Vec3::new(l_uv, l_uv, l_w),
    }
}

/// Exact one-step update of the second-order Dryden shaping filter in
/// normalized coordinates (stationary covariance [[1/2,1/4],[1/4,1/4]]).
fn step_second_order(s: &mut [f64; 2], r: f64, rng: &mut RngStream) {
    // r = a·dt. Transition e^{A dt} = e^{-r} [[1, 0], [r, 1]].
    let decay = fmath::exp(-r);
    let (s1, s2) = (s[0], s[1]);
    let m1 = decay * s1;
    let m2 = decay * (r * s1 + s2);
    // Process-noise covariance (normalized), computed without cancellation.
    let e = fmath::exp(-2.0 * r);
    let one_minus_e = -fmath::exp_m1(-2.0 * r);
    let q11 = one_minus_e / 2.0;
    let q12 = (one_minus_e - 2.0 * r * e) / 4.0;
    let q22 = (2.0 * one_minus_e - e * (4.0 * r + 4.0 * r * r)) / 8.0;
    // Cholesky factor of [[q11, q12], [q12, q22]].
    let l11 = q11.sqrt();
    let l21 = if l11 > 0.0 { q12 / l11 } else { 0.0 };
    let l22 = (q22 - l21 * l21).max(0.0).sqrt();
    let (n1, n2) = (rng.normal(), rng.normal());
    s[0] = m1 + l11 * n1;
    s[1] = m2 + l21 * n1 + l22 * n2;
}

/// Output of the normalized second-order filter, unit variance.
fn second_order_output(s: &[f64; 2]) -> f64 {
    const SQRT3: f64 = 1.732_050_807_568_877_2;
    SQRT3 * s[0] + (1.0 - SQRT3) * s[1]
}

/// Dryden turbulence generator for one vehicle.
#[derive(Debug, Clone)]
pub struct DrydenTurbulence {
    w20: f64,
    rng: RngStream,
    /// Longitudinal state, unit variance.
    u: f64,
    /// Lateral and vertical states, normalized.
    v: [f64; 2],
    w: [f64; 2],
}

impl DrydenTurbulence {
    /// A generator starting from a stationary random state.
    pub fn new(intensity: Intensity, mut rng: RngStream) -> Self {
        // Draw initial states from the stationary distribution so there is
        // no artificial calm start-up transient.
        let mut stationary = || {
            let (a, b) = (rng.normal(), rng.normal());
            // Cholesky of [[1/2, 1/4], [1/4, 1/4]].
            let l11 = 0.5f64.sqrt();
            let l21 = 0.25 / l11;
            let l22 = (0.25 - l21 * l21).sqrt();
            [l11 * a, l21 * a + l22 * b]
        };
        let v = stationary();
        let w = stationary();
        let u = rng.normal();
        Self {
            w20: intensity.w20(),
            rng,
            u,
            v,
            w,
        }
    }

    /// Advances by `dt` seconds and returns the gust velocity [m/s] in the
    /// turbulence frame: x along the horizontal flight direction, y to the
    /// left, z up.
    pub fn step(&mut self, dt: f64, airspeed: f64, altitude_agl: f64) -> Vec3 {
        let p = params(altitude_agl, self.w20);
        let speed = airspeed.max(MIN_AIRSPEED);
        // Longitudinal: exact first-order Gauss–Markov update.
        let phi = fmath::exp(-speed * dt / p.length.x);
        self.u = phi * self.u + (1.0 - phi * phi).max(0.0).sqrt() * self.rng.normal();
        step_second_order(&mut self.v, speed * dt / p.length.y, &mut self.rng);
        step_second_order(&mut self.w, speed * dt / p.length.z, &mut self.rng);
        Vec3::new(
            p.sigma.x * self.u,
            p.sigma.y * second_order_output(&self.v),
            p.sigma.z * second_order_output(&self.w),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn low_altitude_parameters_follow_mil_f_8785c() {
        // At 1000 ft: k = 1, so σ_u = σ_v = σ_w = 0.1·W20 and L = 1000 ft.
        let p = params(1000.0 * FT, 20.0);
        assert!((p.sigma - Vec3::repeat(2.0)).norm() < 1e-12);
        assert!((p.length - Vec3::repeat(1000.0 * FT)).norm() < 1e-9);
        // At 100 ft: k = 0.2593, σ_u = σ_w / k^0.4, L_u = 100 / k^1.2 ft.
        let p = params(100.0 * FT, 20.0);
        let k: f64 = 0.177 + 0.0823;
        assert!((p.sigma.x - 2.0 / k.powf(0.4)).abs() < 1e-9);
        assert!((p.length.x - 100.0 / k.powf(1.2) * FT).abs() < 1e-6);
        assert!((p.length.z - 100.0 * FT).abs() < 1e-9);
    }

    #[test]
    fn high_altitude_scale_length_is_1750_ft() {
        let p = params(5000.0, 20.0);
        assert!((p.length - Vec3::repeat(1750.0 * FT)).norm() < 1e-9);
    }

    #[test]
    fn intensity_presets_are_in_knots() {
        assert!((Intensity::Moderate.w20() - 15.433).abs() < 1e-3);
    }
}
