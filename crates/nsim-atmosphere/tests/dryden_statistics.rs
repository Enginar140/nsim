//! Statistical validation of the Dryden generator against theory:
//! variance σ², the longitudinal autocorrelation e^{-τV/L}, and the
//! lateral/vertical autocorrelation (1 − τV/2L)·e^{-τV/L}. Also checks that
//! the statistics do not depend on the step size — the point of the exact
//! discretization.

use nsim_atmosphere::DrydenTurbulence;
use nsim_atmosphere::dryden::{Intensity, params};
use nsim_core::RngStream;

const V: f64 = 30.0; // airspeed [m/s]
const H: f64 = 3.048; // 10 ft: short scale lengths → many independent samples

fn series(dt: f64, seconds: f64, seed: u64) -> Vec<[f64; 3]> {
    let mut d = DrydenTurbulence::new(Intensity::Moderate, RngStream::new(seed, "dryden"));
    let n = (seconds / dt) as usize;
    (0..n)
        .map(|_| {
            let g = d.step(dt, V, H);
            [g.x, g.y, g.z]
        })
        .collect()
}

fn variance(xs: &[[f64; 3]], axis: usize) -> f64 {
    let n = xs.len() as f64;
    let mean = xs.iter().map(|x| x[axis]).sum::<f64>() / n;
    xs.iter().map(|x| (x[axis] - mean).powi(2)).sum::<f64>() / n
}

/// Normalized autocorrelation at a lag of `lag` samples.
fn autocorrelation(xs: &[[f64; 3]], axis: usize, lag: usize) -> f64 {
    let n = xs.len() - lag;
    let num: f64 = (0..n).map(|i| xs[i][axis] * xs[i + lag][axis]).sum::<f64>() / n as f64;
    num / variance(xs, axis)
}

#[test]
fn variance_matches_sigma_squared() {
    let p = params(H, Intensity::Moderate.w20());
    let xs = series(0.01, 4000.0, 1);
    for axis in 0..3 {
        let expected = p.sigma[axis].powi(2);
        let got = variance(&xs, axis);
        assert!(
            ((got - expected) / expected).abs() < 0.08,
            "axis {axis}: variance {got}, expected {expected}"
        );
    }
}

#[test]
fn longitudinal_autocorrelation_is_exponential() {
    let p = params(H, Intensity::Moderate.w20());
    let dt = 0.01;
    let xs = series(dt, 4000.0, 2);
    // Lag of one correlation time L/V: expected e^{-1}.
    let lag = (p.length.x / V / dt).round() as usize;
    let tau = lag as f64 * dt;
    let expected = (-tau * V / p.length.x).exp();
    let got = autocorrelation(&xs, 0, lag);
    assert!(
        (got - expected).abs() < 0.04,
        "ρ_u = {got}, expected {expected}"
    );
}

#[test]
fn vertical_autocorrelation_matches_dryden_form() {
    let p = params(H, Intensity::Moderate.w20());
    let dt = 0.002;
    let xs = series(dt, 1000.0, 3);
    for multiple in [0.5, 1.0, 2.0] {
        let lag = (multiple * p.length.z / V / dt).round() as usize;
        let x = lag as f64 * dt * V / p.length.z;
        let expected = (1.0 - x / 2.0) * (-x).exp();
        let got = autocorrelation(&xs, 2, lag);
        assert!(
            (got - expected).abs() < 0.03,
            "ρ_w at τV/L = {x:.2}: {got}, expected {expected}"
        );
    }
}

#[test]
fn statistics_do_not_depend_on_step_size() {
    // With a = V/L_w ≈ 9.8 1/s, dt = 0.05 s is a coarse step (a·dt ≈ 0.5)
    // where a forward-Euler filter would visibly distort the variance.
    let p = params(H, Intensity::Moderate.w20());
    let coarse = series(0.05, 20_000.0, 4);
    let expected = p.sigma.z.powi(2);
    let got = variance(&coarse, 2);
    assert!(
        ((got - expected) / expected).abs() < 0.04,
        "variance {got} vs {expected}"
    );
}

#[test]
fn same_seed_same_gusts() {
    assert_eq!(series(0.01, 10.0, 9), series(0.01, 10.0, 9));
    assert_ne!(series(0.01, 10.0, 9), series(0.01, 10.0, 10));
}
