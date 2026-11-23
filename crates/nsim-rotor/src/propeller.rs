//! Propeller aerodynamics from coefficient tables.
//!
//! With `n` the rotational speed [rev/s], `D` the diameter and `ρ` the air
//! density:
//!
//! ```text
//! J = V_axial / (n·D)          advance ratio
//! T = CT(J) · ρ · n² · D⁴      thrust
//! Q = CQ(J) · ρ · n² · D⁵      shaft torque
//! P = 2π · n · Q               shaft power
//! ```
//!
//! This is the standard form of wind-tunnel propeller data (e.g. the UIUC
//! propeller database) and of JSBSim's propeller model. Unlike a constant
//! `T = k·ω²`, thrust drops as the vehicle moves along the thrust axis, and
//! everything scales with air density (altitude, temperature).

use std::f64::consts::PI;

use nsim_core::{Table1D, TableError};

/// Below this rotational speed [rev/s] the advance ratio is ill-defined and
/// the propeller produces no force.
const N_MIN: f64 = 1e-6;

/// Aerodynamic output of a propeller at one operating point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerOutput {
    /// Advance ratio J (dimensionless).
    pub advance_ratio: f64,
    /// Thrust along the thrust axis [N] (negative = reverse thrust).
    pub thrust: f64,
    /// Shaft torque magnitude that the motor must supply [N·m].
    pub torque: f64,
    /// Shaft power [W].
    pub power: f64,
}

/// A fixed-pitch propeller described by CT(J) and CQ(J).
#[derive(Debug, Clone, PartialEq)]
pub struct Propeller {
    diameter: f64,
    ct: Table1D,
    cq: Table1D,
}

impl Propeller {
    /// A propeller of `diameter` [m] with thrust and torque coefficient
    /// tables over the advance ratio.
    pub fn new(diameter: f64, ct: Table1D, cq: Table1D) -> Result<Self, String> {
        if !(diameter.is_finite() && diameter > 0.0) {
            return Err(format!(
                "propeller diameter must be positive, got {diameter}"
            ));
        }
        Ok(Self { diameter, ct, cq })
    }

    /// Diameter [m].
    pub fn diameter(&self) -> f64 {
        self.diameter
    }

    /// Radius [m].
    pub fn radius(&self) -> f64 {
        self.diameter / 2.0
    }

    /// Thrust, torque and power at rotational speed `omega` [rad/s, ≥ 0],
    /// air density `density` [kg/m³] and axial inflow `v_axial` [m/s]
    /// (velocity of the hub relative to the air along the thrust axis;
    /// positive when moving in the thrust direction).
    pub fn evaluate(
        &self,
        omega: f64,
        density: f64,
        v_axial: f64,
    ) -> Result<PropellerOutput, TableError> {
        let n = omega / (2.0 * PI);
        if n < N_MIN {
            return Ok(PropellerOutput {
                advance_ratio: 0.0,
                thrust: 0.0,
                torque: 0.0,
                power: 0.0,
            });
        }
        let d = self.diameter;
        let j = v_axial / (n * d);
        let rho_n2 = density * n * n;
        let thrust = self.ct.eval(j)? * rho_n2 * d.powi(4);
        let torque = self.cq.eval(j)? * rho_n2 * d.powi(5);
        Ok(PropellerOutput {
            advance_ratio: j,
            thrust,
            torque,
            power: 2.0 * PI * n * torque,
        })
    }
}

/// Cheeseman–Bennett ground-effect thrust multiplier for a rotor of radius
/// `radius` with its hub `height` above the ground:
/// `T_IGE / T_OGE = 1 / (1 − (R / 4z)²)`.
///
/// The model diverges as z → R/4; it is capped at z = R/2 (factor 4/3),
/// near the limit of its experimental validity. Returns 1 out of ground
/// effect or when `height` is not positive.
pub fn ground_effect_factor(radius: f64, height: f64) -> f64 {
    if height <= 0.0 {
        return 1.0;
    }
    let z = height.max(radius / 2.0);
    let r = radius / (4.0 * z);
    1.0 / (1.0 - r * r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nsim_core::Extrapolation;

    /// A synthetic 10-inch propeller: CT falls linearly from 0.11 at J = 0
    /// to 0 at J = 0.6; CQ from 0.0075 to 0.0025.
    pub(crate) fn test_prop() -> Propeller {
        let js = vec![0.0, 0.6];
        Propeller::new(
            0.254,
            Table1D::new(js.clone(), vec![0.11, 0.0], Extrapolation::Linear).unwrap(),
            Table1D::new(js, vec![0.0075, 0.0025], Extrapolation::Linear).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn static_thrust_matches_ct_rho_n2_d4() {
        let p = test_prop();
        let omega = 2.0 * PI * 100.0; // 6000 rpm
        let out = p.evaluate(omega, 1.225, 0.0).unwrap();
        let expected = 0.11 * 1.225 * 100.0f64.powi(2) * 0.254f64.powi(4);
        assert!((out.thrust - expected).abs() < 1e-12, "T = {}", out.thrust);
        assert_eq!(out.advance_ratio, 0.0);
        assert!((out.power - 2.0 * PI * 100.0 * out.torque).abs() < 1e-12);
    }

    #[test]
    fn thrust_drops_with_forward_speed() {
        let p = test_prop();
        let omega = 2.0 * PI * 100.0;
        let still = p.evaluate(omega, 1.225, 0.0).unwrap().thrust;
        // V = 7.62 m/s → J = 0.3 → CT = 0.055: half the static thrust.
        let moving = p.evaluate(omega, 1.225, 7.62).unwrap();
        assert!((moving.advance_ratio - 0.3).abs() < 1e-12);
        assert!((moving.thrust / still - 0.5).abs() < 1e-12);
    }

    #[test]
    fn thrust_scales_with_density() {
        let p = test_prop();
        let sea_level = p.evaluate(600.0, 1.225, 0.0).unwrap().thrust;
        let high = p.evaluate(600.0, 0.9093, 0.0).unwrap().thrust; // ~3000 m
        assert!((high / sea_level - 0.9093 / 1.225).abs() < 1e-12);
    }

    #[test]
    fn stopped_propeller_produces_nothing() {
        let out = test_prop().evaluate(0.0, 1.225, 15.0).unwrap();
        assert_eq!((out.thrust, out.torque, out.power), (0.0, 0.0, 0.0));
    }

    #[test]
    fn ground_effect_values() {
        let r = 0.127;
        // z = R: 1 / (1 − 1/16) = 16/15.
        assert!((ground_effect_factor(r, r) - 16.0 / 15.0).abs() < 1e-12);
        // Capped at z = R/2: 1 / (1 − 1/4) = 4/3.
        assert!((ground_effect_factor(r, 0.01) - 4.0 / 3.0).abs() < 1e-12);
        // At 10 m: (R/4z)² ≈ 1.0e-5, i.e. negligible but not zero.
        let q = (r / 40.0) * (r / 40.0);
        assert!((ground_effect_factor(r, 10.0) - 1.0 / (1.0 - q)).abs() < 1e-15);
        assert!(ground_effect_factor(r, 10.0) - 1.0 < 2e-5);
        assert_eq!(ground_effect_factor(r, 0.0), 1.0);
    }

    #[test]
    fn invalid_diameter_is_rejected() {
        let p = test_prop();
        assert!(Propeller::new(0.0, p.ct.clone(), p.cq.clone()).is_err());
    }
}
