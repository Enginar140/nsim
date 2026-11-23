//! Motor models, from simple to electric.
//!
//! **Simple** ([`FirstOrderMotor`]): speed follows `throttle · ω_max` with a
//! first-order lag (separate up/down time constants). Needs no electrical
//! data and ignores the battery — Gazebo's level of fidelity.
//!
//! **Electric** ([`BldcMotor`]): a DC-equivalent brushless motor driven by
//! an ESC from a battery bus:
//!
//! ```text
//! back-EMF  e = ω / Kv                    (Kv in rad/s per volt)
//! current   I = (duty·V_bus − e) / R       (≥ 0: the ESC does not regenerate)
//! torque    τ = (I − I0) / Kv              (I0: no-load current, friction)
//! dynamics  J·dω/dt = τ − Q_prop(ω)
//! ```
//!
//! The back-EMF term is integrated implicitly, so the model is stable for any
//! step size; the propeller load is taken at the start of the step.

use std::f64::consts::PI;

use nsim_core::fmath;

/// Converts a Kv rating in rpm/V (as printed on motors) to rad/s per volt.
pub fn kv_from_rpm_per_volt(rpm_per_volt: f64) -> f64 {
    rpm_per_volt * 2.0 * PI / 60.0
}

/// Simple motor: speed = throttle × maximum speed, with a first-order lag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FirstOrderMotor {
    /// Speed at full throttle [rad/s].
    pub max_omega: f64,
    /// Time constant when speeding up [s].
    pub tau_up: f64,
    /// Time constant when slowing down [s].
    pub tau_down: f64,
}

impl FirstOrderMotor {
    /// New speed after `dt` [s] at `throttle` ∈ [0, 1] (clamped), integrated
    /// exactly.
    pub fn step(&self, omega: f64, throttle: f64, dt: f64) -> f64 {
        let target = throttle.clamp(0.0, 1.0) * self.max_omega;
        let tau = if target > omega {
            self.tau_up
        } else {
            self.tau_down
        };
        if tau <= 0.0 {
            return target;
        }
        target + (omega - target) * fmath::exp(-dt / tau)
    }
}

/// Brushless DC motor plus the rotating inertia it drives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BldcMotor {
    /// Speed constant [rad/s per V].
    pub kv: f64,
    /// Phase-to-phase winding resistance plus ESC losses [Ω].
    pub resistance: f64,
    /// No-load current [A].
    pub no_load_current: f64,
    /// Inertia of rotor plus propeller about the shaft [kg·m²].
    pub inertia: f64,
}

/// Result of one electric motor step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorStep {
    /// Speed at the end of the step [rad/s].
    pub omega: f64,
    /// Current drawn from the bus during the step [A].
    pub current: f64,
    /// Net torque the motor exerts on its rotor (and, reversed, on the
    /// airframe) [N·m].
    pub torque: f64,
}

impl BldcMotor {
    /// Validates the parameters.
    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [
            ("kv", self.kv),
            ("resistance", self.resistance),
            ("inertia", self.inertia),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(format!("motor {name} must be positive, got {v}"));
            }
        }
        if !(self.no_load_current.is_finite() && self.no_load_current >= 0.0) {
            return Err(format!(
                "motor no-load current must be >= 0, got {}",
                self.no_load_current
            ));
        }
        Ok(())
    }

    /// Advances by `dt` [s] with ESC `duty` ∈ [0, 1] (clamped), bus voltage
    /// `v_bus` [V] and propeller load torque `load` [N·m] at the current speed.
    pub fn step(&self, omega: f64, duty: f64, v_bus: f64, load: f64, dt: f64) -> MotorStep {
        let duty = duty.clamp(0.0, 1.0);
        let applied = duty * v_bus;
        let (kv, r, j, i0) = (self.kv, self.resistance, self.inertia, self.no_load_current);
        // Implicit in the back-EMF: ω' (1 + dt/(J R Kv²)) = ω + dt/J (V/(R Kv) − I0/Kv − Q)
        let gain = dt / (j * r * kv * kv);
        let driven = (omega + dt / j * (applied / (r * kv) - i0 / kv - load)) / (1.0 + gain);
        let current = (applied - driven / kv) / r;
        let (omega_next, current) = if current >= 0.0 {
            (driven, current)
        } else {
            // Back-EMF exceeds the applied voltage: the ESC freewheels, no
            // current flows and the rotor coasts against friction and load.
            (omega + dt / j * (-i0 / kv - load), 0.0)
        };
        let omega_next = omega_next.max(0.0);
        let torque = if current > 0.0 {
            (current - i0) / kv
        } else {
            -i0 / kv
        };
        MotorStep {
            omega: omega_next,
            current,
            torque,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 920 rpm/V class motor on a 10" propeller.
    fn motor() -> BldcMotor {
        BldcMotor {
            kv: kv_from_rpm_per_volt(920.0),
            resistance: 0.12,
            no_load_current: 0.5,
            inertia: 2.5e-5,
        }
    }

    fn run(m: &BldcMotor, duty: f64, v: f64, load: f64, dt: f64, steps: usize) -> MotorStep {
        let mut s = MotorStep {
            omega: 0.0,
            current: 0.0,
            torque: 0.0,
        };
        for _ in 0..steps {
            s = m.step(s.omega, duty, v, load, dt);
        }
        s
    }

    #[test]
    fn kv_conversion() {
        assert!((kv_from_rpm_per_volt(60.0) - 2.0 * PI).abs() < 1e-12);
    }

    #[test]
    fn no_load_speed_is_kv_times_voltage_minus_i0_drop() {
        let m = motor();
        let s = run(&m, 1.0, 12.0, 0.0, 1e-3, 5000);
        let expected = m.kv * (12.0 - m.no_load_current * m.resistance);
        assert!(
            (s.omega - expected).abs() / expected < 1e-9,
            "ω = {}",
            s.omega
        );
        assert!((s.current - m.no_load_current).abs() < 1e-9);
    }

    #[test]
    fn loaded_steady_state_matches_motor_equations() {
        // Constant load Q: I = I0 + Q·Kv, ω = Kv (V − I R).
        let m = motor();
        let load = 0.05;
        let s = run(&m, 0.8, 16.0, load, 1e-3, 10_000);
        let current = m.no_load_current + load * m.kv;
        let omega = m.kv * (0.8 * 16.0 - current * m.resistance);
        assert!((s.current - current).abs() < 1e-9, "I = {}", s.current);
        assert!((s.omega - omega).abs() / omega < 1e-9, "ω = {}", s.omega);
        assert!((s.torque - load).abs() < 1e-9, "τ = {}", s.torque);
    }

    #[test]
    fn spin_up_follows_the_mechanical_time_constant() {
        // Without load and friction the speed is first order with
        // τ_m = J R Kv²; after τ_m it reaches 1 − e⁻¹ of the final value.
        let m = BldcMotor {
            no_load_current: 0.0,
            ..motor()
        };
        let tau = m.inertia * m.resistance * m.kv * m.kv;
        let dt = tau / 10_000.0;
        let s = run(&m, 1.0, 10.0, 0.0, dt, 10_000);
        let ratio = s.omega / (m.kv * 10.0);
        assert!(
            (ratio - (1.0 - (-1.0f64).exp())).abs() < 1e-3,
            "ratio = {ratio}"
        );
    }

    #[test]
    fn implicit_integration_is_stable_with_large_steps() {
        // A step 50× the mechanical time constant still converges.
        let m = motor();
        let tau = m.inertia * m.resistance * m.kv * m.kv;
        let s = run(&m, 1.0, 12.0, 0.0, 50.0 * tau, 200);
        let expected = m.kv * (12.0 - m.no_load_current * m.resistance);
        assert!((s.omega - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn throttle_cut_coasts_without_negative_current() {
        let m = motor();
        let spinning = run(&m, 1.0, 12.0, 0.02, 1e-3, 3000);
        let after = m.step(spinning.omega, 0.0, 12.0, 0.02, 1e-3);
        assert_eq!(after.current, 0.0);
        assert!(after.omega < spinning.omega);
        assert!(after.torque < 0.0);
    }

    #[test]
    fn first_order_motor_reaches_63_percent_after_tau() {
        let m = FirstOrderMotor {
            max_omega: 1000.0,
            tau_up: 0.05,
            tau_down: 0.1,
        };
        let w = m.step(0.0, 1.0, 0.05);
        assert!((w - 1000.0 * (1.0 - (-1.0f64).exp())).abs() < 1e-9);
        // Slowing down uses the other time constant.
        let w = m.step(1000.0, 0.0, 0.1);
        assert!((w - 1000.0 * (-1.0f64).exp()).abs() < 1e-9);
    }

    #[test]
    fn invalid_motor_is_rejected() {
        assert!(motor().validate().is_ok());
        let bad = BldcMotor { kv: 0.0, ..motor() };
        assert!(bad.validate().is_err());
    }
}
