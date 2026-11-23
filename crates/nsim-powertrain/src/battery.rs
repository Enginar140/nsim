//! Battery: open-circuit voltage from state of charge, series resistance and
//! an optional RC pair for the slow polarization voltage (Thevenin model).
//!
//! ```text
//! V_terminal = N · OCV(SoC) − I·R0 − V_rc
//! dV_rc/dt   = (I·R1 − V_rc) / (R1·C1)          (integrated exactly)
//! dSoC/dt    = −I / capacity
//! ```
//!
//! Current is positive when discharging.

use nsim_core::{Extrapolation, Table1D, fmath};

/// One RC pair (polarization).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcPair {
    /// Resistance [Ω] (whole pack).
    pub resistance: f64,
    /// Capacitance [F] (whole pack).
    pub capacitance: f64,
}

/// Where a value came from — reported so that estimated defaults are never
/// mistaken for measured data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Provenance {
    /// Given by the user or a catalog entry.
    Specified,
    /// A generic default, not measured for this part.
    Estimated,
}

/// A battery pack.
#[derive(Debug, Clone, PartialEq)]
pub struct Battery {
    cells: u32,
    capacity: f64,
    ocv_cell: Table1D,
    ocv_provenance: Provenance,
    r0: f64,
    rc: Option<RcPair>,
    soc: f64,
    v_rc: f64,
}

impl Battery {
    /// A pack of `cells` in series with `capacity` [A·s], per-cell open
    /// circuit voltage `ocv_cell` over SoC ∈ [0, 1], pack resistance `r0` [Ω],
    /// optional RC pair, fully charged.
    pub fn new(
        cells: u32,
        capacity: f64,
        ocv_cell: Table1D,
        r0: f64,
        rc: Option<RcPair>,
    ) -> Result<Self, String> {
        if cells == 0 {
            return Err("battery needs at least one cell".into());
        }
        if !(capacity.is_finite() && capacity > 0.0) {
            return Err(format!(
                "battery capacity must be positive, got {capacity} A·s"
            ));
        }
        if !(r0.is_finite() && r0 >= 0.0) {
            return Err(format!("battery resistance must be >= 0, got {r0} Ω"));
        }
        if let Some(p) = rc
            && !(p.resistance > 0.0 && p.capacitance > 0.0)
        {
            return Err("RC pair needs positive resistance and capacitance".into());
        }
        Ok(Self {
            cells,
            capacity,
            ocv_cell,
            ocv_provenance: Provenance::Specified,
            r0,
            rc,
            soc: 1.0,
            v_rc: 0.0,
        })
    }

    /// A LiPo pack from the numbers printed on it: cell count, capacity
    /// [mAh] and per-cell internal resistance [Ω]. The discharge curve is a
    /// **generic** LiPo resting-voltage curve (3.30 V empty … 4.20 V full),
    /// flagged as [`Provenance::Estimated`]; use measured data for accuracy.
    pub fn lipo(cells: u32, capacity_mah: f64, cell_resistance: f64) -> Result<Self, String> {
        let soc = vec![0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        let volts = vec![
            3.30, 3.50, 3.68, 3.74, 3.77, 3.79, 3.82, 3.87, 3.92, 3.98, 4.08, 4.20,
        ];
        let curve = Table1D::new(soc, volts, Extrapolation::Clamp).map_err(|e| e.to_string())?;
        let mut b = Self::new(
            cells,
            capacity_mah * 3.6,
            curve,
            cell_resistance * f64::from(cells),
            None,
        )?;
        b.ocv_provenance = Provenance::Estimated;
        Ok(b)
    }

    /// Whether the OCV curve is measured/specified or a generic estimate.
    pub fn ocv_provenance(&self) -> Provenance {
        self.ocv_provenance
    }

    /// State of charge ∈ [0, 1].
    pub fn soc(&self) -> f64 {
        self.soc
    }

    /// Sets the state of charge (e.g. to start a flight half-charged).
    pub fn set_soc(&mut self, soc: f64) {
        self.soc = soc.clamp(0.0, 1.0);
    }

    /// True once the pack is fully discharged.
    pub fn is_depleted(&self) -> bool {
        self.soc <= 0.0
    }

    /// Open-circuit pack voltage at the current SoC [V].
    pub fn open_circuit_voltage(&self) -> f64 {
        let per_cell = self.ocv_cell.eval(self.soc).unwrap_or_else(|_| {
            // Clamp extrapolation cannot fail for SoC in [0, 1].
            unreachable!("OCV table evaluation with clamped SoC")
        });
        f64::from(self.cells) * per_cell
    }

    /// Terminal voltage while delivering `current` [A] [V], never negative.
    pub fn terminal_voltage(&self, current: f64) -> f64 {
        (self.open_circuit_voltage() - current * self.r0 - self.v_rc).max(0.0)
    }

    /// Advances the state by `dt` [s] at a constant `current` [A].
    pub fn step(&mut self, current: f64, dt: f64) {
        self.soc = (self.soc - current * dt / self.capacity).clamp(0.0, 1.0);
        if let Some(rc) = self.rc {
            let tau = rc.resistance * rc.capacitance;
            let decay = fmath::exp(-dt / tau);
            self.v_rc = self.v_rc * decay + current * rc.resistance * (1.0 - decay);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(volts: f64) -> Table1D {
        Table1D::new(vec![0.0, 1.0], vec![volts, volts], Extrapolation::Clamp).unwrap()
    }

    #[test]
    fn full_lipo_reads_4_2_volts_per_cell() {
        let b = Battery::lipo(4, 5000.0, 0.005).unwrap();
        assert!((b.open_circuit_voltage() - 16.8).abs() < 1e-12);
        assert_eq!(b.ocv_provenance(), Provenance::Estimated);
    }

    #[test]
    fn voltage_sags_by_i_times_r0() {
        let b = Battery::lipo(4, 5000.0, 0.005).unwrap(); // R0 = 20 mΩ
        assert!((b.terminal_voltage(50.0) - (16.8 - 1.0)).abs() < 1e-12);
    }

    #[test]
    fn charge_is_counted_in_coulombs() {
        let mut b = Battery::new(1, 3600.0, flat(4.0), 0.0, None).unwrap(); // 1 Ah
        for _ in 0..1800 {
            b.step(1.0, 1.0); // 1 A for 30 min
        }
        assert!((b.soc() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn rc_pair_rises_with_its_time_constant() {
        let rc = RcPair {
            resistance: 0.01,
            capacitance: 1000.0,
        }; // τ = 10 s, V_rc(∞) = 0.5 V at 50 A
        let mut b = Battery::new(1, 1e9, flat(4.0), 0.0, Some(rc)).unwrap();
        for _ in 0..1000 {
            b.step(50.0, 0.01); // 10 s = one time constant
        }
        let expected = 4.0 - 0.5 * (1.0 - (-1.0f64).exp());
        assert!((b.terminal_voltage(50.0) - expected).abs() < 1e-12);
    }

    #[test]
    fn depletion_is_reported_and_soc_never_negative() {
        let mut b = Battery::lipo(3, 100.0, 0.01).unwrap(); // 360 A·s
        b.step(100.0, 10.0);
        assert!(b.is_depleted());
        assert_eq!(b.soc(), 0.0);
        assert!((b.open_circuit_voltage() - 3.0 * 3.30).abs() < 1e-12);
    }

    #[test]
    fn invalid_batteries_are_rejected() {
        assert!(Battery::new(0, 1.0, flat(4.0), 0.0, None).is_err());
        assert!(Battery::new(1, 0.0, flat(4.0), 0.0, None).is_err());
        assert!(Battery::new(1, 1.0, flat(4.0), -1.0, None).is_err());
    }
}
