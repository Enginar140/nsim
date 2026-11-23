//! U.S. Standard Atmosphere 1976 (identical to ICAO ISA up to 32 km).
//!
//! Altitudes are converted from geometric (what NSIM simulates) to
//! geopotential before evaluating the layer equations. A temperature offset
//! models non-standard days ("ISA+15"): temperature shifts, the
//! pressure–altitude relation stays standard, density follows the gas law.

use std::fmt;

use nsim_core::fmath;

/// Standard gravity [m/s²].
pub const G0: f64 = 9.80665;
/// Specific gas constant of dry air, R*/M (US 1976) [J/(kg·K)].
pub const R_AIR: f64 = 8.314_32 / 0.028_964_4;
/// Ratio of specific heats of air.
pub const GAMMA: f64 = 1.4;
/// Sea-level standard pressure [Pa].
pub const P0: f64 = 101_325.0;
/// Sea-level standard temperature [K].
pub const T0: f64 = 288.15;
/// Effective Earth radius for the geopotential conversion [m].
const EARTH_RADIUS: f64 = 6_356_766.0;

/// Layer bases: (geopotential base altitude [m], lapse rate [K/m]).
const LAYERS: [(f64, f64); 7] = [
    (0.0, -0.0065),
    (11_000.0, 0.0),
    (20_000.0, 0.001),
    (32_000.0, 0.0028),
    (47_000.0, 0.0),
    (51_000.0, -0.0028),
    (71_000.0, -0.002),
];
/// Valid geopotential range of the model [m].
const H_MIN: f64 = -5_000.0;
const H_MAX: f64 = 84_852.0;

/// Atmospheric state at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirState {
    /// Static temperature [K].
    pub temperature: f64,
    /// Static pressure [Pa].
    pub pressure: f64,
    /// Density [kg/m³].
    pub density: f64,
    /// Speed of sound [m/s].
    pub speed_of_sound: f64,
}

/// Altitude outside the model's validity range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OutOfRange {
    /// The requested geometric altitude [m].
    pub altitude: f64,
}

impl fmt::Display for OutOfRange {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "altitude {} m is outside the standard atmosphere model (-5 km to 86 km)",
            self.altitude
        )
    }
}

impl std::error::Error for OutOfRange {}

/// Converts geometric altitude to geopotential altitude [m].
pub fn geopotential(geometric: f64) -> f64 {
    EARTH_RADIUS * geometric / (EARTH_RADIUS + geometric)
}

/// The standard atmosphere, optionally offset in temperature.
#[derive(Debug, Clone, PartialEq)]
pub struct Isa {
    temperature_offset: f64,
    /// (base temperature [K], base pressure [Pa]) per layer, precomputed.
    bases: [(f64, f64); 7],
}

impl Default for Isa {
    fn default() -> Self {
        Self::new(0.0)
    }
}

impl Isa {
    /// Standard atmosphere shifted by `temperature_offset` kelvin
    /// (e.g. `15.0` for an "ISA+15" hot day).
    pub fn new(temperature_offset: f64) -> Self {
        let mut bases = [(T0, P0); 7];
        for i in 1..LAYERS.len() {
            let (h_prev, lapse) = LAYERS[i - 1];
            let (t_prev, p_prev) = bases[i - 1];
            let dh = LAYERS[i].0 - h_prev;
            bases[i] = (
                t_prev + lapse * dh,
                layer_pressure(t_prev, p_prev, lapse, dh),
            );
        }
        Self {
            temperature_offset,
            bases,
        }
    }

    /// Air state at a **geometric** altitude above mean sea level [m].
    pub fn at(&self, altitude: f64) -> Result<AirState, OutOfRange> {
        self.at_geopotential(geopotential(altitude))
            .map_err(|_| OutOfRange { altitude })
    }

    /// Air state at a **geopotential** altitude [m] (as in published tables).
    pub fn at_geopotential(&self, h: f64) -> Result<AirState, OutOfRange> {
        if !(H_MIN..=H_MAX).contains(&h) {
            return Err(OutOfRange { altitude: h });
        }
        // Below sea level the first layer is extrapolated (index 0).
        let i = LAYERS.iter().rposition(|(base, _)| h >= *base).unwrap_or(0);
        let (base, lapse) = LAYERS[i];
        let (tb, pb) = self.bases[i];
        let dh = h - base;
        let standard_t = tb + lapse * dh;
        let pressure = layer_pressure(tb, pb, lapse, dh);
        let temperature = standard_t + self.temperature_offset;
        Ok(AirState {
            temperature,
            pressure,
            density: pressure / (R_AIR * temperature),
            speed_of_sound: (GAMMA * R_AIR * temperature).sqrt(),
        })
    }
}

/// Pressure at `dh` above a layer base with temperature `tb`, pressure `pb`.
fn layer_pressure(tb: f64, pb: f64, lapse: f64, dh: f64) -> f64 {
    if lapse == 0.0 {
        pb * fmath::exp(-G0 * dh / (R_AIR * tb))
    } else {
        pb * fmath::pow(tb / (tb + lapse * dh), G0 / (R_AIR * lapse))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    /// Published US Standard Atmosphere 1976 values at geopotential altitude:
    /// (H [m], T [K], p [Pa], ρ [kg/m³]).
    const TABLE: [(f64, f64, f64, f64); 6] = [
        (0.0, 288.15, 101_325.0, 1.2250),
        (1_000.0, 281.65, 89_874.6, 1.1116),
        (11_000.0, 216.65, 22_632.1, 0.363_92),
        (20_000.0, 216.65, 5_474.89, 0.088_035),
        (32_000.0, 228.65, 868.019, 0.013_225),
        (47_000.0, 270.65, 110.906, 0.001_427_5),
    ];

    #[test]
    fn matches_published_tables() {
        let isa = Isa::default();
        for (h, t, p, rho) in TABLE {
            let s = isa.at_geopotential(h).unwrap();
            assert!(
                (s.temperature - t).abs() < 1e-9,
                "T({h}) = {}",
                s.temperature
            );
            assert!(rel(s.pressure, p) < 1e-5, "p({h}) = {}", s.pressure);
            assert!(rel(s.density, rho) < 2e-4, "rho({h}) = {}", s.density);
        }
    }

    #[test]
    fn sea_level_speed_of_sound() {
        let a = Isa::default().at(0.0).unwrap().speed_of_sound;
        assert!((a - 340.294).abs() < 1e-3, "a0 = {a}");
    }

    #[test]
    fn geometric_altitude_is_slightly_above_geopotential() {
        // 10 km geometric ≈ 9984.3 m geopotential.
        assert!((geopotential(10_000.0) - 9_984.29).abs() < 0.01);
        let isa = Isa::default();
        let geometric = isa.at(10_000.0).unwrap().pressure;
        let geopot = isa.at_geopotential(10_000.0).unwrap().pressure;
        assert!(geometric > geopot);
    }

    #[test]
    fn hot_day_keeps_pressure_and_lowers_density() {
        let std_day = Isa::default().at(1_000.0).unwrap();
        let hot = Isa::new(15.0).at(1_000.0).unwrap();
        assert_eq!(hot.pressure, std_day.pressure);
        assert!((hot.temperature - std_day.temperature - 15.0).abs() < 1e-12);
        let expected = std_day.density * std_day.temperature / hot.temperature;
        assert!(rel(hot.density, expected) < 1e-12);
    }

    #[test]
    fn layers_are_continuous() {
        let isa = Isa::default();
        for (base, _) in LAYERS.iter().skip(1) {
            let below = isa.at_geopotential(base - 1e-6).unwrap();
            let above = isa.at_geopotential(*base).unwrap();
            assert!(
                rel(below.pressure, above.pressure) < 1e-9,
                "jump at {base} m"
            );
            assert!((below.temperature - above.temperature).abs() < 1e-6);
        }
    }

    #[test]
    fn below_sea_level_extrapolates_the_troposphere() {
        let s = Isa::default().at_geopotential(-400.0).unwrap(); // Dead Sea shore
        assert!((s.temperature - (288.15 + 0.0065 * 400.0)).abs() < 1e-9);
        assert!(s.pressure > P0);
    }

    #[test]
    fn out_of_range_is_an_error() {
        assert!(Isa::default().at(100_000.0).is_err());
        assert!(Isa::default().at(-6_000.0).is_err());
    }
}
