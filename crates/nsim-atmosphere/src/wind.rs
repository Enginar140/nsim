//! Mean wind with a logarithmic boundary-layer profile (MIL-F-8785C):
//! `W(h) = W20 · ln(h / z0) / ln(20 ft / z0)` with roughness length
//! `z0 = 0.15 ft`, valid from `z0` to 1000 ft; constant above 1000 ft and
//! zero at or below `z0`.

use nsim_core::{Vec3, fmath};

const FT: f64 = 0.3048;
/// Reference height of the wind specification: 20 ft [m].
pub const REFERENCE_HEIGHT: f64 = 20.0 * FT;
const Z0: f64 = 0.15 * FT;
const TOP: f64 = 1000.0 * FT;

/// Steady wind field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeanWind {
    /// Wind velocity at the reference height (20 ft), ENU [m/s]. This is the
    /// direction the air moves **to** (a "north wind" blowing south has a
    /// negative y component).
    pub at_reference: Vec3,
    /// Apply the logarithmic shear profile. If `false`, the wind is the same
    /// at every height.
    pub shear: bool,
}

impl Default for MeanWind {
    fn default() -> Self {
        Self::calm()
    }
}

impl MeanWind {
    /// No wind.
    pub fn calm() -> Self {
        Self {
            at_reference: Vec3::zeros(),
            shear: false,
        }
    }

    /// Wind velocity at `altitude_agl` [m], ENU [m/s].
    pub fn at(&self, altitude_agl: f64) -> Vec3 {
        if !self.shear {
            return self.at_reference;
        }
        let h = altitude_agl.min(TOP);
        if h <= Z0 {
            return Vec3::zeros();
        }
        self.at_reference * (fmath::ln(h / Z0) / fmath::ln(REFERENCE_HEIGHT / Z0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wind() -> MeanWind {
        MeanWind {
            at_reference: Vec3::new(5.0, 0.0, 0.0),
            shear: true,
        }
    }

    #[test]
    fn reference_height_gives_reference_wind() {
        assert!((wind().at(REFERENCE_HEIGHT) - Vec3::new(5.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn wind_grows_with_height_and_saturates_at_1000_ft() {
        let w = wind();
        assert!(w.at(2.0).x < w.at(10.0).x && w.at(10.0).x < w.at(100.0).x);
        assert_eq!(w.at(TOP), w.at(2000.0));
        // ln(1000/0.15) / ln(20/0.15) = 8.8049 / 4.8929 = 1.7995…
        assert!((w.at(TOP).x / 5.0 - 1.799_5).abs() < 1e-4);
    }

    #[test]
    fn no_wind_at_the_ground() {
        assert_eq!(wind().at(0.0), Vec3::zeros());
        assert_eq!(wind().at(-1.0), Vec3::zeros());
    }

    #[test]
    fn without_shear_wind_is_uniform() {
        let w = MeanWind {
            shear: false,
            ..wind()
        };
        assert_eq!(w.at(0.0), w.at(500.0));
    }
}
