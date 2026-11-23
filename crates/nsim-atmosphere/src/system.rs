//! The atmosphere as a simulation system: computes [`LocalAir`] for every
//! rigid body each step, for force models (rotors, wings, …) to read.

use std::collections::BTreeMap;

use nsim_core::{RngStream, Vec3};
use nsim_world::{Body, Entity, Name, Pose, StepContext, System, Velocity, World};

use crate::{DrydenTurbulence, Intensity, Isa, MeanWind};

/// Air at a body's position, valid for the current step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalAir {
    /// Density [kg/m³].
    pub density: f64,
    /// Static temperature [K].
    pub temperature: f64,
    /// Static pressure [Pa].
    pub pressure: f64,
    /// Speed of sound [m/s].
    pub speed_of_sound: f64,
    /// Wind velocity (mean + turbulence), ENU [m/s]. A body's velocity
    /// relative to the air is `v − wind`.
    pub wind: Vec3,
}

/// Configuration of the simulated atmosphere.
#[derive(Debug, Clone, PartialEq)]
pub struct AtmosphereConfig {
    /// Standard atmosphere (with optional temperature offset).
    pub isa: Isa,
    /// Mean wind and shear.
    pub wind: MeanWind,
    /// Turbulence severity, or `None` for smooth air.
    pub turbulence: Option<Intensity>,
    /// Altitude above mean sea level of the world origin (z = 0) [m]. The
    /// ground is the plane z = 0 until terrain support exists.
    pub origin_altitude_msl: f64,
    /// World seed for the turbulence random streams.
    pub seed: u64,
}

impl Default for AtmosphereConfig {
    fn default() -> Self {
        Self {
            isa: Isa::default(),
            wind: MeanWind::calm(),
            turbulence: None,
            origin_altitude_msl: 0.0,
            seed: 0,
        }
    }
}

/// Priority: before force models (which use the default 0).
pub const ATMOSPHERE_PRIORITY: i32 = -100;

/// Writes [`LocalAir`] onto every entity with a [`Body`] during Update.
pub struct AtmosphereSystem {
    config: AtmosphereConfig,
    /// Per-vehicle turbulence generators, keyed by entity.
    gusts: BTreeMap<u64, DrydenTurbulence>,
}

impl AtmosphereSystem {
    /// Creates the system.
    pub fn new(config: AtmosphereConfig) -> Self {
        Self {
            config,
            gusts: BTreeMap::new(),
        }
    }

    fn air_for(
        &mut self,
        entity: Entity,
        name: &str,
        position: Vec3,
        velocity: Vec3,
        dt: f64,
    ) -> Result<LocalAir, String> {
        let agl = position.z.max(0.0);
        let state = self
            .config
            .isa
            .at(self.config.origin_altitude_msl + position.z)
            .map_err(|e| format!("body '{name}': {e}"))?;
        let mean = self.config.wind.at(agl);
        let gust = match self.config.turbulence {
            None => Vec3::zeros(),
            Some(intensity) => {
                let seed = self.config.seed;
                let generator = self.gusts.entry(entity.to_bits().get()).or_insert_with(|| {
                    // Stream named after the body: adding vehicles never
                    // changes the gusts of existing ones.
                    DrydenTurbulence::new(
                        intensity,
                        RngStream::new(seed, &format!("dryden/{name}")),
                    )
                });
                let air_relative = velocity - mean;
                let local = generator.step(dt, air_relative.norm(), agl);
                turbulence_to_world(local, air_relative)
            }
        };
        Ok(LocalAir {
            density: state.density,
            temperature: state.temperature,
            pressure: state.pressure,
            speed_of_sound: state.speed_of_sound,
            wind: mean + gust,
        })
    }
}

/// Rotates a gust from the turbulence frame (x along the horizontal flight
/// direction, y left, z up) into ENU. Without horizontal motion the frame is
/// aligned with east.
fn turbulence_to_world(gust: Vec3, air_relative: Vec3) -> Vec3 {
    let horizontal = Vec3::new(air_relative.x, air_relative.y, 0.0);
    let x = horizontal.try_normalize(1e-6).unwrap_or_else(Vec3::x);
    let y = Vec3::z().cross(&x);
    x * gust.x + y * gust.y + Vec3::z() * gust.z
}

impl System for AtmosphereSystem {
    fn name(&self) -> &str {
        "atmosphere"
    }

    fn priority(&self) -> i32 {
        ATMOSPHERE_PRIORITY
    }

    fn update(&mut self, ctx: &StepContext, world: &mut World) -> Result<(), String> {
        let dt = ctx.dt.as_secs_f64();
        let bodies: Vec<(Entity, String, Vec3, Vec3)> = world
            .query::<(Entity, &Body, &Name, &Pose, &Velocity)>()
            .iter()
            .map(|(e, _, n, p, v)| (e, n.0.clone(), p.0.translation.vector, v.0.linear))
            .collect();
        for (entity, name, position, velocity) in bodies {
            let air = self.air_for(entity, &name, position, velocity, dt)?;
            world
                .insert_one(entity, air)
                .map_err(|e| format!("body '{name}': {e}"))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gust_frame_follows_horizontal_flight_direction() {
        // Flying north: longitudinal gust points north, lateral points west.
        let north = Vec3::new(0.0, 20.0, -3.0);
        let g = turbulence_to_world(Vec3::new(1.0, 2.0, 3.0), north);
        assert!((g - Vec3::new(-2.0, 1.0, 3.0)).norm() < 1e-12, "{g:?}");
    }

    #[test]
    fn hover_uses_east_aligned_frame() {
        let g = turbulence_to_world(Vec3::new(1.0, 0.0, 0.0), Vec3::zeros());
        assert_eq!(g, Vec3::x());
    }
}
