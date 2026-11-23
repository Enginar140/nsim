//! AtmosphereSystem inside a running simulation.

use nsim_atmosphere::{AtmosphereConfig, AtmosphereSystem, Intensity, Isa, LocalAir, MeanWind};
use nsim_core::{SimDuration, Vec3};
use nsim_physics::{BodyDesc, BodyKind, MassProperties};
use nsim_physics_rapier::{RapierBackend, at};
use nsim_world::{Entity, Sim};

fn sim(config: AtmosphereConfig) -> Sim {
    let mut physics = RapierBackend::new();
    nsim_physics::PhysicsBackend::set_gravity(&mut physics, Vec3::zeros());
    let mut s = Sim::new(Box::new(physics), SimDuration::from_millis(10)).unwrap();
    s.add_system(Box::new(AtmosphereSystem::new(config)));
    s
}

fn body(s: &mut Sim, name: &str, z: f64) -> Entity {
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, z)).with_mass(MassProperties::new(
        1.0,
        Vec3::zeros(),
        Vec3::new(0.1, 0.1, 0.1),
    ));
    s.spawn_body(name, &desc).unwrap()
}

fn air(s: &Sim, e: Entity) -> LocalAir {
    *s.world().get::<&LocalAir>(e).expect("LocalAir written")
}

#[test]
fn density_follows_isa_at_body_altitude() {
    let mut s = sim(AtmosphereConfig {
        origin_altitude_msl: 900.0,
        ..Default::default()
    });
    let e = body(&mut s, "uav", 100.0); // 1000 m MSL
    s.step().unwrap();
    let expected = Isa::default().at(1000.0).unwrap();
    assert_eq!(air(&s, e).density, expected.density);
    assert_eq!(air(&s, e).wind, Vec3::zeros());
}

#[test]
fn mean_wind_shear_depends_on_height() {
    let mut s = sim(AtmosphereConfig {
        wind: MeanWind {
            at_reference: Vec3::new(4.0, 0.0, 0.0),
            shear: true,
        },
        ..Default::default()
    });
    let low = body(&mut s, "low", 2.0);
    let high = body(&mut s, "high", 120.0);
    s.step().unwrap();
    assert!(air(&s, low).wind.x < 4.0 && air(&s, high).wind.x > 4.0);
}

fn gust_history(names: &[&str], watch: &str) -> Vec<Vec3> {
    let mut s = sim(AtmosphereConfig {
        turbulence: Some(Intensity::Moderate),
        seed: 99,
        ..Default::default()
    });
    let mut watched = None;
    for n in names {
        let e = body(&mut s, n, 50.0);
        if *n == watch {
            watched = Some(e);
        }
    }
    let e = watched.unwrap();
    (0..200)
        .map(|_| {
            s.step().unwrap();
            air(&s, e).wind
        })
        .collect()
}

#[test]
fn turbulence_is_reproducible_and_isolated_per_vehicle() {
    let alone = gust_history(&["uav-1"], "uav-1");
    assert_eq!(alone, gust_history(&["uav-1"], "uav-1"));
    // Adding another vehicle (before or after) must not change uav-1's gusts.
    assert_eq!(alone, gust_history(&["uav-0", "uav-1", "uav-2"], "uav-1"));
    // Different vehicles get different gusts.
    assert_ne!(alone, gust_history(&["uav-2"], "uav-2"));
    assert!(
        alone.iter().any(|w| w.norm() > 0.1),
        "turbulence produced no gusts"
    );
}
