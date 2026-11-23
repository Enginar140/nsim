//! An open-loop quad X in a running simulation: hover, yaw/roll torques,
//! density, advance ratio and ground effect — each with a physically
//! obvious expected outcome.

use std::f64::consts::PI;

use nsim_atmosphere::{AtmosphereConfig, AtmosphereSystem, Isa};
use nsim_core::{Extrapolation, SimDuration, Table1D, Twist, Vec3};
use nsim_physics::{BodyDesc, BodyKind, MassProperties};
use nsim_physics_rapier::{RapierBackend, at};
use nsim_rotor::{Propeller, Rotor, RotorSystem, Rotors, Spin};
use nsim_world::{Entity, Pose, Sim, Velocity};

const MASS: f64 = 1.5;
const G: f64 = 9.80665;
const ARM: f64 = 0.225;
const D: f64 = 0.254;
const CT0: f64 = 0.11;

fn prop() -> Propeller {
    let js = vec![0.0, 0.6];
    Propeller::new(
        D,
        Table1D::new(js.clone(), vec![CT0, 0.0], Extrapolation::Linear).unwrap(),
        Table1D::new(js, vec![0.0075, 0.0025], Extrapolation::Linear).unwrap(),
    )
    .unwrap()
}

/// Rotor speed [rad/s] at which four rotors carry the weight at density ρ.
fn hover_omega(density: f64) -> f64 {
    let thrust_each = MASS * G / 4.0;
    let n = (thrust_each / (CT0 * density * D.powi(4))).sqrt();
    2.0 * PI * n
}

/// ArduCopter Quad X layout in FLU: front-right and back-left spin CCW,
/// front-left and back-right spin CW.
fn quad_rotors(omega: f64, ground_effect: bool) -> Rotors {
    let k = ARM / 2f64.sqrt();
    let layout = [
        ("m1-front-right", Vec3::new(k, -k, 0.0), Spin::Ccw),
        ("m2-back-left", Vec3::new(-k, k, 0.0), Spin::Ccw),
        ("m3-front-left", Vec3::new(k, k, 0.0), Spin::Cw),
        ("m4-back-right", Vec3::new(-k, -k, 0.0), Spin::Cw),
    ];
    Rotors(
        layout
            .into_iter()
            .map(|(name, position, spin)| Rotor {
                name: name.into(),
                propeller: prop(),
                position,
                axis: Vec3::z(),
                spin,
                omega,
                inplane_drag: 0.0,
                ground_effect,
                output: None,
            })
            .collect(),
    )
}

fn quad_sim(origin_msl: f64, z: f64, rotors: Rotors, twist: Twist) -> (Sim, Entity) {
    let mut sim = Sim::new(Box::new(RapierBackend::new()), SimDuration::from_millis(1)).unwrap();
    sim.add_system(Box::new(AtmosphereSystem::new(AtmosphereConfig {
        origin_altitude_msl: origin_msl,
        ..Default::default()
    })));
    sim.add_system(Box::new(RotorSystem));
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, z))
        .with_mass(MassProperties::new(
            MASS,
            Vec3::zeros(),
            Vec3::new(0.0123, 0.0123, 0.0224),
        ))
        .with_twist(twist);
    let e = sim.spawn_body("quad", &desc).unwrap();
    sim.world_mut().insert_one(e, rotors).unwrap();
    (sim, e)
}

fn state(sim: &Sim, e: Entity) -> (Pose, Velocity) {
    (
        *sim.world().get::<&Pose>(e).unwrap(),
        *sim.world().get::<&Velocity>(e).unwrap(),
    )
}

fn sea_level_hover() -> f64 {
    hover_omega(Isa::default().at(50.0).unwrap().density)
}

#[test]
fn hover_rpm_holds_altitude_without_rotating() {
    let (mut sim, e) = quad_sim(
        0.0,
        50.0,
        quad_rotors(sea_level_hover(), false),
        Twist::default(),
    );
    sim.step_n(3000).unwrap();
    let (pose, vel) = state(&sim, e);
    assert!(
        (pose.0.translation.z - 50.0).abs() < 1e-6,
        "z = {}",
        pose.0.translation.z
    );
    assert!(vel.0.linear.norm() < 1e-6, "v = {:?}", vel.0.linear);
    assert!(vel.0.angular.norm() < 1e-9, "ω = {:?}", vel.0.angular);
}

fn with_speeds(f: impl Fn(&str, f64) -> f64) -> Rotors {
    let mut rotors = quad_rotors(sea_level_hover(), false);
    for r in &mut rotors.0 {
        r.omega = f(&r.name, r.omega);
    }
    rotors
}

#[test]
fn faster_ccw_rotors_yaw_the_body_clockwise() {
    // CCW rotors push the body clockwise (negative z, seen from above) —
    // the same mixing ArduCopter uses to yaw right.
    let rotors = with_speeds(|name, w| {
        if name.starts_with("m1") || name.starts_with("m2") {
            w * 1.02
        } else {
            w * 0.98
        }
    });
    let (mut sim, e) = quad_sim(0.0, 50.0, rotors, Twist::default());
    sim.step_n(200).unwrap();
    let w = state(&sim, e).1.0.angular;
    assert!(w.z < -0.05, "yaw rate {w:?}");
    assert!(
        w.x.abs() < 1e-9 && w.y.abs() < 1e-9,
        "unexpected roll/pitch {w:?}"
    );
}

#[test]
fn faster_left_rotors_roll_right() {
    // More thrust on the left (+y) lifts the left side: positive rotation
    // about +x (forward), i.e. right wing down.
    let rotors = with_speeds(|name, w| if name.contains("left") { w * 1.01 } else { w });
    let (mut sim, e) = quad_sim(0.0, 50.0, rotors, Twist::default());
    sim.step_n(100).unwrap();
    let w = state(&sim, e).1.0.angular;
    assert!(w.x > 0.05, "roll rate {w:?}");
}

#[test]
fn thinner_air_at_altitude_means_descent() {
    // Sea-level hover speed at 3000 m MSL: ~26% less density.
    let (mut sim, e) = quad_sim(
        3000.0,
        50.0,
        quad_rotors(sea_level_hover(), false),
        Twist::default(),
    );
    sim.step_n(500).unwrap();
    let (pose, vel) = state(&sim, e);
    assert!(vel.0.linear.z < -1.0, "vz = {}", vel.0.linear.z);
    assert!(pose.0.translation.z < 50.0);
}

#[test]
fn climbing_raises_advance_ratio_and_lowers_thrust() {
    let climbing = Twist {
        linear: Vec3::z() * 5.0,
        angular: Vec3::zeros(),
    };
    let (mut sim, e) = quad_sim(0.0, 50.0, quad_rotors(sea_level_hover(), false), climbing);
    sim.step().unwrap();
    let rotors = sim.world().get::<&Rotors>(e).unwrap().0.clone();
    let out = rotors[0].output.expect("rotor evaluated");
    assert!(out.advance_ratio > 0.05, "J = {}", out.advance_ratio);
    assert!(out.thrust < MASS * G / 4.0, "T = {}", out.thrust);
    drop(rotors);
    // With less than hover thrust the climb decelerates.
    sim.step_n(100).unwrap();
    assert!(state(&sim, e).1.0.linear.z < 5.0);
}

#[test]
fn ground_effect_pushes_up_near_the_ground() {
    let omega = hover_omega(Isa::default().at(0.15).unwrap().density);
    let (mut near, e1) = quad_sim(0.0, 0.15, quad_rotors(omega, true), Twist::default());
    let (mut far, e2) = quad_sim(0.0, 0.15, quad_rotors(omega, false), Twist::default());
    near.step_n(50).unwrap();
    far.step_n(50).unwrap();
    let vz_near = state(&near, e1).1.0.linear.z;
    let vz_far = state(&far, e2).1.0.linear.z;
    assert!(vz_near > 0.01, "vz with ground effect = {vz_near}");
    assert!(vz_far.abs() < 1e-3, "vz without = {vz_far}");
}

#[test]
fn rotors_without_atmosphere_is_a_clear_error() {
    let mut sim = Sim::new(Box::new(RapierBackend::new()), SimDuration::from_millis(1)).unwrap();
    sim.add_system(Box::new(RotorSystem));
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 5.0)).with_mass(MassProperties::new(
        1.0,
        Vec3::zeros(),
        Vec3::new(0.1, 0.1, 0.1),
    ));
    let e = sim.spawn_body("quad", &desc).unwrap();
    sim.world_mut()
        .insert_one(e, quad_rotors(500.0, false))
        .unwrap();
    let err = sim.step().unwrap_err().to_string();
    assert!(err.contains("add an AtmosphereSystem"), "{err}");
}
