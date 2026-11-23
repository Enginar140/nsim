//! Electric and simple drives in a running simulation.

use nsim_atmosphere::{AtmosphereConfig, AtmosphereSystem};
use nsim_core::{Extrapolation, SimDuration, Table1D, Vec3};
use nsim_physics::{BodyDesc, BodyKind, MassProperties, PhysicsBackend};
use nsim_physics_rapier::{RapierBackend, at};
use nsim_powertrain::{Battery, BldcMotor, FirstOrderMotor, kv_from_rpm_per_volt};
use nsim_rotor::{Drive, PowerBus, Propeller, Rotor, RotorSystem, Rotors, Spin};
use nsim_world::{Entity, Sim, Velocity};

fn prop() -> Propeller {
    let js = vec![0.0, 0.6];
    Propeller::new(
        0.254,
        Table1D::new(js.clone(), vec![0.11, 0.0], Extrapolation::Linear).unwrap(),
        Table1D::new(js, vec![0.0075, 0.0025], Extrapolation::Linear).unwrap(),
    )
    .unwrap()
}

fn motor() -> BldcMotor {
    BldcMotor {
        kv: kv_from_rpm_per_volt(920.0),
        resistance: 0.12,
        no_load_current: 0.5,
        inertia: 2.5e-5,
    }
}

fn rotor(name: &str, x: f64, y: f64, spin: Spin, drive: Drive, throttle: f64) -> Rotor {
    let mut r = Rotor::new(name, prop(), Vec3::new(x, y, 0.0), Vec3::z(), spin);
    r.drive = drive;
    r.throttle = throttle;
    r.ground_effect = false;
    r
}

fn sim(gravity: Vec3) -> Sim {
    let mut physics = RapierBackend::new();
    physics.set_gravity(gravity);
    let mut s = Sim::new(Box::new(physics), SimDuration::from_millis(1)).unwrap();
    s.add_system(Box::new(AtmosphereSystem::new(AtmosphereConfig::default())));
    s.add_system(Box::new(RotorSystem));
    s
}

/// A quad bolted to a thrust stand (fixed body), electric drives, 3S 300 mAh.
fn thrust_stand(throttle: f64) -> (Sim, Entity) {
    let mut s = sim(Vec3::new(0.0, 0.0, -9.80665));
    let e = s
        .spawn_body("stand", &BodyDesc::new(BodyKind::Fixed, at(0.0, 0.0, 1.0)))
        .unwrap();
    let k = 0.16;
    let d = || Drive::Electric(motor());
    let rotors = Rotors(vec![
        rotor("m1", k, -k, Spin::Ccw, d(), throttle),
        rotor("m2", -k, k, Spin::Ccw, d(), throttle),
        rotor("m3", k, k, Spin::Cw, d(), throttle),
        rotor("m4", -k, -k, Spin::Cw, d(), throttle),
    ]);
    let bus = PowerBus::new(Battery::lipo(3, 300.0, 0.008).unwrap());
    s.world_mut().insert(e, (rotors, bus)).unwrap();
    (s, e)
}

#[test]
fn battery_sag_lowers_rotor_speed_and_charge_is_conserved() {
    let (mut s, e) = thrust_stand(0.6);
    let mut charge_drawn = 0.0; // A·s, integrated from the bus current
    let sample = |s: &Sim| -> (PowerBus, f64) {
        let bus = (*s.world().get::<&PowerBus>(e).unwrap()).clone();
        let omega = s.world().get::<&Rotors>(e).unwrap().0[0].omega;
        (bus, omega)
    };
    let mut early = None;
    for step in 1..=30_000 {
        s.step().unwrap();
        let (bus, omega) = sample(&s);
        charge_drawn += bus.current * 0.001;
        if step == 1000 {
            early = Some((bus.voltage, omega));
        }
    }
    let (v_early, w_early) = early.unwrap();
    let (bus, w_late) = sample(&s);
    assert!(
        bus.voltage < v_early - 0.3,
        "voltage {v_early} → {}",
        bus.voltage
    );
    assert!(w_late < w_early * 0.97, "ω {w_early} → {w_late}");
    // Coulomb counting: SoC dropped exactly by the charge drawn.
    let capacity = 300.0 * 3.6;
    assert!(
        ((1.0 - bus.battery.soc()) - charge_drawn / capacity).abs() < 1e-9,
        "SoC {} vs drawn {charge_drawn} A·s",
        bus.battery.soc()
    );
    assert!(bus.current > 4.0, "total current {} A", bus.current);
}

#[test]
fn spin_up_kicks_the_airframe_in_yaw() {
    // Only the two CCW rotors run. While they accelerate, the motor torque
    // (inertia + aero) exceeds the steady aero torque, so the airframe's yaw
    // acceleration is larger at start-up than later.
    let mut s = sim(Vec3::zeros());
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 10.0)).with_mass(MassProperties::new(
        1.5,
        Vec3::zeros(),
        Vec3::new(0.0123, 0.0123, 0.0224),
    ));
    let e = s.spawn_body("quad", &desc).unwrap();
    let k = 0.16;
    let d = || Drive::Electric(motor());
    let rotors = Rotors(vec![
        rotor("m1", k, -k, Spin::Ccw, d(), 0.5),
        rotor("m2", -k, k, Spin::Ccw, d(), 0.5),
    ]);
    let bus = PowerBus::new(Battery::lipo(3, 5000.0, 0.005).unwrap());
    s.world_mut().insert(e, (rotors, bus)).unwrap();

    let yaw_rate = |s: &Sim| s.world().get::<&Velocity>(e).unwrap().0.angular.z;
    let accel_at = |s: &mut Sim| {
        let before = yaw_rate(s);
        s.step().unwrap();
        (yaw_rate(s) - before) / 0.001
    };
    s.step_n(5).unwrap();
    let start_up = accel_at(&mut s);
    s.step_n(2000).unwrap();
    let steady = accel_at(&mut s);
    // CCW rotors → clockwise (negative) yaw.
    assert!(start_up < 0.0 && steady < 0.0, "{start_up} {steady}");
    assert!(
        start_up.abs() > 2.0 * steady.abs(),
        "start-up {start_up} vs steady {steady}"
    );
}

#[test]
fn simple_drive_needs_no_battery() {
    let mut s = sim(Vec3::zeros());
    let e = s
        .spawn_body("stand", &BodyDesc::new(BodyKind::Fixed, at(0.0, 0.0, 1.0)))
        .unwrap();
    let m = FirstOrderMotor {
        max_omega: 1500.0,
        tau_up: 0.05,
        tau_down: 0.08,
    };
    let rotors = Rotors(vec![rotor(
        "m1",
        0.0,
        0.0,
        Spin::Ccw,
        Drive::Simple(m),
        0.4,
    )]);
    s.world_mut().insert_one(e, rotors).unwrap();
    s.step_n(1000).unwrap();
    let r = &s.world().get::<&Rotors>(e).unwrap().0[0];
    assert!((r.omega - 600.0).abs() < 1e-3, "ω = {}", r.omega);
    assert!(r.output.unwrap().thrust > 0.0);
}

#[test]
fn electric_drive_without_bus_is_a_clear_error() {
    let mut s = sim(Vec3::zeros());
    let e = s
        .spawn_body("stand", &BodyDesc::new(BodyKind::Fixed, at(0.0, 0.0, 1.0)))
        .unwrap();
    let rotors = Rotors(vec![rotor(
        "m1",
        0.0,
        0.0,
        Spin::Ccw,
        Drive::Electric(motor()),
        0.5,
    )]);
    s.world_mut().insert_one(e, rotors).unwrap();
    let err = s.step().unwrap_err().to_string();
    assert!(err.contains("no PowerBus"), "{err}");
}
