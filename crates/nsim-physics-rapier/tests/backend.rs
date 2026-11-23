//! Behavioural tests of the Rapier backend against analytic results.

use nsim_core::{Iso3, Quat, SimDuration, Twist, Vec3, Wrench};
use nsim_physics::{
    BodyDesc, BodyId, BodyKind, ColliderDesc, MassProperties, PhysicsBackend, PhysicsError, Shape,
};
use nsim_physics_rapier::{RapierBackend, at};

const G: f64 = 9.80665;

fn dt() -> SimDuration {
    SimDuration::from_millis(1)
}

/// A 2 kg point-like body with unit-ish inertia, no colliders.
fn floating_body(backend: &mut RapierBackend, z: f64) -> BodyId {
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, z)).with_mass(MassProperties::new(
        2.0,
        Vec3::zeros(),
        Vec3::new(0.1, 0.2, 0.3),
    ));
    backend.add_body(&desc).unwrap()
}

fn ground(backend: &mut RapierBackend) {
    let desc = BodyDesc::new(BodyKind::Fixed, Iso3::identity())
        .with_collider(ColliderDesc::new(Shape::HalfSpace { normal: Vec3::z() }));
    backend.add_body(&desc).unwrap();
}

fn run(backend: &mut RapierBackend, steps: usize) {
    for _ in 0..steps {
        backend.step(dt()).unwrap();
    }
}

#[test]
fn free_fall_matches_kinematics() {
    let mut b = RapierBackend::new();
    let id = floating_body(&mut b, 100.0);
    run(&mut b, 1000); // 1 s
    let s = b.body_state(id).unwrap();
    // Semi-implicit Euler: v exact, z off by g·dt·t/2 ≈ 4.9 mm.
    assert!(
        (s.twist.linear.z + G).abs() < 1e-9,
        "v = {}",
        s.twist.linear.z
    );
    let z_exact = 100.0 - 0.5 * G;
    assert!((s.pose.translation.z - z_exact).abs() < 6e-3);
}

#[test]
fn wrench_applies_for_exactly_one_step() {
    let mut b = RapierBackend::new();
    b.set_gravity(Vec3::zeros());
    let id = floating_body(&mut b, 0.0);
    // 20 N on 2 kg for 1 ms → Δv = 0.01 m/s, then no further acceleration.
    b.add_wrench(id, Wrench::from_force(Vec3::x() * 20.0))
        .unwrap();
    b.step(dt()).unwrap();
    let v1 = b.body_state(id).unwrap().twist.linear.x;
    run(&mut b, 10);
    let v2 = b.body_state(id).unwrap().twist.linear.x;
    assert!((v1 - 0.01).abs() < 1e-12, "v1 = {v1}");
    assert_eq!(v1, v2);
}

#[test]
fn wrenches_submitted_in_one_step_are_summed() {
    let mut b = RapierBackend::new();
    b.set_gravity(Vec3::zeros());
    let id = floating_body(&mut b, 0.0);
    b.add_wrench(id, Wrench::from_force(Vec3::x() * 10.0))
        .unwrap();
    b.add_wrench(id, Wrench::from_force(Vec3::x() * 10.0))
        .unwrap();
    b.step(dt()).unwrap();
    let v = b.body_state(id).unwrap().twist.linear.x;
    assert!((v - 0.01).abs() < 1e-12, "v = {v}");
}

#[test]
fn force_equal_to_weight_hovers() {
    let mut b = RapierBackend::new();
    let id = floating_body(&mut b, 10.0);
    for _ in 0..2000 {
        b.add_wrench(id, Wrench::from_force(Vec3::z() * 2.0 * G))
            .unwrap();
        b.step(dt()).unwrap();
    }
    let s = b.body_state(id).unwrap();
    assert!((s.pose.translation.z - 10.0).abs() < 1e-9);
    assert!(s.twist.linear.norm() < 1e-9);
}

#[test]
fn torque_gives_angular_acceleration_tau_over_inertia() {
    let mut b = RapierBackend::new();
    b.set_gravity(Vec3::zeros());
    let id = floating_body(&mut b, 0.0);
    // Izz = 0.3 kg·m², τ = 0.6 N·m for 1 ms → Δω = 2 rad/s² · 1 ms = 0.002 rad/s.
    b.add_wrench(
        id,
        Wrench {
            force: Vec3::zeros(),
            torque: Vec3::z() * 0.6,
        },
    )
    .unwrap();
    b.step(dt()).unwrap();
    let w = b.body_state(id).unwrap().twist.angular;
    assert!((w.z - 0.002).abs() < 1e-12, "ω = {w:?}");
    assert!(w.x.abs() < 1e-15 && w.y.abs() < 1e-15);
}

#[test]
fn ball_comes_to_rest_on_the_ground() {
    let mut b = RapierBackend::new();
    ground(&mut b);
    let ball = b
        .add_body(
            &BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 1.0))
                .with_collider(ColliderDesc::new(Shape::Ball { radius: 0.1 })),
        )
        .unwrap();
    run(&mut b, 3000);
    let z = b.body_state(ball).unwrap().pose.translation.z;
    assert!((z - 0.1).abs() < 5e-3, "z = {z}");
}

#[test]
fn cylinder_axis_is_local_z() {
    // A tall cylinder (half height 0.5, radius 0.1) standing upright must
    // rest with its center at z = 0.5. If the axis were y it would lie flat
    // or rest at z = 0.1.
    let mut b = RapierBackend::new();
    ground(&mut b);
    let cyl = b
        .add_body(
            &BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.6)).with_collider(ColliderDesc::new(
                Shape::Cylinder {
                    half_height: 0.5,
                    radius: 0.1,
                },
            )),
        )
        .unwrap();
    run(&mut b, 2000);
    let s = b.body_state(cyl).unwrap();
    assert!(
        (s.pose.translation.z - 0.5).abs() < 5e-3,
        "z = {}",
        s.pose.translation.z
    );
    assert!(s.pose.rotation.angle() < 1e-3, "tipped over");
}

#[test]
fn derived_mass_uses_water_density() {
    // 0.1 m cube of water = 1 kg. A 9.80665 N upward force must hover it.
    let mut b = RapierBackend::new();
    let id = b
        .add_body(
            &BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 5.0)).with_collider(ColliderDesc::new(
                Shape::Cuboid {
                    half_extents: Vec3::new(0.05, 0.05, 0.05),
                },
            )),
        )
        .unwrap();
    for _ in 0..100 {
        b.add_wrench(id, Wrench::from_force(Vec3::z() * G)).unwrap();
        b.step(dt()).unwrap();
    }
    assert!(b.body_state(id).unwrap().twist.linear.norm() < 1e-9);
}

#[test]
fn initial_twist_and_teleport_are_respected() {
    let mut b = RapierBackend::new();
    b.set_gravity(Vec3::zeros());
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0))
        .with_mass(MassProperties::new(
            1.0,
            Vec3::zeros(),
            Vec3::new(1.0, 1.0, 1.0),
        ))
        .with_twist(Twist {
            linear: Vec3::x() * 3.0,
            angular: Vec3::zeros(),
        });
    let id = b.add_body(&desc).unwrap();
    run(&mut b, 1000);
    let x = b.body_state(id).unwrap().pose.translation.x;
    assert!((x - 3.0).abs() < 1e-9, "x = {x}");
    let target = Iso3::from_parts(
        nalgebra::Translation3::new(-1.0, 2.0, 3.0),
        Quat::from_euler_angles(0.1, 0.2, 0.3),
    );
    b.set_body_pose(id, target).unwrap();
    assert_eq!(b.body_state(id).unwrap().pose, target);
}

#[test]
fn invalid_requests_are_explicit_errors() {
    let mut b = RapierBackend::new();
    // Half-space on a dynamic body.
    let bad = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0))
        .with_collider(ColliderDesc::new(Shape::HalfSpace { normal: Vec3::z() }));
    assert!(matches!(
        b.add_body(&bad),
        Err(PhysicsError::InvalidInput(_))
    ));
    // Dynamic body without mass or colliders.
    let massless = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0));
    assert!(matches!(
        b.add_body(&massless),
        Err(PhysicsError::InvalidInput(_))
    ));
    // Negative radius.
    let neg = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0))
        .with_collider(ColliderDesc::new(Shape::Ball { radius: -1.0 }));
    assert!(matches!(
        b.add_body(&neg),
        Err(PhysicsError::InvalidInput(_))
    ));
    // NaN wrench.
    let id = floating_body(&mut b, 0.0);
    let nan = Wrench::from_force(Vec3::new(f64::NAN, 0.0, 0.0));
    assert!(matches!(
        b.add_wrench(id, nan),
        Err(PhysicsError::InvalidInput(_))
    ));
    // Zero step.
    assert!(matches!(
        b.step(SimDuration::ZERO),
        Err(PhysicsError::InvalidInput(_))
    ));
    // Failed add_body calls must not have created bodies: only `id` exists.
    assert_eq!(id, BodyId(0));
}

#[test]
fn removed_body_becomes_unknown() {
    let mut b = RapierBackend::new();
    let id = floating_body(&mut b, 0.0);
    b.remove_body(id).unwrap();
    assert_eq!(b.body_state(id), Err(PhysicsError::UnknownBody(id)));
    assert_eq!(b.remove_body(id), Err(PhysicsError::UnknownBody(id)));
}
