//! # nsim-physics-conformance
//!
//! The behavioural contract of [`PhysicsBackend`]. Every backend (Rapier,
//! MuJoCo, PhysX, Jolt, …) must pass this suite, which is what makes backends
//! interchangeable: a model that flies on one flies the same way on another,
//! within the stated tolerances.
//!
//! Each check compares against an analytic result. Use the
//! [`physics_conformance_tests!`] macro in a backend crate's tests:
//!
//! ```ignore
//! nsim_physics_conformance::physics_conformance_tests!(|| {
//!     Box::new(my_backend::MyBackend::new())
//! });
//! ```

use nalgebra::Translation3;
use nsim_core::{Iso3, Quat, SimDuration, Twist, Vec3, Wrench};
use nsim_physics::{
    BodyDesc, BodyId, BodyKind, ColliderDesc, MassProperties, PhysicsBackend, PhysicsError, Shape,
};

/// Creates a fresh backend for each check.
pub type Factory = dyn Fn() -> Box<dyn PhysicsBackend>;

const G: f64 = 9.80665;

fn dt() -> SimDuration {
    SimDuration::from_millis(1)
}

fn at(x: f64, y: f64, z: f64) -> Iso3 {
    Iso3::from_parts(Translation3::new(x, y, z), Quat::identity())
}

/// 2 kg body with diagonal inertia (0.1, 0.2, 0.3) and no colliders.
fn floating_body(b: &mut dyn PhysicsBackend, z: f64) -> BodyId {
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, z)).with_mass(MassProperties::new(
        2.0,
        Vec3::zeros(),
        Vec3::new(0.1, 0.2, 0.3),
    ));
    b.add_body(&desc).expect("add floating body")
}

fn ground(b: &mut dyn PhysicsBackend) {
    let desc = BodyDesc::new(BodyKind::Fixed, Iso3::identity())
        .with_collider(ColliderDesc::new(Shape::HalfSpace { normal: Vec3::z() }));
    b.add_body(&desc).expect("add ground");
}

fn run(b: &mut dyn PhysicsBackend, steps: usize) {
    for _ in 0..steps {
        b.step(dt()).expect("step");
    }
}

/// Default gravity is standard gravity pointing down (−z, ENU).
pub fn default_gravity_is_standard_down(make: &Factory) {
    let b = make();
    let g = b.gravity();
    assert!(
        (g - Vec3::new(0.0, 0.0, -G)).norm() < 1e-12,
        "gravity = {g:?}"
    );
}

/// After 1 s of free fall: v = −g·t exactly, z within the error of a
/// first-order integrator (g·dt·t/2 ≈ 4.9 mm at 1 kHz).
pub fn free_fall_matches_kinematics(make: &Factory) {
    let mut b = make();
    let id = floating_body(b.as_mut(), 100.0);
    run(b.as_mut(), 1000);
    let s = b.body_state(id).unwrap();
    assert!(
        (s.twist.linear.z + G).abs() < 1e-9,
        "vz = {}",
        s.twist.linear.z
    );
    let z_exact = 100.0 - 0.5 * G;
    assert!(
        (s.pose.translation.z - z_exact).abs() < 6e-3,
        "z = {}",
        s.pose.translation.z
    );
}

/// A wrench acts for exactly one step: Δv = F/m·dt, then no more.
pub fn wrench_lasts_one_step(make: &Factory) {
    let mut b = make();
    b.set_gravity(Vec3::zeros());
    let id = floating_body(b.as_mut(), 0.0);
    b.add_wrench(id, Wrench::from_force(Vec3::x() * 20.0))
        .unwrap();
    b.step(dt()).unwrap();
    let v1 = b.body_state(id).unwrap().twist.linear.x;
    run(b.as_mut(), 10);
    let v2 = b.body_state(id).unwrap().twist.linear.x;
    assert!((v1 - 0.01).abs() < 1e-12, "v1 = {v1}");
    assert!((v1 - v2).abs() < 1e-15, "force persisted: {v1} -> {v2}");
}

/// Wrenches submitted for the same step are summed.
pub fn wrenches_are_summed(make: &Factory) {
    let mut b = make();
    b.set_gravity(Vec3::zeros());
    let id = floating_body(b.as_mut(), 0.0);
    b.add_wrench(id, Wrench::from_force(Vec3::x() * 10.0))
        .unwrap();
    b.add_wrench(id, Wrench::from_force(Vec3::x() * 10.0))
        .unwrap();
    b.step(dt()).unwrap();
    let v = b.body_state(id).unwrap().twist.linear.x;
    assert!((v - 0.01).abs() < 1e-12, "v = {v}");
}

/// An upward force equal to the weight holds a body still for 2 s.
pub fn weight_force_hovers(make: &Factory) {
    let mut b = make();
    let id = floating_body(b.as_mut(), 10.0);
    for _ in 0..2000 {
        b.add_wrench(id, Wrench::from_force(Vec3::z() * 2.0 * G))
            .unwrap();
        b.step(dt()).unwrap();
    }
    let s = b.body_state(id).unwrap();
    assert!(
        (s.pose.translation.z - 10.0).abs() < 1e-9,
        "z = {}",
        s.pose.translation.z
    );
    assert!(s.twist.linear.norm() < 1e-9);
}

/// Torque about a principal axis: Δω = τ/I·dt, no cross-axis coupling.
pub fn torque_gives_tau_over_inertia(make: &Factory) {
    let mut b = make();
    b.set_gravity(Vec3::zeros());
    let id = floating_body(b.as_mut(), 0.0);
    let w = Wrench {
        force: Vec3::zeros(),
        torque: Vec3::z() * 0.6,
    };
    b.add_wrench(id, w).unwrap();
    b.step(dt()).unwrap();
    let omega = b.body_state(id).unwrap().twist.angular;
    assert!((omega.z - 0.002).abs() < 1e-12, "ω = {omega:?}");
    assert!(
        omega.x.abs() < 1e-15 && omega.y.abs() < 1e-15,
        "ω = {omega:?}"
    );
}

/// An off-center center of mass: a force through the body origin also
/// produces rotation (r × F about the CoM).
pub fn offset_center_of_mass_is_respected(make: &Factory) {
    let mut b = make();
    b.set_gravity(Vec3::zeros());
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0)).with_mass(MassProperties::new(
        1.0,
        Vec3::new(0.0, 0.0, -0.5),
        Vec3::new(0.1, 0.1, 0.1),
    ));
    let id = b.add_body(&desc).unwrap();
    let s = b.body_state(id).unwrap();
    assert!((s.center_of_mass - Vec3::new(0.0, 0.0, -0.5)).norm() < 1e-12);
    // Force +x at the body origin, 0.5 m above the CoM: τ = (0,0,0.5)×(1,0,0) = (0,0.5,0).
    let wrench = Wrench::from_force_at(Vec3::x(), Vec3::zeros(), s.center_of_mass);
    b.add_wrench(id, wrench).unwrap();
    b.step(dt()).unwrap();
    let omega = b.body_state(id).unwrap().twist.angular;
    assert!((omega.y - 0.5 / 0.1 * 0.001).abs() < 1e-12, "ω = {omega:?}");
}

/// A ball dropped on the ground comes to rest at z = radius.
pub fn ball_rests_on_ground(make: &Factory) {
    let mut b = make();
    ground(b.as_mut());
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 1.0))
        .with_collider(ColliderDesc::new(Shape::Ball { radius: 0.1 }));
    let ball = b.add_body(&desc).unwrap();
    run(b.as_mut(), 3000);
    let z = b.body_state(ball).unwrap().pose.translation.z;
    assert!((z - 0.1).abs() < 5e-3, "z = {z}");
}

/// Cylinders are z-aligned: an upright one rests with its center at z = h/2.
pub fn cylinder_axis_is_local_z(make: &Factory) {
    let mut b = make();
    ground(b.as_mut());
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.6)).with_collider(
        ColliderDesc::new(Shape::Cylinder {
            half_height: 0.5,
            radius: 0.1,
        }),
    );
    let cyl = b.add_body(&desc).unwrap();
    run(b.as_mut(), 2000);
    let s = b.body_state(cyl).unwrap();
    assert!(
        (s.pose.translation.z - 0.5).abs() < 5e-3,
        "z = {}",
        s.pose.translation.z
    );
    assert!(s.pose.rotation.angle() < 1e-3, "tipped over");
}

/// Mass derived from colliders uses water density (0.1 m cube = 1 kg).
pub fn derived_mass_uses_water_density(make: &Factory) {
    let mut b = make();
    let desc = BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 5.0)).with_collider(
        ColliderDesc::new(Shape::Cuboid {
            half_extents: Vec3::new(0.05, 0.05, 0.05),
        }),
    );
    let id = b.add_body(&desc).unwrap();
    for _ in 0..100 {
        b.add_wrench(id, Wrench::from_force(Vec3::z() * G)).unwrap();
        b.step(dt()).unwrap();
    }
    let v = b.body_state(id).unwrap().twist.linear.norm();
    assert!(v < 1e-9, "|v| = {v}");
}

/// Initial velocity is used; teleporting sets the exact pose.
pub fn initial_twist_and_teleport(make: &Factory) {
    let mut b = make();
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
    run(b.as_mut(), 1000);
    let x = b.body_state(id).unwrap().pose.translation.x;
    assert!((x - 3.0).abs() < 1e-9, "x = {x}");
    let target = Iso3::from_parts(
        Translation3::new(-1.0, 2.0, 3.0),
        Quat::from_euler_angles(0.1, 0.2, 0.3),
    );
    b.set_body_pose(id, target).unwrap();
    let got = b.body_state(id).unwrap().pose;
    assert!((got.translation.vector - target.translation.vector).norm() < 1e-12);
    assert!(got.rotation.angle_to(&target.rotation) < 1e-12);
}

/// Invalid requests are explicit errors and never leave partial state.
pub fn invalid_requests_are_errors(make: &Factory) {
    let mut b = make();
    let cases = [
        // Half-space on a dynamic body.
        BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0))
            .with_collider(ColliderDesc::new(Shape::HalfSpace { normal: Vec3::z() })),
        // Dynamic body without mass or colliders.
        BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0)),
        // Negative radius.
        BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, 0.0))
            .with_collider(ColliderDesc::new(Shape::Ball { radius: -1.0 })),
    ];
    for desc in &cases {
        assert!(
            matches!(b.add_body(desc), Err(PhysicsError::InvalidInput(_))),
            "accepted invalid body: {desc:?}"
        );
    }
    let id = floating_body(b.as_mut(), 0.0);
    let nan = Wrench::from_force(Vec3::new(f64::NAN, 0.0, 0.0));
    assert!(matches!(
        b.add_wrench(id, nan),
        Err(PhysicsError::InvalidInput(_))
    ));
    assert!(matches!(
        b.step(SimDuration::ZERO),
        Err(PhysicsError::InvalidInput(_))
    ));
}

/// A removed body is unknown afterwards.
pub fn removed_body_is_unknown(make: &Factory) {
    let mut b = make();
    let id = floating_body(b.as_mut(), 0.0);
    b.remove_body(id).unwrap();
    assert_eq!(b.body_state(id), Err(PhysicsError::UnknownBody(id)));
    assert_eq!(b.remove_body(id), Err(PhysicsError::UnknownBody(id)));
}

/// Two identical runs of a contact-rich scene end in bit-identical states.
pub fn identical_runs_are_identical(make: &Factory) {
    let final_states = |b: &mut dyn PhysicsBackend| {
        ground(b);
        let ids: Vec<BodyId> = (0..8)
            .map(|i| {
                let pose = Iso3::from_parts(
                    Translation3::new(0.05 * i as f64, 0.0, 0.3 + 0.25 * i as f64),
                    Quat::from_euler_angles(0.3 * i as f64, 0.2, 0.1),
                );
                let collider = ColliderDesc::new(Shape::Cuboid {
                    half_extents: Vec3::new(0.1, 0.08, 0.06),
                });
                b.add_body(&BodyDesc::new(BodyKind::Dynamic, pose).with_collider(collider))
                    .unwrap()
            })
            .collect();
        run(b, 800);
        ids.iter()
            .map(|id| b.body_state(*id).unwrap())
            .collect::<Vec<_>>()
    };
    let a = final_states(make().as_mut());
    let c = final_states(make().as_mut());
    assert_eq!(a, c);
}

/// `capabilities().save_state` is truthful, and a restored state continues
/// exactly like the original.
pub fn save_state_matches_capabilities(make: &Factory) {
    let mut b = make();
    let id = floating_body(b.as_mut(), 10.0);
    run(b.as_mut(), 100);
    if !b.capabilities().save_state {
        assert!(matches!(b.save_state(), Err(PhysicsError::Unsupported(_))));
        return;
    }
    let snap = b.save_state().unwrap();
    run(b.as_mut(), 100);
    let expected = b.body_state(id).unwrap();
    let mut restored = make();
    restored.load_state(&snap).unwrap();
    run(restored.as_mut(), 100);
    assert_eq!(restored.body_state(id).unwrap(), expected);
}

/// Generates one `#[test]` per conformance check for the backend produced by
/// `$factory` (an expression of type `Fn() -> Box<dyn PhysicsBackend>`).
#[macro_export]
macro_rules! physics_conformance_tests {
    ($factory:expr) => {
        $crate::physics_conformance_tests!(@tests $factory;
            default_gravity_is_standard_down,
            free_fall_matches_kinematics,
            wrench_lasts_one_step,
            wrenches_are_summed,
            weight_force_hovers,
            torque_gives_tau_over_inertia,
            offset_center_of_mass_is_respected,
            ball_rests_on_ground,
            cylinder_axis_is_local_z,
            derived_mass_uses_water_density,
            initial_twist_and_teleport,
            invalid_requests_are_errors,
            removed_body_is_unknown,
            identical_runs_are_identical,
            save_state_matches_capabilities,
        );
    };
    (@tests $factory:expr; $($check:ident),+ $(,)?) => {
        $(
            #[test]
            fn $check() {
                $crate::$check(&$factory);
            }
        )+
    };
}
