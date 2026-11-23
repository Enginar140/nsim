//! Determinism tests: identical runs, save/restore continuity, and a golden
//! hash that CI checks on both Linux and Windows (cross-platform determinism).

use nsim_core::{Iso3, Quat, SimDuration, Vec3};
use nsim_physics::{BodyDesc, BodyId, BodyKind, ColliderDesc, PhysicsBackend, Shape};
use nsim_physics_rapier::RapierBackend;

/// Hash of the final state of the reference scene after [`STEPS`] steps.
/// If this changes, either determinism broke or the physics changed on
/// purpose (e.g. a Rapier upgrade) — in that case update it deliberately.
const GOLDEN_HASH: u64 = 0xd4ee_452c_436c_49df;
const STEPS: usize = 1500;

/// Tiny deterministic generator (no external RNG: its output must never
/// change between versions).
struct Lcg(u64);

impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
}

/// Ground plus 30 mixed shapes dropped into a pile — chaotic enough that any
/// nondeterminism shows up as a different final state.
fn scene() -> (RapierBackend, Vec<BodyId>) {
    let mut b = RapierBackend::new();
    let ground = BodyDesc::new(BodyKind::Fixed, Iso3::identity())
        .with_collider(ColliderDesc::new(Shape::HalfSpace { normal: Vec3::z() }));
    b.add_body(&ground).unwrap();

    let mut rng = Lcg(42);
    let mut ids = Vec::new();
    for i in 0..30 {
        let shape = match i % 3 {
            0 => Shape::Cuboid {
                half_extents: Vec3::new(
                    rng.range(0.05, 0.2),
                    rng.range(0.05, 0.2),
                    rng.range(0.05, 0.2),
                ),
            },
            1 => Shape::Ball {
                radius: rng.range(0.05, 0.15),
            },
            _ => Shape::Cylinder {
                half_height: rng.range(0.05, 0.2),
                radius: rng.range(0.05, 0.15),
            },
        };
        let pose = Iso3::from_parts(
            nalgebra::Translation3::new(
                rng.range(-0.4, 0.4),
                rng.range(-0.4, 0.4),
                0.5 + 0.25 * i as f64,
            ),
            Quat::from_euler_angles(
                rng.range(-3.0, 3.0),
                rng.range(-3.0, 3.0),
                rng.range(-3.0, 3.0),
            ),
        );
        let mut collider = ColliderDesc::new(shape);
        collider.restitution = 0.3;
        ids.push(
            b.add_body(&BodyDesc::new(BodyKind::Dynamic, pose).with_collider(collider))
                .unwrap(),
        );
    }
    (b, ids)
}

fn run(b: &mut RapierBackend, steps: usize) {
    for _ in 0..steps {
        b.step(SimDuration::from_millis(1)).unwrap();
    }
}

/// FNV-1a over the exact bits of every body's pose and velocity.
fn state_hash(b: &RapierBackend, ids: &[BodyId]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for id in ids {
        let s = b.body_state(*id).unwrap();
        let values = s
            .pose
            .translation
            .vector
            .iter()
            .chain(s.pose.rotation.coords.iter())
            .chain(s.twist.linear.iter())
            .chain(s.twist.angular.iter());
        for v in values {
            for byte in v.to_bits().to_le_bytes() {
                h ^= u64::from(byte);
                h = h.wrapping_mul(0x0100_0000_01b3);
            }
        }
    }
    h
}

#[test]
fn identical_runs_produce_identical_states() {
    let (mut a, ids) = scene();
    let (mut b, _) = scene();
    run(&mut a, STEPS);
    run(&mut b, STEPS);
    assert_eq!(state_hash(&a, &ids), state_hash(&b, &ids));
}

#[test]
fn restored_state_continues_identically() {
    let (mut original, ids) = scene();
    run(&mut original, STEPS / 2);
    let snapshot = original.save_state().unwrap();
    run(&mut original, STEPS - STEPS / 2);

    let mut restored = RapierBackend::new();
    restored.load_state(&snapshot).unwrap();
    run(&mut restored, STEPS - STEPS / 2);

    assert_eq!(state_hash(&original, &ids), state_hash(&restored, &ids));
}

#[test]
fn final_state_matches_golden_hash_on_every_platform() {
    let (mut b, ids) = scene();
    run(&mut b, STEPS);
    let hash = state_hash(&b, &ids);
    assert_eq!(
        hash, GOLDEN_HASH,
        "state hash {hash:#018x} differs from golden {GOLDEN_HASH:#018x}"
    );
}

#[test]
fn the_pile_actually_settles() {
    // Guards the test scene itself: if bodies fell through the ground or
    // exploded, the hashes above would be meaningless.
    let (mut b, ids) = scene();
    run(&mut b, STEPS);
    for id in &ids {
        let s = b.body_state(*id).unwrap();
        assert!(
            s.pose.translation.z > 0.0 && s.pose.translation.z < 3.0,
            "{s:?}"
        );
        assert!(s.pose.translation.vector.xy().norm() < 5.0, "{s:?}");
    }
}
