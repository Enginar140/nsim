//! Behaviour of the simulation loop and scheduler.

use std::sync::{Arc, Mutex};

use nsim_core::{SimDuration, SimTime, Vec3, Wrench};
use nsim_physics::{BodyDesc, BodyKind, MassProperties};
use nsim_physics_rapier::{RapierBackend, at};
use nsim_world::{ExternalWrench, Name, Pose, Sim, SimError, StepContext, System, Velocity, World};

const G: f64 = 9.80665;

fn sim() -> Sim {
    Sim::new(Box::new(RapierBackend::new()), SimDuration::from_millis(1)).unwrap()
}

fn drone_desc(z: f64) -> BodyDesc {
    BodyDesc::new(BodyKind::Dynamic, at(0.0, 0.0, z)).with_mass(MassProperties::new(
        1.5,
        Vec3::zeros(),
        Vec3::new(0.02, 0.02, 0.04),
    ))
}

/// Adds a constant upward force to the entity with the given name.
struct Thrust {
    target: &'static str,
    newtons: f64,
}

impl System for Thrust {
    fn name(&self) -> &str {
        "thrust"
    }
    fn update(&mut self, _ctx: &StepContext, world: &mut World) -> Result<(), String> {
        for (name, w) in world.query_mut::<(&Name, &mut ExternalWrench)>() {
            if name.0 == self.target {
                w.0 += Wrench::from_force(Vec3::z() * self.newtons);
            }
        }
        Ok(())
    }
}

#[test]
fn update_system_force_reaches_physics() {
    let mut s = sim();
    let e = s.spawn_body("drone", &drone_desc(10.0)).unwrap();
    s.add_system(Box::new(Thrust {
        target: "drone",
        newtons: 1.5 * G,
    }));
    s.step_n(1000).unwrap();
    let pose = s.world().get::<&Pose>(e).unwrap().0;
    let vel = s.world().get::<&Velocity>(e).unwrap().0;
    assert!(
        (pose.translation.z - 10.0).abs() < 1e-9,
        "z = {}",
        pose.translation.z
    );
    assert!(vel.linear.norm() < 1e-9);
    assert_eq!(s.time(), SimTime::ZERO + SimDuration::from_secs(1));
    assert_eq!(s.step_index(), 1000);
}

#[test]
fn wrench_is_cleared_after_each_step() {
    struct Once(bool);
    impl System for Once {
        fn name(&self) -> &str {
            "once"
        }
        fn update(&mut self, _: &StepContext, world: &mut World) -> Result<(), String> {
            if !self.0 {
                for w in world.query_mut::<&mut ExternalWrench>() {
                    w.0 += Wrench::from_force(Vec3::x() * 15.0);
                }
                self.0 = true;
            }
            Ok(())
        }
    }
    let mut s = sim();
    let e = s.spawn_body("b", &drone_desc(100.0)).unwrap();
    s.add_system(Box::new(Once(false)));
    s.step_n(50).unwrap();
    // 15 N on 1.5 kg for one 1 ms step: Δv = 0.01 m/s, never re-applied.
    let vx = s.world().get::<&Velocity>(e).unwrap().0.linear.x;
    assert!((vx - 0.01).abs() < 1e-12, "vx = {vx}");
    assert_eq!(s.world().get::<&ExternalWrench>(e).unwrap().0, Wrench::ZERO);
}

type Log = Arc<Mutex<Vec<String>>>;

struct Recorder {
    label: &'static str,
    priority: i32,
    log: Log,
}

impl System for Recorder {
    fn name(&self) -> &str {
        self.label
    }
    fn priority(&self) -> i32 {
        self.priority
    }
    fn pre_update(&mut self, ctx: &StepContext, _: &mut World) -> Result<(), String> {
        let t = ctx.time.as_nanos();
        self.log
            .lock()
            .unwrap()
            .push(format!("pre:{}@{t}", self.label));
        Ok(())
    }
    fn update(&mut self, ctx: &StepContext, _: &mut World) -> Result<(), String> {
        let t = ctx.time.as_nanos();
        self.log
            .lock()
            .unwrap()
            .push(format!("upd:{}@{t}", self.label));
        Ok(())
    }
    fn post_update(&mut self, ctx: &StepContext, _: &World) -> Result<(), String> {
        let t = ctx.time.as_nanos();
        self.log
            .lock()
            .unwrap()
            .push(format!("post:{}@{t}", self.label));
        Ok(())
    }
}

#[test]
fn phases_run_in_order_and_priority_then_registration_order() {
    let log: Log = Arc::default();
    let mut s = sim();
    for (label, priority) in [("b", 0), ("a", -10), ("c", 0)] {
        s.add_system(Box::new(Recorder {
            label,
            priority,
            log: log.clone(),
        }));
    }
    s.step().unwrap();
    let expected = [
        "pre:a@0",
        "pre:b@0",
        "pre:c@0", //
        "upd:a@0",
        "upd:b@0",
        "upd:c@0", //
        // PostUpdate sees the end-of-step time (1 ms).
        "post:a@1000000",
        "post:b@1000000",
        "post:c@1000000",
    ];
    assert_eq!(*log.lock().unwrap(), expected);
}

#[test]
fn post_update_observes_the_integrated_state() {
    struct Check {
        seen: Arc<Mutex<Option<f64>>>,
    }
    impl System for Check {
        fn name(&self) -> &str {
            "check"
        }
        fn post_update(&mut self, _: &StepContext, world: &World) -> Result<(), String> {
            let mut q = world.query::<&Velocity>();
            let v = q.iter().next().map(|v| v.0.linear.z);
            *self.seen.lock().unwrap() = v;
            Ok(())
        }
    }
    let seen = Arc::default();
    let mut s = sim();
    s.spawn_body("falling", &drone_desc(50.0)).unwrap();
    s.add_system(Box::new(Check {
        seen: Arc::clone(&seen),
    }));
    s.step().unwrap();
    let vz = seen.lock().unwrap().expect("post_update ran");
    assert!((vz + G * 0.001).abs() < 1e-12, "vz = {vz}");
}

#[test]
fn system_errors_name_the_system_and_phase() {
    struct Broken;
    impl System for Broken {
        fn name(&self) -> &str {
            "broken-sensor"
        }
        fn post_update(&mut self, _: &StepContext, _: &World) -> Result<(), String> {
            Err("no data".into())
        }
    }
    let mut s = sim();
    s.add_system(Box::new(Broken));
    let err = s.step().unwrap_err();
    assert_eq!(
        err,
        SimError::System {
            system: "broken-sensor".into(),
            phase: "post_update",
            message: "no data".into(),
        }
    );
    assert_eq!(
        err.to_string(),
        "system 'broken-sensor' failed in post_update: no data"
    );
}

#[test]
fn zero_step_size_is_rejected() {
    assert!(matches!(
        Sim::new(Box::new(RapierBackend::new()), SimDuration::ZERO),
        Err(SimError::World(_))
    ));
}

#[test]
fn snapshot_restore_continues_identically() {
    let mut s = sim();
    let e = s.spawn_body("drone", &drone_desc(10.0)).unwrap();
    s.add_system(Box::new(Thrust {
        target: "drone",
        newtons: 1.2 * 1.5 * G, // climbing
    }));
    s.step_n(300).unwrap();
    let snap = s.snapshot().unwrap();
    s.step_n(300).unwrap();
    let reference = (s.time(), s.world().get::<&Pose>(e).unwrap().0);

    s.restore(&snap).unwrap();
    assert_eq!(s.step_index(), 300);
    s.step_n(300).unwrap();
    let again = (s.time(), s.world().get::<&Pose>(e).unwrap().0);
    assert_eq!(reference, again);
}
