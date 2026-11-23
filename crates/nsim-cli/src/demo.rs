//! `nsim demo`: a headless drop test that exercises the whole stack
//! (scheduler, force system, physics backend) and prints a determinism hash.

use std::time::Instant;

use nalgebra::Translation3;
use nsim_core::{Iso3, Quat, SimDuration, Vec3, Wrench};
use nsim_physics::{BodyDesc, BodyKind, ColliderDesc, MassProperties, PhysicsBackend, Shape};
use nsim_world::{ExternalWrench, Name, Pose, Sim, StepContext, System, Velocity, World};

const G: f64 = 9.80665;

/// Holds the body named "hover" at constant height by cancelling gravity —
/// a stand-in for the force models (rotors, aerodynamics) of later stages.
struct AntiGravity {
    mass: f64,
}

impl System for AntiGravity {
    fn name(&self) -> &str {
        "anti-gravity"
    }
    fn update(&mut self, _: &StepContext, world: &mut World) -> Result<(), String> {
        for (name, w) in world.query_mut::<(&Name, &mut ExternalWrench)>() {
            if name.0 == "hover" {
                w.0 += Wrench::from_force(Vec3::z() * self.mass * G);
            }
        }
        Ok(())
    }
}

fn pose(x: f64, y: f64, z: f64, roll: f64) -> Iso3 {
    Iso3::from_parts(
        Translation3::new(x, y, z),
        Quat::from_euler_angles(roll, 0.3 * roll, 0.0),
    )
}

/// Runs the demo for `seconds` of simulated time and prints a summary.
pub fn run(backend: Box<dyn PhysicsBackend>, seconds: u32) -> Result<(), String> {
    let backend_name = backend.name();
    let dt = SimDuration::from_millis(1);
    let mut sim = Sim::new(backend, dt).map_err(|e| e.to_string())?;

    let ground = BodyDesc::new(BodyKind::Fixed, Iso3::identity())
        .with_collider(ColliderDesc::new(Shape::HalfSpace { normal: Vec3::z() }));
    sim.spawn_body("ground", &ground)
        .map_err(|e| e.to_string())?;
    for i in 0..12 {
        let shape = if i % 2 == 0 {
            Shape::Cuboid {
                half_extents: Vec3::new(0.1, 0.1, 0.1),
            }
        } else {
            Shape::Ball { radius: 0.1 }
        };
        let desc = BodyDesc::new(
            BodyKind::Dynamic,
            pose(
                0.03 * i as f64,
                -0.02 * i as f64,
                0.5 + 0.3 * i as f64,
                0.4 * i as f64,
            ),
        )
        .with_collider(ColliderDesc::new(shape));
        sim.spawn_body(&format!("block-{i}"), &desc)
            .map_err(|e| e.to_string())?;
    }
    let hover_mass = 1.5;
    let hover = BodyDesc::new(BodyKind::Dynamic, pose(3.0, 0.0, 2.0, 0.0)).with_mass(
        MassProperties::new(hover_mass, Vec3::zeros(), Vec3::new(0.02, 0.02, 0.04)),
    );
    sim.spawn_body("hover", &hover).map_err(|e| e.to_string())?;
    sim.add_system(Box::new(AntiGravity { mass: hover_mass }));

    println!("nsim demo — backend: {backend_name}, dt = 1 ms, {seconds} s simulated");
    let started = Instant::now();
    for second in 1..=seconds {
        sim.step_n(1000).map_err(|e| e.to_string())?;
        let (top, hover_z) = summary(sim.world());
        println!("  t = {second:>3} s   highest block z = {top:6.3} m   hover z = {hover_z:.6} m");
    }
    let wall = started.elapsed().as_secs_f64();
    let steps = sim.step_index();
    println!(
        "{steps} steps in {wall:.2} s wall time ({:.0} steps/s, real-time factor {:.1}x)",
        steps as f64 / wall,
        sim.time().as_secs_f64() / wall
    );
    println!(
        "state hash: {:#018x}  (same on every run and every OS)",
        state_hash(sim.world())
    );
    Ok(())
}

fn summary(world: &World) -> (f64, f64) {
    let mut top = f64::MIN;
    let mut hover = f64::NAN;
    for (name, pose) in world.query::<(&Name, &Pose)>().iter() {
        let z = pose.0.translation.z;
        if name.0 == "hover" {
            hover = z;
        } else if name.0.starts_with("block-") {
            top = top.max(z);
        }
    }
    (top, hover)
}

/// FNV-1a over the exact bits of all poses and velocities, in name order.
fn state_hash(world: &World) -> u64 {
    let mut rows: Vec<(String, [f64; 13])> = world
        .query::<(&Name, &Pose, &Velocity)>()
        .iter()
        .map(|(n, p, v)| {
            let t = p.0.translation.vector;
            let q = p.0.rotation.coords;
            let (l, a) = (v.0.linear, v.0.angular);
            let values = [
                t.x, t.y, t.z, q.x, q.y, q.z, q.w, l.x, l.y, l.z, a.x, a.y, a.z,
            ];
            (n.0.clone(), values)
        })
        .collect();
    rows.sort_by(|a, b| a.0.cmp(&b.0));
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for (_, values) in rows {
        for byte in values.iter().flat_map(|v| v.to_bits().to_le_bytes()) {
            h ^= u64::from(byte);
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
    }
    h
}
