//! # nsim-physics-rapier
//!
//! The default NSIM physics backend, built on Rapier (`rapier3d-f64`) with the
//! `enhanced-determinism` feature: results are bit-identical across operating
//! systems and CPUs as long as bodies are created in the same order.

mod convert;

use std::collections::BTreeMap;

use nalgebra::Translation3;
use nsim_core::{Iso3, Quat, SimDuration, Twist, Vec3, Wrench};
use nsim_physics::{
    BodyDesc, BodyId, BodyKind, BodyState, Capabilities, ColliderDesc, PhysicsBackend,
    PhysicsError, Shape,
};
use rapier3d_f64::prelude::{
    ColliderBuilder, MassProperties as RapierMass, PhysicsWorld, RigidBodyBuilder, RigidBodyHandle,
    SharedShape,
};

use convert::{pose_from_rapier, pose_to_rapier, quat_to_rapier, vec_from_rapier, vec_to_rapier};

/// Density used to derive mass from colliders when no explicit mass is given
/// (water, so derived masses are physically plausible) [kg/m³].
const DEFAULT_DENSITY: f64 = 1000.0;

/// Standard gravity, ENU world frame [m/s²].
const STANDARD_GRAVITY: f64 = 9.80665;

/// Rapier-based physics backend.
pub struct RapierBackend {
    world: PhysicsWorld,
    /// Wrenches to apply during the next step, in body-id order.
    pending: BTreeMap<BodyId, Wrench>,
}

impl Default for RapierBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl RapierBackend {
    /// An empty world with standard gravity pointing down (−z in ENU).
    pub fn new() -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = vec_to_rapier(&Vec3::new(0.0, 0.0, -STANDARD_GRAVITY));
        Self {
            world,
            pending: BTreeMap::new(),
        }
    }

    fn handle(&self, id: BodyId) -> Result<RigidBodyHandle, PhysicsError> {
        let index = (id.0 & 0xFFFF_FFFF) as u32;
        let generation = (id.0 >> 32) as u32;
        let handle = RigidBodyHandle::from_raw_parts(index, generation);
        if self.world.bodies.contains(handle) {
            Ok(handle)
        } else {
            Err(PhysicsError::UnknownBody(id))
        }
    }

    fn id_of(handle: RigidBodyHandle) -> BodyId {
        let (index, generation) = handle.into_raw_parts();
        BodyId(u64::from(index) | (u64::from(generation) << 32))
    }
}

fn invalid(msg: impl Into<String>) -> PhysicsError {
    PhysicsError::InvalidInput(msg.into())
}

fn check_positive(name: &str, value: f64) -> Result<(), PhysicsError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(invalid(format!(
            "{name} must be positive and finite, got {value}"
        )))
    }
}

fn check_pose(pose: &Iso3) -> Result<(), PhysicsError> {
    let finite = pose.translation.vector.iter().all(|v| v.is_finite())
        && pose.rotation.coords.iter().all(|v| v.is_finite());
    if finite {
        Ok(())
    } else {
        Err(invalid("pose contains NaN or infinity"))
    }
}

/// Builds a Rapier collider. Rapier cylinders are y-aligned; NSIM's are
/// z-aligned, so their local pose is rotated +90° about x (y → z).
fn build_collider(c: &ColliderDesc, kind: BodyKind) -> Result<ColliderBuilder, PhysicsError> {
    check_pose(&c.pose)?;
    let mut local = c.pose;
    let shape = match &c.shape {
        Shape::Cuboid { half_extents } => {
            for (axis, v) in ["x", "y", "z"].iter().zip(half_extents.iter()) {
                check_positive(&format!("cuboid half extent {axis}"), *v)?;
            }
            SharedShape::cuboid(half_extents.x, half_extents.y, half_extents.z)
        }
        Shape::Ball { radius } => {
            check_positive("ball radius", *radius)?;
            SharedShape::ball(*radius)
        }
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            check_positive("cylinder half height", *half_height)?;
            check_positive("cylinder radius", *radius)?;
            let y_to_z = Quat::from_axis_angle(&Vec3::x_axis(), std::f64::consts::FRAC_PI_2);
            local = Iso3::from_parts(local.translation, local.rotation * y_to_z);
            SharedShape::cylinder(*half_height, *radius)
        }
        Shape::Capsule {
            half_height,
            radius,
        } => {
            check_positive("capsule half height", *half_height)?;
            check_positive("capsule radius", *radius)?;
            SharedShape::capsule_z(*half_height, *radius)
        }
        Shape::HalfSpace { normal } => {
            if kind != BodyKind::Fixed {
                return Err(invalid("a half-space can only belong to a fixed body"));
            }
            let n = normal.try_normalize(1e-12).ok_or_else(|| {
                invalid(format!(
                    "half-space normal must be non-zero, got {normal:?}"
                ))
            })?;
            SharedShape::halfspace(vec_to_rapier(&n))
        }
    };
    if !(c.friction.is_finite() && c.friction >= 0.0) {
        return Err(invalid(format!(
            "friction must be >= 0, got {}",
            c.friction
        )));
    }
    if !(c.restitution.is_finite() && (0.0..=1.0).contains(&c.restitution)) {
        return Err(invalid(format!(
            "restitution must be in [0, 1], got {}",
            c.restitution
        )));
    }
    Ok(ColliderBuilder::new(shape)
        .position(pose_to_rapier(&local))
        .friction(c.friction)
        .restitution(c.restitution))
}

impl PhysicsBackend for RapierBackend {
    fn name(&self) -> &'static str {
        "rapier"
    }

    fn capabilities(&self) -> Capabilities {
        Capabilities {
            save_state: true,
            cross_platform_deterministic: true,
        }
    }

    fn set_gravity(&mut self, gravity: Vec3) {
        self.world.gravity = vec_to_rapier(&gravity);
    }

    fn gravity(&self) -> Vec3 {
        vec_from_rapier(self.world.gravity)
    }

    fn add_body(&mut self, desc: &BodyDesc) -> Result<BodyId, PhysicsError> {
        check_pose(&desc.pose)?;
        if desc.kind == BodyKind::Dynamic && desc.mass.is_none() && desc.colliders.is_empty() {
            return Err(invalid(
                "a dynamic body needs explicit mass properties or at least one collider",
            ));
        }
        // Validate everything before touching the world, so a failed call
        // leaves no partial body behind.
        let colliders = desc
            .colliders
            .iter()
            .map(|c| build_collider(c, desc.kind))
            .collect::<Result<Vec<_>, _>>()?;

        let mut builder = match desc.kind {
            BodyKind::Dynamic => RigidBodyBuilder::dynamic(),
            BodyKind::Fixed => RigidBodyBuilder::fixed(),
            BodyKind::Kinematic => RigidBodyBuilder::kinematic_position_based(),
        }
        .pose(pose_to_rapier(&desc.pose))
        .linvel(vec_to_rapier(&desc.twist.linear))
        .angvel(vec_to_rapier(&desc.twist.angular))
        .ccd_enabled(desc.ccd);

        let density = match &desc.mass {
            Some(m) => {
                m.validate()?;
                builder =
                    builder.additional_mass_properties(RapierMass::with_principal_inertia_frame(
                        vec_to_rapier(&m.center_of_mass),
                        m.mass,
                        vec_to_rapier(&m.principal_inertia),
                        quat_to_rapier(&m.inertia_frame),
                    ));
                0.0 // explicit mass: colliders contribute shape only
            }
            None => DEFAULT_DENSITY,
        };

        let handle = self.world.bodies.insert(builder.build());
        for collider in colliders {
            self.world.colliders.insert_with_parent(
                collider.density(density).build(),
                handle,
                &mut self.world.bodies,
            );
        }
        // Rapier only refreshes world-space mass properties during a step, so
        // until then `center_of_mass()` reports the origin. Force models that
        // compute lever arms before the first step need the real value.
        let w = &mut self.world;
        w.bodies[handle].recompute_mass_properties_from_colliders(&w.colliders);
        Ok(Self::id_of(handle))
    }

    fn remove_body(&mut self, id: BodyId) -> Result<(), PhysicsError> {
        let handle = self.handle(id)?;
        let w = &mut self.world;
        w.bodies.remove(
            handle,
            &mut w.islands,
            &mut w.colliders,
            &mut w.impulse_joints,
            &mut w.multibody_joints,
            &mut w.soft_bodies,
            true,
        );
        self.pending.remove(&id);
        Ok(())
    }

    fn body_state(&self, id: BodyId) -> Result<BodyState, PhysicsError> {
        let rb = &self.world.bodies[self.handle(id)?];
        Ok(BodyState {
            pose: pose_from_rapier(rb.position()),
            twist: Twist {
                linear: vec_from_rapier(rb.linvel()),
                angular: vec_from_rapier(rb.angvel()),
            },
            center_of_mass: vec_from_rapier(rb.center_of_mass()),
        })
    }

    fn set_body_pose(&mut self, id: BodyId, pose: Iso3) -> Result<(), PhysicsError> {
        check_pose(&pose)?;
        let handle = self.handle(id)?;
        self.world.bodies[handle].set_position(pose_to_rapier(&pose), true);
        Ok(())
    }

    fn set_body_twist(&mut self, id: BodyId, twist: Twist) -> Result<(), PhysicsError> {
        let finite = twist
            .linear
            .iter()
            .chain(twist.angular.iter())
            .all(|v| v.is_finite());
        if !finite {
            return Err(invalid("twist contains NaN or infinity"));
        }
        let handle = self.handle(id)?;
        let rb = &mut self.world.bodies[handle];
        rb.set_linvel(vec_to_rapier(&twist.linear), true);
        rb.set_angvel(vec_to_rapier(&twist.angular), true);
        Ok(())
    }

    fn add_wrench(&mut self, id: BodyId, wrench: Wrench) -> Result<(), PhysicsError> {
        if !wrench.is_finite() {
            return Err(invalid("wrench contains NaN or infinity"));
        }
        self.handle(id)?;
        *self.pending.entry(id).or_insert(Wrench::ZERO) += wrench;
        Ok(())
    }

    fn step(&mut self, dt: SimDuration) -> Result<(), PhysicsError> {
        if dt.as_nanos() <= 0 {
            return Err(invalid(format!(
                "step size must be positive, got {} ns",
                dt.as_nanos()
            )));
        }
        let pending = std::mem::take(&mut self.pending);
        let mut applied = Vec::with_capacity(pending.len());
        for (id, w) in &pending {
            let handle = self.handle(*id)?;
            let rb = &mut self.world.bodies[handle];
            rb.add_force(vec_to_rapier(&w.force), true);
            rb.add_torque(vec_to_rapier(&w.torque), true);
            applied.push(handle);
        }
        self.world.integration_parameters.dt = dt.as_secs_f64();
        self.world.step();
        // Rapier keeps user forces until reset; NSIM wrenches last one step.
        for handle in applied {
            if let Some(rb) = self.world.bodies.get_mut(handle) {
                rb.reset_forces(false);
                rb.reset_torques(false);
            }
        }
        Ok(())
    }

    fn save_state(&self) -> Result<Vec<u8>, PhysicsError> {
        if !self.pending.is_empty() {
            return Err(PhysicsError::State(
                "cannot save while wrenches are pending; save between steps".into(),
            ));
        }
        postcard::to_stdvec(&self.world).map_err(|e| PhysicsError::State(e.to_string()))
    }

    fn load_state(&mut self, state: &[u8]) -> Result<(), PhysicsError> {
        let world: PhysicsWorld =
            postcard::from_bytes(state).map_err(|e| PhysicsError::State(e.to_string()))?;
        self.world = world;
        self.pending.clear();
        Ok(())
    }
}

/// Convenience: an identity pose translated to `(x, y, z)`.
pub fn at(x: f64, y: f64, z: f64) -> Iso3 {
    Iso3::from_parts(Translation3::new(x, y, z), Quat::identity())
}
