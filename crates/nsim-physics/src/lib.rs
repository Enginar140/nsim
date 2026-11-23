//! # nsim-physics
//!
//! Engine-independent physics backend interface. A backend owns rigid bodies
//! and their collision shapes, integrates them in fixed steps and can save and
//! restore its complete state.
//!
//! Lessons from Gazebo's gz-physics that shape this API:
//! - **No silent fallbacks.** Anything a backend cannot do is an explicit
//!   [`PhysicsError::Unsupported`], never a logged-and-ignored request.
//! - **Forces are inputs to a step.** Force models (aerodynamics, rotors,
//!   hydrodynamics) live outside the backend and submit [`Wrench`]es with
//!   [`PhysicsBackend::add_wrench`]; they are summed and applied for exactly
//!   one step.
//! - **Deterministic by construction.** Body ids are assigned in creation
//!   order and backends must iterate in a stable order.
//!
//! Conventions: SI units, world frame ENU, body frames FLU (see `nsim-core`).
//! Cylinders and capsules have their axis along the shape's local **z**
//! (as in SDF/URDF); backends with a different native axis convert.

use std::fmt;

use nsim_core::{Iso3, Quat, SimDuration, Twist, Vec3, Wrench};

/// Opaque identifier of a body inside one backend instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(pub u64);

/// How a body moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BodyKind {
    /// Moved by forces, gravity and contacts.
    #[default]
    Dynamic,
    /// Never moves (ground, static structures).
    Fixed,
    /// Moved only by explicitly setting its pose or velocity.
    Kinematic,
}

/// Collision shape geometry, in the collider's local frame.
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// Box given by its half extents along x, y, z [m].
    Cuboid {
        /// Half extents [m].
        half_extents: Vec3,
    },
    /// Sphere [m].
    Ball {
        /// Radius [m].
        radius: f64,
    },
    /// Cylinder with its axis along local z [m].
    Cylinder {
        /// Half of the length along z [m].
        half_height: f64,
        /// Radius [m].
        radius: f64,
    },
    /// Capsule (cylinder with hemispherical caps) along local z [m].
    Capsule {
        /// Half of the length of the cylindrical part along z [m].
        half_height: f64,
        /// Radius [m].
        radius: f64,
    },
    /// Infinite plane; everything on the side opposite to `normal` is solid.
    HalfSpace {
        /// Outward normal in the collider frame (normalized by the backend).
        normal: Vec3,
    },
}

/// One collision shape attached to a body.
#[derive(Debug, Clone, PartialEq)]
pub struct ColliderDesc {
    /// Geometry.
    pub shape: Shape,
    /// Pose of the shape relative to the body frame.
    pub pose: Iso3,
    /// Coulomb friction coefficient.
    pub friction: f64,
    /// Restitution (bounciness), 0 = no bounce, 1 = perfectly elastic.
    pub restitution: f64,
}

impl ColliderDesc {
    /// A collider at the body origin with friction 0.5 and no restitution.
    pub fn new(shape: Shape) -> Self {
        Self {
            shape,
            pose: Iso3::identity(),
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

/// Mass properties of a body, in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProperties {
    /// Mass [kg].
    pub mass: f64,
    /// Center of mass in the body frame [m].
    pub center_of_mass: Vec3,
    /// Principal moments of inertia about the center of mass [kg·m²].
    pub principal_inertia: Vec3,
    /// Orientation of the principal inertia axes in the body frame.
    pub inertia_frame: Quat,
}

impl MassProperties {
    /// Mass properties with the principal axes aligned to the body frame.
    pub fn new(mass: f64, center_of_mass: Vec3, principal_inertia: Vec3) -> Self {
        Self {
            mass,
            center_of_mass,
            principal_inertia,
            inertia_frame: Quat::identity(),
        }
    }

    /// Checks physical validity: positive finite mass, non-negative inertia
    /// satisfying the triangle inequality.
    pub fn validate(&self) -> Result<(), PhysicsError> {
        let [a, b, c] = [
            self.principal_inertia.x,
            self.principal_inertia.y,
            self.principal_inertia.z,
        ];
        if !(self.mass.is_finite() && self.mass > 0.0) {
            return Err(PhysicsError::InvalidInput(format!(
                "mass must be positive and finite, got {}",
                self.mass
            )));
        }
        if [a, b, c].iter().any(|v| !v.is_finite() || *v < 0.0) {
            return Err(PhysicsError::InvalidInput(format!(
                "principal inertia must be finite and non-negative, got {:?}",
                [a, b, c]
            )));
        }
        // Relative tolerance: real inertia tensors from CAD round slightly.
        let tol = 1e-9 * (a + b + c);
        if a + b < c - tol || a + c < b - tol || b + c < a - tol {
            return Err(PhysicsError::InvalidInput(format!(
                "principal inertia {:?} violates the triangle inequality",
                [a, b, c]
            )));
        }
        Ok(())
    }
}

/// Everything needed to create a body.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyDesc {
    /// Motion type.
    pub kind: BodyKind,
    /// Initial pose of the body frame in the world.
    pub pose: Iso3,
    /// Initial velocity (center of mass, world frame).
    pub twist: Twist,
    /// Explicit mass properties. `None` = derive from colliders at
    /// 1000 kg/m³ (only sensible for simple test objects).
    pub mass: Option<MassProperties>,
    /// Collision shapes.
    pub colliders: Vec<ColliderDesc>,
    /// Continuous collision detection for fast, thin objects.
    pub ccd: bool,
}

impl BodyDesc {
    /// A body of the given kind at the given pose, without colliders.
    pub fn new(kind: BodyKind, pose: Iso3) -> Self {
        Self {
            kind,
            pose,
            twist: Twist::default(),
            mass: None,
            colliders: Vec::new(),
            ccd: false,
        }
    }

    /// Adds a collider (builder style).
    pub fn with_collider(mut self, collider: ColliderDesc) -> Self {
        self.colliders.push(collider);
        self
    }

    /// Sets explicit mass properties (builder style).
    pub fn with_mass(mut self, mass: MassProperties) -> Self {
        self.mass = Some(mass);
        self
    }

    /// Sets the initial velocity (builder style).
    pub fn with_twist(mut self, twist: Twist) -> Self {
        self.twist = twist;
        self
    }
}

/// Kinematic state of a body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    /// Pose of the body frame in the world.
    pub pose: Iso3,
    /// Velocity of the center of mass and angular velocity, world frame.
    pub twist: Twist,
    /// Center of mass in world coordinates.
    pub center_of_mass: Vec3,
}

/// Optional features a backend may provide.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Capabilities {
    /// [`PhysicsBackend::save_state`] / [`PhysicsBackend::load_state`] work.
    pub save_state: bool,
    /// Results are bit-identical across operating systems and CPUs.
    pub cross_platform_deterministic: bool,
}

/// Errors reported by a physics backend.
#[derive(Debug, Clone, PartialEq)]
pub enum PhysicsError {
    /// The body id does not exist (never created or already removed).
    UnknownBody(BodyId),
    /// The backend does not implement the requested feature.
    Unsupported(&'static str),
    /// The request contains physically or numerically invalid data.
    InvalidInput(String),
    /// Saving or restoring state failed.
    State(String),
}

impl fmt::Display for PhysicsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownBody(id) => write!(f, "unknown body {}", id.0),
            Self::Unsupported(what) => write!(f, "not supported by this backend: {what}"),
            Self::InvalidInput(msg) => write!(f, "invalid input: {msg}"),
            Self::State(msg) => write!(f, "state error: {msg}"),
        }
    }
}

impl std::error::Error for PhysicsError {}

/// A physics engine behind NSIM.
pub trait PhysicsBackend: Send {
    /// Human-readable backend name, e.g. `"rapier"`.
    fn name(&self) -> &'static str;

    /// Optional features this backend provides.
    fn capabilities(&self) -> Capabilities;

    /// Sets the gravity vector, world frame [m/s²].
    fn set_gravity(&mut self, gravity: Vec3);

    /// Current gravity vector, world frame [m/s²].
    fn gravity(&self) -> Vec3;

    /// Creates a body. Ids are assigned deterministically in creation order.
    fn add_body(&mut self, desc: &BodyDesc) -> Result<BodyId, PhysicsError>;

    /// Removes a body and its colliders.
    fn remove_body(&mut self, id: BodyId) -> Result<(), PhysicsError>;

    /// Current kinematic state of a body.
    fn body_state(&self, id: BodyId) -> Result<BodyState, PhysicsError>;

    /// Teleports a body to a new pose (velocities are kept).
    fn set_body_pose(&mut self, id: BodyId, pose: Iso3) -> Result<(), PhysicsError>;

    /// Sets a body's velocity (center of mass, world frame).
    fn set_body_twist(&mut self, id: BodyId, twist: Twist) -> Result<(), PhysicsError>;

    /// Adds a wrench (world frame, at the center of mass) for the **next step
    /// only**. Multiple calls are summed.
    fn add_wrench(&mut self, id: BodyId, wrench: Wrench) -> Result<(), PhysicsError>;

    /// Advances the simulation by `dt`.
    fn step(&mut self, dt: SimDuration) -> Result<(), PhysicsError>;

    /// Serializes the complete simulation state.
    fn save_state(&self) -> Result<Vec<u8>, PhysicsError> {
        Err(PhysicsError::Unsupported("save_state"))
    }

    /// Restores a state produced by [`PhysicsBackend::save_state`] of the same
    /// backend and version.
    fn load_state(&mut self, _state: &[u8]) -> Result<(), PhysicsError> {
        Err(PhysicsError::Unsupported("load_state"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_mass_properties_pass() {
        let m = MassProperties::new(2.0, Vec3::zeros(), Vec3::new(0.1, 0.2, 0.25));
        assert!(m.validate().is_ok());
    }

    #[test]
    fn non_positive_mass_is_rejected() {
        for mass in [0.0, -1.0, f64::NAN] {
            let m = MassProperties::new(mass, Vec3::zeros(), Vec3::new(1.0, 1.0, 1.0));
            assert!(matches!(m.validate(), Err(PhysicsError::InvalidInput(_))));
        }
    }

    #[test]
    fn inertia_violating_triangle_inequality_is_rejected() {
        let m = MassProperties::new(1.0, Vec3::zeros(), Vec3::new(0.1, 0.1, 0.5));
        assert!(matches!(m.validate(), Err(PhysicsError::InvalidInput(_))));
    }

    #[test]
    fn thin_rod_inertia_is_accepted() {
        // A thin rod along x: Ixx ≈ 0, Iyy = Izz — boundary of the inequality.
        let m = MassProperties::new(1.0, Vec3::zeros(), Vec3::new(0.0, 0.08, 0.08));
        assert!(m.validate().is_ok());
    }

    #[test]
    fn errors_have_readable_messages() {
        assert_eq!(
            PhysicsError::UnknownBody(BodyId(7)).to_string(),
            "unknown body 7"
        );
        assert_eq!(
            PhysicsError::Unsupported("joints").to_string(),
            "not supported by this backend: joints"
        );
    }
}
