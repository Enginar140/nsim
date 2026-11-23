//! Built-in components.

use nsim_core::{Iso3, Twist, Wrench};
use nsim_physics::BodyId;

/// Human-readable entity name (unique names are not enforced).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Name(pub String);

/// Links an entity to a rigid body in the physics backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Body(pub BodyId);

/// Pose of the body frame in the world (ENU). Written by the physics phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose(pub Iso3);

/// Velocity of the center of mass and angular velocity, world frame.
/// Written by the physics phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity(pub Twist);

/// Sum of the forces and torques that force models want applied during the
/// current step (world frame, at the center of mass). Cleared after each
/// physics step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ExternalWrench(pub Wrench);
