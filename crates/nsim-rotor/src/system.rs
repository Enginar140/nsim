//! Rotors attached to rigid bodies, and the system that turns them into
//! forces each step.

use nsim_atmosphere::LocalAir;
use nsim_core::{Vec3, Wrench};
use nsim_world::{CenterOfMass, ExternalWrench, Name, Pose, StepContext, System, Velocity, World};

use crate::propeller::{Propeller, PropellerOutput, ground_effect_factor};

/// Sense of rotation about the thrust axis (right-hand rule).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Spin {
    /// Counter-clockwise about the thrust axis (looking against it, from
    /// where the thrust points). The body feels a clockwise reaction torque.
    Ccw,
    /// Clockwise about the thrust axis.
    Cw,
}

impl Spin {
    /// +1 for CCW, −1 for CW.
    pub fn sign(self) -> f64 {
        match self {
            Self::Ccw => 1.0,
            Self::Cw => -1.0,
        }
    }
}

/// One propeller or rotor mounted on a body.
#[derive(Debug, Clone, PartialEq)]
pub struct Rotor {
    /// Name used in error messages (e.g. "m1").
    pub name: String,
    /// Aerodynamic model.
    pub propeller: Propeller,
    /// Hub position in the body frame [m].
    pub position: Vec3,
    /// Thrust direction in the body frame (unit vector).
    pub axis: Vec3,
    /// Rotation sense about `axis`.
    pub spin: Spin,
    /// Rotational speed [rad/s], ≥ 0. Set by a motor model or controller.
    pub omega: f64,
    /// In-plane (H-force) drag coefficient [N·s/(m·rad)]: the hub feels
    /// `−k·ω·v_inplane` (Martin & Salaün). 0 disables it.
    pub inplane_drag: f64,
    /// Apply Cheeseman–Bennett ground effect (flat ground at z = 0).
    pub ground_effect: bool,
    /// Last computed aerodynamic output (for telemetry and motor models).
    pub output: Option<PropellerOutput>,
}

/// All rotors of one body.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Rotors(pub Vec<Rotor>);

/// Computes rotor forces from [`LocalAir`] and adds them to
/// [`ExternalWrench`] during Update (default priority 0, after the
/// atmosphere system).
#[derive(Debug, Default)]
pub struct RotorSystem;

/// Wrench on the body (about its center of mass) produced by one rotor.
fn rotor_wrench(
    rotor: &mut Rotor,
    pose: &Pose,
    velocity: &Velocity,
    com: Vec3,
    air: &LocalAir,
) -> Result<Wrench, String> {
    let rotation = pose.0.rotation;
    let hub = pose.0 * nalgebra::Point3::from(rotor.position);
    let axis = (rotation * rotor.axis)
        .try_normalize(1e-12)
        .ok_or_else(|| format!("rotor '{}': thrust axis is zero", rotor.name))?;
    // Velocity of the hub relative to the air.
    let v_hub = velocity.0.linear + velocity.0.angular.cross(&(hub.coords - com));
    let v_air = v_hub - air.wind;
    let v_axial = v_air.dot(&axis);
    let out = rotor
        .propeller
        .evaluate(rotor.omega, air.density, v_axial)
        .map_err(|e| format!("rotor '{}': {e}", rotor.name))?;
    rotor.output = Some(out);

    let mut thrust = out.thrust;
    if rotor.ground_effect && axis.z > 0.0 {
        thrust *= ground_effect_factor(rotor.propeller.radius(), hub.z);
    }
    let v_inplane = v_air - axis * v_axial;
    let force = axis * thrust - v_inplane * (rotor.inplane_drag * rotor.omega);
    // The air resists the propeller's rotation: reaction torque opposite to
    // the spin, along the axis.
    let reaction = -axis * (rotor.spin.sign() * out.torque);
    let at_hub = Wrench::from_force_at(force, hub.coords, com);
    Ok(Wrench {
        force: at_hub.force,
        torque: at_hub.torque + reaction,
    })
}

impl System for RotorSystem {
    fn name(&self) -> &str {
        "rotors"
    }

    fn update(&mut self, _ctx: &StepContext, world: &mut World) -> Result<(), String> {
        for (name, rotors, pose, vel, com, air, wrench) in world.query_mut::<(
            &Name,
            &mut Rotors,
            &Pose,
            &Velocity,
            &CenterOfMass,
            Option<&LocalAir>,
            &mut ExternalWrench,
        )>() {
            let air = air.ok_or_else(|| {
                format!(
                    "body '{}' has rotors but no LocalAir — add an AtmosphereSystem",
                    name.0
                )
            })?;
            for rotor in &mut rotors.0 {
                wrench.0 += rotor_wrench(rotor, pose, vel, com.0, air)?;
            }
        }
        Ok(())
    }
}
