//! Rotors attached to rigid bodies, how they are driven, and the system that
//! turns them into forces each step.

use nsim_atmosphere::LocalAir;
use nsim_core::{Vec3, Wrench};
use nsim_powertrain::{Battery, BldcMotor, FirstOrderMotor};
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

/// How a rotor's speed is produced — the level of detail is the user's
/// choice (see the "progressive detail" principle).
#[derive(Debug, Clone, PartialEq)]
pub enum Drive {
    /// `Rotor::omega` is set directly by a controller or test.
    Direct,
    /// Throttle × max speed with a first-order lag; no battery needed.
    Simple(FirstOrderMotor),
    /// Brushless motor driven by the ESC duty (= throttle) from the body's
    /// [`PowerBus`]; battery sag lowers speed and thrust.
    Electric(BldcMotor),
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
    /// How the rotor speed is produced.
    pub drive: Drive,
    /// Throttle / ESC command ∈ [0, 1], set by a controller or autopilot
    /// bridge (ignored by [`Drive::Direct`]).
    pub throttle: f64,
    /// Rotational speed [rad/s], ≥ 0. Output of the drive (input for
    /// [`Drive::Direct`]).
    pub omega: f64,
    /// In-plane (H-force) drag coefficient [N·s/(m·rad)]: the hub feels
    /// `−k·ω·v_inplane` (Martin & Salaün). 0 disables it.
    pub inplane_drag: f64,
    /// Apply Cheeseman–Bennett ground effect (flat ground at z = 0).
    pub ground_effect: bool,
    /// Last aerodynamic output (telemetry).
    pub output: Option<PropellerOutput>,
    /// Last motor current [A] (electric drive only).
    pub current: f64,
}

impl Rotor {
    /// A directly driven rotor at rest, with ground effect on and no in-plane
    /// drag; adjust the public fields as needed.
    pub fn new(name: &str, propeller: Propeller, position: Vec3, axis: Vec3, spin: Spin) -> Self {
        Self {
            name: name.to_owned(),
            propeller,
            position,
            axis,
            spin,
            drive: Drive::Direct,
            throttle: 0.0,
            omega: 0.0,
            inplane_drag: 0.0,
            ground_effect: true,
            output: None,
            current: 0.0,
        }
    }
}

/// All rotors of one body.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Rotors(pub Vec<Rotor>);

/// The electrical bus of a body: a battery feeding its electric drives.
#[derive(Debug, Clone, PartialEq)]
pub struct PowerBus {
    /// The battery.
    pub battery: Battery,
    /// Terminal voltage at the end of the last step [V].
    pub voltage: f64,
    /// Total current drawn during the last step [A].
    pub current: f64,
}

impl PowerBus {
    /// A bus fed by `battery`, initially unloaded.
    pub fn new(battery: Battery) -> Self {
        let voltage = battery.terminal_voltage(0.0);
        Self {
            battery,
            voltage,
            current: 0.0,
        }
    }
}

/// Computes rotor forces from [`LocalAir`], advances the drives and the
/// battery, and adds the result to [`ExternalWrench`] during Update
/// (default priority 0, after the atmosphere system).
#[derive(Debug, Default)]
pub struct RotorSystem;

/// Aerodynamic force (no reaction torque yet) of one rotor, as a wrench
/// about the center of mass, plus the propeller output.
fn aero_wrench(
    rotor: &Rotor,
    pose: &Pose,
    velocity: &Velocity,
    com: Vec3,
    air: &LocalAir,
) -> Result<(Wrench, Vec3, PropellerOutput), String> {
    let hub = pose.0 * nalgebra::Point3::from(rotor.position);
    let axis = (pose.0.rotation * rotor.axis)
        .try_normalize(1e-12)
        .ok_or_else(|| format!("rotor '{}': thrust axis is zero", rotor.name))?;
    let v_hub = velocity.0.linear + velocity.0.angular.cross(&(hub.coords - com));
    let v_air = v_hub - air.wind;
    let v_axial = v_air.dot(&axis);
    let out = rotor
        .propeller
        .evaluate(rotor.omega, air.density, v_axial)
        .map_err(|e| format!("rotor '{}': {e}", rotor.name))?;
    let mut thrust = out.thrust;
    if rotor.ground_effect && axis.z > 0.0 {
        thrust *= ground_effect_factor(rotor.propeller.radius(), hub.z);
    }
    let v_inplane = v_air - axis * v_axial;
    let force = axis * thrust - v_inplane * (rotor.inplane_drag * rotor.omega);
    Ok((Wrench::from_force_at(force, hub.coords, com), axis, out))
}

/// Advances the rotor's drive by `dt`. Returns the torque the drive exerts
/// on the rotor (the airframe feels the opposite) and the bus current.
fn advance_drive(
    rotor: &mut Rotor,
    aero_torque: f64,
    bus_voltage: Option<f64>,
    dt: f64,
) -> Result<(f64, f64), String> {
    match &rotor.drive {
        Drive::Direct => Ok((aero_torque, 0.0)),
        Drive::Simple(motor) => {
            rotor.omega = motor.step(rotor.omega, rotor.throttle, dt);
            Ok((aero_torque, 0.0))
        }
        Drive::Electric(motor) => {
            let v = bus_voltage.ok_or_else(|| {
                format!(
                    "rotor '{}' has an electric drive but its body has no PowerBus",
                    rotor.name
                )
            })?;
            let s = motor.step(rotor.omega, rotor.throttle, v, aero_torque, dt);
            rotor.omega = s.omega;
            rotor.current = s.current;
            Ok((s.torque, s.current))
        }
    }
}

impl System for RotorSystem {
    fn name(&self) -> &str {
        "rotors"
    }

    fn update(&mut self, ctx: &StepContext, world: &mut World) -> Result<(), String> {
        let dt = ctx.dt.as_secs_f64();
        for (name, rotors, pose, vel, com, air, bus, wrench) in world.query_mut::<(
            &Name,
            &mut Rotors,
            &Pose,
            &Velocity,
            &CenterOfMass,
            Option<&LocalAir>,
            Option<&mut PowerBus>,
            &mut ExternalWrench,
        )>() {
            let air = air.ok_or_else(|| {
                format!(
                    "body '{}' has rotors but no LocalAir — add an AtmosphereSystem",
                    name.0
                )
            })?;
            let bus_voltage = bus.as_ref().map(|b| b.voltage);
            let mut total_current = 0.0;
            for rotor in &mut rotors.0 {
                let (aero, axis, out) = aero_wrench(rotor, pose, vel, com.0, air)?;
                rotor.output = Some(out);
                let (drive_torque, current) = advance_drive(rotor, out.torque, bus_voltage, dt)?;
                total_current += current;
                // Newton's third law: the airframe feels the opposite of the
                // torque driving the rotor about its axis.
                let reaction = -axis * (rotor.spin.sign() * drive_torque);
                wrench.0 += Wrench {
                    force: aero.force,
                    torque: aero.torque + reaction,
                };
            }
            if let Some(bus) = bus {
                bus.battery.step(total_current, dt);
                bus.current = total_current;
                bus.voltage = bus.battery.terminal_voltage(total_current);
            }
        }
        Ok(())
    }
}
