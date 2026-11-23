//! The simulation loop.

use std::fmt;

use hecs::{Entity, World};
use nsim_core::{SimDuration, SimTime, Wrench};
use nsim_physics::{BodyDesc, BodyId, PhysicsBackend, PhysicsError};

use crate::components::{Body, ExternalWrench, Name, Pose, Velocity};

/// Timing information passed to systems.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepContext {
    /// Simulation time. During PreUpdate/Update this is the time at the
    /// **start** of the step; during PostUpdate it is the time at the **end**
    /// (the instant the freshly integrated state belongs to).
    pub time: SimTime,
    /// Step size.
    pub dt: SimDuration,
    /// Number of completed steps before this one.
    pub step_index: u64,
}

/// A unit of simulation logic. All methods are optional.
pub trait System: Send {
    /// Name used in error messages.
    fn name(&self) -> &str;

    /// Ordering within a phase: lower runs first; ties keep registration order.
    fn priority(&self) -> i32 {
        0
    }

    /// Controllers and bridges: read inputs, write commands.
    fn pre_update(&mut self, _ctx: &StepContext, _world: &mut World) -> Result<(), String> {
        Ok(())
    }

    /// Force models: add to [`ExternalWrench`] components.
    fn update(&mut self, _ctx: &StepContext, _world: &mut World) -> Result<(), String> {
        Ok(())
    }

    /// Sensors and publishers: observe the new state (read-only).
    fn post_update(&mut self, _ctx: &StepContext, _world: &World) -> Result<(), String> {
        Ok(())
    }
}

/// Errors from the simulation loop.
#[derive(Debug, Clone, PartialEq)]
pub enum SimError {
    /// The physics backend reported an error.
    Physics(PhysicsError),
    /// A system failed.
    System {
        /// Name of the failing system.
        system: String,
        /// Phase in which it failed.
        phase: &'static str,
        /// The system's error message.
        message: String,
    },
    /// The world is inconsistent (e.g. a body component without a pose).
    World(String),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Physics(e) => write!(f, "physics: {e}"),
            Self::System {
                system,
                phase,
                message,
            } => write!(f, "system '{system}' failed in {phase}: {message}"),
            Self::World(msg) => write!(f, "world: {msg}"),
        }
    }
}

impl std::error::Error for SimError {}

impl From<PhysicsError> for SimError {
    fn from(e: PhysicsError) -> Self {
        Self::Physics(e)
    }
}

/// Saved simulation state.
///
/// Captures time and the complete physics state. **Not yet captured:** the
/// internal state of systems and non-physics components; restoring is only
/// exact for worlds whose systems are stateless. (Tracked for a later stage.)
#[derive(Debug, Clone, PartialEq)]
pub struct Snapshot {
    time: SimTime,
    step_index: u64,
    physics: Vec<u8>,
}

struct Registered {
    system: Box<dyn System>,
    order: usize,
}

/// A simulated world: entities, systems and a physics backend, advanced in
/// fixed steps.
pub struct Sim {
    world: World,
    physics: Box<dyn PhysicsBackend>,
    systems: Vec<Registered>,
    time: SimTime,
    dt: SimDuration,
    step_index: u64,
}

impl Sim {
    /// Creates a simulation with the given backend and fixed step size.
    pub fn new(physics: Box<dyn PhysicsBackend>, dt: SimDuration) -> Result<Self, SimError> {
        if dt.as_nanos() <= 0 {
            return Err(SimError::World(format!(
                "step size must be positive, got {} ns",
                dt.as_nanos()
            )));
        }
        Ok(Self {
            world: World::new(),
            physics,
            systems: Vec::new(),
            time: SimTime::ZERO,
            dt,
            step_index: 0,
        })
    }

    /// Current simulation time.
    pub fn time(&self) -> SimTime {
        self.time
    }

    /// Fixed step size.
    pub fn dt(&self) -> SimDuration {
        self.dt
    }

    /// Number of completed steps.
    pub fn step_index(&self) -> u64 {
        self.step_index
    }

    /// The entity world (read-only).
    pub fn world(&self) -> &World {
        &self.world
    }

    /// The entity world, for adding non-physical entities and components.
    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// The physics backend (read-only), e.g. for queries.
    pub fn physics(&self) -> &dyn PhysicsBackend {
        self.physics.as_ref()
    }

    /// Creates a rigid body and an entity carrying [`Name`], [`Body`],
    /// [`Pose`], [`Velocity`] and [`ExternalWrench`].
    pub fn spawn_body(&mut self, name: &str, desc: &BodyDesc) -> Result<Entity, SimError> {
        let id = self.physics.add_body(desc)?;
        let state = self.physics.body_state(id)?;
        Ok(self.world.spawn((
            Name(name.to_owned()),
            Body(id),
            Pose(state.pose),
            Velocity(state.twist),
            ExternalWrench::default(),
        )))
    }

    /// Registers a system. It runs from the next step on.
    pub fn add_system(&mut self, system: Box<dyn System>) {
        let order = self.systems.len();
        self.systems.push(Registered { system, order });
        // Stable: (priority, registration order). Re-sorted on every
        // registration so stepping never allocates or sorts.
        self.systems.sort_by_key(|r| (r.system.priority(), r.order));
    }

    /// Advances the simulation by one step.
    pub fn step(&mut self) -> Result<(), SimError> {
        let ctx = StepContext {
            time: self.time,
            dt: self.dt,
            step_index: self.step_index,
        };
        for r in &mut self.systems {
            r.system
                .pre_update(&ctx, &mut self.world)
                .map_err(|m| system_error(r.system.name(), "pre_update", m))?;
        }
        for r in &mut self.systems {
            r.system
                .update(&ctx, &mut self.world)
                .map_err(|m| system_error(r.system.name(), "update", m))?;
        }
        self.physics_phase()?;

        self.time += self.dt;
        self.step_index += 1;
        let post = StepContext {
            time: self.time,
            ..ctx
        };
        for r in &mut self.systems {
            r.system
                .post_update(&post, &self.world)
                .map_err(|m| system_error(r.system.name(), "post_update", m))?;
        }
        Ok(())
    }

    /// Advances the simulation by `n` steps.
    pub fn step_n(&mut self, n: u64) -> Result<(), SimError> {
        for _ in 0..n {
            self.step()?;
        }
        Ok(())
    }

    /// Hands wrenches to the backend, steps it and writes the new state back.
    /// Bodies are processed in [`BodyId`] order for determinism.
    fn physics_phase(&mut self) -> Result<(), SimError> {
        let mut wrenches: Vec<(BodyId, Wrench)> = self
            .world
            .query_mut::<(&Body, &mut ExternalWrench)>()
            .into_iter()
            .map(|(body, wrench)| (body.0, std::mem::take(&mut wrench.0)))
            .filter(|(_, w)| *w != Wrench::ZERO)
            .collect();
        wrenches.sort_by_key(|(id, _)| *id);
        for (id, w) in wrenches {
            self.physics.add_wrench(id, w)?;
        }

        self.physics.step(self.dt)?;

        for (body, pose, vel) in self.world.query_mut::<(&Body, &mut Pose, &mut Velocity)>() {
            let state = self.physics.body_state(body.0)?;
            pose.0 = state.pose;
            vel.0 = state.twist;
        }
        Ok(())
    }

    /// Saves time and physics state. See [`Snapshot`] for what is captured.
    pub fn snapshot(&self) -> Result<Snapshot, SimError> {
        Ok(Snapshot {
            time: self.time,
            step_index: self.step_index,
            physics: self.physics.save_state()?,
        })
    }

    /// Restores a snapshot taken from this simulation (same bodies and
    /// entities) and refreshes [`Pose`]/[`Velocity`] from physics.
    pub fn restore(&mut self, snapshot: &Snapshot) -> Result<(), SimError> {
        self.physics.load_state(&snapshot.physics)?;
        self.time = snapshot.time;
        self.step_index = snapshot.step_index;
        for (body, pose, vel, wrench) in
            self.world
                .query_mut::<(&Body, &mut Pose, &mut Velocity, &mut ExternalWrench)>()
        {
            let state = self.physics.body_state(body.0)?;
            pose.0 = state.pose;
            vel.0 = state.twist;
            wrench.0 = Wrench::ZERO;
        }
        Ok(())
    }
}

fn system_error(name: &str, phase: &'static str, message: String) -> SimError {
    SimError::System {
        system: name.to_owned(),
        phase,
        message,
    }
}
