//! Rapier must pass the shared physics conformance suite.

nsim_physics_conformance::physics_conformance_tests!(|| {
    Box::new(nsim_physics_rapier::RapierBackend::new()) as Box<dyn nsim_physics::PhysicsBackend>
});
