//! Physics backends compiled into this binary, selectable by name.

use nsim_physics::PhysicsBackend;

/// A backend that can be created by name.
pub struct BackendInfo {
    /// Name used on the command line and in world files.
    pub name: &'static str,
    /// One-line description.
    pub description: &'static str,
    create: fn() -> Box<dyn PhysicsBackend>,
}

/// All backends enabled by Cargo features, default first.
// Each push is behind its own Cargo feature, so the list cannot be a literal.
#[allow(clippy::vec_init_then_push, unused_mut)]
pub fn available() -> Vec<BackendInfo> {
    let mut list = Vec::new();
    #[cfg(feature = "rapier")]
    list.push(BackendInfo {
        name: "rapier",
        description: "Rapier 0.36 (f64), cross-platform deterministic — default",
        create: || Box::new(nsim_physics_rapier::RapierBackend::new()),
    });
    list
}

/// Creates the backend called `name`, or explains which ones exist.
pub fn create(name: &str) -> Result<Box<dyn PhysicsBackend>, String> {
    let list = available();
    match list.iter().find(|b| b.name == name) {
        Some(b) => Ok((b.create)()),
        None => {
            let names: Vec<&str> = list.iter().map(|b| b.name).collect();
            Err(format!(
                "unknown physics backend '{name}'; available: {}",
                if names.is_empty() {
                    "none (built without backend features)".to_owned()
                } else {
                    names.join(", ")
                }
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(feature = "rapier")]
    fn rapier_is_available_and_creatable() {
        assert!(available().iter().any(|b| b.name == "rapier"));
        assert_eq!(create("rapier").unwrap().name(), "rapier");
    }

    #[test]
    fn unknown_backend_lists_alternatives() {
        let err = create("bullet").err().expect("bullet is not a backend");
        assert!(err.starts_with("unknown physics backend 'bullet'"), "{err}");
    }
}
