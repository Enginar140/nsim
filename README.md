# NSIM

[![CI](https://github.com/Enginar140/nsim/actions/workflows/ci.yml/badge.svg)](https://github.com/Enginar140/nsim/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

> **Status: early development. Nothing works end-to-end yet.**

NSIM is an open-source robotics simulator written in Rust, focused on **realistic UAV
physics** and **first-class flight-controller integration**, running natively on
**Windows and Linux**.

## Goals
- **Deterministic.** Integer-nanosecond time, seeded per-world/per-sensor RNG streams,
  stable iteration order: the same seed gives the same flight, bit for bit.
- **Flight physics beyond the usual simplifications.** Table-based fixed-wing aerodynamics
  (CD0 and induced drag, stall, control surfaces), propellers with CT/CQ vs. advance ratio,
  ESC–motor–battery power trains with voltage sag, Dryden turbulence, and sensor noise
  specified as Allan-variance densities.
- **ArduPilot SITL in lockstep** (JSON interface) for fixed-wing and multirotor first;
  PX4 later.
- **Headless core, separate viewer.** The simulation is a library; the Bevy-based viewer,
  CLI and (later) Python/Gymnasium API are clients of it.
- **Swappable physics.** Rapier (default, f64, cross-platform deterministic); MuJoCo, PhysX
  and Jolt as optional backends later. Aero, rotor and hydro models are backend-independent.
- **Models in [VDF](https://github.com/Enginar140/nvdf)**, an open vehicle description
  format with importers for URDF, SDF, MJCF and JSBSim.

## Roadmap (short)
0. Repository skeleton, CI ✓
1. Core: world, scheduler, snapshots, Rapier backend, determinism tests ✓
2. Flight physics ← *next*: atmosphere, rotor, power train, fixed-wing aero, validation
3. Sensors + ArduPilot JSON bridge: ArduCopter and ArduPlane flights
4. Bevy viewer
5. VDF scenes, terrain · 6. Zenoh/ROS 2, Python · 7. Camera/lidar · 8. Marine, ground, arms

Detailed plan (Turkish): [`docs/plan/00_yol_haritasi.md`](docs/plan/00_yol_haritasi.md).
A study of Gazebo's architecture and physics models (Turkish): [`docs/inceleme/`](docs/inceleme/00_OZET.md).

## Try it
```sh
cargo run --release -p nsim-cli -- backends          # physics engines compiled in
cargo run --release -p nsim-cli -- demo --physics rapier
```
The demo drops a pile of blocks next to a body held aloft by a force system and prints a
state hash that is identical on every run and on every operating system.

## Swappable physics
The simulation loop only knows the `PhysicsBackend` trait. Force models (aerodynamics,
rotors, hydrodynamics) never call the engine: they write an `ExternalWrench` component,
so vehicle models work unchanged on any backend. Every backend must pass the shared
**conformance suite** (`nsim-physics-conformance`, 15 checks against analytic results);
optional backends are Cargo features and are selected by name at run time.

## Repository layout
| Path | Contents |
|---|---|
| `crates/nsim-core` | Simulation time, math types, ENU/FLU ↔ NED/FRD frames |
| `crates/nsim-physics` | Engine-independent physics backend trait |
| `crates/nsim-physics-rapier` | Default backend (Rapier, f64, cross-platform deterministic) |
| `crates/nsim-physics-conformance` | Test suite every backend must pass |
| `crates/nsim-world` | Entities, phased scheduler, simulation loop |
| `crates/nsim-cli` | `nsim` command-line runner |
| `docs/` | Plans and research notes |

## Türkçe özet
NSIM; gerçekçi İHA fiziği, ArduPilot ile kilitli adımda SITL ve deterministik simülasyon
hedefleyen, Windows ve Linux'ta yerel çalışan, Rust ile yazılmış açık kaynak bir robotik
simülatörüdür. Ayrıntılı plan ve Gazebo incelemesi `docs/` altında, Türkçedir.

## License
Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in this project by you, as defined in the Apache-2.0 license, shall be dual
licensed as above, without any additional terms or conditions.

NSIM is not affiliated with Open Robotics or the Gazebo project.
