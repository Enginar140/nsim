# Contributing

Thanks for your interest! The project is at an early design stage, so the best
contributions right now are **issues and design discussion**.

## Before opening a pull request
```sh
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```
CI runs the same checks on Linux and Windows, plus a build on the minimum supported
Rust version (1.89).

## Conventions
- Commit messages: `<type>: <description>` where type is one of
  `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `perf`, `ci`.
- New behaviour comes with tests. Physics models come with validation tests (analytic solutions, JSBSim or real flight logs)
  and determinism tests where relevant.
- No silent fallbacks: unsupported input is an explicit error or a reported loss.

## License
By contributing you agree that your contributions are dual licensed under
MIT OR Apache-2.0, as described in the README.
