# RocketSim Utils

Workspace of single-purpose, faster-than-RocketSim re-implementations of
Rocket League physics. Each crate copies the RocketSim logic it needs rather
than wrapping it, so **every `rocketsim` release requires a manual sync**
(see `.agents/skills/update-rocketsim/`).

## Crates

- `ball_sim` — 1 ball, no cars/pads. Needs `collision_meshes/`. Has its own
  bullet dispatcher/world copies plus planning APIs (`clear_persistent_manifolds`).
- `ball_sim_cpp` — C++ bindings over `ball_sim` (cxx). No direct `rocketsim` dep.
- `drive_sim` — 1 car on the floor with boost pads, no ball. Floor-only
  simplified vehicle model (`[bool; 4]` wheel contacts, always grounded,
  `Arena` hardcodes Octane). No demos, no dynamic bodies.
- `air_sim` — 1 airborne car, no ball/pads/ground. No wheels or ground contact.
- `turn_sim` — pure in-air rotation. No state beyond rot/ang-vel.

`rocketsim` appears only in `[dev-dependencies]` for parity tests.
`Cargo.lock` is gitignored. `collision_meshes/` is gitignored but required at
test runtime for `ball_sim` and RS-side parity setup.

## Commands

```sh
cargo test --workspace
cargo clippy --workspace --all-targets
cargo +nightly fmt --check
```

`rustfmt.toml` uses nightly-only options (`imports_granularity`,
`group_imports`), so stable `cargo fmt` silently skips them — always use
`cargo +nightly fmt`.

## Conventions

- Match RocketSim **verbatim** (including `f32` op order and rounding) over
  local simplification — these are bit-parity ports, not re-derivations.
- Parity tests compare against `rocketsim` with tolerances (`POS_TOL`,
  `VEL_TOL`); prefer round inputs there and cover fractional/edge inputs in
  throwaway harnesses, not the repo.
- Scratch work (crate downloads, external check binaries) goes in `/tmp`,
  never in the repo.
- `drive_sim` sticky force uses raw throttle; vehicle friction input is
  unconditional `boost → 1.0`. `Vec3` vs `Vec3A` in control APIs must track
  upstream (`pyr()`).
