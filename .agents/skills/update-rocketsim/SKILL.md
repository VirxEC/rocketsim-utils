---
name: Update RocketSim utils after a RocketSim release
description: Use when a new rocketsim crate version is published. Syncs the physics copies in air_sim, ball_sim, drive_sim, and turn_sim to match the new RocketSim release, then verifies with parity tests.
---

# Update utils after a RocketSim release

The utils (`air_sim`, `ball_sim`, `drive_sim`, `turn_sim`) are intentionally
simplified, faster re-implementations of RocketSim physics, not wrappers.
Every RocketSim release can change behavior, so each util with copied logic
must be diffed and re-synced. `ball_sim_cpp` is FFI over `ball_sim` and has no
direct `rocketsim` dependency — it never needs changes here.

## 1. Find the current pin and the new version

```sh
grep -rn 'rocketsim =' --include="Cargo.toml" . | grep -v target
```

Pins live in the `[dev-dependencies]` of `air_sim`, `ball_sim`, `drive_sim`,
and `turn_sim` (used only by parity tests). `Cargo.lock` is gitignored.

## 2. Diff the upstream crates

Download the old and new `.crate` files from
`https://crates.io/api/v1/crates/rocketsim/<version>/download` into a scratch
dir **outside the repo** (e.g. `/tmp/opencode/rsdiff`), unpack each with
`tar xzf ... --strip-components=1`, then compare:

```sh
diff -rq <old> <new> | grep -v -e Cargo -e README -e cargo_vcs
```

Incremental diffs (`old→new` only) are what matter. Ignore `Cargo.toml`
(version bump), `README.md`, and `.cargo_vcs_info.json`.

## 3. Map changed upstream files to utils

| Upstream file | Util | Notes |
|---|---|---|
| `src/sim/car/base.rs` driving (`update_wheels`, friction input) | `drive_sim/src/sim/car/base.rs` | Single car, always grounded; `wheels_with_contact` is `[bool; 4]` |
| `src/sim/car/base.rs` air torque / flip / air throttle | `air_sim/src/sim/car/base.rs` | No wheels/ground; `update_double_jump_or_flip` is the dodge source |
| `src/sim/car/base.rs` turn torque | `turn_sim/src/lib.rs` | Pure rotation; check its quantize helper against upstream's |
| `src/shared/quantize.rs` input helpers | helpers in `drive_sim` + `air_sim` car `base.rs` | Copy verbatim (see rules below) |
| `src/sim/car/car_controls.rs` | `air_sim/.../car_controls.rs` | Only `pyr()`/`with_pyr()`-style API changes apply; `drive_sim`/`turn_sim` have reduced controls without `pyr` |
| `src/sim/boost_pad/base.rs` | `drive_sim/src/sim/boost_pad/base.rs` | Only util with boost pads |
| `src/sim/ball/base.rs`, `src/sim/arena/base.rs`, `src/sim/phys_state.rs` | `ball_sim/...` | Manifold clear/count APIs, `set_state` clearing, derive changes |
| `src/sim/consts.rs` | each util's `consts` | Utils keep intentional subsets — only port values they actually use |
| `src/bullet/.../vehicle_rl.rs` reorderings | usually N/A | `drive_sim` has a simplified floor-only SIMD vehicle with no ground-stick/pushback/dynamic bodies; `ball_sim` has its own dispatcher |
| demolish/respawn, 3-wheel configs, `is_on_ground` flows | N/A | Utils have no demos, no jumping off ground (`drive_sim`), no air-to-ground transitions (`air_sim`) |

Past examples: 0.2.6 added input quantization (throttle/steer/pyr through
`quantize_axis_inputs`, `pyr(): Vec3→Vec3A`); 0.2.7 replaced suspension consts
with per-body `suspension_strengths()` and reworked dodge-dir computation;
0.2.5 fixed the boost-pad AABB to use `cyl_radius`.

## 4. Port rules

- Copy upstream logic **verbatim**, including doc comments and `f32` operation
  order. Never "simplify" rounding or reorder arithmetic — utils must be
  bit-parity with RocketSim, and 1-ulp differences are real (e.g. per-body
  suspension values like 35.735 vs the old 35.75 const).
- Known invariants (re-verify against the diff, don't assume): sticky force
  keeps **raw** throttle; vehicle friction input stays unconditional
  `boost → 1.0`; dodge deadzone checks stay raw while the dodge vector is
  quantized; `quantize_axis_inputs` (`.round()`) and `quantize_axis_input`
  (`round_ties_even`) are byte-equivalent by construction (the `>> 1` folds
  the tie difference away).
- Distinguish real drift from intentional simplifications: if a util lacks the
  concept entirely (demos, dynamic-body contacts, 3 wheels), it's N/A. If it
  has the concept with copied code, it must match.
- `drive_sim`'s `Arena` hardcodes `CarBodyConfig::OCTANE`; per-body changes
  still apply via the shared formula since inputs (axle offsets, mass) match
  upstream exactly.

## 5. Bump, then verify

```sh
sed -i 's/rocketsim = "<old>"/rocketsim = "<new>"/' air_sim/Cargo.toml turn_sim/Cargo.toml ball_sim/Cargo.toml drive_sim/Cargo.toml
cargo update -p rocketsim
cargo test --workspace
cargo clippy --workspace --all-targets
cargo +nightly fmt --check
```

`rustfmt.toml` uses nightly-only options, so stable `cargo fmt` silently
skips them — always use `cargo +nightly fmt`.

A bump-only run first is diagnostic: failures pinpoint real drift
(e.g. `turning_drive_parity` caught the 0.2.6 steer quantization).

## 6. Prove parity on edge cases with a throwaway harness

Repo parity tests use round inputs that often quantize to themselves. Build a
temporary crate in the scratch dir (never in the repo) depending on the util
by path plus the new `rocketsim`, using **fractional** inputs that exercise
the change (throttle 0.37, steer 0.63, cancelling yaw/roll like `yaw=0.5,
roll=-0.5`, partial-throttle-plus-boost). Harness gotchas:

- RocketSim arenas spawn a ball and `air_sim` has none: park it away with
  `set_ball_state` (e.g. `(0, 5000, 1900)`) or the car will "collide" with it.
- `air_sim` has no ground plane either: keep checks above ground-contact
  altitude (~130 ticks max from z=500) or divergence is a harness artifact.
- `drive_sim` needs no meshes but RS init needs `collision_meshes/`; use the
  absolute repo path since the harness runs from the scratch dir.
- Prove sensitivity: `git stash` the util fix, show the harness fails
  (e.g. vel diff 1.86 at tick 0 for the 0.2.7 dodge change), `git stash pop`,
  show it passes.

Clean up the scratch dir when done. Do not commit harness code.
