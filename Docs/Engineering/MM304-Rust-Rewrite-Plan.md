# MM304 Rust rewrite, Rust backend on main, telemetry — plan

Date: 2026-10-29. Status: proposed.

## Where things stand

- `main` (b3d4df3) flies the C backend in `CLanding/`.
- `origin/rust-landing-backend` (d975804) replaces that backend with `rust/landing`,
  a c2rust 0.22 translation of the C at f1eebdc (13:53 today). It is byte-identical
  to the C build, keeps C symbol names and `#[repr(C)]` layouts, and `Validation/`
  links against its `liblanding.a`.
- That branch lacks these `main` commits: 7c7ee56, f41d6f5 (deorbit HAC-energy
  ranking), e23dd5c (spiral bank unload), 264514b, add4930, b3d4df3 (MM304
  forecast fixes). The MM304 ones are superseded below; the others must be ported.
- Live evidence (equatorial 72 km checkpoint, AOA_CHAIN trace): the current MM304
  is a closed loop through its own forecast. Forecasting at the current command
  made the low-energy cap cycle every 4-7 s; forecasting on the schedule still
  cycles (14 transitions) because the cap is bang-bang between 35 deg (+15 km short)
  and 22 deg (-250 km long). Bank follows AoA. The forecast also used a different
  law (constant bank, constant AoA) from the guidance it predicts.

## Goal

MM304 delivers the orbiter to a physically feasible, energy-sufficient entrance of
MM305's spiral HAC, with predictions that reproduce what the vehicle will actually
fly, and stable commands, observable in telemetry. Verified end to end in native
simulation and in a full live flight.

## Phase A — Rust backend on main, current algorithms

1. Merge `origin/rust-landing-backend` into `main`. The C backend sources go away
   as on the branch; headers in `CLanding/include` stay as the C-ABI contract for
   `Validation/`.
2. Port the non-MM304 C changes since f1eebdc into the translation:
   - e23dd5c spiral bank unload → `taem_planner.rs` (spiral route).
   - 7c7ee56 / f41d6f5 deorbit ranking by HAC handoff energy, handoff-Mach energy
     recording, `taem_hac_energy_*` fields → `predictor.rs`, struct layouts.
   - The MM304 forecast commits are replaced by Phase B, not ported.
3. Gate: `cargo build --release`, `cargo test`, `make -C CLanding test` against
   `liblanding.a`, and `ClosedLoopTrace` MM305/HAC/Final scenarios byte-identical
   to a C build of b3d4df3 (MM304 scenarios excluded: Phase B changes MM304).

## Phase B — new MM304 in idiomatic Rust (`rust/landing/src/mm304/`)

One guidance law, used by live guidance, the predictor, and the deorbit witness.

- `state.rs` — `Mm304State`, a `#[repr(C)]` plain-data struct stored inline in
  `GuidanceMachine` (copied by value, so every prediction gets an independent
  copy; no globals, no thread-local caches, no pointers into live state).
- `law.rs` — per-tick law: plan + measured state → AoA and signed bank commands.
  AoA follows the schedule (40 deg high-Mach plateau, ramp down; the test
  staircase when set) plus the plan's continuous AoA bias. Bank magnitude comes
  from the plan. Roll-rate and roll-acceleration limits, reversal dwell, lateral
  capture, stall, g-load, q, sideslip and authority limits, with the same
  filters live and predicted.
- `predict.rs` — closed-loop predictor: 3-DOF over the rotating sphere, the
  measured-calibrated aero table (live data wins), atmosphere, gravity, a
  first-order attitude response matched to measured roll and pitch dynamics, and
  `law.rs` in the loop on a cloned state. Outputs the trajectory, the terminal
  state at the MM305 admission boundary, and flags: converged, inside the validated
  aero envelope, admission envelope met. A non-converged or out-of-envelope
  forecast is "uncertain", never a success.
- `terminal.rs` — HAC-entrance feasibility: the predicted handoff state is scored
  by MM305's own spiral geometry (entrance, quartic spiral, G2 exit, Final),
  computing the path length and energy loss along the curve, via the native MM305
  functions. Result: energy surplus or deficit (m), geometry admissible yes or no,
  limiting constraint. The configured Mach/altitude admission band is enforced,
  not relaxed.
- `plan.rs` — receding-horizon coupled planner on the worker thread (every
  prediction interval). Decision variables: AoA bias (continuous, from 0 down to
  max-L/D) and bank magnitude (continuous), with the reversal policy from
  `lateral.rs` inside the forecast. Objective, continuous in the physics: an
  asymmetric terminal-energy cost (a deficit costs far more than a surplus,
  because MM305 can shed surplus but cannot make up a deficit), plus terminal
  geometry cost (along, cross, heading against the admission envelope), plus
  authority margin. A 2-D bounded search, warm-started from the active plan.
  Hysteresis is physical: a new plan replaces the active one only if its
  predicted cost improves by more than the cost uncertainty measured from recent
  predicted-vs-actual residuals. No output smoothing. If nothing is feasible,
  report the limiting constraint and fly the energy-preserving extreme (deficit:
  max-L/D wings-level toward the HAC; surplus: max drag and max bank inside the
  limits).
- `lateral.rs` — reversals chosen by predicted terminal cross-track and heading
  under the current bank sign versus reversed, including roll time, measured
  lateral velocity and available lift. No reversal near zero error or during
  centreline capture. No fixed schedules or distance thresholds.
- Integration: the translated `entry_program_guidance` and the worker's MM304
  entry job call into `mm304`. `entry_exec` phases and the MM305 ownership gate
  stay, and the gate requires `terminal.rs` feasibility, so MM304 keeps the
  vehicle if the downstream route is not qualified.

## Phase C — agreement

The deorbit witness flies `mm304::predict`, and the deorbit target, the MM304 plan,
the MM305 admission decision and MM305 execution all use `terminal.rs`'s
requirement.

## Phase D — telemetry

- The backend snapshot gains an `mm304` block: the active plan (AoA bias, bank,
  reversal plan), the decision and its reason (plan kept, replaced or infeasible,
  with the limiting constraint), the predicted trajectory (downsampled), the
  predicted terminal state and feasibility, and predicted-vs-actual residuals
  (position, heading, altitude, speed, energy, AoA, bank, cross-track) at matched
  horizons, split into aero-model and guidance-model error.
- The flight log gains `mm304Plan` and `mm304Residual` records.
- Display: an MM304 panel in `ksl tui` (plan, terminal feasibility, residuals, last
  decisions); WebTelemetry overlays the predicted ground track and
  altitude-vs-range on the actual trajectory, plus a decision timeline.

## Phase E — validation

- Rust tests: prediction determinism and isolation; predicted vs executed
  commands; continuity across heading wrap and runway overflight; stable AoA and
  bank near energy-mode transitions; infeasible energy and geometry; reversal
  dynamics; spiral entrance feasibility and G2 transitions; MM304/MM305 agreement;
  polar, equatorial, high-energy and low-energy states.
- Replay of existing flight logs: prediction error, old C forecast versus new
  predictor, from identical initial states.
- Native ShuttleSim end to end (equatorial 72 km live checkpoint scenario, 54 deg
  polar, high and low energy) through MM305 spiral, Final and touchdown.
- Live: a full flight from the 86 km equatorial quicksave, deorbit to rollout.
  Report what happened; no claim of success from admission or reaching the HAC.

## Commits

One per phase step, each built and tested.
