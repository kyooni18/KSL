# Validation

All checked-in qualification paths are offline. They build and exercise the
native C guidance stack without opening KSP, a serial device, or a kRPC server.

- `make -C CLanding clean all` builds the C17 backend with the vendored kRPC
  C-Nano 0.6.0 and nanopb 0.4.9.1 sources plus SQLite.
- `make test` (repo root, ~25 s) is the offline gate. It runs
  `make -C CLanding test` (16 fast native targets, listed in
  `Validation/README.md`) and pytest over `tests/` (mission layer, ascent,
  CLI, TUI, one closed-loop ShuttleSim golden replay) and `Validation/`.
- `make -C CLanding qualification` adds `taem-native-stack-test` (passing) and
  `mm305-feasibility-test`, an open physical gate that currently fails its
  non-vacuity check. A green build alone does not qualify landing or rollout.
  Model selection follows the explicit `KSP_LANDER_MODEL_PROFILE` profile;
  record its effective files and hashes.
- Closed-loop evidence comes from ShuttleSim (`ShuttleSim/scripts/run_guidance.py`).
  Use `--terminal-atmosphere/--terminal-aero/--terminal-aero-book/--terminal-attitude`
  to give guidance a different model from the plant; without them guidance has
  perfect knowledge of the plant's aerodynamics, which is not evidence of
  robustness. The simulator's attitude-servo parameters are no longer
  published to guidance (live KSP has none); `KSP_LANDER_SIM_PUBLISH_SERVO=1`
  restores them for A/B comparison.
- By default ShuttleSim's attitude plant is an ideal AoA/bank servo and the
  atmospheric FCS is bypassed. `run_guidance.py --direct-control` instead runs
  the backend's `flight_control_step` against a surface-moment attitude plant
  (stick inputs, q-scaled control/restoring moments, sideslip). Its moment
  coefficients are a generic lifting-body model, not a KSP identification, so
  it checks that the FCS closes its loops, not that its gains are right for
  the KSP vehicle.
- `python3 Validation/BackendProtocolTests.py` verifies the UI-to-C NDJSON
  protocol and the serial C-Nano configuration schema without connecting.

For memory/undefined-behavior checks, `make test` passes under gcc
AddressSanitizer + UndefinedBehaviorSanitizer (verified 2026-09-25 on Linux):

```
make -C CLanding CC=gcc BUILD=build-asan \
  CFLAGS="-O1 -g -std=c17 -pthread -fsanitize=address,undefined -fno-omit-frame-pointer -w" \
  LDFLAGS="-pthread -fsanitize=address,undefined" test   # ASAN_OPTIONS=detect_leaks=0
```

`-Werror` is off there because gcc's `-Wmisleading-indentation` and
`-Wformat-truncation` fire on the dense one-line controller code; the default
clang build keeps `-Werror`. Leak detection was not part of that check.

Live KSP tests are a separate acceptance stage. `run_headless.command --live`
and the optional independent safety guard must only be used deliberately with a
configured kRPC serial-protocol endpoint. Offline qualification never loads a
quicksave or sends flight controls.

The flight-operations console is checked offline with the scripted NDJSON backend
and Textual's test driver. Coverage includes insertion-only launch checkpoints,
explicit orbit operations, controller ownership conflicts, deorbit review without
engagement, revision-bound execution, replanning, changed orbit/mass/identity and
expired burn rejection, external abort/release, and executing an externally
started run's reviewed plan from the TUI. Console tests verify that settings
edits invalidate review and that simulation/live context limits available actions.
The layout is inspected at 120×40 and 80×24 terminal sizes. These checks do not
constitute a new live flight qualification.

On 2026-10-12, `bin/ksl land --from orbit -p sts-85km-10deg` completed one
guarded live orbit-to-rollout flight from an existing 85 km / 10° checkpoint.
The shuttle touched down 580 m along KSC runway 09 at 52.4 m/s and stopped
at 1466 m along, 3.5 m off centerline, with the body flap intact. Evidence is
`runs/20261011T164103Z-live-orbit-retry/` and the separately saved
`ksl-autonomous-20261012-orbit-runway-success` scene. The first attempt from
the same save safely aborted before burn when KSP's post-load placeholder mass
was used for planning; orbital checkpoint loading now waits for the physics
refresh. A separate MM305-only continuation overran the runway. These flights
demonstrate the integrated handoffs and one successful recovery trajectory,
while the slow MM305 non-vacuity gate and broader flight envelope remain open.
The live aerodynamic residual exceeded prior-flight model uncertainty late in
the successful approach; neither the simulator plant nor the chute inflation
has been certified by this result.
