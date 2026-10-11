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
