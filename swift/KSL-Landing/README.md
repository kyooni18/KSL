# KSL Landing — native Swift 6 rewrite

A standalone, object-oriented shuttle guidance, control, simulation, replay, and kRPC package. The flight runtime is Swift only. It flies modeled unpowered entry, variable-radius spiral HAC, final approach, preflare, flare, touchdown, and runway stop using the same flight computer used by the live adapter.

**Status:** implemented, compiled, and offline-tested. The delivered suite has 37 passing Swift tests and 12 passing independent TCP protocol/failure tests. The complete 19-case landing matrix, including unsuccessful cases, is in [Documentation/Validation.md](Documentation/Validation.md). **No live KSP touchdown or rollout was verified.** Some off-nominal entries remain outside the demonstrated controller envelope.

The original KSL landing implementation was preserved separately. This package does not modify or depend on ascent, OMS, or deorbit code, and it does not start/restart KSP.

## Build and run

Requires Swift 6 and Swift Package Manager. No remote Swift package dependencies are needed. The verified platform is Linux x86_64 with Swift 6.0.3; macOS 13 is declared but not build-tested here.

```sh
swift build -c release
swift test -c release

# Nominal final approach, touchdown, and stop
.build/release/ksl-landing simulate --require-success --output final.json

# Entry with an 8-degree heading error: demonstrated entry through spiral HAC
.build/release/ksl-landing simulate --entry --heading-error 8 \
  --require-success --log entry.jsonl --output entry.json

# Recorded polar entry initial conditions, rerun in the new offline model
.build/release/ksl-landing simulate \
  --scenario Fixtures/sts-kerbin-20261029-live-entry-72km.ini \
  --require-success --output historical-initial-condition.json

# All unit/integration tests, protocol faults, and the full case matrix
bash Tools/run-validation.sh
```

The command-line `simulate` output always says `offline-model-only`. A fixture containing the word `live` identifies where its initial condition came from; it does not make a new simulation a live flight. Use `--require-success` when a shell command must fail on a modeled noncompletion.

## Package map

| Directory | Purpose |
| --- | --- |
| `Sources/KSLCore` | SI-valued states/configuration, geometry, commands, phases, protocols |
| `Sources/KSLPhysics` | Kerbin atmosphere, STS polar, force/moment dynamics, integration, prediction |
| `Sources/KSLGuidance` | Entry search, energy, spiral planning, tracking, approach, touchdown decisions |
| `Sources/KSLControl` | Surface/attitude control and rollout steering/braking |
| `Sources/KSLKRPC` | Native bounded protobuf codec, TCP session, telemetry/frame conversion, dispatch |
| `Sources/KSLRuntime` | Shared deterministic flight computer and ordered asynchronous runtime |
| `Sources/KSLSimulation` | Scenarios, plant disturbances, deterministic closed-loop execution |
| `Sources/KSLDiagnostics` | Streaming logs, native replay, conservative archival import |
| `Sources/KSLLanding` | CLI executable |
| `Tests` | Mathematical, physical, guidance, ownership, concurrency, replay, and flight regressions |
| `Fixtures` | Original historical starting conditions used by validation |
| `Validation` | Actual reports, exact scenarios, compressed logs, plans, test logs, plots, source hashes |
| `Tools` | Independent wire fixture, reproducible matrix/report/plot orchestration |
| `Documentation` | Engineering design, evidence audit, operation, results, precise limitations |

## Geometry and planning

The HAC uses `r(theta) = Rf + k*theta²`, positive growth, both sides, analytic curvature, and a finite family of turn amounts including more than a full revolution. C2 acquisition and unload curves connect it to the measured state and straight final. Radius/growth/turn selection is assessed against lift, trim, bank/rate, q, load, drag work, final speed, and braking distance. It is not a fixed-radius circle.

Entry forecasts evaluate lateral feedback and reversal history throughout each candidate. The leading forecasts invoke the same full terminal planner. A rejected speed-based handoff continues to use a control-dependent forecast. The final capture curve is both energy-priced and flown. An observed-force effectiveness filter assists lift inversion under model mismatch. A finite candidate search and stress warnings do not establish global or robust feasibility.

## Replay and archives

Record every 100 ms control step for exact replay:

```sh
.build/release/ksl-landing simulate --log final.jsonl --log-every 1 --output final.json
.build/release/ksl-landing replay --input final.jsonl --verify-commands

# A nondefault mission requires the same configuration at replay time
.build/release/ksl-landing replay --input live-flight.jsonl \
  --configuration live-flight.jsonl.configuration.json --verify-commands
```

The packaged nominal-final log is full rate; most other packaged matrix logs are sampled at 1 Hz to reduce evidence size. A sampled log is for trajectory inspection and is not a command-identical replay input. Decompress `.jsonl.gz` before passing it to the native reader.

For the original schema-5 logs, strict parsing is the default. Salvage is explicit:

```sh
.build/release/ksl-landing legacy-audit --input original-vehicle.jsonl --skip-malformed
.build/release/ksl-landing import-legacy --input original-vehicle.jsonl \
  --skip-malformed --session SESSION_ID --at-time UT --output recovered-scenario.json
```

Sessions are never merged. A damaged delta chain waits for a complete keyframe. Imported telemetry is not automatically labeled measured/live, and missing records are not fabricated.

## Native KSP connection

```sh
# Read-only inspection
.build/release/ksl-landing inspect --host 127.0.0.1 --port 50000 --vessel STS

# Writes controls to the already airborne, named active vessel
.build/release/ksl-landing fly --host 127.0.0.1 --port 50000 --vessel STS \
  --airbrake-group 1 --log live-flight.jsonl
```

`fly` disengages the kRPC autopilot and maintains zero throttle, SAS off, RCS off, and the configured airbrake action group off. Wheel braking is allowed only on the ground. `SIGINT`, runtime failures, and normal completion have explicit cleanup. No automatic command retry occurs after an ambiguous acknowledgement. Reconnection is read-only and requires a fresh mission before arming.

The actual craft's action-group mapping, installed server behavior, moment signs, and timing need live qualification. Detailed prerequisites, wire types, shutdown semantics, and configuration are in [KRPC-Integration.md](Documentation/KRPC-Integration.md).

## Engineering documents

- [Architecture, module dependencies, ARC, concurrency](Documentation/Architecture.md)
- [Physics, frames, units, and model provenance](Documentation/Physics-and-Frames.md)
- [Guidance, spiral geometry, energy, approach, and control](Documentation/Guidance.md)
- [Original-project investigation and historical-log audit](Documentation/Reference-Audit.md)
- [Quantitative final validation](Documentation/Validation.md)
- [Native kRPC integration and commands](Documentation/KRPC-Integration.md)
- [Unfinished and unverified functionality](Documentation/Known-Limitations.md)

The evidence boundary is deliberate: offline completion, local wire compatibility, and actual live touchdown/rollout are distinct claims. Only the first two have been tested in this delivery.
