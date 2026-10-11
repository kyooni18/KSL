# KSL architecture

## Layers

```text
            bin/ksl (CLI)          ksl tui (Textual)
                    \                 /
                     MissionController            ksl/controller.py
                     ├─ PhaseMachine              ksl/phases.py
                     ├─ RunLog (runs/<id>/)       ksl/runlog.py
                     └─ phase programs            ksl/programs.py
        ┌──────────────┬──────────────┬─────────────────┬───────────────────┐
  AscentProgram  OrbitTrimProgram  LandingProgram      SimulationProgram
  child process  child process     NDJSON client       child process
  ksl.ascent.    ksl.ascent.       ksl/backend.py      ShuttleSim/scripts/
  flight         orbit_trim             │              run_guidance.py
        │              │                │                 │         │
     Python kRPC    Python kRPC   CLanding/build/      backend   ShuttleSim
        └──────┬───────┘          landing_backend (C)  (same     plant (C)
               │                        │ C-Nano kRPC   binary)
               └────────── KSP / kRPC ──┘
```

**Separation rules**

- *Guidance and control laws* live only in `ksl/ascent/{guidance,vehicle,
  allocation,attitude}.py` (ascent, OMS) and in `CLanding/` (deorbit through
  rollout, atmospheric FCS). The mission layer contains none.
- *Simulation* is a separate process (`ShuttleSim/`). It never links guidance
  code. The backend sees it through the simulator transport, exactly as it
  would see KSP.
- *Vehicle models*: the ascent vehicle model is discovered at runtime from kRPC
  part topology (`ksl/ascent/vehicle.py`). The landing models (aero,
  atmosphere, attitude) are selected by `KSP_LANDER_MODEL_PROFILE`
  (`ShuttleSim/scripts/model_paths.py`, `CLanding/telemetry/sim_telemetry.c`).
- *Interfaces* (CLI, TUI, PyQt, web telemetry) only read run/backend state and
  issue operator commands.

## The C/Python boundary

There are two boundaries, both process boundaries:

1. **Landing backend NDJSON protocol v1** (stdin/stdout of `landing_backend`).
   The operator sends methods (`updateConfiguration`, `connect`, `createPlan`,
   `engage`, `engageReentry`, `engageFinalTest`, `abort`, `saveCheckpoint`,
   `shutdown`, …). The backend streams `snapshot` messages. `ksl/backend.py`
   is the single blocking client, and `Tools/headless_flight.py` subclasses it.
   The PyQt UI keeps a `QProcess` client because it needs Qt's event loop.
2. **Phase program stdout**. The ascent and OMS programs print human-readable
   logs, plus `@@ksl {json}` milestone/telemetry lines (`ksl/ascent/events.py`).
   The controller maps the milestones onto mission phases.

Neither boundary needed C or Python rewrites. The backend still never shares a
process with Python, and Python never talks to kRPC on the landing path.

## Phase state machine

`ksl/phases.py` defines the 11 mission phases, the valid entry phases, and the
only mappings from program vocabularies to mission phases:

| Mission phase | Owner | Source signal |
|---|---|---|
| Prelaunch | AscentProgram | start / `preflight` |
| Launch | AscentProgram | `ignition`, `liftoff` |
| Ascent | AscentProgram | `gravity-turn`, `booster-separation` |
| Orbit | Ascent → OrbitTrimProgram | `meco`, `et-separation`, `insertion`; trim reports |
| Deorbit | LandingProgram | backend `Planning`, `Coast to Burn`, `Burn Setup`, `Deorbit Burn` |
| MM304 | LandingProgram | backend `Entry Interface`, `MM304 Entry` |
| MM305 | LandingProgram | backend `TAEM`, `Heading Alignment` |
| Final Approach | LandingProgram | backend `Final Approach`, `Flare` |
| Touchdown / Rollout / Complete | LandingProgram | backend `Touchdown`, `Rollout`, `Complete` |

The machine is forward-only (skips allowed, regressions rejected and logged).
Backend `Abort`/`Fault` fail the mission. `Idle`, `Paused` and `Attitude Recovery`
are statuses, not progress. A unit test parses `phase_string()` in
`CLanding/vehicle/models.c` and fails if a new backend phase is left unmapped.

Live holds: `prelaunch` (read-only preflight), `orbit`, `complete`.
Continuation (`ksl continue`): orbit → deorbit; MM304/MM305/Final resume
their own phase through `engageReentry`/`engageFinalTest` after the guard's
verified direct-control handoff. An interrupted ascent cannot be resumed.
Continuation restores the resolved configuration recorded with the original
run. An explicit profile or new overrides replace its values. Save loading
uses the independent guard's settled-scene/UT verification before a program
starts; profile and phase validation precede the load.

## Live landing sequence (LandingProgram)

This mirrors the safety steps of the headless live harness:

1. Start the backend with `KSP_LANDER_MODEL_PROFILE`. Wait for `ready`, send
   `updateConfiguration` (the profile's kRPC endpoint applied), then `connect`
   and wait for `connected`.
2. From Deorbit: pause KSP through the independent guard, run `createPlan`,
   resume. Require entry-stage qualification (and strict full-route capture if
   `landing.requireStrictPlan`). Then `engage`.
   From MM304/MM305/Final: run the guard's `prepare-reentry` handoff (SAS off,
   zero throttle/thrust, 1× warp, all verified), then `engageReentry` /
   `engageFinalTest`.
3. Follow snapshots to `Complete` (success) or `Abort`/`Fault` (failure). Write
   KSP saves at configured phases through the backend's `saveCheckpoint`.
4. On operator abort or error, send `abort` and then `shutdown`. The native
   safe-control path then runs.

The native rollout program keeps its wider contact envelope to retain braking
control near the runway edge. `Complete` requires a stopped speed inside the
configured runway bounds; an overrun becomes a failed landing after braking.

## Shared infrastructure

| Concern | Where |
|---|---|
| kRPC endpoint | profile `connection` → `--address/--rpc-port/--stream-port` for ascent and OMS, `connection.rpcHost/rpcPort` for the backend, `KSL_KRPC_*` for the guard |
| configuration | `ksl/config.py` + `Configuration/missions/*.json` |
| logging / status / telemetry | `ksl/runlog.py` (`runs/<id>/`) |
| checkpoints | `checkpoint.json` + optional KSP saves (`checkpoints.kspSaveOnPhases`) |
| error handling | typed errors (`ConfigError`, `ConnectionFailure`, `BackendError`, `GuardError`, `ProgramUnavailable`, `PhaseTransitionError`) → mission state + exit code |
| replay / export | `ksl/replay.py` (runs, ascent CSVs, ShuttleSim recordings) |

The native backend's internal structure is documented in
`CLanding/ARCHITECTURE.md` and `CLanding/README.md`. The ascent control model is
in `Docs/Ascent/README.md`.
