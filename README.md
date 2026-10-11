# KSL: KSP Shuttle Launcher + Lander

KSL flies a Space Shuttle in Kerbal Space Program through one complete mission:

```text
Prelaunch → Launch → Ascent → Orbit → Deorbit → MM304 Entry → MM305 TAEM
          → Final Approach → Touchdown → Rollout → Complete
```

It combines two former projects that now share one repository and history:

- **Lander** (this repository, formerly KSPShuttleLander). A native C17 backend
  owns deorbit planning, entry (MM304), TAEM (MM305), final approach, flare,
  touchdown and rollout guidance, plus the atmospheric flight-control system.
  It speaks kRPC directly through the vendored C-Nano client.
- **Launcher** (formerly KSPShuttleLauncher, imported with full history). Python
  kRPC programs own the STS ascent (SSME/SRB allocation, roll program, gravity
  turn, SRB and ET separation, PEG MECO) and the OMS orbit trim.

The `ksl` mission controller sequences both through one explicit phase state
machine. It uses one mission profile, one run record (events, telemetry,
status, checkpoints), one CLI and one TUI.

## Quick start

```sh
make setup          # .venv with krpc, textual, pytest
make                # build the native landing backend and ShuttleSim
make test           # offline regression gate (~25 s, no KSP needed)

bin/ksl validate --connect              # profile, build, deps, and a read-only KSP probe
bin/ksl run --dry-run                   # print the full mission plan, fly nothing
bin/ksl simulate l5-mm305               # offline closed-loop landing (native backend + ShuttleSim)
bin/ksl tui                             # dashboard / mission setup / replay
```

`bin/ksl` runs from a checkout. `pip install -e .` installs the same CLI as `ksl`.

## Flying a mission (live KSP)

Load the **STS** on the Kerbin launch pad. Start the kRPC server (RPC 50000,
stream 50001). Make sure no other autopilot (MechJeb etc.) is active.

```sh
bin/ksl run -p sts-85km-10deg                     # pad → 85 km / 10° orbit → KSC runway 09
bin/ksl launch -p sts-kerbin                       # pad → orbit, then hold in orbit
bin/ksl land -p sts-kerbin                         # deorbit → rollout from the current orbit
bin/ksl land --from mm305                          # take over an orbiter already in TAEM
bin/ksl continue latest                            # resume the last run from its checkpoint
bin/ksl continue latest --use-save                 # …after loading the checkpoint's KSP save
bin/ksl status --watch                             # follow a running mission from another shell
```

Entry phases are `prelaunch`, `orbit`, `deorbit`, `mm304`, `mm305` and
`final-approach` (`ksl phases`). A live mission can hold at `prelaunch`
(read-only preflight), `orbit`, or `complete`. Powered ascent and the unpowered
glide cannot be paused partway through a phase. Ctrl-C (or **a a** in the TUI)
runs the owning program's safe-release path. The ascent programs release
throttle and autopilot. The landing backend disengages autopilot/SAS,
neutralizes axes, throttle and wheel steering, and restores engines it
activated.

## Commands

| Command | Purpose |
|---|---|
| `run [--from P] [--to P]` | fly a mission between phases (live, or `--dry-run`) |
| `launch` / `land` | shortcuts: prelaunch→orbit; deorbit (or `--from`)→complete |
| `continue [RUN]` | resume from a run's checkpoint (orbit→deorbit, landing phase→same phase) |
| `simulate [SCENARIO]` | offline closed loop against ShuttleSim; `--list` shows named scenarios |
| `status [RUN] [--watch]`, `status --live` | run status; or a read-only probe of the game |
| `runs`, `checkpoints` | list recorded runs and resumable checkpoints |
| `replay SRC [--events] [--play RATE]` | summarize/play back a run, ascent CSV, or ShuttleSim recording |
| `export SRC -f csv\|jsonl [-o FILE]` | export telemetry |
| `validate [--connect]` | preflight checks |
| `profiles [--show P]`, `phases` | configuration and phase reference |
| `tui` | terminal UI |

Common options: `-p/--profile`, `--set section.key=value` (repeatable),
`--json` (machine-readable, for scripts and agents), `-v` (raw program output).
Exit codes: `0` ok/holding, `1` mission failed, `2` usage/config error,
`3` kRPC connection failure, `130` aborted.

## Configuration

A **mission profile** (`Configuration/missions/*.json`) configures a whole
mission. Unset keys take the defaults in `ksl/config.py`.

| Section | Contents |
|---|---|
| `connection` | kRPC address/ports, shared by the Python programs and the C backend |
| `ascent` | target altitude/inclination, insertion offset, SRB/SSME limiters, pad settle, extra flags |
| `orbit` | OMS trim on/off, `minBurnDeltaV` (corrections at or below it are skipped; default 0.1 m/s) |
| `landing` | backend configuration file, `KSP_LANDER_MODEL_PROFILE`, auto-warp, strict-plan policy, timeouts |
| `simulation` | default scenario, telemetry rate, max sim time |
| `checkpoints` | phases at which live missions write KSP saves |

The native backend keeps its own detailed configuration
(`Configuration/sts-live.json` for the STS, `default.json` for STS-N). The
profile references it and overrides its kRPC endpoint.

## Runs, telemetry and replay

Every run (live, simulated or dry-run) writes `runs/<run-id>/`:
`config.json`, `events.jsonl`, `telemetry.jsonl`, `status.json`,
`checkpoint.json`, per-program logs (`ascent.log`, `orbit-trim.log`,
`backend.stderr.log`, …), plus program artifacts (`ascent.csv`,
`deorbit-plan.json`, `simulation-result.json`). The CLI and the TUI read the
same files, so the TUI can watch a mission started from the CLI.

## Repository layout

```text
ksl/                 unified mission system (Python)
  phases.py          phase state machine + mappings from backend/ascent phases
  controller.py      MissionController (shared by CLI and TUI)
  programs.py        Ascent / OrbitTrim / Landing (live) / Simulation programs
  backend.py         NDJSON client for the native landing backend
  config.py, runlog.py, replay.py, scenarios.py, krpc_link.py, kspguard.py
  cli.py, tui.py     interfaces
  ascent/            ascent + OMS programs (from the Launcher)
    flight.py        kRPC run loop / CLI      guidance.py  ascent laws, PEG
    vehicle.py       STS model, staging        geometry.py  frames and roll math
    allocation.py    bounded thrust allocation attitude.py  custom attitude controller
    orbit_trim.py    OMS trim                  events.py    @@ksl milestone lines
CLanding/            native landing backend (C17; guidance, FCS, kRPC C-Nano, SQLite store)
ShuttleSim/          standalone entry-to-landing simulator + closed-loop scripts
Configuration/       mission profiles, backend configurations, checkpoints, vehicles
Tools/               headless live harness, kRPC safety guard, analysis/probe tools
PyQtApp/, WebTelemetry/  legacy PyQt operator UI and web telemetry viewer
Validation/          C and Python regression tests for the native stack
tests/               mission-layer and ascent tests
Docs/                architecture, algorithms, campaign logs and experiments
```

See [Docs/ARCHITECTURE.md](Docs/ARCHITECTURE.md) for how the parts fit together,
[VALIDATION.md](VALIDATION.md) for the test surface and current verification
status, and [Docs/Ascent/README.md](Docs/Ascent/README.md) for the ascent
control model.

## Other operator surfaces

- `./run_headless.command --live …` (`Tools/headless_flight.py`) is the
  live landing **test harness**: quickload, crash guard, campaign pinning and
  descent checkpoints. It uses the same shared backend client and guard calls.
- `./run_pyqt.command` is the PyQt landing UI (landing phases only).
- `Tools/telemetry_web.py` is the browser telemetry viewer.

## History

The Launcher's 136 commits are merged into this repository's history
(non-squashed subtree merge `874c59a`). Its pre-import tip is tagged
`launcher-history`, so `git log launcher-history -- shuttle/flight.py` shows
the ascent history at its original paths. Lander history is unchanged.

## Dependencies

The native backend vendors kRPC C-Nano 0.6.0 (LGPL-3.0-or-later) and nanopb
0.4.9.1, and links SQLite 3. Python: `krpc`, `textual`; `pytest` for tests;
PyQt6 only for the legacy UI.
