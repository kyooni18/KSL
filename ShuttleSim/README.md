# ShuttleSim

Standalone entry-to-landing simulator for STS-N on Kerbin.

ShuttleSim is intentionally **not** part of Guidance. It is a separate process and does not link Guidance code. Guidance consumes simulator telemetry and emits only high-level attitude/gear commands through the protocol boundary.

## Architecture

```text
Guidance  -- UDP command :8795 -->  ShuttleSim
Guidance  <-- telemetry :8796 ----  ShuttleSim
Telemetry Web <-- telemetry :8797 - ShuttleSim
                                      |
                                      +-- JSONL stdout
                                      +-- optional JSONL recording
```

ShuttleSim owns:

- simulation clock and fast-forward pacing
- Kerbin-centered inertial state propagation
- rotating Kerbin and stock position/UT-dependent atmosphere
- inverse-square Kerbin gravity
- KSP-calibrated STS-N Mach/AoA/q aerodynamic force model
- finite, q-dependent attitude response to AoA/bank commands
- 20 ms KSP parity physics cadence by default
- 86 km orbital initial state and deorbit burn/impulse
- spherical runway-relative coordinates
- touchdown and simple rollout physics
- telemetry publication and deterministic recording

Guidance owns none of those. It sees only telemetry and outputs high-level commands.

## Current fidelity status

The runtime architecture is implemented and exercised end-to-end. The files under `data/*_seed.*` are **seed models**, not claims of KSP parity. Exact parity requires fitting the tables and attitude response from the project's real KSP telemetry, then validating on held-out flights.

This separation is deliberate: simulator mechanics can be developed and tested now without smuggling Guidance logic into the physics engine, while calibration can replace data files later without redesigning the runtime.

## Build

```sh
cmake -S . -B build
cmake --build build -j
```

The code is dependency-free C11/POSIX and is intended to build with Apple Clang on macOS as well as Clang/GCC on Linux.

## Basic run

```sh
./build/shuttlesim \
  --scenario scenarios/orbit86km.ini \
  --atmosphere data/kerbin_atmosphere_seed.csv \
  --aero data/stsn_aero_seed.csv \
  --attitude data/stsn_attitude_seed.ini \
  --rate max \
  --record run.jsonl
```

Without Guidance connected, the scenario's initial attitude command is held. For a physics smoke test only:

```sh
./build/shuttlesim \
  --scenario scenarios/orbit86km.ini \
  --fixed-aoa 35 \
  --fixed-bank 0 \
  --rate max
```

`--fixed-*` is a test driver, not Guidance logic.

## Fast time pass

`--rate max` runs as fast as the CPU allows while keeping the physics timestep fixed. Numeric rates pace simulated time relative to wall time:

```sh
--rate 1
--rate 10
--rate 100
--rate 1000
--rate max
```

Fast-forward never multiplies the physics timestep. It runs more fixed steps per wall-clock second.

Normal runs use a 0.02 s physics step to match KSP's fixed-update cadence. Use a smaller `--dt` only as a numerical-convergence diagnostic; do not retune the plant against the smaller step.
The free-flight translational update mirrors the PhysX fixed-tick ordering (force/acceleration -> velocity -> position) rather than using a higher-order continuum integrator that KSP itself does not use.

## Open-loop KSP replay

For aerodynamic/world validation, convert a real flight to:

```csv
time_s,aoa_deg,bank_deg,gear_down
0.00,0,0,0
0.02,0.2,0,0
...
```

Then run:

```sh
./build/shuttlesim \
  --scenario scenarios/real-86km-checkpoint.ini \
  --attitude-replay actual-attitude.csv \
  --replay-mode actual \
  --rate max \
  --record sim-replay.jsonl
```

`actual` bypasses ShuttleSim's attitude servo and imposes the recorded actual AoA/bank. This answers the calibration question: **given the same attitude history, does the simulated vehicle follow the same trajectory as KSP?**

For attitude-response validation instead:

```sh
--attitude-replay guidance-command-history.csv --replay-mode command
```

That feeds recorded commands through the simulator's finite response model.

## Real 86 km checkpoint

`Scenario` accepts either the seed latitude/longitude circular-orbit description or exact inertial Cartesian state:

```ini
ut0=12345.67
position_x_m=...
position_y_m=...
position_z_m=...
velocity_x_mps=...
velocity_y_mps=...
velocity_z_mps=...
mass_kg=...
```

For KSP parity, use the actual saved 86 km state. Do not tune the seed orbital geometry to imitate a later checkpoint.

## Data fitting

See `CALIBRATION.md`. The included tools generate replacement atmosphere, aerodynamic, and attitude model files from normalized telemetry.

## Tests

```sh
./tests/run_all.sh
```

The tests compile the simulator, exercise 86 km -> touchdown, exercise recorded attitude replay, and verify UDP command ingestion.

## Operational local setup

The simulator is a separate process from Guidance. Default local transport:

- UDP 8795: Guidance/high-level attitude commands -> ShuttleSim
- UDP 8796: ShuttleSim telemetry -> Guidance
- UDP 8797: ShuttleSim telemetry -> Telemetry Web adapter

Start the calibrated real-86-km simulator:

    ./ShuttleSim/scripts/run_local.sh

It intentionally starts paused so consumers can bind before simulated time advances.
Then send the first attitude command and release the clock:

    ./ShuttleSim/scripts/simctl.py attitude --aoa 25 --bank -40
    ./ShuttleSim/scripts/simctl.py resume

Pause or change gear with:

    ./ShuttleSim/scripts/simctl.py pause
    ./ShuttleSim/scripts/simctl.py gear down
    ./ShuttleSim/scripts/simctl.py gear up

Inspect the Web telemetry UDP stream without running Guidance:

    ./ShuttleSim/scripts/telemetry_probe.py --port 8797 --count 5

Useful environment overrides:

    SIM_RATE=max
    SIM_MAX_TIME=2400
    SIM_TELEMETRY_HZ=20
    SIM_SCENARIO=/path/to/scenario.ini
    SIM_RECORD=/path/to/run.jsonl
    ./ShuttleSim/scripts/run_local.sh

The default launcher uses the real pre-deorbit 86 km KSP state, fitted Kerbin
atmosphere, the cross-flight direct-force data book, the robust Mach/AoA fallback
table, and the telemetry-derived attitude response model.

For aerodynamic parity work, tools/normalize_ksp_vehicle_log.py decodes native
delta-compressed KSP flight logs, tools/build_force_book.py constructs the
independent-flight force/q data book, and tools/compare_ksp_time.py plus
tools/compare_ksp_csv.py compare held-out KSP trajectories against ShuttleSim.
