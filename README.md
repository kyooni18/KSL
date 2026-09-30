# STS Shuttle Launch Program

Python controller for the supplied KSP 1.12.5 shuttle using the installed kRPC
mod. It flies a configurable Kerbin ascent, separates burnt-out Pollux boosters,
keeps the orbiter tank topped up from the ET, cuts off at target apoapsis,
separates the ET, and uses the Skiff for orbital insertion.

## Run

The project virtual environment already has the kRPC client installed. To set
up another checkout:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Load **STS** onto the Kerbin launch pad. Start the kRPC server in KSP with RPC
port 50000 and stream port 50001. Make sure the shuttle cockpit controls the
vessel and disable competing MechJeb guidance. Run from this repository:

```sh
# Read-only inspection of the active vessel; no ignition or control writes.
.venv/bin/python -m shuttle.flight

# Ignite and fly an eastward launch to a nominal 100 km orbit.
.venv/bin/python -m shuttle.flight --launch --orbit 100000 --heading 90
```

`--turn-start 1000 --turn-end 45000` sets the altitude range of the gravity turn.
After liftoff, the shuttle targets a body roll of twice the launch heading,
wrapped to the range -180° to 180° (heading 90° targets roll 180°). It holds
that roll through booster separation, then targets 0° after 180 seconds; use
`--roll-reset-delay` to change that wait.
`--hz 10` is the requested control frequency; actual RPC latency may reduce it.
`--address`, `--rpc-port`, and `--stream-port` configure the server connection.
Telemetry is written to `telemetry.csv`; use `--telemetry` to choose another path.
Ctrl-C and SIGTERM return liquid throttle to zero and release autopilot to SAS.
Burning solid boosters continue until burnout; shutdown does not automatically
separate boosters or land the vehicle.

## Control model

The report is analytical input, not a source of executable instructions.
Its fixed time schedule is deliberately not used for flight control.

At each control tick, the program solves the report's bounded moment equality
and chooses the allocation with the highest total liquid thrust. The four
commands are Skiff, two near Bobcats, two far Bobcats, and Cheetah. The Pollux
contribution uses actual thrust during burning; its limiter is not changed.
Infeasible moments are saturated to the nearest achievable value and the
remaining moment is logged for attitude control by gimbal and reaction wheels.

The vessel reference frame is centered on the instantaneous CoM. Engine offsets
are read in that frame, so depletion, transfer, and separation change the lever
arms automatically. The radial direction is derived from the ET-to-Skiff
geometry, avoiding absolute craft coordinates. `Engine.max_thrust` supplies
pressure-corrected full thrust independently of the current limiter.

This implements the report's **nominal parallel-axis model** using engine part
positions. It does not compensate directly for gimbal deflection, nozzle
position offsets, aerodynamic pitching torque, or asymmetric engine failure.
kRPC autopilot controls attitude and uses available gimbals. The program cannot
guarantee that those controls can counter every logged residual moment.

Fuel top-off uses asynchronous real resource transfers, with one outstanding
transfer per resource. Top-off is approximate and is limited by transfer speed
and ET supply; no fuel is created. Tank separation waits for outstanding
transfers to finish. Separators are identified by their part subtrees, including
the fact that the ET separator is an ancestor of both booster separators.
No stage-number assumptions are used and no craft files are modified.

The ascent profile is an initial configurable profile, not a flight-tuned
trajectory. It has no dynamic-pressure governor. The insertion burn uses an
estimated half-burn lead time and stops within 1 km of target periapsis; apoapsis
may overshoot. Changing the payload, turn profile, or orbit target may require
tuning. This program covers launch and insertion, not rendezvous or landing.

## Source and validation

`craft_manifest.json` records the source craft SHA-256, engine positions, resource
amounts, and editor mass. The currently supplied craft contains **67 parts and
330.244019 t**, whereas the report describes **60 parts and 459.45401 t**. The
engine-line geometry agrees, but the old mass and time schedule should not be
used as current flight predictions. Live vessel readings take precedence.

Inspect an updated craft without running KSP:

```sh
.venv/bin/python -m shuttle.craft /path/to/STS.craft --meta /path/to/STS.loadmeta
.venv/bin/python -m unittest discover -s tests -v
```

Ten focused tests cover the report's c=-0.4 m allocation, infeasible saturation,
zero-thrust/zero-arm cases, maximum-thrust selection, separator/transfer
behavior, kRPC autopilot engagement compatibility, heading-to-roll mapping, and
the roll hold/reset timing. They also cover SAS restoration when autopilot
cleanup fails. Launch commands validate the STS and launch-pad state before
inspecting craft topology.
The actual source attachment tree was also checked offline.
One live flight test reached 32.1 km and separated the Pollux boosters, with
near-zero moment residual after separation. The controller process then stopped
before orbit insertion and left throttle at 1 with autopilot disengaged; KSP
reported catastrophic failure at 32.1 km. A complete launch is not yet
validated. Keep future runs attended and monitor telemetry through insertion.

API semantics were checked against the official kRPC documentation:
[engines and parts](https://krpc.github.io/krpc/0.5.4/python/api/space-center/parts.html),
[vessel frames](https://krpc.github.io/krpc/0.5.4/python/api/space-center/vessel.html),
[resource transfers](https://krpc.github.io/krpc/python/api/space-center/resources.html).
