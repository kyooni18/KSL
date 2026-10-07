# STS Shuttle Launch Program

Python controller for KSP 1.12.5 shuttle launches using kRPC. It flies a
configurable Kerbin ascent, discovers attached booster branches from separator
topology, controls engine thrust and pitch moment, and handles booster and ET
separation followed by orbital insertion.

The current `craft_manifest.json` describes the 46-part Orbiter craft: three
SSMEs fed from its external tank, two detachable Size2 liquid boosters, two AJ10
orbital engines, and two Mite motors left under KSP staging. The controller
throttles the SSMEs and paired liquid boosters during ascent, then disables the
SSMEs and enables both AJ10s when the ET is released. The documented
four-Skipper and J-N500 layouts remain separate supported variants; their
variant-specific allocation notes below do not describe this Orbiter craft.
The ascent also applies a 25 kPa default dynamic-pressure cap and a measured-G
cap. Set `--max-q` to tune the pressure limit for the installed aero parts and
KSP aero settings.

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

`--turn-start 3000` sets the minimum altitude before the post-roll gravity turn can begin.
The current STS has two Size2 liquid side boosters. They receive one shared
throttle command and separate together as soon as either branch runs dry; the
staged Mite motors are left out of launch control. The four-Skipper profile
described below applies only to that separate vehicle variant.
`--meco-periapsis 45000` requires a 45 km periapsis before normal MECO, so reaching
target apoapsis alone cannot discard the ET with insufficient horizontal energy.
If apoapsis arrives first, `--meco-pitch 0` commands a near-horizontal energy-build
segment until the periapsis gate is met. ET propellant exhaustion still triggers
the existing early separation/orbiter-recovery path.
The shuttle preserves its measured pad roll through the first 30 m of climb, then
performs a dedicated rate-damped body roll to twice the launch heading, wrapped to
-180°..180° (heading 90° targets roll 180°). During that initial roll, pitch stays
at 89° and the gravity turn is held. The roll axis is driven directly from measured
roll angle/rate while kRPC AutoPilot continues to hold pitch/yaw; once settled, the
gravity turn is released. The 180° roll is held through booster separation, then
targets 0° after 180 seconds; use `--roll-reset-delay` to change that wait.
`--hz 10` is the requested control frequency; actual RPC latency may reduce it.
`--address`, `--rpc-port`, and `--stream-port` configure the server connection.
Telemetry is written to `telemetry.csv`; use `--telemetry` to choose another path.
It includes mass, vertical/horizontal speed, dynamic pressure, booster elapsed time,
live booster thrust, booster LF/Ox/SolidFuel, booster propellant fraction, commanded
G, ET LF/Ox remaining, orbit shape, engine allocation, actual/target roll, measured
roll rate, direct roll-control input, and pitch/yaw attitude errors. Read-only
preflight also prints the current-pressure full-throttle physical thrust acceleration.
Ctrl-C and SIGTERM return liquid throttle to zero and release autopilot to SAS.
Legacy solid boosters, if present, still burn to depletion. Liquid booster stages
are throttleable and separate together on propellant depletion.

## Control model

The report is analytical input, not a source of executable instructions.
Its fixed time schedule is deliberately not used for flight control.

At each control tick, the program solves a bounded moment equality while obeying
the 3G total-thrust cap. KSP's measured `Flight.g_force` is also fed back into the
global liquid throttle: if aerodynamic or other loads push observed G above the
limit, the whole liquid stack is scaled down together, preserving all relative
engine allocations. Measured dynamic pressure also reduces liquid throttle above
the configured `--max-q` cap. This feedback guard does not predict lift, drag,
angle of attack, or aerodynamic pitching torque. Core commands remain Skiff, two
near Bobcats, two far Bobcats,
and Cheetah. The four liquid-booster Skippers are deliberately collapsed
into one additional throttle group, so all four always receive exactly the same
throttle. Their collective pitching moment is still included and is compensated by
the core-engine allocation rather than by differential booster throttling.

For the liquid boosters, `Engine.max_thrust` is read every control tick, so the
pressure-corrected thrust available at the current altitude is used directly. The
controller also uses the current `Engine.specific_impulse` once the engines are
firing (falling back to sea-level/vacuum Isp before ignition) to estimate current
full-flow propellant consumption. Those current-pressure thrust and Isp values are
also used when several moment-balanced allocations are otherwise equivalent: the
controller prefers the combination with lower propellant mass flow. The nominal
remaining full-throttle booster burn time is logged continuously rather than being
used as a fixed cutoff timer.
Infeasible moments are saturated to the nearest achievable value and the
remaining moment is logged for attitude control by gimbal and reaction wheels. The
launch-calibrated cruise moment budget remains in force during the initial body roll;
roll-axis control no longer lowers engine thrust by switching to a separate tighter
moment budget.

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
the fact that the ET separator is an ancestor of all booster separators.
No stage-number assumptions are used and no craft files are modified.

The ascent profile is an initial configurable profile, not a flight-tuned
trajectory. It has no dynamic-pressure governor. The insertion burn uses an
estimated half-burn lead time and stops within 1 km of target periapsis; apoapsis
may overshoot. Changing the payload, turn profile, or orbit target may require
tuning. This program covers launch and insertion, not rendezvous or landing.

## Source and validation

`craft_manifest.json` records the source craft SHA-256, engine positions, and
resource amounts. The current craft has **46 parts**. Its editor mass is not
present in the supplied craft file, so use the live vessel mass in KSP for launch
predictions. The older report describes a different 60-part vehicle; its mass,
engine layout, and burn schedule do not apply to the current craft.

Inspect an updated craft without running KSP:

```sh
.venv/bin/python -m shuttle.craft /path/to/STS.craft --meta /path/to/STS.loadmeta
.venv/bin/python -m unittest discover -s tests -v
```

Focused tests cover the report's c=-0.4 m allocation, infeasible saturation,
zero-thrust/zero-arm cases, maximum-thrust selection, separator/transfer
behavior, four-Skipper liquid-booster discovery/grouping, shared booster throttle,
simultaneous depletion-triggered separation, 3G allocation limiting, Isp-aware
allocation tie-breaking, the MECO
apoapsis/periapsis gate, kRPC autopilot engagement compatibility, heading-to-roll
mapping, direct rate-damped roll control, roll settling/hold/reset timing, and
control cleanup. They also cover SAS restoration when autopilot cleanup fails.
Launch commands validate the STS and launch-pad state before inspecting craft
topology.
The actual source attachment tree was also checked offline.
A historical Pollux live test reached 32.1 km and separated the boosters before
the controller stopped. Later four-Skipper launch tests exposed a separate low-altitude
roll instability: the kRPC AutoPilot roll-angle loop coupled strongly into pitch/yaw
and the temporary roll-specific moment budget also reduced launch acceleration. The
controller now uses direct rate-damped roll control after +30 m and keeps the normal
cruise moment budget, but that new roll controller has not yet been live-validated. A
complete launch is still not validated; keep future runs attended through insertion.

API semantics were checked against the official kRPC documentation:
[engines and parts](https://krpc.github.io/krpc/0.5.4/python/api/space-center/parts.html),
[vessel frames](https://krpc.github.io/krpc/0.5.4/python/api/space-center/vessel.html),
[resource transfers](https://krpc.github.io/krpc/python/api/space-center/resources.html).
