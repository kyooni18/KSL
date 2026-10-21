# STS Shuttle Launch Program

Python controller for KSP 1.12.5 shuttle launches using kRPC. It flies a
configurable Kerbin ascent, discovers attached booster branches from separator
topology, controls engine thrust and pitch moment, and handles booster and ET
separation followed by orbital insertion.

The active STS observed over kRPC has three SSMEs fed from its external tank,
two Clydesdale solid boosters, two Terrier OMS engines, and one Mite motor
attached to each booster. The controller throttles the SSMEs, accounts for the
fixed booster thrust, separates both Clydesdales after burnout, then disables
the SSMEs and enables both Terrier OMS engines when the ET is released. The
Mites remain under stock staging. The documented
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

**One command to orbit** (circular orbit of a given altitude in km and inclination
in degrees; logs and telemetry go to `runs/`):

```sh
bin/ksl launch --set ascent.targetAltitudeKm=90 --set ascent.inclinationDeg=34
bin/ksl launch -p sts-polar --set ascent.revertToLaunch=true   # revert first
```

It sets the Clydesdale limiters to 0.9, launches with the SSME limiter at 0.38,
flies to an `altitude` x `altitude-30` km insertion orbit (PEG), separates the ET
5 s after MECO and translates clear on RCS, then circularizes with the OMS at
apoapsis (reaction wheels only, closed-loop re-planning). Attitude is flown by
`ksl/ascent/attitude.py` (own controller, not the kRPC AutoPilot), which feeds
forward the canted-engine thrust moment about the live centre of mass.

Lower-level entry points:

```sh
# Read-only inspection of the active vessel; no ignition or control writes.
.venv/bin/python -m ksl.ascent.flight

# Fly an eastward STS suborbit with a 75 km apoapsis and 50 km periapsis.
.venv/bin/python -m ksl.ascent.flight --launch --orbit 75000 --heading 90 --meco-periapsis 50000
```

`--turn-start 3000` sets the minimum altitude before the post-roll gravity turn can begin.
The turn biases 20° below velocity direction at low dynamic pressure, tapering
to 8° by 25 kPa to build horizontal speed earlier without pitching far below
prograde in the dense atmosphere. The pitch command changes at up to 0.8°/s by
default; `--pitch-slew` tunes this rate. The slower slew keeps the shuttle higher
through SRB separation so the low post-booster TWR can continue building orbit
without dropping the nose to horizontal too early.
The Orbiter controller also supports two Size2 liquid side boosters, but the
active STS uses two Clydesdale SRBs. Their thrust is treated as fixed while the
SSME group is allocated around it, and both boosters separate together when
their Clydesdale motors run out of fuel. Mite motors attached at the nose are
held inactive during the Clydesdale burn and ignite immediately before booster
separation. On the live STS, all three SSMEs ignite at a 100%
thrust limiter while the clamps still hold. The controller verifies at least
95% of their combined maximum thrust and holds for a 3-second lead interval by
default; `--ssme-lead-time` changes that interval. Only then does the next
staging action light both SRBs and release the clamps. The four-Skipper profile described
below applies only to that separate vehicle variant.
The controller reads each SRB's configured thrust limiter for pad acceleration
and burn-time estimates. With both live Clydesdales at 70%, the current 520.7 t
STS has an estimated 1.36 G at ignition with the three SSMEs at full thrust.
`--meco-periapsis 50000` with a 75 km apoapsis target aims for a suborbit whose
periapsis is positive and below the 70 km atmosphere boundary. Reaching target
apoapsis alone cannot discard the ET with insufficient horizontal energy.
If apoapsis arrives first, `--meco-pitch 0` commands a near-horizontal energy-build
segment through the configured pitch slew until the periapsis gate is met. If ET propellant runs low first, the
program separates the tank and hands off to the Terrier OMS engines rather than allowing an
SSME fuel-out.
The shuttle preserves its measured pad roll through the first 30 m of climb, then
performs a dedicated rate-damped body roll to twice the launch heading, wrapped to
-180°..180° (heading 90° targets roll 180°). During that initial roll, pitch stays
at 89° and the gravity turn is held. The roll axis is driven directly from measured
roll angle/rate while kRPC AutoPilot continues to hold pitch/yaw; the turn is
released after roll error stays within 3° for the configured settle interval.
The roll settle check also requires body roll rate below 1.5°/s to tolerate
the small residual oscillation measured on the active STS.
The body roll is rate-capped at 3°/s, isolates aerodynamic roll inputs until
settled, and pauses if nose error exceeds 5°.
The 180° roll is held through booster separation, then
targets 0° after 180 seconds; use `--roll-reset-delay` to change that wait.
`--hz 10` is the requested control frequency; actual RPC latency may reduce it.
`--address`, `--rpc-port`, and `--stream-port` configure the server connection.
Telemetry is written to `telemetry.csv`; use `--telemetry` to choose another path.
It includes mass, vertical/horizontal speed, dynamic pressure, booster elapsed time,
live booster thrust, booster LF/Ox/SolidFuel, booster propellant fraction, commanded
G, ET LF/Ox remaining, orbit shape, engine allocation, actual/target roll, measured
roll rate, direct roll-control input, and pitch/yaw attitude errors. Read-only
preflight also prints current-pressure acceleration using the configured SRB limits
and the expected full-thrust ignition acceleration.
Ctrl-C and SIGTERM return liquid throttle to zero and release autopilot to SAS.
Legacy solid boosters, if present, still burn to depletion. Liquid booster stages
are throttleable and separate together on propellant depletion.

## Control model

The report is analytical input, not a source of executable instructions.
Its fixed time schedule is deliberately not used for flight control.

At each control tick, the program solves a bounded moment equality while obeying
the configured total-thrust target. In ordinary ascent, KSP's measured
`Flight.g_force` is fed back into global liquid throttle and measured dynamic
pressure reduces throttle above `--max-q`. During the STS Clydesdale burn,
master throttle stays at 100% and the common SSME limiter tracks the pitch moment
needed to balance the boosters about the live center of mass. The controller uses
pressure-corrected full SSME thrust and canted mounting geometry, with a 30–100%
limiter range and the normal throttle slew. It retains this balance through the
last fueled booster tick; the post-booster clearance/roll schedule starts at
burnout and separation, without an early low-fuel throttle drop. If the required
balance exceeds full SSME thrust, the remaining moment needs attitude control.
Combined SRB and SSME thrust can exceed the G or q targets during this phase.
The feedback guard does not
predict lift, drag, angle of attack, or aerodynamic pitching torque. Core
commands remain Skiff, two
near Bobcats, two far Bobcats,
and Cheetah. The four liquid-booster Skippers are deliberately collapsed
into one additional throttle group, so all four always receive exactly the same
throttle. Their collective pitching moment is still included and is compensated by
the core-engine allocation rather than by differential booster throttling.

After the STS Clydesdales separate, SSME throttle is allocated against the G
target without the pad-calibrated moment cap; the autopilot gimbals steer the
engines. This avoids the observed post-separation throttle stall near 0.30.

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

`craft_manifest.json` records the last saved craft-file SHA-256, engine positions,
and resource amounts. That 46-part liquid-booster snapshot does not match the
active KSP vessel observed for this update, which has 41 parts and two Clydesdale
boosters; no matching saved craft file was available to hash. Use live vessel
mass in KSP for launch predictions. The older report describes a different
60-part vehicle; its mass, engine layout, and burn schedule do not apply here.

Inspect an updated craft without running KSP:

```sh
.venv/bin/python -m ksl.ascent.craft /path/to/STS.craft --meta /path/to/STS.loadmeta
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

## Orbit profiles, trajectory corridor, trim and live controls (2026-10-13)

**Status: offline-verified only (unit tests, fake kRPC node frames, dry runs). None
of this has flown in KSP yet.** First live flight checklist is at the end.

### Profile kinds (`profile.kind`, selectable in the TUI Mission tab)

| kind | ascent (ET) | orbit operations (OMS) |
|---|---|---|
| `circular` | `targetAltitudeKm` × insertion Pe, `inclinationDeg` | exact one-burn trim, ≤ `orbit.maxBurns` (2) |
| `plane` | waits for the pad to pass under `planeTarget`'s orbital plane (e.g. Minmus), PEG steers to that plane normal | trim apsides, keep the as-flown plane |
| `gto` | semi-GTO: Pe `gtoPerigeeKm`, Ap `semiGtoApoapsisKm` (default 800 km) | `geo_transfer`: apogee raise to synchronous altitude, stay in GTO |
| `geo` | semi-GTO whose Ap is *designed* so a 2-rev coast lines the apogee up with `geoLongitudeDeg` | perigee phasing burn → GTO injection → circularize at apogee (`geoCircularize`: auto = only if OMS dv ≥ need + `reserveDeltaV`) |
| `rendezvous` | plane launch to `planeTarget` (a vessel) | trim to parking orbit, then `rendezvous`: Lambert transfer to `standoffM` behind, midcourse, velocity match, RCS approach to `stopDistanceM` |

Examples: `Configuration/missions/sts-{gto,geo,minmus-plane,rendezvous}.json`.

GEO longitude math (`ksl/ascent/geo.py`): apogee longitude =
L_perigee + 180° − ω·(n·P_semi + P_GTO/2). The launch time cannot move it (the
ground track repeats every day), so it is placed by the number of semi-GTO revs `n`
and a small perigee phasing burn that retunes P_semi. `insertionLongitudeDeg` (default
−60°) is only the pre-flight estimate used to design the ET apoapsis; after MECO the
plan is recomputed from the measured perigee.

### Ascent trajectory following (`ksl/ascent/trajectory.py`)

Recorded flights (e.g. run 20261012T123634Z) lofted to Ap 105 km for an 85 km target
with ~700 m/s vertical speed at 67 km: the pitch-over began at 1.5 km where q was
already ~6 kPa, so the q·α bound pinned the nose to prograde. Changes:

* `--turn-start` 1500 → 500 m: pitch over while q is low enough for a useful α.
* Apoapsis corridor: above 6 km the pitch is corrected by −0.35°/km of predicted-Ap
  error against a reference Ap(altitude) (scaled to the target), ±10°, still inside the
  q·α bound. Altitude/apoapsis are mass independent, so a heavy payload follows the same
  shape, only slower. Disable with `--no-corridor` in `ascent.extraArgs`.
* `--transfer` disables the circular-ascent Ap hold for GTO insertions.

Note: in the recorded SRB phase the CSV's vertical speed, UT and altitude are mutually
inconsistent (∫vz dt ≈ 2× altitude), so that data cannot calibrate a simulator; the
corridor gains are first-principles and need a live check.

### Orbit trim

`orbit_trim.plan_exact` solves one impulsive burn reaching the exact Ap, Pe and
inclination (scan over burn time including exact target-radius crossings), converts
the dv vector to node components by probing kRPC's own `burn_vector` (no frame sign
assumptions), and flies it. A second burn only removes the residual (RCS forward
translation when ≤ 2 m/s). `--legacy-plan` restores the staged apsis-by-apsis planner,
which is also the automatic fallback.

### Live operator controls

Every child program polls `runs/<id>/controls.json` (`ksl/controls.py`). TUI keys
on the watched run: `f` warp now, `x` stop warp, `g` auto-warp on/off, `o` orbit
operations on/off, `h` hold in orbit, `t` trigger orbit operations. Warping is
re-evaluated every 0.25 s, so turning auto-warp off drops to 1× immediately (unlike
`SpaceCenter.warp_to`). Initial values: `operations.autoWarp`, `orbit.trim`,
`orbit.holdForTrigger`.

### First live flight checklist

1. Circular 85 km / 10°: watch the pitch-over at ~500 m (q, α, wing loads) and the
   corridor correction in the telemetry; compare max Ap against the 105 km baseline.
2. Trim: confirm `exact plan` predicted elements vs the result after burn 1.
3. Only then GTO/GEO (check `geo-plan` / `gto-apogee` events) and rendezvous.
