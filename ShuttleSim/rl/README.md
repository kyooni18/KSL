# Offline guidance-learning experiment

**Simulator only. Not a landing solution or a live-flight policy.** See
[REPORT.md](REPORT.md) for measured results and blocking limitations.

## Run

From the repository root (C11 compiler, CMake, Python 3 standard library):

```sh
cmake -S ShuttleSim -B ShuttleSim/build-offline -DSHUTTLESIM_OFFLINE_API=ON
cmake --build ShuttleSim/build-offline -j2
python3 ShuttleSim/rl/test_physics.py ShuttleSim/build-offline/libshuttlesim_offline.dylib
python3 -m unittest discover -s ShuttleSim/rl -p test_environment.py
sh ShuttleSim/tests/run_all.sh
python3 ShuttleSim/rl/experiment.py train --simulator-only --output /tmp/shuttle-train
python3 ShuttleSim/rl/experiment.py evaluate --simulator-only \
  --policy /tmp/shuttle-train/policy.json --output /tmp/shuttle-eval
```

On Linux replace `.dylib` with `.so` in the physics-test command. Output
directories must not already exist: runs are never silently overwritten.
Training defaults to two generations, four candidates, seed 7, 3600-second
maximum episodes, and `--curriculum mixed`.  The mixed curriculum runs short
entry-corridor, interface-corridor, terminal-corridor, and one full 86 km case
so smoke runs do not spend all samples timing out hundreds of kilometers from
the TAEM interface.  Use `--curriculum full` for the legacy 86 km-only
distribution, or `--curriculum entry` / `--curriculum interface` for narrower
diagnostic runs.  `--horizon`, `--population`, `--generations`, and `--seed`
customize run size. Evaluation does not update weights or simulator parameters.

## Architecture and integration boundary

`physics.c` wraps existing `sim_init`, `sim_apply_command`, and `sim_step` in
independent handles. The offline duration ABI advances the calculated
simulator interval directly; it derives exact RK4 substeps from the configured
physics timestep and has no fixed tick-count or wall-clock pacing rule. The
native executable remains available with a configured small `--dt` for
high-fidelity validation. `expert.c` adapts telemetry to the
existing CLanding deterministic guidance, linked into a separate offline
library. `Environment(simulator_only=True)` is the guidance adapter: reset,
observe, project, execute a calculated duration, terminate, and log. It opens
no network connections and never calls kRPC. The CMake option defaults OFF;
normal UDP operation remains separate. The raw physics ABI is not a safety
controller and must not be used as a shortcut around `Environment.step`.

`reset(seed, policy_version)` returns observation and metadata;
`step([aoa_residual, bank_residual])` returns observation, reward, terminated,
truncated, info. `close()` releases both handles. The environment can be reset
and independently interleaved. Explicit simulator opt-in is mandatory in the
Python API and CLI. Checkpoint loading validates schema, feature ordering,
finite weights, simulator-only marker and content version. Invalid runtime
policy output falls back to the deterministic command, not a zero attitude.

## Observation and action contract

Schema `shuttlesim-guidance-v2` has **37 features**. `environment.FEATURES`
exports ordered names; physical ratios are derived from active world, vehicle,
or modeled-authority state and clipped to [-5, 5]. It includes
phase; altitude/radar altitude; air, horizontal and vertical speeds; Mach/q/FPA;
AoA/bank and measured finite-difference attitude/course rates; runway along/cross
track, range/bearing, heading/course error; projected TAEM range error; gear and
brakes; phase age; deterministic AoA/bank; residual eligibility; HAC radius,
remaining angle/capture/violation and committed reversal state. These are
telemetry or guidance-derived values, not simulator futures, hidden forces,
randomization factors, or an oracle trajectory. Model labels and randomization
factors appear only in diagnostic logs, not the policy input.

The initial experiment intentionally learns only bounded entry AoA and bank
residuals. Actions [-1,1]^2 map into the currently available AoA/bank command
headroom exported by the deterministic expert, and target slew is derived from
the production control-authority response-time envelope. It does **not** learn
actuator, pitch/yaw torque, throttle, SAS, phase transitions, gear, airbrake,
reversal commitment, HAC size or final handoff decisions. Those stay with the
existing deterministic machine. The simulator also does not model a low-level
attitude servo: the commanded target is followed kinematically, with only the
configured pitch and roll rate limits applied. This limited scope avoids a new
low-level controller and is not represented as a full trajectory-learning
solution.

The terminal curriculum reports two separate contracts. `initial_handoff` and
`curriculum_handoff` describe the local MM304 interface set only. A terminal
path is counted only when the production diagnostics expose a valid,
non-degraded candidate with a finite non-negative live-energy margin and the
path has actually been committed. Terminal-corridor episodes that begin inside
the MM304 set therefore do not earn a handoff event or promotion credit by
themselves; promotion reports `terminal_path_unproven` when that downstream
contract is never observed.

Safety projection is fail-closed: learning is allowed only in ENTRY_ENERGY,
above 25 km, speed >500 m/s, q >50 Pa and <80% configured maximum, conservative
stall fraction <0.7, and valid finite guidance state. Advisory continuation warnings do not themselves
remove simulator-only authority. Abort/fault and every negative signed margin in
the shared control-authority envelope remain hard physical gates. It applies
existing protective AoA floors and bank-authority limits, forbids changing bank
sign, and derives residual target slew from the same modeled control-response
time used by production guidance. Deterministic recovery/abort overrides learned
residuals immediately.
Those are residual limits, not a replacement for the guidance target limits or
attitude servo rate limits. Abort/fault prevents another physics tick. Negative
signed margins from the shared control-authority envelope terminate rather than
trusting reward shaping. No live safety, touchdown, SAS-off or no-powered-approach
gates are relaxed.

## Reward and success

Potential differences reward reductions in absolute projected TAEM range,
runway cross track and course errors. Costs cover time, residual discontinuity,
excess sink, predicted infeasibility/warnings, energy error, unsafe AoA and late
final. Terminal reward is +1000 only for valid stopped runway rollout; unsafe,
abort or runway miss is -1000; horizon truncation is -200. Exact terms are logged
individually in JSONL; synthetic terminal tests ensure low altitude, contact
alone, COMPLETE alone and an abort plus contact cannot masquerade as success.

Success requires explicit simulator touchdown on the configured physical runway,
gear down, impact sink/speed no greater than the configured touchdown targets,
then the simulator's numerically stopped ground state inside that same runway and
deterministic COMPLETE with no unsafe condition. Runway length/width come from
the active scenario and touchdown targets come from the production vehicle/guidance
configuration; the RL adapter no longer carries a second hard-coded landing box.
`metrics.touchdown` counts contact; `success` and `rollout_valid` count the much
stronger combined gate.

## Algorithm and reproducibility

The runtime needs no NumPy, Torch, Gym or ML service. A compact linear-tanh
policy has 76 parameters (2 x (37+1)), deterministic inference and portable JSON
export. Zero residual exactly initializes to the existing teacher; this is an
expert warm start, **not** a claim of supervised behavior-cloning training.
CEM selection includes a tiny simulator-only active-residual exploration term to
avoid remaining permanently tied to the zero policy during short curriculum
smoke tests; deployment promotion is still fail-closed and requires validation
results, not exploration activity.
Seeded cross-entropy-method (CEM) parameter search evaluates full episodes,
retains elites and records mean/stddev, RNG state and checkpoints per generation.
CEM is adequate for a cheap, reproducible smoke test, not a promise of
sample-efficient learning of long sparse-reward landing.  The default mixed
curriculum is explicitly diagnostic: it samples bounded simulator-only
entry/interface/terminal-corridor states plus one full-entry case so failure
reasons and residual authority are visible before expensive full 86 km runs.
It does not certify a live terminal policy or relax deployment gates.

Training seeds: 11,12. Diagnostic validation: 101,102. Unseen evaluation:
1001,1002,1003. Held-out scenario: `ksp86km-heldout-20260915.ini`, seeds 2001,2002.
Stress seed: 3001. Validation is not used to select evaluation seeds or modify
physics. Compare deterministic zero residual, a projected fixed 25-degree AoA /
30-degree bank target, and the exported policy under identical cases. This
fixed baseline remains constrained; it is not unsafe open-loop actuator control.

## Domain randomization

At reset only, seeded independent perturbations apply to in-memory copies. The
existing model uncertainty varies mass +/-2%, atmospheric density +/-5%,
lift/drag +/-5%, pitch/roll attitude-following rate +/-5%, and inertial X velocity
/-1 m/s. Randomized episodes now begin from a pre-deorbit orbital condition:
apoapsis and periapsis are sampled in the 70--400 km band with periapsis no
higher than apoapsis, inclination is sampled from 0--87 degrees, and a
retrograde apoapsis burn targets a 40--69.5 km post-deorbit periapsis. The burn
is applied before the guidance episode starts. The resulting post-deorbit
altitude, speed, flight-path angle and ground-track geometry are therefore
derived from the orbit rather than independently perturbed. A conservative
MM304/TAEM pre-sampler rejects combinations with an unusable 70 km entry FPA
or speed, excessive deorbit delta-v, insufficient upstream runway range,
excessive crossrange, or an unreachable runway corridor. Its current limits
are FPA from -3.2 to -0.25 degrees, speed 1,800--2,700 m/s, upstream distance
150--1,500 km, crossrange <=1,200 km, range 150--1,800 km, and burn delta-v
5--140 m/s. This is a conservative two-body pre-sampler; the atmospheric
simulation remains the authority for actual MM304-to-TAEM behavior. Requested
pre-deorbit elements, burn delta-v and realized post-deorbit elements are
logged as diagnostics only; they are not policy inputs. The force book and
fallback aero table are scaled consistently. No fitted data files are written.
Scales and initial conditions are fixed throughout an episode and logged.
Stress cases use density x0.90, mass x1.05 (values in `run_campaign.py`) and
attitude-following rate x0.90 (an additional milder response stress, not the
campaign's x0.70/x0.85 actuator cases).

Density stress here changes density only, unlike campaign generation which also
scales pressure. It is a sensitivity test, not a complete thermodynamic weather
model or bit-for-bit campaign reproduction. The simulator currently has no
center-of-mass or aerodynamic-moment model, so CG shifts are not randomized or
claimed; mass and attitude-response variation are the available bounded
vehicle-dynamics proxies. Independent bank-effectiveness, sensor noise,
jitter and disturbances are also deferred: the reduced telemetry adapter does
not yet justify claiming live parity for those dimensions.

## Parity limitations and stop condition

The offline telemetry/expert adapter and `guidance_runner` now share the common
post-decode metadata preparation for physics/aerodynamic confidence, configured
vehicle fallbacks, response authority, site-relative geometry, and vessel
identity. They are still not bit-identical: the offline expert updates at 1 Hz,
uses simulator force estimates and simplified rates, and does not reproduce all
adaptive calibration/control-loop state or the runner's transport-specific
ground-track reconstruction. Radar altitude uses the runway height plane, not
terrain sensing. Simulator ground/atmosphere/aero and attitude models remain
approximations. There is no validated terminal curriculum. Historical `actual`
replay bypasses servo commands and cannot validate a learned controller.

The initial gate audit found that an advisory continuation warning was being
treated as a hard policy veto, which produced zero eligible steps. The current
simulator-only adapter separates that advisory condition from hard physical
guards; fresh smoke training now shows action-dependent residual steps across
the varied initial-condition cases, but no touchdown or valid rollout. Do not
loosen the remaining gates or modify fitted physics to manufacture success.
Reconcile telemetry, expert cadence/calibration, stall applicability and
baseline terminal geometry in a separate parity investigation before increasing
policy authority or running long training. Live integration remains out of
scope.

## Artifacts

`results/smoke-01/` and later numbered smoke directories hold exported policy,
checkpoints, optimizer states, training and validation JSONL and reports.
`results/evaluation-01/` and later evaluation directories hold comparison JSONL
and aggregate/episode metrics. Every reset logs the realized initial condition
and diagnostic orbital elements; every step logs policy/schema, requested
action, projected command/residual, fallback reason and reward terms. Terminal
records include physical touchdown metrics, outcome, phase counts, q/G maxima,
authority counts, randomization, initial conditions and return. `REPORT.md`
records commands and negative results. No artifact is a flight-ready policy.

## Telemetry dashboard

The existing telemetry web server observes the newest RL result directory and
shows its filtered initial orbital condition, learner-active progress,
fallbacks, and validation outcome in the `RL / OFFLINE MONITOR` panel. This is
read-only: it does not launch training and cannot command the KSP vehicle.

To pin the dashboard to a particular run, restart the normal service with:

```sh
KSP_LANDER_WEB_MODE=simulator \
KSP_LANDER_RL_OUTPUT=ShuttleSim/rl/results/while-playing-01 \
  ./Tools/telemetry_web_service.sh restart
```

If `KSP_LANDER_RL_OUTPUT` is unset, the server selects the most recently
updated directory containing `train.jsonl` or `training-report.json`.
