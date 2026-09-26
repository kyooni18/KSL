# Closed-loop qualification work, 2026-09-26

## Baseline and provenance

Starting HEAD: `11569ae`. The worktree already contained changes to FCS,
Final pitch release, simulator contact mechanics, telemetry, model selection,
and untracked identification tools/tables. They are preserved, not attributed
to this investigation. `provenance.json` records source/model hashes and the
initial tracked-diff hash. No live KSP process has been started or controlled.

Foundation and the separate ChatGPT research surface are unavailable in this
session: no callable tools were exposed and the connector-directory search
found no matching connection. No recalled knowledge or independent ChatGPT
review is claimed. This log is the durable local fallback, not a Foundation
write. Recheck access before subsequent architectural decisions.

The available attitude identification report covers Mach <=0.9, with held-out
R² of 0.758 pitch, 0.245 roll, 0.369 yaw. Its fitted pitch rate coefficient is
positive but its exported damping ratio is clamped to zero. Therefore even the
reported residuals do not validate the exported simulator dynamics. Supersonic
rigid-body identification and uncertainty bounds remain unproven. Do not tune
major FCS changes against this model as if it were qualified.

## Experiment 1: qualification must detect its own failures

Command: `make -C CLanding BUILD=build-research -j4 test` (Clang 18, Linux arm64).
Initial environment lacked Clang and SQLite headers; these were installed.
The earlier incomplete GCC build is not flight evidence.

Effective profile: `identified` (the existing uncommitted profile default).
Configuration: normalized built-in defaults, not `Configuration/default.json`.
Final fixture: `ShuttleSim/scenarios/final-mm305-ideal-3p5km-185mps-aoa3.ini`;
initial speed 185 m/s, sink 63.274 m/s, radar height 1273.896 m.
Direct FCS; ideal-servo comparison and model-error runs have not yet been run.
Raw output: `baseline-gate.log`.

Observed: process returned success despite admission/planner mismatch 2/2
and Final touchdown sink 6.05 m/s, speed 54.19 m/s, along 1620.6 m,
cross 0.0 m, pitch 5.2 degrees. The existing limits are <=3 m/s sink,
60–75 m/s speed, along 0–2500 m and cross +/-35 m. No rollout is exercised
by this fixture; these are contact results only.

Both tests conditioned failures on optional environment variables. Removed
that behavior: their default exit codes now enforce the stated contracts.
The admission fixture also had zero attitude-model coefficients, reversed
crossrange and tangent-plane altitude/course errors. Replaced conversion with
the production spherical frame and checked round-trip geometry and state
validity. This fixes the test, not the guidance or plant.

Discriminating rerun: `make -C CLanding BUILD=build-research -j2 -k contract-tests`.
Raw output: `hard-contract-gates.log`. Both gates return failure. Corrected
admission fixtures still have 0/2 accepted routes; rejection is native vertical
profile energy feasibility, with solves about 3.4–3.7 s on this host. Samples
remain synthetic telemetry with prescribed forces, not physically consistent
closed-loop handoffs. They disprove the software implication for those inputs,
but do not quantify the live false-admission rate or prove physical impossibility.

Remaining chain: consistent measured-state admission campaign; temporally valid
downstream certificates; planner profiling/deadlines; TAEM energy supervision;
touchdown reachability and Final experiments; identified plant validation;
contact/rollout validation; dispersed end-to-end qualification. These gates
intentionally remain red until the actual contracts are met.

## Experiment 2: reduced forecast admission parity

After commit `4813ce8`, code inspection found the exact-interface branch in
`predictor_simulate_entry_core` was unreachable from the sole public caller
(`stop_at_taem=false`). The active reporting branch instead used only altitude
and speed. Replaced both divergent checks with the shared measured-state
admission envelope, populated from modeled state including course-to-site
error and lift-based load. The forecast remains advisory and reports an
admission opportunity, not a planner certificate or executive completion.

`EntryForecastAdmissionTests.c` calls the public forecast and live envelope.
Before: closing matched, receding gave envelope=false/forecast=true (veto 0x80).
After: closing, receding, range, Mach, altitude and structural-q cases agree.
Additional reciprocal runway checks pass in the regression run. Before testing
used the existing pre-edit predictor object (`make -o prediction/entry_simulation.inc`);
after testing rebuilt it normally. Logs: `forecast-before.log`,
`forecast-after.log`, `forecast-regression.log`. All nine component suites pass;
the same two physical contract gates fail. This resolves reduced forecast
policy inconsistency only. The admission envelope itself still needs downstream
feasibility and temporal validity.

## Experiment 3: paired Final servo/direct-control baseline

Runner: `ShuttleSim/scripts/run_guidance.py`, scenario
`final-mm305-ideal-3p5km-185mps-aoa3.ini`, `--engage engageFinalTest`,
`--backend-build-dir CLanding/build-research --sim-build-dir ShuttleSim/build-research
--skip-build --no-mirror --quiet-progress --max-sim-time 180`; one run adds
`--direct-control`. Profile identified, runtime `Configuration/default.json`,
model=plant (NOT robustness evidence), no speedbrakes. Exact run IDs, effective
configuration and hashes are retained in `final-{servo,fcs}-result.json`.
The modified reduced predictor has no ownership of either Final experiment.

| Mode | Main contact sink | Speed | Along | Pitch | Outcome |
|---|---:|---:|---:|---:|---|
| Ideal servo | 1.821 m/s | 63.253 m/s | 1523.342 m | 7.680 deg | runner failed |
| Direct FCS | 4.323 m/s | 66.815 m/s | 1367.378 m | 5.411 deg | runner failed |

This is different from the built-in-config C fixture (6.05 m/s direct sink).
Configuration and sample cadence differ: do not attribute that difference to
an FCS improvement. Runtime speed gate is 0.85–1.15 times configured touchdown
speed, whereas the C fixture uses 60–75 m/s; agreement remains to be established.

The servo result demonstrates reachable low-sink main contact for this modeled
state/plant. It does not prove live STS-N authority or general Final reachability.
The direct-vs-servo difference warrants response/command telemetry analysis,
not a gain change before plant validation.

Both rollout models contain a decisive defect: `stopped=true` with upward
velocity 8.909 m/s (servo) or 7.093 m/s (direct), main_contact=false and large
nose load. Maximum gear loads are 44.757 g / 35.172 g. Code latches stop from
horizontal speed alone and then freezes the state. These are simulation/model
failures, not successful rollout. The ground-contact integrator, low-speed
body attitude and dissipative friction need discriminating tests before any
rollout qualification. The runner additionally appears to assess an earlier
telemetry sample; final recorded contact=true but rolloutValid=false. That
classification inconsistency must not be used to obscure the physical defects.
