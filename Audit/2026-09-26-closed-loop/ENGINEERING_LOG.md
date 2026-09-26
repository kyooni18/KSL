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
