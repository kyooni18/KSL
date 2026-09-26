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

## Experiment 4: Final campaigns and the Final energy band (Claude)

Tool: `ShuttleSim/scripts/run_final_campaign.py` (runway-relative Final
checkpoints, mirrored cross-track pairs, simulator-truth classification,
`--ld-band` required-L/D filter, outcome distributions and mirror asymmetry
in `summary.json`). Profile identified, model = plant (`--nominal-plant`),
ideal servo, no speedbrakes, seed 1, 24 cases. Required L/D is
(target - along) / (h + (V^2 - v_td^2)/2g) with v_td 75 m/s, target 375 m.
Campaign directories are under `ShuttleSim/campaigns/` (not committed).

Mechanisms found and fixed (commits 0e389fb, 6e51dc1, 5e3386f):

1. Lateral capture ignored the Final roll limiter (0.99 deg/s^2): ±300 m
   cross-track oscillation. Median |cross| at 30 m height 249 -> 86 m.
2. Preflare profile used s = s_sgs + sqrt(2a dh), whose required pull grows
   without bound as sink nears the shallow-glide sink; the latch used the
   correct s^2 = s_sgs^2 + 2a dh. Late hard pull -> balloon -> stall.
3. Arc hold/pull switch held any flight path (including a climb) until the
   required pull reached 0.9 of design; arc command floored at zero (no push).
4. Linear sink blend from 25 m plus a 0.5 m/s float from 3.5 m spent ~18 s
   near the ground; at g/(L/D) ~3 m/s^2 that is ~50 m/s of speed.
5. Fixed outer-glide speed reference 1.6 v_td: this vehicle is drag-dominated
   at approach speeds (L/D ~1.9-2.3 through the arc, measured), so a 0.25 g
   arc from a 24 deg glide bled 120 -> 85 m/s before the shallow glide.
   Replaced by a point-mass touchdown-speed prediction with learned aero and a
   0.5 g design pull (arc loss scales with arc time).

Feasible-band campaign (required L/D 2.0-3.4), HEAD 0e389fb -> 5e3386f:
touchdown sink median 7.1 -> 2.8 m/s, speed median 43.9 -> 59.1 m/s,
along median 668 -> 305 m, passes 0 -> 2. The C touchdown-quality fixture
(direct FCS) fails at 0e389fb and passes at 5e3386f (sink 1.86 m/s,
62.3 m/s, along 1293 m).

Outcome versus required L/D (both bands, current code):

| Required L/D | Outcome |
|---|---|
| <= 1.6 | energy excess: lands 1300-1560 m or overruns (needs MM305 path lengthening) |
| ~1.8-2.25 | only band reaching the zone at gate speed; flare quality inconsistent |
| >= 2.3 | energy-short: touchdown 46-62 m/s |

Implication for the MM305 -> Final contract: without speedbrakes the usable
Final energy band is narrow; MM305 must deliver required L/D ~1.8-2.25, and
states outside it are infeasible-initial-state, not Final guidance failures.
The nominal handoff fixture (3.5 km, 20 deg, 185 m/s) needs L/D ~1.4.

Open: flare ringing (incidence lag ~1.7 s from pitch wn 1.5, zeta 1.3, 5.9
deg/s rate limit vs a 1 s flare time constant); lateral capture for |cross|
> ~100 m at 3-7 km; a measured-acceleration sink lead was tried and rejected
(p90 sink 9.3 m/s, mirror asymmetry 14 m/s). Not yet run: model != plant
dispersions, direct FCS campaigns.

Build hygiene: `build/sim.o`/`scenario.o` lacked header dependency tracking,
so the touchdown fixture linked a stale AeroTable layout (e3fa2cb).
`mm305-feasibility-test` fails identically on clean HEAD ("2/2 admitted
states have no qualified route") - pre-existing, not caused by this work.

## Experiment 5: why mm305-feasibility-test fails (Claude)

`Validation/MM305FeasibilityPropertyTests.c` admits two states (26 km / M2.5,
30 km before threshold; 22.5 km / M2.35, 28 km, 5 km crossrange) and MM305
finds no route for either ("no HAC join has an energy-feasible native vertical
profile"). Fails identically on clean HEAD and under every model profile
(identified, fitted-legacy; reference-b finds 1/2).

Diagnosis (temporary instrumentation in terminal_solver.c, reverted):
- Every candidate route (20-140 km) was profiled. 20 km routes are too short
  to descend; every route >= 30 km reaches the ground part-way.
- The constant-AoA profile flights are identical at every AoA because the
  lateral demand of the lead turn exceeds thin-air lift: the loop drives AoA
  to its cap and banks 46-69 deg at 26 km / 790 m/s, vertical lift collapses,
  flight path steepens -8 -> -15 -> -26 -> -51 deg, cross-track error grows
  and demand reaches hundreds of m/s^2 at 90 deg bank: a spiral dive to the
  ground ~27 km along a 59 km route at 51 m/s. The planner's rejection is
  physically correct.
- Identified supersonic aero: max L/D ~0.65-0.75 (M1.5-4) vs ~2.8 at M0.3.
  Point-mass straight glides from 26 km / M2.5 reach 3 km altitude after
  46-66 km (38-57 km from 22.5 km / M2.35). These states are only 20-22 km
  from the Final interface, so MM305 must add >=25 km of path by turning at
  a speed where the vehicle cannot turn tightly.

Conclusion: the defect is admission, not the planner. The MM304 -> MM305
admission (`entry_mm305_admission_envelope`) checks a lower energy bound, a
maximum range, Mach/altitude windows, but no energy-excess (minimum range)
bound and no turn-feasibility bound, so it certifies states MM305 cannot fly.
The Shuttle-like built-in TAEM interface range is 96.5 km; the runtime JSON
uses 40 km. Fix options (a design decision): (1) admission requires a
qualified MM305 route (the downstream-feasibility contract; seconds per solve,
so asynchronous), or (2) a conservative physical energy-excess / minimum-range
veto consistent with the planner, with MM304 targeting a handoff range the
vehicle can actually use. Either makes these test samples inadmissible, so
the test's sample set must come from the real admission set.

## Experiment 6: MM304 -> MM305 downstream-feasibility contract (Claude)

Implemented option 1 (user decision): ownership transfers only with a
replay-qualified MM305 route from a measured MM304 state (<= 10 s old, same
model snapshot); MM305 adopts that route. Planner profiling now brackets the
feasible route length by bisection (commits 37957c0, 8e65364). Test states
from 26 km / M2.5: 28-30 km out and 70-80 km out have no route (held in
MM304); 60 km qualifies (7.2 km HAC, 47 km lead). `make test` all green.
Open: MM304 must target the ~45-66 km handoff window; no live or full-chain
sim run yet with the contract; live KSP checkpoint `50km-Main` is available
(not yet flown).
