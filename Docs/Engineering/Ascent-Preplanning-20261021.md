# Prelaunch ascent optimization — 2026-10-21

User direction: calculate the efficient ascent trajectory from the actual
vehicle mass, CoM, propulsion and control constraints before flight. The prior
elapsed-UT fix corrected an execution bug, but did not meet that planning goal.

Commit 3dd2ada introduces the default STS/SRB prelaunch direct-shooting search,
read-only vehicle snapshot, saved model/reference, and bounded live tracking.
See `Docs/Ascent/Preplanned.md` for the model, workflow, and explicit limitations.

## Retained experiments

Compact models, reference CSVs and exact results are in
`Docs/Ascent/PreplannedEvidence/`. These are synthetic vehicles, not live STS
identification. The 35-degree gimbal envelope is deliberately generous; it must
not be interpreted as a measured SSME setting.

| Case | Initial mass | Propellant | Predicted cutoff Ap × Pe | Duration | Qualified |
|---|---:|---:|---:|---:|---|
| Baseline | 520 t | 340.217 t | 90.790 × 60.564 km | 225.859 s | yes |
| Additional payload at (0, 8, 1) m | 530 t | 343.736 t | 90.545 × 59.970 km | 249.110 s | yes |

Both cases target 90 × 60 km, change mass/CoM through fuel consumption and
booster release, and pass finer-grid endpoint, peak-Ap, q, G, q-alpha and trim
checks. The additional payload changes both CoM and the selected pitch history;
it is not the same altitude schedule flown more slowly. Peak modeled apoapsis
stays below the permitted 93 km envelope. These results demonstrate planning
behavior and numerical targeting, not actual propellant savings over a tuned
KSP flight or proof of global optimality.

The no-gimbal case with an off-axis payload fails qualification. Search failure
is retained as an unqualified result and never interpreted as proof that the
physical vehicle cannot fly.

## Numerical investigation

An early coarse simulation appeared to reach the target, then missed by several
kilometres on a smaller step. Applying the end-of-step attitude and limiter to
the whole interval biased the result. The retained integrator uses midpoint
translation/atmosphere/propulsion, mean interval throttle, exact first-order
attitude response to a slewing command, and event-aware roll timing. Refinement
uses smaller steps and checks the candidate again at half the step size.
Coarse predictions alone do not permit ignition.

## Validation

- `pytest tests/ascent tests/test_mission.py tests/test_operations.py -q`:
  **141 passed, 12 subtests passed**, with two mission-test background-thread
  warnings after temporary-log directory cleanup. That unrelated cleanup path
  was not modified.
- Four focused tests cover depletion/staging CoM and inertia, full optimization
  to both apsides without lofting, rejection of missing authority, and saved
  model/reference plus bounded feedback.
- Offline baseline and changed-payload optimization completed independently of
  KSP. Models and parameters are retained for reruns with
  `python -m ksl.ascent.preplan`.
- A read-only standard-kRPC connection attempt was refused. No live vehicle
  snapshot, launch, staging, controls, warp or save change was performed. Jev was
  unavailable. Actual STS drag, control response, orbit accuracy and efficiency
  remain to be flight-validated.
