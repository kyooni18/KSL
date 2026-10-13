# Intelligent ascent development evidence — 2026-10-13

**No completed live target orbit has been accepted.** These are development
experiments, not a reliability qualification of the final source.

## Automated verification

- Broad Python suite: `pytest -q tests Validation -k 'not golden_replay'` passed
  199 tests and 56 subtests at the final regression checkpoint.
- Final ascent/mission suite: `pytest -q tests/ascent tests/test_mission.py -k
  'not golden_replay'` passed 118 tests and 12 subtests.
- TUI headless verification mounted the app, selected intelligent guidance,
  saved/reloaded its profile and elliptical periapsis, and confirmed that the
  default profile still selects legacy guidance.
- The excluded landing golden replay failed at touchdown speed (65.263 m/s vs
  64.182 m/s expected). Running the same test against Python code extracted from
  pre-change commit `24ca7d0`, using the same native artifacts, reproduced the
  identical failure. Landing files were not changed by this ascent work.

## Independent point-mass experiments

`point-mass-matrix.jsonl`: nine combinations of 10, 30 and 80 tonne initial mass
and 80, 120 and 200 km circular targets. All achieved both apsides within 1 km.

`staged-matrix.json`: changing thrust, Isp, mass and discarded stages; includes a
200 × 90 km target. Both cases met the 1 km apsis tolerance. The 80 tonne case
reached its target before consuming its final configured stage; it must not be
counted as verification of every possible stage transition.

The plant uses an independent RK4 integrator, exponential atmosphere and mass
flow. Attitude is ideal, inclination is zero, and aerodynamic instability,
structural failures, finite control authority and kRPC latency are not simulated.
Its final elements are computed from integrated position/velocity, not copied
from the planner's predicted result. Live inclination accuracy is unverified.

## Live development trials

`live-summary.json` records the outcome and measured values. CSVs are decimated
development traces retaining the first, final and maximum-q samples. Original
full traces remain in their named `runs/` directories. Configuration snapshots
are included. They describe the development code loaded when each run began;
they do not certify later fixes that had not yet flown.

1. Pioneer, Desert site, 100 km / 10°: automatic launch, both depleted SRBs
   separated as a group, core burn reached the target apoapsis. A default-frame
   velocity read falsely reported zero climb during coast and stopped guidance.
   Explicit body-frame reads fixed that defect.
2. Insertion recovery from that flight planned a 163.8 m/s correction. No burn
   progress was accepted because of a large attitude error. Last recorded
   elements were Ap 100.035 km, Pe −70.098 km: **suborbital, not successful**.
   Later code disables competing SAS, checks actual thrust alignment independently
   of roll, budgets alignment time from torque/inertia, and bounds stalled burns.
   Those changes still need a completed live flight.
3. Pioneer, MahiMahi site (latitude −49.80°), 100 km / 55°: climbed to 14.4 km,
   then KSP changed from flight to SpaceCenter/editor and the RPC failure stopped
   guidance. Later ownership guards explicitly detect this transition.

Earlier trials also exposed an autopilot engagement API difference (property
instead of methods), surface/inertial speed confusion, and insufficient ignition
settling time. Focused regression tests cover those contracts.

Observed q reached about 36 kPa with a 25 kPa governor setting. Fixed solid thrust
cannot be reduced by liquid throttle; this is an unresolved flight-envelope
qualification issue, not evidence that the pressure constraint was met.

At the end of this development session KSP remained in the editor. No further
launch was forced while editing was active. Further qualification needs complete
live flights on multiple configurations, especially canted/asymmetric vehicles,
controlled reusable stages, and stricter orbit tolerances.
