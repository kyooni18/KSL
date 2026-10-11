# Measured-state restart and landing matrix

Earlier fixtures seeded course from body heading. At the first measured TAEM
frame these differ by 11.37° for L5 and 19.48° for L7. That changes both the
path and the incidence of a banked vehicle. The new fixtures reconstruct
surface velocity from native inertial velocity minus planetary rotation, then
preserve measured body Euler orientation and angular rates separately.

`prepare.py` checks reconstructed speed, FPA, AoA and sideslip against the
native frame and records input hashes, geometry and rejected cached frames in
`sources.json`. L5 uses TAEM UT 2162.610 and Final UT 2412.130. Independent L6
recording `2026-10-10T08-28-20Z-STS-vehicle.jsonl` uses UT 2163.990 and
2415.030; 14 earlier Final frames failed coherence checks. Original L7 TAEM
uses UT 6566.590; Final comes from the separate approved repeat flight at UT
6924.610, after rejecting 17 cached frames. These are separate continuations,
not a continuous L7 trajectory. The high case is synthetic: L5 TAEM with
altitude increased by 2000 m, preserving the other inlet values.

Commit 61fb088 adds optional complete body-pose/rate groups to scenario loading.
All 21 runs preserve six measured Euler/rate values in the initial simulator
packet within 0.0001 and course within 0.001°. Runtime checks reject incomplete
pose/rate groups. `make test` passes 104 tests and 44 subtests with historical
L5/L7 goldens unchanged. No production attitude coefficients changed.

## Results

`run.py` compares three unqualified body-coordinate plants, with fixed flare
and unchanged configuration. `result.json` and `high-result.json` retain
commands, binary/input provenance, initial packets, phases and acceptance.

| Plant | L5 MM305 speed / sink / along | L6 MM305 | L7 MM305 | High MM305 |
|---|---|---|---|---|
| Existing coefficients | 63.06 / 1.30 / 754 | **Pass:** 65.86 / 1.69 / 307 | 35.82 / 15.80 / −7409, gear up | **Pass:** 68.40 / 1.56 / 312 |
| L4 fit | 58.50 / 1.75 / 747 | 59.61 / 1.53 / 241 | 65.62 / 3.02 / 1951 | 57.08 / 0.67 / 777 |
| L7a fit | 47.47 / 0.88 / 1322 | 61.86 / 1.04 / 412 | 57.46 / 2.62 / 2205 | 53.20 / 1.29 / 823 |

Units are m/s, m/s and m. Unmarked cells fail acceptance. Every MM305 case
observes Final and Flare; existing-coefficient L7 reaches contact with gear up.
The pinned L4-fit CLI matrix (`run_matrix.py`, `matrix.json`) exposes
`l5-measured-mm305`, `l6-measured-mm305`, `l7-measured-mm305` and
`high-measured-mm305`. All four reach Final and Flare with gear down, but
**all four fail landing acceptance**: L5/L6/high speed, L7 contact/rollout.
Phase reachability is verified; successful landing on this matrix is open.

Isolated Final restarts also distinguish the plants: L4 passes L5 and L6,
while L7a gives L5 58.52 m/s, 1.27 m/s sink, 639 m along (live about
55.7 / 1.13 / 621). No model passes the full measured matrix or establishes
transferable live fidelity. All three L7 Final commands are rejected while
Idle; their zero-valued contact summaries are defaults, not touchdown data.

## Limits and next work

The restart reconstructs scalar/geographic motion and body state, not exact
Cartesian history, actuator state or guidance memory. Runway elevation/ground
model remain approximate. A Final-test engagement does not restore live
low-energy delivery memory. These limitations prevent attributing each failure
solely to a pitch coefficient. Predictor command servo still differs from the
body FCS plant, and roll/yaw dynamics remain generic. Earlier body-model and
MM304 comparisons used approximate course inlets; correct MM304 geometry
before accepting upstream targeting changes. Historical goldens remain useful
regressions, but do not establish live fidelity. No flare tuning, production
plant switch or gate relaxation is accepted from these experiments.
