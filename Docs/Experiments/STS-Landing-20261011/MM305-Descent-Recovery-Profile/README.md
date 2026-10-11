# Sustained descent and recovery profile experiment

This profile family does not repair focused STS admission. Neither its
production-bound 24° run nor two explicitly broader 36° reference-bound runs
qualifies any route. The experimental bound was not adopted. Current
production guidance, configuration, tests and acceptance gates are unchanged.

| Variant | Reference bound | Recovery begins | Completed / 2184 trials | Joint reference-feasible | Best tracking error, m | Same trial exit speed, m/s |
|---|---:|---:|---:|---:|---:|---:|
| Baseline Hermite + bow | 24° | existing curve | 1287 | 0 | 3040.205 | 70.107 |
| Descent/recovery | 24° | 70% station | 1448 | 0 | 2899.520 | 74.488 |
| Descent/recovery | experimental 36° | 70% station | 1405 | 0 | 3238.414 | 89.234 |
| Descent/recovery | experimental 36° | 85% station | 1393 | 0 | 3428.520 | 68.107 |

All four focused fixtures fail non-vacuity, with zero gate/adoption violations.
The table uses joint observations from the smallest-tracking-error completed
trial, not unrelated minima. The best new 24° trial meets endpoint altitude
within 44.537 m but misses the altitude reference by 2.90 km and exits far
below the unchanged Final alignment target of 160 m/s. Endpoint accuracy
alone is not feasibility. None of the broader-bound variants even reaches
the joint slope/tracking/endpoint reference gates.

The temporary analytic profile smoothly ramps from the measured inlet slope
to a constant interior slope, then smoothly recovers to the Final slope. The
interior slope is solved analytically from the altitude integral so both
endpoint altitudes are preserved. Existing 13 midpoint-bow samples parameterize
the first ramp completion at 5–65% of station; the two recovery fractions
provide separate searches. Endpoint slopes remain −8°/−20° in this fixture.
The existing initial sag/authority correction is retained.

The shape is a finite geometric family, not a state-dependent energy/authority
optimizer. It may transition to steep descent too early or too late for the
actual Mach/q history and lateral maneuver. The recorded L5/L6 steep segments
therefore do not prove this parameterization reproduces their paths.

The 36° variants deliberately change the *temporary* configuration's
`taemGlideSlope`, broadening the reference-slope gate and changing the existing
lead-length shortlist heuristic. They are not qualification under the current
24° contract, and the comparison is not an isolated ceiling change. No
tracking, endpoint, speed, load, q, replay or Final gate is relaxed. Those
later gates cannot be claimed verified because the candidate profiles fail
before qualification. A production-bound pass is not obtained.

[run.py](run.py) creates a temporary planner source/object, separate fixture
binary and disposable configurations. It never replaces production source,
backend or fixture binaries. Sample 0 uses 26 km altitude, Mach 2.5, along
−30 km, cross zero, mass 25,123.15 kg and the October 10 aerodynamic table.
The baseline is asserted against prior counts and tracking results. Full
five-sample qualification, landing matrix and live flights are not run for
these rejected variants. The unchanged production suites were not rerun.

Run `python3 Docs/Experiments/STS-Landing-20261011/MM305-Descent-Recovery-Profile/run.py`
from the repository root. [result.json](result.json) records hashes, bounds,
shape parameters, per-run contract results and joint rows. Ignored logs are
regenerated. All temporary builds ran to the expected qualification failure.
The analytic altitude integral was checked at both endpoints for 24
length/ramp/recovery combinations; no additional compile-time tests were added.

Next: evaluate a path reference tied to propagated Mach/q and lateral lift
reserve, rather than station-only ramp fractions or a steeper scalar ceiling.
Every eventual candidate still needs production-bound full-route, Final and
landing qualification, plus a qualified plant before flare tuning.
