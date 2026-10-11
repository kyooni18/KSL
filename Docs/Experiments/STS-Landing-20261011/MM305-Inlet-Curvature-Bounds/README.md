# Turn-aware inlet curvature bounds

Adding a lower inlet curvature bound does not repair sample-0 admission.
Both baseline and experiment test 2,184 smooth profiles, complete 1,287 and
find zero jointly feasible profiles. All 2,184 printed joint trial rows are
identical between runs. Both focused fixtures fail non-vacuity with zero
handoff/adoption violations. No production change is adopted.

The existing inlet limiter bounds only excessive upward curvature. The
experiment also accounts for the minimum positive vertical lift accompanying
lateral acceleration within the bank limit:

```
vertical_min = abs(lateral_acceleration) / tan(bank_limit)
curvature_min = (vertical_min - free_flight_normal_demand) / (airspeed * ground_speed)
curvature_target = clamp(reference_curvature, curvature_min, curvature_max)
```

The existing finite sag correction then adjusts initial curvature while
preserving inlet altitude/FPA and the downstream analytic profile. The
experiment rejects an empty curvature interval rather than relaxing authority.
This is an inlet-only correction; it does not make the whole curve feasible.

The lower bound is activated 1,336 times and the existing upper bound 762 times
among profile-generation calls. Passive traces prove the correction reaches
the analytic reference. For the previously selected route (62,326.028 m,
midpoint bow −1,150.990 m), initial curvature changes from −2.38670e−5 to
−1.06630e−5 rad/m, using a +3.99573 m bump over 3,081.579 m. Among 42 early
packets from both mirrored turns, reference altitude differs by at most
3.94886 m and reference FPA by 0.22174 degrees. Actual altitude, actual AoA,
commanded AoA and bank remain exactly identical at printed precision. This
rules out an experiment that failed to modify the reference; its small change
does not change the chosen/shaped controls in those frames.

The unchanged best completed tracking trial still has a 3,040.205 m altitude
error, exits at 70.107 m/s and ends 181.164 m away from the target altitude.
The later transonic turn/pull-up shortage is not addressed by this local bump.
The finite experiment does not prove physical unreachability or justify
loosening the existing 300 m tracking, 150 m endpoint or Final-speed constraints.

[run.py](run.py) compiles temporary planner and passively instrumented solver
objects into a separate fixture binary. Production source and binaries are
preserved. It compares baseline with an environment-enabled lower bound;
all aerodynamic, attitude, profile, replay and Final gates remain unchanged.
Only focused sample 0 is run, not the full qualification batch or flight
matrix. The zero-option baseline is asserted against previous counts/results.

Run `python3 Docs/Experiments/STS-Landing-20261011/MM305-Inlet-Curvature-Bounds/run.py`
from the repository root. [result.json](result.json) records hashes, joint
trial results, curvature correction examples and all 42 early packets per
mode. Raw logs are ignored and regenerated. Both temporary builds compiled
and ran to the expected qualification failure; no production change was
retained, so mission and native test suites were not rerun.

Next: move from inlet-only corrections to a propagated path/energy envelope.
The current reference can demand infeasible downward rotation while turning,
and later an infeasible pull-up after slowing. Satisfying either boundary
alone is insufficient to deliver a qualified Final state.
