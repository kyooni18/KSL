# Offline local lead-profile search

The existing local lead-in altitude adjustment does not repair focused STS
sample-0 admission. Five signed offsets tested 10,920 smooth profiles; none
passed the existing joint slope, tracking and endpoint constraints. Every
focused fixture still fails non-vacuity with zero gate/adoption violations.
This finite search is not a proof of physical unreachability.

| Local bow / altitude drop | Completed | Jointly feasible | Smallest tracking error, m | Same trial exit speed, m/s | Same trial max slope, degrees |
|---|---:|---:|---:|---:|---:|
| +0.0 | 1287 | 0 | 3040.205 | 70.107 | 23.248 |
| -0.1 | 1285 | 0 | 3471.499 | 66.202 | 33.975 |
| -0.2 | 1280 | 0 | 5093.263 | 77.704 | 43.083 |
| +0.1 | 1285 | 0 | 3398.900 | 67.694 | 37.157 |
| +0.2 | 1285 | 0 | 3325.187 | 67.194 | 47.824 |

Each adjustment is a fraction of the inlet-to-Final altitude drop. The existing
local shape starts at route fraction 0.08, peaks at 0.24 and returns to zero
at 0.40. Negative values lower the early reference; positive values raise it.
All 13 native midpoint bows and the unchanged lateral route search remain.
Mass, atmosphere, aero, attitude, inlet and Final/flight constraints are fixed.
The table reports joint observations from the completed trial with smallest
tracking error in each run, not independent minima masquerading as one flight.

Even the baseline's best tracking case misses its altitude reference by
3,040 m (limit 300 m) and exits at 70.1 m/s (target 160 m/s). Its reference
slope is within the 24-degree limit. The local perturbations increase the
best tracking error and steepen those same-route profiles. The closest
endpoint-altitude examples in result.json also fail tracking/slope, despite
small endpoint errors. This rejects simply activating the unused local bow.

[run.py](run.py) derives compiler/linker flags from the existing fixture target,
compiles a temporary copy of terminal_solver.c and replaces only its object
in a temporary fixture binary. Production source files, backend and fixture
binary are not replaced by the experiment. The temporary copy activates the
existing local offset in solve_smooth_profile and retains its analytic metadata
when tabulating a successful reference. All qualification checks are unchanged.
The zero-offset trial matches the previously recorded baseline counts and
closest-altitude speed; the script asserts that comparison.

Run `python3 Docs/Experiments/STS-Landing-20261011/MM305-Lead-Profile-Search/run.py`
from the repository root. Ignored per-offset logs are regenerated beside the
script. [result.json](result.json) preserves solver/source/input/binary/log
hashes, exit codes, trial counts and joint rows. Full five-sample qualification,
closed-loop replays and live flights were not run for this rejected experiment.
No production behavior change was retained, so test suites were not rerun.

Next: trace where the smallest-error, slope-compliant profile leaves its
reference and how much lift/attitude authority is available there. A profile
must preserve both achievable path curvature and energy, rather than merely
matching endpoint geometry or lowering the acceptance gates.
