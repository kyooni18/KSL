# Expanded radius search for focused MM305 admission

Larger circles do not repair the focused sample's admission failure. The
instrumented search tests 2,808 smooth profiles (baseline 2,184); none is
jointly feasible. The fixture still fails non-vacuity with one physical inlet,
zero qualified handoffs and zero gate/adoption violations. No production
change was adopted. This finite search does not establish unreachability.

The inner search treats its supplied radius as an upper bound and samples
seven radii down to 3 km. The outer admission planner already supplies both
12 km and 18 km upper bounds. Thus the baseline already includes larger
circles than the configured 12 km nominal radius. The temporary experiment
retains both native searches and adds 1.5× and 2× each supplied upper bound:
18/24 km in the first pass and 27/36 km in the second. The 18 km family is
repeated; only 24/27/36 km are new upper families. Existing pruning and the
six-profile budget per side/radius remain unchanged.

| Radius family | Profile checks | Profile route length range, km | Rejection reasons |
|---|---:|---:|---|
| 18 km (including repeat) | 24 | 22.132–100.058 | 8 too short for slope; 16 propagation timeouts |
| 24 km | 12 | 22.160–115.083 | 4 too short for slope; 8 timeouts |
| 27 km | 12 | 22.173–148.345 | 2 too short; 2 too long for energy; 8 timeouts |
| 36 km | 12 | 22.250–222.519 | 2 too short; 2 too long for energy; 8 timeouts |

The smallest tracking errors among completed large-radius smooth trials are
still about 18.5 km: these are short routes that arrive far above the Final
altitude and require reference slopes over 60 degrees. Their high exit speeds
are not feasible Final delivery. Longer routes commonly fail propagation or
energy completion. Neither choosing the slowest turn nor minimizing route
length alone solves the combined constraints.

[run.py](run.py) creates a temporary copy of taem_candidate_search.c, appends
only the extra radius checks, then links its object into a separate temporary
fixture. Production source, backend and fixture binaries are not replaced.
It preserves the search selection policy and all native profile, replay and
Final gates. The inlet, mass 25,123.15 kg, atmosphere and October 10 polar are
unchanged. This is only sample 0 (26 km, Mach 2.5, along −30 km, cross zero),
not a full five-sample qualification or closed-loop flight.

Run `python3 Docs/Experiments/STS-Landing-20261011/MM305-Expanded-Radius-Search/run.py`
from the repository root. [result.json](result.json) contains hashes,
reproduction arguments, per-radius joint smooth-trial rows, profile summaries
and the failed contract result. The raw search log is ignored and regenerated.
The temporary program compiled and ran to the expected qualification failure;
no production change was retained, so test suites were not rerun.

Next: the inlet curvature limiter currently bounds only excessive upward
rotation. The earlier trace asks for zero vertical lift while spending lift
on a turn. Check the corresponding lower curvature bound from the minimum
vertical lift needed to make that lateral acceleration within the bank limit.
That may improve reference feasibility locally, but transonic deficits later
in the route still require full propagation and qualification.
