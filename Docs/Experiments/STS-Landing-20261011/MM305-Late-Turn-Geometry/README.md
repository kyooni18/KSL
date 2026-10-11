# Gentler opening lead geometry

Constraining early lead curvature changes the chosen cubic routes but does
not qualify the focused STS inlet. All four runs still have zero jointly
reference-feasible profiles, fail non-vacuity and report zero gate/adoption
violations. No production geometry or acceptance constraint changed.

| Variant, 2184 trials each | Completed | Best tracking error, m | Same trial route length, m | Same trial speed, m/s | Same trial slope | Same trial endpoint error, m |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 1287 | 3040.205 | 62326.028 | 70.107 | 23.248° | 181.164 |
| First 4 km, half inlet authority | 633 | 2149.308 | 55602.311 | 108.654 | 26.459° | 8.432 |
| First 8 km, half inlet authority | 549 | 2310.524 | 59242.447 | 107.566 | 26.091° | 6.618 |
| First 8 km, full inlet authority | 684 | 3297.785 | 49548.682 | 86.602 | 29.891° | 406.205 |

The table preserves joint observations from each run's smallest-tracking-error
completed profile. Improved endpoint altitude in the half-authority cases
still accompanies kilometre-scale tracking error, a slope above the unchanged
24° limit and insufficient speed for the 160 m/s alignment target. Neither
smaller tracking error nor accurate endpoint altitude establishes feasibility.

The temporary planner evaluates peak curvature along the first 4–8 km of each
candidate cubic and restricts it to half or all of the current inlet curvature
authority. Existing endpoint tangents and HAC circle primitives are preserved;
handle selection chooses different paths. Native forward regularity, whole
curve peak bounds, profile generation, replay and Final checks remain. The
prefix sampling uses 128 curve intervals and conservatively includes the
first sample beyond the requested distance. The criterion always includes the
original initial curvature and never loosens that original bound.

This is a deliberately conservative search-shaping restriction, not a claim
that inlet authority remains the true limit throughout the prefix. Density
and speed change there. A gentle initial cubic is also not an explicit straight
descent segment: it can shift or distribute turn demand across the whole lead.
The finite experiment rejects this restriction as an admission remedy, not
all possible turn-delay strategies or physical reachability.

[run.py](run.py) compiles a temporary planner object into a separate fixture
binary and preserves production source/backend/fixture binaries. The baseline
matches previous counts. It uses sample 0 only (26 km/Mach 2.5, along −30 km,
cross zero, mass 25,123.15 kg) and the October 10 polar/borrowed attitude servo.
No full five-sample qualification, closed-loop matrix or live flight was run;
no production change was retained, so test suites were not rerun.

Run `python3 Docs/Experiments/STS-Landing-20261011/MM305-Late-Turn-Geometry/run.py`
from the repository root. [result.json](result.json) records hashes, parameters,
joint rows, representative selected geometry and contract results. Raw logs
are ignored and regenerated. All temporary builds ran to the expected failure.

Next: distinguish a true straight descent prefix followed by a tangent-continuous
turn from reshaping a single cubic. Its propagated exit must seed the later
route, and the combined path must still satisfy all endpoint/energy constraints.
