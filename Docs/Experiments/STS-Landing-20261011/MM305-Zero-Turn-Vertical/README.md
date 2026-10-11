# Zero-turn vertical delivery diagnostic

Removing all lateral demand lets the native tracker closely follow several
24°-limited profiles, but none of 448 sampled profiles also meets the current
160 ±12% m/s arrival-speed diagnostic. The fastest completed trial passing
slope, altitude tracking and endpoint checks arrives at 127.814 m/s, below the
140.8 m/s lower bound. The turn is therefore not the only failure in these
sampled analytic profiles. This is a finite diagnostic, not a physical
impossibility proof, new route qualification, or basis for lowering a gate.

Run `.venv/bin/python Docs/Experiments/STS-Landing-20261011/MM305-Zero-Turn-Vertical/run.py`
from the integration checkout. The runner uses a temporary fixture and a
temporary planner object exposing the unchanged private native vertical-profile
function. Production source/executables are preserved. Source, inputs, binary
and log hashes are in `result.json`; every trial is retained in `trials.csv`.
Raw logs are ignored. Compilation retains native `-Werror` flags.

The configured sample-0 inlet is physically admitted: nominal 26 km, Mach 2.5,
30 km before the runway, aligned, AoA 18°, FPA −8°, STS October 10 aero,
25,123.15 kg, native atmosphere and attitude servo. Reference endpoint is the
configured Final altitude, approximately 2,980.20 m ASL, from 8 km / 20°.

First search seven path lengths 40–100 km every 10 km, each with 13 bow offsets
spanning ±0.3 of inlet-to-endpoint altitude drop (91 trials). Refine 60–80 km
every 1 km with bows −2,000 to +2,000 m every 250 m (357 additional trials).
Native Hermite altitude/FPA/vertical-curvature evaluation and initial upper
vertical-authority shaping are retained. Length denotes integrated ground
travel, not a valid runway route. The lateral reference is reset to the current
position/course with zero curvature each tick, so demanded lateral acceleration
is asserted zero. No fictitious relocation, speed reset or energy replenishment
occurs. This moving lateral anchor deliberately removes route following and
turn cost; it is not a realizable runway endpoint or a mathematical upper bound
on all banked trajectories.

Native unpowered propagation uses at most 0.5 s ticks, trapezoidal ground-speed
station integration and a shortened last tick. Pressure, normal lift load,
ground and 420 s timeout are checked every tick including the exit. The analytic
reference slope is sampled at 4,097 uniform stations, a finite estimate of its
peak rather than an exact extremum proof. Diagnostic thresholds are unchanged:
24° reference, 300 m maximum altitude error, 150 m endpoint altitude error,
160 ±12% m/s speed. No landing or Final admission is inferred from these checks.

| Result | Value |
|---|---:|
| Trials | 448 |
| Completed | 411 |
| Timeout / ground before endpoint | 33 / 4 |
| Slope-compliant completed | 267 |
| Slope + tracking + endpoint passing | 149 |
| All four diagnostics passing | 0 |
| Largest observed pressure / normal load across trials | 19.64 kPa / 2.44 g |

All sampled trajectories stayed under configured 45 kPa / 3.5 g limits up to
their termination. The best tracked slope-compliant case, length 68 km and bow
+500 m, has 14.917 m maximum altitude error and 0.562 m endpoint error, but
arrives at 125.996 m/s. The fastest slope/tracking/endpoint-compliant case is
length 70 km / bow +1,150.990 m: slope 23.956°, tracking 19.451 m, endpoint
7.798 m, elapsed 244.890 s, speed 127.814 m/s, actual FPA −21.650°.

A 60 km / bow +1,150.990 m case tracks within 16.934 m and arrives at
130.625 m/s, but reference slope reaches 27.240° and speed still fails. A fast
269.446 m/s case has a ~50° reference and ends over 7 km above target; fast
travel alone is not successful delivery. These observations retain energy and
endpoint cost instead of counting a short/fast segment as a solution.

No gate, profile coefficient, production command or golden was changed. No full
five-state MM305 batch, combined lateral route, Final tail, plant qualification
matrix or live flight was performed. The next investigation should inspect
which arrival-speed/geometry contract the live low-energy Final resolver and
the planner each enforce, and whether a coupled profile can preserve that
contract. Do not interpret this diagnostic as permission to reduce the speed
requirement or a proof that all possible profile families fail.
