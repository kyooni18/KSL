# Propagated-authority reference experiment

A reference integrated using the evolving lift envelope reduces some tracking
errors but does not deliver a qualified Final state. Both response variants
fail focused admission with zero joint reference-feasible profiles and zero
gate/adoption violations. The experiment is rejected as an admission remedy.
Production reference/Final/landing constraints are unchanged, including the
24-degree reference slope, 300 m tracking error, 150 m endpoint altitude error
and 160 m/s alignment-speed target with its existing tolerance.

| Run, 2184 trials each | Completed | Joint reference-feasible | Smallest tracking error, m | Same trial max reference descent | Same trial endpoint error, m | Same trial exit speed, m/s |
|---|---:|---:|---:|---:|---:|---:|
| Baseline | 1287 | 0 | 3040.205 | 23.248° | 181.164 | 70.107 |
| 2.05 s response | 1291 | 0 | 532.220 | 53.639° | 1307.509 | 80.214 |
| 8 s response | 1294 | 0 | 488.149 | 21.865° | 17420.947 | 508.809 |

These are joint rows, not separate minima. The 8-second trial is a short route
that ends roughly 17 km above its required altitude, retaining excess energy.
Its small relative tracking error is not a usable Final delivery. The best
slope-compliant 2.05-second trial similarly ends 17.8 km from the required
altitude. The existing endpoint and speed gates are essential and retained.

The temporary solver independently integrates reference FPA and altitude from
the initial state over projected station; it does not reset them to the
actual trajectory to erase tracking error. At each propagated state, it
computes allowable table lift using current Mach/q, incidence ceiling and
load cap. Lateral demand and bank limit define the remaining vertical lift
interval. Gamma-rate demand from the geometric curve and altitude closure is
clipped to this interval, with the existing tracker/servo then controlling the
vehicle. This is an instantaneous envelope, not actuator/inertia qualification.

A finite parameter sweep uses the existing 13 bows and lateral candidate
families. Endpoint altitude/FPA is not automatically restored after a rate
constraint becomes active. The experiment demonstrates why following an
achievable local reference alone does not solve the boundary-value problem:
turn timing, altitude/density history, energy and final capture must be planned
together. A later replay must still verify a generated tabulated profile;
none qualifies here, so later replay/Final/load outcomes cannot be claimed.

[run.py](run.py) compiles a temporary solver object and separate fixture binary,
preserving production source and binaries. Baseline counts match the prior
experiment. Sample 0 is 26 km/Mach 2.5, along −30 km/cross zero, measured mass
25,123.15 kg with the October 10 provisional table and borrowed command servo.
No full five-sample qualification, landing matrix or live flight was run.
No production change was retained, so mission/native suites were not rerun.

Run `python3 Docs/Experiments/STS-Landing-20261011/MM305-Propagated-Reference/run.py`
from the repository root. [result.json](result.json) preserves input/source/
binary/log hashes, contract results and separate slope-compliant joint rows.
Ignored per-run logs are regenerated. All temporary programs compiled and
ran to the expected qualification failure.

Next: test whether delaying the early lateral turn until after a useful
altitude/density change reduces the later turn/pull-up burden, rather than
only bending the vertical reference around a fixed lateral route. Preserve
endpoint, speed and downstream qualification in that experiment.
