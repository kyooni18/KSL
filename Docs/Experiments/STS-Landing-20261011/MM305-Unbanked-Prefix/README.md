# Unbanked descent before the turn

A true propagated unbanked prefix did not rescue focused sample-0 downstream
feasibility. Four prefix cases and the reproduced baseline each searched 2,184
smooth profiles; none was reference feasible, and `mm305_plan` found no route.
This rejects these finite fixed-incidence prefixes as an admission remedy,
not every composite path or global reachability.

Run `.venv/bin/python Docs/Experiments/STS-Landing-20261011/MM305-Unbanked-Prefix/run.py`
from the integration checkout. The runner compiles a temporary replacement
fixture, links production objects, and preserves source/input/binary/log hashes
in `result.json`. No production source or executable is replaced. Raw logs are
ignored. This diagnostic uses STS October 10 aero, native attitude servo,
25,123.15 kg, configured atmosphere, and sample 0: nominal 26 km, Mach 2.5,
30 km before the runway, aligned, initial FPA −8°, AoA 18°.

The original physical capture must pass. Prefix propagation uses zero commanded
bank and 3° or 10° commanded incidence, native attitude state, and unpowered
native forces. Distance integrates ground speed by the trapezoidal rule; ticks
are at most 0.5 s with a shortened final tick. It is a nearly straight ground
track, not a forced fixed-course line: final cross track stays below 0.55 m
and course drift below 0.002°. The downstream route is seeded from the complete
propagated exit state, so its initial tangent follows that state. Every prefix
sample, including the exit, checks dynamic pressure, normal lift load, ground
clearance and elapsed-time limit. Measured peak pressure is at most 4.29 kPa,
normal load 0.616 g, against configured 45 kPa/3.5 g limits.

| Prefix / commanded AoA | Elapsed s | Exit altitude above runway m | Exit Mach | Energy change kJ/kg | Completed profiles | Best tracking m | Its exit speed m/s |
|---|---:|---:|---:|---:|---:|---:|---:|
| None | 0 | 25,932 | 2.497 | 0 | 1,287 | 3,040 | 70.107 |
| 4 km / 3° | 5.44 | 25,308 | 2.416 | −25.63 | 942 | 3,852 | 65.130 |
| 8 km / 3° | 11.11 | 24,499 | 2.339 | −50.73 | 556 | 3,299 | 45.586 |
| 8 km / 10° | 11.13 | 24,597 | 2.320 | −53.96 | 565 | 3,360 | 67.436 |
| 12 km / 10° | 17.09 | 23,735 | 2.226 | −82.18 | 418 | 3,200 | 64.125 |

Energy change is air-relative kinetic energy plus gravitational potential;
it includes rotating-atmosphere effects and is not an exact inertial drag-work
identity. Prefix drag work is also recorded. The baseline's 2,184/1,287 trial
counts reproduce the earlier fixture. All five searches return a valid planner
result with `found=0`; exit code 1 denotes no downstream route, not execution
failure. Best tracking remains far above the 300 m bound. The 12 km case can
end within 115 m of endpoint altitude but still fails tracking, 24° reference
slope (24.226°), and speed delivery.

This is a staged feasibility diagnostic, not the original admission property
batch. It does not adopt a route, reinterpret the propagated exit as a new
physical handoff, construct a composite path for full replay, or qualify a
landing tail. Even a positive downstream result would require combined-route
validation from the original state. No result was positive. The production
24° reference, 300 m tracking, 150 m endpoint, speed and Final admission gates
remain unchanged. No full five-state batch, plant qualification matrix, new
compile-time tests or live flight was performed.

A fixed-incidence straight prefix loses energy while moving closer to the
runway and leaves essentially the same later turning problem. A useful next
investigation should trace the longest-prefix downstream candidate against
the baseline to locate whether its first failure is the same transonic
combined turn/pull-up deficit; do not adopt another untraced prefix variant.
