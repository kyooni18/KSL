# Rejected MM305 profile authority trace

The slope-compliant, smallest-tracking-error smooth profile fails through
coupled lateral and vertical authority demands. It first stays above the
reference, then loses altitude in a heavily banked transonic turn. A local
altitude adjustment alone does not address both failures. No production
command, coefficient or acceptance constraint was changed.

Instrumentation selects route length 62,326.028 m and midpoint bow −1,150.990 m
in the unchanged sample-0 search. Both mirrored turns were recorded. Trace 2
matches the previously reported smallest-error trial; trace 1 has a slightly
larger tracking error due to the spherical/rotating geometry.

| Trace 2 observation | First 300 m departure | Maximum command-envelope shortfall | Largest altitude departure |
|---|---:|---:|---:|
| Elapsed, s | 9.5 | 68.0 | 97.5 |
| Projected route station, m | 6543 | 32344 | 36764 |
| Altitude error, m | +325 | −212 | −3040 |
| Speed, m/s | 696.5 | 264.0 | 194.0 |
| Actual FPA, degrees | −9.72 | −38.51 | −22.58 |
| Reference FPA, degrees | −15.32 | −23.09 | −22.62 |
| Actual bank, degrees | +60.27 | +70.00 | +0.04 |
| Required vertical lift acceleration, m/s² | 0.00 | 44.04 | 18.70 |
| Required lateral acceleration, m/s² | 6.63 | 25.62 | 0.24 |
| Maximum table lift at permitted command incidence, m/s² | 9.01 | 9.64 | 19.94 |

Initially the reference asks for more rapid downward rotation while the turn
requires positive lift. The tracker clips negative vertical lift demand to
zero, but satisfying lateral demand within its bank limit still supplies
positive vertical lift. At 9.5 s, selected vertical lift is 2.45 m/s² and
actual vertical lift is 3.57 m/s²; the vehicle cannot follow that reference
with the current positive-incidence command allocation. This is not an
initial shortage of maximum lift.

By 58.5 s, Mach 1.015, the requested acceleration vector already exceeds the
available incidence envelope: vertical 13.86 and lateral 15.04 m/s² versus
maximum lift 14.58 m/s². At 59 s, Mach 0.999, the subsonic incidence ceiling
reduces maximum permitted-incidence table lift to 8.48 m/s². Actual incidence
remains above the new command cap temporarily as the shaped command decays.
The discontinuity compounds an existing turn/pull-up deficit; removing the
ceiling alone is not established as a fix. The trace spends 45 seconds with
requested lift-vector magnitude more than 0.25 m/s² above this envelope.

At 97.5 s the vehicle has nearly recovered the reference FPA and now has
sufficient model lift, but accumulated altitude error is 3 km. It approaches
the exit at only about 70 m/s. Endpoint geometry does not restore the lost
energy. Plan lateral curvature and vertical descent against the evolving
Mach/q/incidence envelope, rather than sizing them independently at the inlet.

Maximum lift here is evaluated at the polar's incidence knots and the allowed
command ceiling, with no g-load cap or actuator limits. It is not an absolute
instantaneous bound: observed model incidence can exceed a newly lowered
command ceiling. All these observations concern the provisional predictor,
not an identified STS attitude plant or proof of physical unreachability.

Run `run.py`, then `analyze.py` in this directory using Python from the
repository root. The runner compiles an instrumented solver object into a
temporary fixture binary, preserving production source and binaries. Native
search, command shaping, propagation and gates are unchanged. The fixture
still fails non-vacuity with no qualified route. [provenance.json](provenance.json)
contains input/source/binary/log hashes; [result.json](result.json) preserves
joint observations, and trace CSVs give a 5-second history plus key events.
The raw trace log is ignored and regenerated. No full qualification matrix
or live flight was run; no production change was retained.
