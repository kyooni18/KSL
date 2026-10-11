# Why the unbanked prefix fails

Passive traces show that the 12 km / 10° unbanked prefix does not avoid the
baseline's coupled turn/pull-up deficit. Its shorter remaining lead has an
earlier lateral lift-demand shortfall, and it reaches Mach 1 at higher altitude
with less lift available. Production commands, propagation and gates remain
unchanged. The prefix remains rejected, not adopted.

Run `.venv/bin/python Docs/Experiments/STS-Landing-20261011/MM305-Prefix-Authority-Comparison/run.py`
from the integration checkout. The runner reuses the exact source builders in
`MM305-Unbanked-Prefix/run.py` and `MM305-Profile-Authority-Trace/run.py`, before
their execution sections, and links both temporary sources into a temporary
fixture. Source/input/binary/log hashes are saved in `result.json`. It asserts
that all 2,184 smooth-trial rows and the prefix exit packet exactly reproduce
the uninstrumented run in each case. Compilation uses the native `-Werror`
flags. Raw logs are ignored; CSVs retain five-second history and key events.

The traced routes are each case's best-tracking completed smooth candidate:
baseline length 62,326.028 m / bow −1,150.990 m, and prefix downstream length
53,174.425 m / bow −1,041.155 m. Both mirrored baseline trials match the target;
one prefix trial matches. The baseline table below uses trace 2, which is the
smallest-tracking-error mirror. The comparison does not hold route geometry
fixed: it explains why the respective selected candidates fail.

| Observation | Baseline | 12 km prefix downstream |
|---|---:|---:|
| First lift-vector command-envelope shortfall, downstream elapsed s | 52.5 | 1.5 |
| At that event: requested lift-vector magnitude m/s² | 17.23 | 11.26 |
| At that event: maximum permitted-incidence table lift m/s² | 16.95 | 10.76 |
| First 300 m altitude departure, downstream elapsed s | 9.5 | 11.0 |
| That altitude error m | +324.55 | +314.20 |
| Largest altitude departure m | −3,040.20 | −3,199.95 |
| Command-envelope shortfall duration s | 45.0 | 84.5 |
| Mach-1 crossing downstream elapsed s | 59.0 | 41.0 |
| Mach-1 crossing total elapsed including prefix s | 59.0 | 58.09 |
| Mach-1 crossing altitude ASL m | 15,762 | 17,577 |
| Mach-1 crossing dynamic pressure Pa | 4,042.72 | 2,832.29 |
| Mach-1 crossing permitted-incidence maximum lift m/s² | 8.48 | 5.94 |

At 1.5 s of the prefix's downstream route, vertical lift demand is clipped to
zero, while lateral demand is already 11.26 m/s². Even the optimistic maximum
table lift is only 10.76 m/s², and bank-limited lateral availability is 10.11.
Actual AoA is 12.60° and bank −8.58°; the command is still ramping toward 18.79°
and −16.43°. Thus response lag also matters, but removing lag alone cannot
supply the requested lateral acceleration within that incidence envelope.
The fixed prefix has moved closer to the runway without propagating authority
constraints into the remaining lead geometry. This finite observation does
not establish that every alternative lead has the same failure.

By 11 s, the route asks for a −19.70° reference FPA while actual FPA is −14.07°.
Vertical demand remains zero, but the positive lift required for the turn keeps
the craft above its reference. Later the native tracker tries to pull up while
still at approximately 70° bank. At 48.5 s, actual FPA −38.43° versus reference
−24.20°, requested vertical/lateral acceleration is 46.36/−7.66 m/s² versus
7.11 m/s² maximum allowed-incidence table lift. Largest accumulated altitude
loss appears at 85 s even after FPA has nearly recovered.

Adding the 17.087 s prefix makes the Mach-1 crossing time nearly the same as
the baseline. At that crossing, the prefix case is 1.82 km higher; pressure and
maximum permitted-incidence lift are about 30% lower. It still has approximately
70° bank and actual FPA −31.59°. The command-cap transition at Mach 1 further
reduces table lift from 10.11 to 5.94 m/s², while requested lift-vector magnitude
rises from 14.68 to 15.72. The baseline suffers the same transition from 14.58
to 8.48. Both have deficits before the cap transition; removing the cap is not
validated as a remedy.

The lift envelope is evaluated at allowed AoA knots plus the command ceiling,
without load cap or actuator limits. It is optimistic, not an absolute physical
bound: actual incidence can temporarily exceed the command ceiling. Shortfall
means requested vector magnitude exceeds this envelope by more than 0.25 m/s².
Elapsed times are downstream unless explicitly including the prefix. The prefix
itself remains within pressure/load bounds; its cost and state are retained.

Both planner runs still return `found=0`. No original-state handoff, composite
route, Final tail, plant qualification or landing acceptance is established.
No full five-state batch, new compile-time tests or live flight was performed.
The useful next direction is a coupled altitude/energy/lateral-curvature search
that preserves endpoint and speed constraints; further untraced fixed-prefix
variants or cap removal would not follow from this evidence.
