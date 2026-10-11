# Held-out L5/L6/L7 aerodynamic force comparison

The current October 10 polar represents the observed transonic force ratios
reasonably well at matched measured states. Low L/D is also present in native
telemetry; it is not solely an artifact of either table. The October 11
supersonic correction is not transferable across all bands and is not adopted.
No fitting, guidance change, simulator qualification or live flight occurred.

The comparison uses the original L5, L6 and L7 native logs (October 10–11),
held out from the documented October 9 polar source flights. It evaluates
`q * 250 m² * coefficient(Mach, AoA)` using each measured state, separately
from atmosphere prediction, attitude propagation and path tracking. Forces
come from native cached lift/drag vector magnitudes (`telemetry.inc`), rather
than trajectory acceleration. Samples require MM304/TAEM, q >= 300 Pa,
Mach 0.5–3.5, AoA 0–30°, sideslip within 5°, no airbrakes and thrust <= 1 N.

| Mach band | Flight | Median measured L/D | Measured / October 10 drag | Measured / October 11 drag |
|---|---|---:|---:|---:|
| 1–1.5 | L5 | 0.682 | 1.074 | 1.074 |
| 1–1.5 | L6 | 0.684 | 1.077 | 1.077 |
| 1–1.5 | L7 | 0.690 | 1.075 | 1.075 |
| 1.5–2 | L5 | 0.848 | 1.048 | 0.725 |
| 1.5–2 | L6 | 0.853 | 1.049 | 0.716 |
| 1.5–2 | L7 | 0.864 | 1.029 | 0.717 |
| 2–2.5 | L5 | 0.876 | 0.883 | 0.636 |
| 2–2.5 | L6 | 0.876 | 0.895 | 0.640 |
| 2–2.5 | L7 | 0.880 | 0.853 | 0.622 |
| 2.5–3.5 | L5 | 0.617 | 1.195 | 0.978 |
| 2.5–3.5 | L6 | 0.816 | 0.834 | 0.645 |
| 2.5–3.5 | L7 | 0.618 | 1.213 | 0.989 |

These are duration-weighted medians, not independent statistical confidence
intervals. Transonic bands have about 16–36 seconds of selected flight data
per flight. Ratios below 1 indicate table overprediction. October 10 lift
ratios in Mach 1–2.5 stay within about 5% of unity. Its drag RMS error is still
10–20% in several bands, and Mach 2.5–3.5 differs between L6 and L5/L7; this
check does not qualify the table or justify a pooled coefficient correction.
The observed incidence spans differ across bins, so the results do not
validate unsampled AoA or control-surface configurations.

This evidence rejects simply reducing transonic drag to avoid the predictor's
energy loss. The next inquiry is how a vertical reference can accommodate
strong deceleration while preserving the existing Final interface and flight
constraints. The current 24° slope constraint and shallow reference shape
need explicit feasibility analysis. No gate should be lowered just to pass
these inlet samples; the results do not establish physical impossibility.

Run `python3 Docs/Experiments/STS-Landing-20261011/Aero-Heldout-Crosscheck/analyze.py`
from the repository root. [result.json](result.json) records exact log/table
hashes, selection policy, duration, incidence/q coverage, force bias/RMSE,
and weighted 5th/95th percentile ratios. Repeated/non-forward UT and gaps over
0.5 seconds are excluded. Both bilinear interpolators were checked at every
table knot and cell midpoint. Production source was unchanged; prior passing
backend and mission tests were not rerun for this analysis.
