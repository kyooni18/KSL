# MM305 provisional aerodynamic table comparison

Changing only the existing supersonic aerodynamic table does not qualify the
focused STS admission sample. No coefficient, production profile or gate was
changed. This is evidence about the implemented search, not a proof of physical
unreachability. Both polars are provisional and use extrapolation.

| Sample 0, 26 km / Mach 2.5 / along −30 km | October 10 table | October 11 table |
|---|---:|---:|
| Smooth trials | 2184 | 2184 |
| Completed | 1287 | 1368 |
| Jointly feasible | 0 | 0 |
| Closest endpoint altitude error, m | 2.016 | 0.003 |
| Same trial exit speed, m/s | 95.251 | 109.673 |
| Same trial maximum reference descent, degrees | 36.587 | 33.138 |
| Same trial maximum altitude tracking error, m | 8036.088 | 4868.963 |

Both invocations fail the retained non-vacuity assertion with zero gate/adoption
violations. The October 10 run is the previous diagnostic baseline, collected
before the diagnostic logging commit. The October 11 run uses source `7eb7bdd`.
The diagnostic-only changes do not alter trajectories. No full five-sample
batch was run with the alternative polar.

The best tabulated positive-incidence L/D within 0–30 degrees is only 0.894
at Mach 1.5 in both tables. At Mach 2.581 it is 0.834 (October 10) or 0.596
(October 11), corresponding to unbanked steady glide angles of 50.17 or
59.22 degrees. Those are far steeper than the configured 24-degree TAEM
reference limit. A transient shallow descent can trade speed for lift; these
steady estimates do not establish impossibility. They do identify a strong
energy cost that must be reconciled with the aerodynamic data and the planned
profile before trying to qualify a solution. The maxima are over tabulated
rows, not a continuous optimizer or a live measurement.

The October 11 profile comment documents scaling from October 9 entry data
(Mach 2–4 drag previously 9–51% low). It supplies higher drag in this band,
not a remedy for exhausted terminal energy. Attitude parameters, atmosphere,
inlet, mass and admission constraints were held constant.

Reproduce the alternative from the repository root after building the fixture:

```sh
KSP_LANDER_MODEL_PROFILE=sts-live-20261010 KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS=1 CLanding/build/mm305_feasibility_tests --configuration Configuration/sts-live.json --mass-kg 25123.15 --sample-index 0 --aero ShuttleSim/reference-model/sts_aero_live20261011.csv
```

[analyze.py](analyze.py) reads ignored local `baseline.log` and `aero11.log`,
and both checked-in tables. [result.json](result.json) retains joint trial
rows, per-Mach L/D estimates, hashes and reproduction arguments. No production
source changed, so the previously passing test suites were not rerun.

Next: compare these Mach-band force predictions with held-out L5/L7 native
force telemetry, then assess an energy-aware vertical profile. Do not lower
speed, slope or tracking gates to make this sample pass.
