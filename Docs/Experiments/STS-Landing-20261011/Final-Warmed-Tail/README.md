# Warmed native Final tails

Passive experiment on the continuous native L5/L7 replays, after `c426693`.
The original MM305 prefix retains controller, estimator and direct FCS history.
Temporary instrumentation in Final records the actual evolving plan and
command, and evaluates a force-only snapshot of the late learned model on
future **actual** states/incidences. It changes no command or production file.
This is a retrospective model diagnostic, not a new forecast or a checkpoint
restart implementation. The native surrogate plant remains unqualified live.

## Parity

Both landing summaries and success verdicts equal the prior frozen-response
runs. All 3,654 L5 and 4,928 L7 published physical/control telemetry packets
match exactly, excluding only wall-clock `sim_rate`. This includes attitude,
direct actuator inputs, gear/brakes and contact fields. All 725 L5 and 930 L7
`FINAL_TRACE` records match field-for-field except their process pointer;
intentional post-latch NaN tokens are compared consistently. This verifies the
instrumentation preserved the same warmed native tails, beyond summary parity.

L5 remains accepted: 66.129 m/s, 1.844 m/s sink, 297.613 m along, stop
1101.616 m. L7 remains unaccepted: 61.929 m/s, 3.353 m/s sink, 376.232 m short.
The current L7 aim is 150 m. No golden is changed.

## Force learning on actual future incidences

Integrate `drag * airspeed / mass` by trapezoidal rule on common Final trace
timestamps. The seed and updating models see exactly the same actual Mach,
pressure and achieved AoA. The frozen snapshot's scalar areas and fixed
incidence arrays are the only copied fields used; its pointers are unused.

| Native tail | Common interval | Measured work | Updating learned model | Frozen seed model |
|---|---:|---:|---:|---:|
| L5 | 36.32 s | 12.069 kJ/kg | 11.994 (−0.62%) | 11.614 (−3.77%) |
| L7 | 11.40 s | 2.464 kJ/kg | 2.427 (−1.53%) | 2.284 (−7.31%) |

These values end at the last pre-contact guidance sample. Simulator telemetry
through first contact gives 12.085 and 2.481 kJ/kg respectively. Sampling and
endpoint differences explain why that integral differs from the trace interval.

The seed force model therefore misses drag as later incidences are flown.
Allowing actual in-flight learning reduces that error substantially. This is
not a causal decomposition of touchdown-speed error: evaluating a frozen
model on a trajectory produced by the updating controller does not yield the
trajectory a different predictor/controller would fly.

## Evolving control/profile

| Native tail | Seed wheel height | Time to contact | Time below 12 m | Achieved AoA maximum |
|---|---:|---:|---:|---:|
| L5 | 819.204 m | 36.42 s | 6.20 s | 5.864° |
| L7 | 120.520 m | 11.50 s | 3.50 s | 7.634° |

At L5's seed (UT 2462.649863), the preflare plan is invalid, so its stored
819.565 m trigger is unusable and response is 1.595 s. The next actual tick
restores a valid 816.110 m trigger and response rises to 4.105 s; pull latches
at 815.553 m. The first five latched samples above 12 m still have no finite
arc demand and negative vertical acceleration demand. The native arc becomes
active at UT 2463.249863, initially with zero acceleration demand. The latch
does not mean the fixed ideal sink-profile acceleration starts immediately.

L7 has no valid preflare plan/trigger at the corresponding seed or pull.
It latches one tick later at 118.433 m, with an immediately active arc demand
of 1.080 m/s². Its response varies from 1.5 to 2.301 s. In both runs the design
arrest coefficient stays at 3.4335 m/s², while the actual demand varies.

The prior frozen ideal-profile L5 forecast completes in 32 s and its retained
one-second history first falls below 12 m at 28 s (approximately four seconds
remaining). The actual tail takes 36.42 s and spends 6.20 s below 12 m. That
comparison uses discrete, differently sampled histories and is not a precise
crossing-time error bound. L7's frozen total is 12 s versus actual 11.50 s.
The much closer L7 speed forecast does not establish transferable accuracy.

## Decision and reproduction

Keep production unchanged. The next forecast experiment must account for
native stage/onset logic and preserve learned force/control history; another
shared speed rescaler or frozen incidence servo is not supported. Future
model learning cannot simply be copied from this retrospective trace into a
predictor. Live plant validation and the pending aim/drogue check remain open.

Run from the integration worktree:

```sh
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Final-Warmed-Tail/run.py
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Final-Warmed-Tail/analyze.py
```

The runner builds a temporary backend with the original native objects and
instrumented Final translation unit (`-Werror`), then removes that executable.
`result.json` retains input manifests, evidence hashes, exact parity checks
and full numeric analysis. CSVs retain two-second samples and event points;
original raw traces remain in ignored ShuttleSim run folders.
