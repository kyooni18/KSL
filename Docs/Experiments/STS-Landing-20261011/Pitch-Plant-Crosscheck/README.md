# Pitch plant cross-check against live L7

The existing L4 pitch-response regression was refitted and evaluated against
held-out L5 and the approved live L7 MM305 continuation. Source logs are
identified by path and SHA256 in [result.json](result.json). The L4/L5 files
were copied read-only from the primary checkout into this worktree's ignored
FlightLogs directory. No live flight or primary-checkout writes were performed.

Reproduce from the integration worktree:

```sh
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Pitch-Plant-Crosscheck/analyze.py
```

The experiment retains the existing selection: unpowered flying, radar height
at least 15 m, dynamic pressure at least 500 Pa, Mach at most 0.9, bank within
5 degrees, sideslip within 2 degrees, adjacent samples 0.05–0.15 s apart.
Each coefficient set and pressure cap is fitted on L4 only. L5 and L7 are
never included in the fit. The causal control-delay sweep is diagnostic;
no delay/model was selected using the held-out scores.

## Results

| Acceleration coordinate, zero control delay | L4 fit R² | L5 held-out R² | L7 held-out R² |
|---|---:|---:|---:|
| Body pitch | 0.449 | 0.335 | **−2.171** |
| AoA | 0.086 | 0.091 | **−0.131** |

Zero-delay sample counts are L4 815, L5 838, L7 4233. Negative R² means
the coefficient prediction is worse than predicting that dataset's mean
acceleration. The body fit fails even in the overlapping 4–7 kPa band
(L7 568 samples, R² −29.40); pressure-distribution shift alone cannot explain
the cross-flight failure. That band's target variance may be small, so R²
does not quantify the absolute acceleration error by itself.

Delays 0, 40, 80, 120, 200, 300 and 400 ms were checked. Historical applied
pitch was taken only from preceding selected observations within 150 ms of
the delayed timestamp. No tested delay gave nonnegative overall L7 R².
Increasing delay also degraded L5 held-out prediction; a constant actuator
delay does not rescue this regression. Different delays retain different
sample subsets; this sweep is not a controlled estimate of physical latency.

## Decision: do not adopt these coefficients

`ShuttleSim/src/attitude.c:direct_step` evolves **AoA** using the moment
coefficients and its AoA rate, while the current `att-live.ini` documents
coefficients identified from **body pitch** on a different STS-N vehicle.
Body pitch acceleration and AoA acceleration are related through the changing
flight-path angle and bank/sideslip kinematics; they are not interchangeable.
The simulator's configured damping ratio also yields damping proportional to
sqrt(dynamic pressure), whereas the regression fits constant rate damping.

Simply replacing the current coefficients with the L4 body fit would retain
that coordinate mismatch and has failed cross-flight validation. The direct
AoA fit has insufficient predictive quality as well. No active plant file,
guidance coefficient, acceptance gate, or golden was changed by this analysis.
Next investigation should account for flight-path kinematics, assess absolute
residuals and excitation, and validate both pitch response and closed-loop
touchdown against held-out flights before adopting a plant change.

## Related failed-rollout correction

Commit `9fdfa0a` fixes the live check's nonterminating grounded warning recovery:
continue braking while moving; once grounded with finite nonnegative speed
below 1.5 m/s, latch Abort and disengage automation. A prior failed contact
stays failed even if later telemetry returns inside the runway. Nonfinite
speed does not establish a stop. The regression was added to the existing
MM305 recovery-monitor binary and failed on the original implementation.

The failure exit exposed a simulator evidence defect: the harness's SIGTERM
previously discarded the simulator's touchdown summary. ShuttleSim now handles
SIGTERM/SIGINT by leaving its physics and lockstep loops and emitting the
existing measured final telemetry/summary. It does not advance physics to
invent a stop or change a failed landing to success.

`make test` passes: 104 tests and 44 subtests, plus native checks. Explicit CLI
replays preserve the L5 golden (66.129 m/s, 1.844 m/s sink, stop 1101.616 m;
all gates pass), and L7 now exits with a failed mission and its measured
gear-down flare/contact evidence (55.5 m/s, 0.91 m/s sink, 226 m short).
The current-build `make -C CLanding taem-native-stack-test` also passed.
Run directories are `runs/20261011T081917Z-l5-mm305` and
`runs/20261011T081919Z-l7-mm305`. This controller fix remains unverified live.
