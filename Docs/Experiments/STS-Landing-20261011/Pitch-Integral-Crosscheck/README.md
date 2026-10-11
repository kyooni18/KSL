# Integral pitch identification and coordinate check

This diagnostic uses L4/L5 plus both approved L7 MM305 continuations. It tests
whether short finite differences or the direct plant's pressure-dependent
rate damping explain the failed pitch-fit transfer. It also checks the
kinematic assumption behind substituting body-moment coefficients into an
AoA integrator. No production plant, flight-control coefficient or golden
was changed, and no flight was started for this experiment.

Reproduce with:

```sh
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Pitch-Integral-Crosscheck/analyze.py
```

`result.json` preserves source paths/SHA256, every tested duration/delay,
cross-flight training results and the kinematic check. The script uses the
existing dependency-free weighted least-squares solver. Training cap choice
uses only training R²; held-out scores do not choose a production model.

## Integrated moment fits still do not generalize

Selected data are unpowered flying states, radar ≥15 m, Q ≥0.5 kPa, Mach ≤0.9,
|bank| <5° and |sideslip| <2°. Each interval is contiguous, with gaps ≤0.2 s;
intervals do not overlap except for their boundary. The response is the
endpoint rate change divided by interval duration. Regressors are trapezoidal
time averages of control moment, sqrt(Q)-scaled damping, Q×AoA and Q.

The model tests `deltaRate/dt = average(min(Q,cap)×gain×input + sqrt(Q)×damping×rate
− Q×stiffness×(AoA−trim))`, matching the current plant's damping shape instead
of the previous constant damping coefficient. Both body and AoA coordinates
are evaluated with intervals 0.2/0.5/1/2 s and control delays 0/0.1/0.2 s.
L4 trains the sweep; L5 and both L7 flights are held out.

Representative 0.5 s, zero-delay body results:

| Training flight | Training R² | Held-out L4 | Held-out L5 | Held-out L7 first | Held-out L7 repeat |
|---|---:|---:|---:|---:|---:|
| L4 | 0.372 | — | 0.263 | −2.090 | −1.232 |
| L5 | 0.730 | −11.818 | — | −48.041 | −36.881 |
| L7 first | 0.179 | −1.717 | −2.400 | — | 0.241 |
| L7 repeat | 0.304 | −2.585 | −3.368 | 0.100 | — |

Every L4-trained body candidate in the duration/delay sweep has negative
held-out R² on both L7 flights. AoA fits are weak; no swept candidate has
physical coefficient signs and positive scores across all held-out flights.
A higher training R² cannot establish a transferable plant. The modest L7
pair transfer supports repeatability within that regime, not a common L4–L7
model. This rejects simple adoption of the integrated fit, not all possible
pitch models.

## A constant control-torque term is not identified

At 0.5 s and zero delay, an extra pressure-independent input term tests a
possible contribution from reaction wheels/manual torque. All four body
fits give **negative** constant gain (−11.0, −16.1, −6.43, −6.60 deg/s² per
unit input). That is not a valid positive reaction-wheel authority estimate.
Although training and some L7 pair scores improve, none has physical signs
or broad transfer. The apparent term could instead reflect pressure/Mach
nonlinearity or closed-loop confounding; this experiment cannot distinguish
them. No negative control torque is installed.

## Flight-path rotation materially changes AoA

For the same near-level 0.5 s intervals, compare the recorded AoA increment
with two predictions: the integrated body pitch rate alone, or that rotation
minus the measured flight-path-angle increment. This is a kinematic check,
with no fitted coefficients. The wings-level approximation omits small
bank, sideslip and heading-coupling terms, so exact agreement is not expected.

| Flight | Intervals | Body-only RMSE | With FPA change RMSE | Body-only R² | With FPA change R² |
|---|---:|---:|---:|---:|---:|
| L4 | 180 | 0.490° | 0.119° | −4.217 | 0.692 |
| L5 | 144 | 0.439° | 0.139° | −4.866 | 0.412 |
| L7 first | 714 | 0.159° | 0.0695° | −1.391 | 0.545 |
| L7 repeat | 659 | 0.195° | 0.0878° | −1.437 | 0.504 |

Subtracting flight-path rotation reduces increment RMSE by 55–76% in all
four flights. The current airborne `direct_step` integrates surface moments
as AoA acceleration, and `flight_physics_airborne_step` reconstructs body
orientation relative to the newly rotated velocity. It therefore omits the
kinematic contribution required when its coefficients represent body moments.
The ground pitch-release path already distinguishes body pitch from FPA.

The appropriate next implementation is to preserve body rotational state
and derive incidence from body orientation relative to the air-relative
velocity, then identify moments in that coordinate. A near-level correction
alone does not validate banked TAEM; full flight-frame rotation matters there.
Do not tune flare or rebaseline acceptance against the present fit as if this
coordinate check had validated moment coefficients.

## Limits

Recorded state groups are partly cached and asynchronously refreshed;
trapezoidal integration reduces dependence on short differences but does not
recover missing sample times or independent actuator excitation. The fits
remain passive closed-loop diagnostics, with different pressure/Mach coverage
and correlated input/incidence. No coefficient is accepted, and neither the
simulator touchdown tolerance nor the full scenario matrix is satisfied by
these results alone. Jev was unavailable; the retained local runner/results
provide the investigation evidence.
