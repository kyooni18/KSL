# Measured pitch authority and inertia cross-check

This passive diagnostic tests whether the pitch-fit transfer failure is
explained by omitting native available control torque and pitch inertia. It
extends the integral cross-check with independent L6 and the third L7 flight.
No production plant or flight-control coefficient is changed.

`analyze.py` uses the same nonoverlapping 0.5 s, zero-delay wings-level
intervals: unpowered flying, radar ≥15 m, Q ≥0.5 kPa, Mach ≤0.9, |bank| <5°,
|sideslip| <2°, no sample gaps above 0.2 s. L4 alone trains. L5/L6/L7a/L7b/L7c
remain held out; no held-out score selects a coefficient. `result.json`
retains source SHA256, interval counts, coefficients and all scores.

Two alternatives replace capped Q×control with measured
`availablePitchTorque / pitchInertia × control`, converted to deg/s².
Both retain sqrt(Q) rate damping, Q×AoA stiffness and Q trim. The second
scales passive terms by 1e6/pitchInertia. AvailableTorque is maximum control
authority rather than actual net moment; this is an explanatory-variable
experiment, not a direct measurement of aerodynamic moments.

| Model | L4 train R² | L5 | L6 | L7a | L7b | L7c |
|---|---:|---:|---:|---:|---:|---:|
| Prior capped-Q model | 0.372 | 0.263 | 0.163 | −2.090 | −1.232 | −2.492 |
| Measured authority | 0.052 | −0.017 | −0.041 | −0.976 | −0.483 | −1.050 |
| Authority + inertia scaling | 0.052 | −0.016 | −0.041 | −0.925 | −0.453 | −0.996 |

There are 180 training intervals and respectively 144, 164, 714, 659 and
715 held-out intervals. The alternatives have positive control gain and
negative damping/stiffness coefficients, but every held-out R² is negative.
That fails transfer validation. Their inferred trim is also about −3.6°,
which is not validated as a physical shuttle trim. No coefficients are
adopted. Adding measured authority/inertia alone does not resolve the model.

Recorded state groups remain partly cached, controls and incidence are
correlated under closed-loop feedback, and available torque can aggregate
multiple control sources. The alternatives improve some L7 errors versus
the prior model but degrade L5/L6 and training performance; that does not
establish a useful plant. The sim/live touchdown gap remains open. Further
identification must account for actuator/aerodynamic state or obtain adequate
independent excitation rather than treating maximum authority as net moment.
