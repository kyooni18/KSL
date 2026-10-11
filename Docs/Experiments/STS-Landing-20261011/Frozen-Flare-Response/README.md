# Frozen flare response diagnostic

Adding an ideal AoA servo to the same frozen scalar forecast does not explain
L5's 10.47 m/s late forecast error. Retaining its stored trigger, despite that
plan being invalid at the sampled instant, does not correct the error either.
These experiments are rejected as production forecasting remedies. Actual
closed-loop flight is unchanged in both L5/L7 replays.

Run `run.py`, then `analyze.py` here with the repository `.venv/bin/python` from
the integration checkout. The runner compiles a temporary backend with a cloned
forecast beside the real controller. It samples UT 2462.649863 in L5 and
7021.450098 in L7: the same late pre-latch states identified in the preceding
comparison. The frozen clone takes the real learned force coefficients and
lift ratio, measured speed/sink/height/AoA/rate, atmosphere anchor and current
plan. It never changes the main guidance result or estimator. Both replay
landing summaries must exactly equal their prior uninstrumented summaries.
Mode 0 must reproduce the original forecast to 1e−8 m/s. Native compile flags
include `-Werror`; source/binary/log hashes and replay manifests are retained
in `result.json`. One-second histories and contact steps are in the CSVs.

## What is isolated

- Mode 0: unchanged original point-mass forecast, dt 0.25 s.
- Mode 1: same scalar trajectory, but post-latch sink acceleration follows the
  learned lift actually delivered by the selected, quantized incidence instead
  of applying the requested acceleration ideally.
- Mode 2: mode 1 plus native `attitude_step` ideal AoA servo using the selected
  attitude INI. Seed measured AoA and measured AoA rate; set its initial command
  to measured AoA. Disable direct control and bank for this counterfactual.

Modes 1/2 also run at dt 0.05 s. The experiment retains the original scalar
sink equations and pre-latch constant-FPA assumption. It is **not** native 3D
propagation, the legacy direct-control moment plant, or a full future Final/FCS
replay. The estimator and plan do not update. Thus it measures a limited
response substitution, not the correctness of a complete response-aware
predictor. Original contact overshoot is retained; no new interpolation gate
is introduced. All variants reach scalar ground within their 300 s limit.

L5's current usable trigger is zero because its preflare plan is invalid at
the sampled instant. Its stored trigger remains 819.564985 m. Two extra L5
cases use that stored trigger **only as a counterfactual**, while the actual
controller's gate remains intact. The following actual tick restores a valid
plan and latches pull-up near 815.6 m. This future plan change is absent from
the frozen forecast. The counterfactual tests whether changing timing alone
explains the discrepancy; it does not authorize using an invalid plan.

| Case | L5 speed m/s | L5 scalar sink m/s | L7 speed m/s | L7 scalar sink m/s |
|---|---:|---:|---:|---:|
| Original, dt 0.25 | 76.595 | 1.856 | 61.796 | 1.889 |
| Achieved lift, dt 0.25 | 72.160 | 1.657 | 55.002 | 1.574 |
| Achieved lift + servo, dt 0.25 | 76.139 | 3.989 | 61.745 | 4.078 |
| Achieved lift, dt 0.05 | 73.076 | 1.768 | 55.101 | 1.609 |
| Achieved lift + servo, dt 0.05 | 75.943 | 4.087 | 60.824 | 3.669 |
| Retained trigger + original, dt 0.25 | 79.286 | 1.854 | — | — |
| Retained trigger + servo, dt 0.05 | 75.088 | 2.696 | — | — |
| Actual native flight contact | 66.129 | 1.844 | 61.929 | 3.353 |

The dt comparison is a sensitivity check, not proof of convergence. Servo
results change by ~0.20 m/s in L5 and ~0.92 m/s in L7 between these steps;
the quantized instantaneous-lift case changes ~0.92 m/s in L5. No coarse
forecast should be treated as an exact contact prediction.

## Flight cost and remaining discrepancy

From the sampled state to actual first contact, integrate native recorded
`drag_n * airspeed / mass` by trapezoidal UT intervals. This is an approximate
recorded drag-work measure, not a complete inertial energy identity. L5 has
365 samples, L7 116. No ground rollout samples after first contact are included.

| Observation | L5 | L7 |
|---|---:|---:|
| Original frozen elapsed s | 32.00 | 12.00 |
| Original frozen drag work J/kg | 11,482.96 | 2,568.79 |
| Actual flight elapsed s | 36.42 | 11.50 |
| Actual recorded drag work J/kg | 12,084.97 | 2,481.31 |
| Original frozen maximum incidence ° | 4.50 | 6.60 |
| Actual maximum measured incidence ° | 5.864 | 7.634 |

L5 spends longer airborne and loses more recorded drag work than its frozen
forecast. The forecast's missing future controller/plan/estimator changes and
its instantaneous lift allocation cannot be replaced by a common drag multiplier
or this isolated servo substitution. Moving its frozen trigger up causes
scalar latch immediately, but makes the ideal forecast faster (79.286 m/s),
not closer to actual contact. This rejects the timing-only hypothesis for this
scalar model, even though plan timing clearly differs from the flown controller.

The experiment preserves L5's exact passing golden and L7's current failed
landing (61.929 m/s, sink 3.353 m/s, 376 m short, gear-down/no crash). Neither
surrogate plant is newly qualified. No production source, flight gate, golden,
new test or live flight changes. Next compare a complete native Final control
replay from the same preserved late state/history, including its changing
preflare plan, incidence command, attitude response and estimator, before
replacing the existing forecast or tuning the last 10 m arrest.
