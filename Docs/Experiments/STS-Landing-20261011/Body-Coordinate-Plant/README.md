# Body-coordinate plant implementation and offline comparison

**Superseded comparison:** the initial decoder mixed body and geometric
angles when calculating rates. [Corrected reruns](../Body-Coordinate-Rate-Fix/README.md)
remove that artifact; the large L5 departures and isolated Final results
below must not be used as current physical model-validation evidence.

Commit `0951e4a` implements an opt-in `direct_body_coordinates=1` direct
plant. Aerodynamic moments advance body p/q/r and the inertial quaternion;
incidence, sideslip and wind-axis bank are derived from the new air-relative
velocity. Velocity/FPA rotation therefore changes incidence without inventing
body rotation. Initial bank rotates the wind/lift plane before incidence is
applied, keeping the initial AoA and sideslip consistent at bank. The new
mode exports actual body angular rates and Euler angles to native telemetry.
Legacy INIs retain their existing coordinate model and goldens.

The ground pitch-release path initializes its rate from body pitch rate in
this mode. Body-rate availability is withdrawn after pitch release; the
existing ground model then owns pitch. This is a diagonal moment surrogate,
not a measured inertia-tensor rigid-body model: gyroscopic cross-coupling,
pressure/Mach-dependent actuator response, measured initial angular velocity,
and lateral aerodynamic forces still need identification. Below the existing
minimum direct-control pressure it uses the established servo fallback.

## Verification

`make` and `make test` passed: native checks, 104 Python tests and 44 subtests.
A focused helper in the existing touchdown test verifies zero-torque inertial
attitude preservation when the flight path rotates, at both 0° and 60° bank;
it also verifies positive pitch input rotates the nose upward with the
configured acceleration. The same test checks actual body-rate publication
and decoder availability, while legacy packets keep body rates unavailable.
The added protocol assertions passed `make -C CLanding touchdown-quality-test`.

Explicit baseline continuations retained the exact established outcomes:
L5 66.129 m/s, 1.844 m/s sink, stop 1101.616 m, all gates pass; L7 55.499 m/s,
0.907 m/s sink, 225.778 m short, failed acceptance. Runs:
`runs/20261011T090406Z-body-mode-legacy-l5` and
`runs/20261011T090451Z-body-mode-legacy-l7` (see status/summary locally).
No golden was rebased. No TAEM guidance, production profile or flare setting
was changed. No live flight was started.

## Coefficient experiments are not accepted

`run.py` compares three body-coordinate plants on archived L5/L7 MM305 states:

- `existing`: the previous STS-N-derived pitch coefficients, interpreted as
  body moments instead of incidence moments;
- `l4`: the physical-sign 0.5 s, zero-delay L4 integral fit;
- `l7a`: the corresponding fit trained on the first approved L7 continuation.

The latter fits already lack cross-flight validation; they are diagnostic
candidates, not accepted identified profiles. Roll/yaw use the existing
generic coefficients, which are not live-validated. Guidance's terminal
predictor and the simulator receive the same candidate INI through the runner,
but this does not establish direct-control parity: `terminal_propagator_step`
still advances a commanded AoA/bank servo, without a predicted body-rate FCS
loop. Its existing model-validation flag is not a new qualification of these
candidates. That predictor/control difference also remains open.

| Plant / inlet | Contact speed | Sink | Along | Cross | Gear | Result |
|---|---:|---:|---:|---:|---|---|
| existing / L5 MM305 | 51.35 | 13.51 | +28563 m | −29199 m | up | failed, recovery timeout |
| existing / L7 MM305 | 60.15 | 0.37 | −313 m | +0.49 m | down | failed runway acceptance |
| L4 fit / L5 MM305 | 31.44 | 24.42 | −6731 m | −4914 m | up | failed, recovery timeout |
| L4 fit / L7 MM305 | 47.88 | 4.87 | +2429 m | +2.50 m | down | failed runway acceptance |
| L7 fit / L5 MM305 | 32.81 | 19.86 | −9164 m | −4675 m | up | failed, recovery timeout |
| L7 fit / L7 MM305 | 57.25 | 1.50 | +2224 m | +0.36 m | down | failed runway acceptance |

Speeds and sink are m/s. Use `simulatorSummary.touchdown_gear` for actual
contact gear; the runner's separate `touchdownGearDown` flag can reflect later
ground recovery and is not a substitute. Stop fields are defaults when
`stopped=false`, not measured stopping positions. All candidates fail the
full MM305 replay. The L5 cases never reach Final/Flare, while all L7 cases
do; correcting coordinate bookkeeping alone does not validate a model.

## Isolated L5 Final does not yet reproduce live touchdown

`run_final.py` starts at the first native Final frame of live L5 (UT 2412.130,
129.02 m/s, altitude 2275 m, near level bank). This approximate restart uses
body heading as course and lacks initial rates/history. Its source SHA256
and full decoded frame are in `l5-live-final-source.json`. Final test engagement
is an offline injection, not proof that MM305 can deliver this state.

| Final plant | Contact speed | Sink | Along | Simulator acceptance |
|---|---:|---:|---:|---|
| legacy coordinate model | 68.74 | 1.23 | 312 m | pass |
| body / existing coefficients | 72.21 | 1.25 | 153 m | pass |
| body / L4 fit | 69.03 | 0.97 | 232 m | pass |
| body / L7 fit | 58.46 | 1.28 | 426 m | **fail speed gate** |
| live L5 reference | ~55.7 | ~1.13 | ~621 m | below acceptance speed floor |

Although the L7-fit candidate is closer to live touchdown speed, it does not
meet the full plant validation requirement: it fails the speed gate, remains
about 195 m short of the live contact, and fails full L5 MM305 and held-out
moment checks. No pass is claimed for that candidate, and no acceptance gate
was lowered. Pitch/refit and banked/lateral dynamics remain open before any
flare tuning can rely on this plant.

## Reproduction

After `make`, run in the integration worktree:

```sh
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Plant/run.py
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Plant/run_final.py
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Plant/collect.py
```

`result.json` retains arguments, exit codes, separate simulator/backend results,
model sources and build/input provenance. Raw logs and native simulator
streams remain under ShuttleSim/runs. The INIs explicitly label these
unqualified coefficient experiments. Jev was unavailable; local targeted
experiments provided the comparison evidence.
