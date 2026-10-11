# Correct body-coordinate telemetry rates

Commit `c88eba5` fixes a decoder error introduced with the opt-in body plant:
Euler angles were replaced **after** computing coordinate rates. Current
geometric `FPA + AoA` and ground-track heading were therefore differenced
against the preceding body's Euler pitch and nose heading. At bank those
coordinates differ even without body rotation. In the original L5 body run,
recovery triggered at UT 2178.85 with reported pitch rate 20.88°/s and heading
rate 71.43°/s while body pitch rate was −0.07°/s.

The decoder now selects body angles before validation/differentiation. Body
attitude availability is separate from body-rate availability, preserving
orientation through initial packets and ground pitch release. Course rate
uses ground-track heading; Euler heading rate uses nose heading. Published
wind-bank rate does not substitute for body Euler roll rate. The first
simulator packet uses the body mode's consistent wind-bank quaternion.

## Regression and validation

One additional case in the existing touchdown test freezes a banked body
orientation across two packets. It fails the previous decoder's zero pitch
rate assertion and passes the corrected decoder, including zero roll,
heading and course rates. Logs: `/tmp/ksl-body-rate-regression-before.log`
and `/tmp/ksl-body-rate-regression-after.log`.

`make` and `make test` pass: native checks, 104 Python tests, 44 subtests.
Existing production INIs/goldens remain unchanged. No TAEM law, acceptance
gate or flare setting changed, and no live flight was started.

A small diagnostic compares decoded coordinate rates with ZYX body-rate
kinematics in the existing-coefficient L5 runs. Before/after selected snapshot
counts are 13/35; the trajectories differ, so these are not matched-state
statistical validation. Pitch-rate absolute-error p95 falls from 21.48°/s
to 0.63°/s; heading-rate error falls from 71.55°/s to 0.56°/s. The controlled
stationary regression establishes the fix; these trajectory numbers show
its relevance. `check_rates.py` retains that diagnostic in `rate-check.json`.

## Corrected MM305 comparisons

The same three body models are rerun without changing coefficients. All six
now reach Final and flare. The earlier large L5 departures were decoder
artifacts and must not be used as physical model-validation evidence.

| Plant / inlet | Contact speed | Sink | Along | Stop | Acceptance |
|---|---:|---:|---:|---:|---|
| existing / L5 | 56.39 | 2.24 | 213 m | 869 m | fail speed |
| existing / L7 | 58.13 | 1.95 | −307 m | not measured | fail runway |
| L4 fit / L5 | 67.64 | 0.90 | 210 m | 1079 m | pass |
| L4 fit / L7 | 68.11 | 1.98 | 1820 m | not measured | failed rollout/runway envelope |
| L7 fit / L5 | 49.92 | 1.61 | 796 m | 1390 m | fail speed |
| L7 fit / L7 | 58.08 | 1.52 | 1948 m | not measured | failed rollout/runway envelope |

Speed and sink are m/s; actual contact gear is down in all six. Unmeasured
stops mean `simulatorSummary.stopped=false`; zero stop positions in those
records are defaults. L7 failures still automatically terminate after
braking; contact position alone does not establish valid rollout. The L4-fit
plant's L5 pass does not establish live-model agreement or L7 qualification.

## Corrected isolated live-L5 Final comparisons

These retain the same approximate archived Final-state restart, now with
consistent initial/body-rate decoding:

| Plant | Contact speed | Sink | Along | Simulator acceptance |
|---|---:|---:|---:|---|
| legacy | 68.74 | 1.23 | 312 m | pass, unchanged |
| body / existing | 67.33 | 1.25 | 417 m | pass |
| body / L4 fit | 60.24 | 1.34 | 534 m | fail speed |
| body / L7 fit | 55.92 | 1.41 | 527 m | fail speed |
| live L5 reference | ~55.7 | ~1.13 | ~621 m | below acceptance speed floor |

The L7-fit Final case is now close to live speed, with 0.28 m/s more sink and
about 94 m earlier contact. This is encouraging **isolated Final** evidence,
not full plant validation: full L5 MM305 remains slower, both L7 outcomes
fail acceptance, held-out moment fits still fail, and the restart approximates
course and omits angular velocity/history. No coefficient is adopted, no
production model is switched, and no gate is weakened. The previous isolated
Final comparison is also superseded by these corrected results.

## Remaining validation

The body mode is an opt-in diagonal moment surrogate, with generic roll/yaw
and no measured inertia cross-coupling or lateral-force model. Accurate
inlets, held-out response checks and the L5/L6/L7/high-energy matrix remain
necessary. The terminal predictor still uses an AoA/bank servo rather than a
predicted body-rate FCS loop, so matching INI source paths are insufficient
for direct-control parity. This fix removes a measurement artifact; it does
not finish the sim/live refit or justify flare tuning against every case.

Reproduce after `make`:

```sh
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Rate-Fix/run.py
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Rate-Fix/run_final.py
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Rate-Fix/collect.py
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Body-Coordinate-Rate-Fix/check_rates.py
```

`result.json` preserves simulator contact, backend acceptance, model sources,
build/input provenance and raw stream locations. The recorded Final inlet
and its source hash remain in the preceding Body-Coordinate-Plant directory.
Raw run logs remain local. Jev was unavailable.
