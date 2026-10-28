# MM304 predictor / plant parity — 2026-10-28

Status: investigation, not flight-qualified. Work is isolated on
`mm304-planned-sturn` from `d2a05bc`. Main's local changes and flyable binary
are untouched. No merge or live rebuild is authorized by these results.

## Forecast model mismatch

The schedule-policy predictor never populated `AeroContext.entry_table`.
The guidance-shadow predictor already used the captured STS table. Schedule
forecasts instead used online force samples where available and a fallback
polar elsewhere. Comparing at the same altitude without matching Mach and
actual AoA concealed the sign and size of the force error.

`Tools/compare_entry_forecast.py` matches trace samples within 0.05 Mach and
0.25 degrees actual AoA. PRED_AERO D/L are accelerations, so it multiplies by
plant mass before dividing by q. Counts represent forecast samples, not
independent physical observations. Altitude-only density comparisons can
include differences in latitude and UT.

Using the supplied branch run
`mm304-guidance-20261028T025306Z-pfm_dx80` as the plant reference:

| Mach bin | AoA | Old D/q / plant | Old L/q / plant | Captured D/q / plant | Captured L/q / plant |
|---|---:|---:|---:|---:|---:|
| 3.0–3.5 | 28° | 1.510 | 0.528 | 1.006 | 1.006 |
| 4.0–4.5 | 28° | 1.408 | 0.519 | 1.000 | 1.001 |
| 4.5–5.0 | 28° | 1.375 | 0.520 | 1.000 | 1.000 |

The earlier trace `mm304-guidance-20261028T023149Z-il24bopm` has density
17–18% high at 40–52 km. A fresh baseline's evolving scalar calibration
reduces its median error to about 4–5%; this does not repair its force polar.
The captured spatial atmosphere is within about 0.1% of the plant at matched
24, 28, 32, 40, 48 and 52 km samples.

Changes in checkpoint `98a58de`:

- Pass the captured, policy/frame-validated TerminalModel to policy forecasts.
- Evaluate its spatial atmosphere at forecast latitude, longitude and UT.
- Use its clean whole-vessel force/q table and geometric bank rotation;
  do not multiply it by residuals calibrated against the fallback polar.
- Trace forces with the actual AoA/bank used for that force evaluation.

Checkpoint `9e79a62` additionally initializes forecasts from measured attitude
and rates, scores handoff using the same atmosphere as propagation, and stops
forced Entry policies at the physical MM305 boundary. The old rollout continued
Entry commands toward the ground, counted post-handoff reversals, and repeatedly
ran an irrelevant route-admission solve. General entry/admission forecasts retain
their existing ownership checks.

One regression function added to the existing predictor white-box test checks
Mach 2.5–5, AoA 18/23/28, table/atmosphere parity, independence from fallback
scalar residuals, and termination at the physical handoff. `make -C CLanding
test` passes for both checkpoints.

## Live flight is not an exact plant twin

Source: `FlightLogs/2026-10-27T23-59-56Z-STS-vehicle.jsonl` in the main checkout.
Evaluating the captured aero table at each live sample's exact q/Mach/AoA
with `Tools/entry_aero_table_probe.c` gives:

| Mach bin | Median actual AoA | Table D/q / live | Table L/q / live |
|---|---:|---:|---:|
| 2.5–3.0 | 18.40° | 1.102 | 1.095 |
| 3.0–3.5 | 18.37° | 1.114 | 1.090 |
| 3.5–4.0 | 18.81° | 1.111 | 1.103 |
| 4.0–4.5 | 23.90° | 1.070 | 1.130 |
| 4.5–5.0 | 25.04° | 1.040 | 1.129 |

Captured spatial density agrees with live samples within about 0.4% at the
sampled heights. The residual aero discrepancy above remains; matching the
chosen ShuttleSim profile is not evidence of exact live aero parity.

At Mach 5 live AoA was about 25.8° while commanded AoA was 28°. Around Mach
3.5 actual AoA was 18.8° versus a 20.1° command. The branch's default alpha
schedule is flat, and a fresh plan replaces it with the candidate AoA. Its
late changes are not a fixed Mach-3.5 transition: plan changes, yaw reserve,
drag modulation and attitude tracking all matter. No invented Mach gate has
been added to reproduce that one trajectory.

## Replay evidence

Configuration: `runs/20261027T235952Z-deorbit/backend-configuration.json`.
Profile: `sts-live-20261015-live-only`. All runs use the isolated simulator
and backend; no main binary is replaced.

| Variant | Run | Touchdown along / cross | Outcome |
|---|---|---|---|
| Supplied d2a05bc branch witness | `mm304-guidance-20261028T025306Z-pfm_dx80` | +17.474 / −16.477 km | Crash |
| Captured model, 40° cap | `mm304-guidance-20261028T051354Z-xo16lyvg` | +45.112 / −16.752 km | Crash |
| Measured initial attitude, 40° cap | `mm304-measured-state-20261028T051952Z-6i2az68n` | +43.340 / −18.665 km | Crash |

A fresh baseline trace run was stopped after collecting comparison samples;
its unfinished flight is not used as a landing result.

The corrected model finds no low-cost candidate in the original early search:
best heading error about 109°, distance about 48 km for a 49.6 km target.
An isolated diagnostic allowing the configured vehicle limit (70°) finds
heading 2.9°, distance 44.4 km, score 8.2. Thus correcting the predictor exposes
a planner-envelope limitation rather than establishing landing success.

The configured-envelope experiment removes the private 40° cap, retains the
vehicle/dynamic/yaw/load bounds, and searches the full configured bank envelope
instead of treating the scalar straight-policy energy solution as a lateral
upper bound. With the configured limit, the replay makes a safe centerline touchdown but
remains short: −2407.613 m along / −0.665 m cross. Its score rises from about
6–10 early to 34 at handoff. The first configured-envelope regression campaign
retains all three existing passes; the two old 72 km cases still crash but
closer to the site.

A due-now reversal candidate avoids continuously moving a five-second deadline
forward. It improves the main replay to −128.440 m along / −13.632 m cross,
but still fails the runway test and regresses the 40 km case to 4.216 m/s
sink at touchdown (on runway, but not qualified). It is not an accepted result.

A further control-parity experiment passes the actual EntryLateralState into
each forecast and uses the shared lateral allocator for scheduled policies.
It respects capture/dwell and command lead/acceleration limits. Guidance also
honors the plan's bank magnitude rather than adding an unpriced zero-effort
crossrange bank demand on top. A regression assertion checks that a due-now
reversal cannot bypass an unfinished physical bank exchange. Full replay and
regressions preserved the three previous passes. The shared-control main run
still landed short: −1717.651 / +0.413 m, safe touchdown but runway failure.
Its final heading error was small, but range contribution kept cost around 33.
`make test` passes.

The next experiment uses the captured attitude model's q-dependent response,
initialized from measured angles/rates and integrated at 20 ms. Generic and
direct-control response paths retain their previous behavior. This avoids
pricing a generic response while the plant flies the captured actuator model.

A further search correction retains the existing plan's absolute deadlines as
an explicitly evaluated continuation candidate. Rebuilding only a coarse grid
relative to each request omitted the very policy currently being flown, and
successive replans introduced bank exchanges not present in earlier forecasts.
The continuation competes on the same score; there is no switching penalty or
new timing threshold. These last two experiments are still awaiting complete
replay/regression evidence and are not flight-qualified.

## Regression comparison before the envelope experiment

Both d2a05bc and the captured-model checkpoint were run against the same six
scenarios and configuration. Touchdown values are identical:

| Scenario | Along / cross (m) | Result on both |
|---|---:|---|
| sts-live-oms72-entry-20261009 | 134996.442 / 6692.130 | Crash |
| sts72km-run9-entry-interface | 217638.198 / −10436.100 | Crash |
| sts-kerbin-20261022-live-orbit86-mm304-40km | 813.890 / 0.422 | Pass |
| sts-kerbin-20261028-live-overhead-mm305 | −597.938 / 0.114 | Safe touchdown short; runway failure |
| sts-kerbin-20261022-live-orbit86-mm305 | 662.171 / −0.009 | Pass |
| sts-kerbin-20261021-quicksave-mm305 | 674.383 / −0.116 | Pass |

Unchanged pre-existing failures are not counted as passes.
