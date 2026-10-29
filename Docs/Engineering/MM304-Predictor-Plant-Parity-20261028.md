# MM304 predictor / plant parity — 2026-10-28

Status: offline checkpoint complete; **not regression-free or live-qualified**.
The earlier short-landing interpretation below is superseded by the runway-fixture
correction documented at the end of this report. Work is isolated on
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
new timing threshold. Complete results for these last two experiments are recorded below. Neither
is flight-qualified.

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


## Captured-response-only result

`mm304-captured-response-20261028T054626Z-_efvu9jo` landed at −4095.348 m
along / −2.726 m cross, with 15.905 m/s sink and a tail strike. It is rejected
as a landing solution.


## Final checkpoint result (a14f910)

Run `mm304-continuation-20261028T055005Z-pnozsmov` completed without a simulator
crash, but `success=false`, `on_runway=false`: touchdown **−1540.809 m along /
−2.542 m cross**, sink **4.052 m/s**, speed 60.677 m/s, gear down. Compared
with the supplied branch witness (+17.474 / −16.477 km, crash), this is a
large improvement in delivery, but it does not meet landing acceptance.

The remaining forecast error at handoff is much smaller than the original
late-entry divergence:

| Plant Mach | Latest plan score |
|---:|---:|
| 5.0 | 7.1 |
| 4.5 | 5.4 |
| 4.0 | 8.9 |
| 3.5 | 12.6 |
| 3.0 | 15.1 |
| 2.5 | 14.7 |

At the first plant sample inside Mach 2.5 below 30 km (UT 148743778.286316),
altitude is 25487.030 m, speed 756.591 m/s, distance to the configured site
46247.510 m, course 147.692°, bearing 135.844°. The independently calculated
realized score is **15.20** (11.85° heading error plus 3.35 km distance error
against the logged 49.6 km target), versus the latest forecast **14.7**.
That is a useful handoff forecast, not evidence of runway landing success.
The remaining short touchdown must be investigated from this inlet through
MM305/Final; this work does not establish which downstream component owns it.

Final six-scenario comparison, all using the same requested profile and
`runs/20261027T235952Z-deorbit/backend-configuration.json` for both baseline
and candidate:

| Scenario | d2a05bc result | Final along / cross (m) | Final result |
|---|---|---:|---|
| sts-live-oms72-entry-20261009 | Crash, +134996 / +6692 m | −533.476 / +59.956 | No crash; short/off runway |
| sts72km-run9-entry-interface | Crash, +217638 / −10436 m | +39970.550 / +5027.852 | Crash |
| sts-kerbin-20261022-live-orbit86-mm304-40km | Pass | +815.619 / −0.467 | Pass, sink 2.938 m/s |
| sts-kerbin-20261028-live-overhead-mm305 | Runway failure | −597.938 / +0.114 | Same runway failure |
| sts-kerbin-20261022-live-orbit86-mm305 | Pass | +662.171 / −0.009 | Pass, unchanged |
| sts-kerbin-20261021-quicksave-mm305 | Pass | +674.383 / −0.116 | Pass, unchanged |

All previously passing scenarios remain passing. The other three remain
failures, not qualification successes. Final `make -C CLanding test` passed;
no additional broad test runs were needed after the final documentation edit.
All replay processes have finished. Trace instrumentation is retained because
the requested landing condition remains unmet. Main was not merged or rebuilt.

### Reproduction

Worktree: `/Users/kyooni18/.codex/worktrees/mm304-predictor/KSL`.
From its ShuttleSim directory, the final replay used:

```sh
KSP_LANDER_MODEL_PROFILE=sts-live-20261015-live-only \
KSP_LANDER_TRACE_S_TURN=1 python3 scripts/run_guidance.py \
  --scenario scenarios/sts-kerbin-20261028-live-entry-72km.ini \
  --engage engageReentry \
  --configuration /Users/kyooni18/Code/KSL/runs/20261027T235952Z-deorbit/backend-configuration.json \
  --quiet-progress --compact-guidance-log --skip-build --label continuation
```

Per-run manifests and compressed telemetry are retained in the worktree's
`ShuttleSim/runs/`. The final regression IDs begin `mm304-parity-continuation-`.
The baseline used a separately compiled d2a05bc source archive; no main build
or user WIP files were used as build output.


## Continued offline validation: runway fixture correction

`0ca2b64` corrects an independent validation-fixture defect. Both October 28
scenarios used the east/RW27 threshold (`-74.4896008327406`) with heading 90°,
while the recorded backend configuration targeted the west/RW09 threshold
(`-74.7283333333`). Those thresholds are one runway length (2,500 m) apart.
The simulator therefore placed its physical runway beyond the controller's
selected runway. This explains the exact 2,500 m discrepancy between snapshot
telemetry and Final's reframed status coordinates. It is not evidence that
Final needs another 2,500 m of range tuning.

Both fixtures now use the recorded configuration's west threshold. Vehicle
initial position, velocity, attitude, mass and UT are unchanged. A runner
preflight check rejects inconsistent physical runway endpoints, elevation or
width before launching processes; an actual reciprocal threshold/heading pair
is accepted. The focused test reproduces the original error and exercises both
corrected fixtures and reciprocal orientation.

The corrected overhead MM305 replay passes at +1,891.369 m along / +0.117 m
cross, 1.863 m/s sink, valid rollout, no tail strike or departure:
`mm304-parity-runway-parity-sts-kerbin-20261028-live-overhead-mm305-20261028T061353Z-cr4g6zsx`.
This is a new replay, not an arithmetic relabeling of the old result; moving the
plant runway also corrects its ground-contact geometry.

The three previously passing cases still pass: orbit86 MM304 at +815.619 m /
-0.467 m, sink 2.938; orbit86 MM305 at +662.171 m / -0.009 m, sink 2.948;
quicksave MM305 at +674.383 m / -0.116 m, sink 2.672.
Full-entry and remaining regressions are pending. C tests pass; Python discovery
passes 20 tests with nine integration tests skipped (29 discovered).


## MM305 inside-cone recovery checkpoint (`523eb26`)

The first corrected full replay (`mm304-runway-parity-20261028T061255Z-143n9ta0`)
did not pass: -7,262 m along / -1,761 m cross, sink 14.73 m/s. Its final
MM304 forecast cost was 15.8; the realized handoff at UT 148743777.186316
had range 45,599.26 m, heading error 11.6047°, and cost **15.6055** against
the 49,600 m range target. The late forecast is now accurate; the remaining
failure occurs in MM305 acquisition.

Inside the heading-alignment cone, no forward tangent sometimes exists.
The acquisition law then steered toward the centre while waiting for tangent
heading capture. It crossed the diameter, acquired an additional half orbit,
and delivered Final at only 783 m altitude, 9 km before the runway. The
existing radial HAC controller has outward-bank authority to recover from
inside, but previously never owned this state.

An unrestricted early radial-capture experiment passed the frozen handoff
but regressed the quicksave MM305 case. It prematurely assigned the shorter
established-arc energy reference to a fast radial inlet. That experiment is
rejected; its unfinished full-entry/old-entry runs were stopped deliberately.
The committed recovery uses the existing model's energy requirement for the
remaining local arc: it captures radially only when no tangent exists, the
vehicle is inside the cone, and that arc already costs more energy than is
available. Normal acquisition remains unchanged with sufficient energy.

`sts-kerbin-20261028-predictor-mm305.ini` freezes the corrected full replay's
Mach-2.5 state with a fresh guidance machine. It is a diagnostic supplement,
not a replacement for full-entry validation. Before recovery it crashes;
after recovery it passes at +283.184 m / +8.280 m, sink 2.927 m/s, speed
65.618 m/s, no bounce/tail strike/departure, valid rollout to +1,143 m.
The focused C regression exercises the recorded inside-cone geometry with
both short and adequate energy. The full C test suite passes.

All three MM305 cases and the 40 km entry retain passes with this refinement.
The corrected-fixture OMS72 replay before the cone change also passes
(+799.561 m / +0.364 m, sink 2.940); run9 still fails (+39,582.962 m /
+3,627.569 m, sink 47.316). Final full-entry and old-entry results are recorded below.


## Final offline results and limitations

At `523eb26`, the corrected full 72 km entry replay
`mm304-cone-energy-full-20261028T062901Z-ird9ouzj` passed: touchdown
+703.663 m along / +3.242 m cross, sink 2.648 m/s, speed 63.420 m/s,
gear down, no tail strike or runway departure, and valid rollout to +1,527 m.
The forecast costs at Mach 5, 4.5, 4, 3.5, 3 and 2.5 were respectively
7.6, 5.7, 8.9, 12.3, 13.3 and 14.6. At the realized Mach 2.5 handoff,
the actual heading-and-range cost was 14.43 versus the last forecast 14.6.
The replay produced 256 replans; timing depends on solver wall time.

For an equal-geometry comparison, the original `d2a05bc` binary with the
corrected 72 km runway fixture crashed at -79.157 m along / -26,302.869 m
cross, sink 47.167 m/s (`/tmp/mm304-runway-baseline.log`). This is a single
replay for each version, not a statistical reliability claim.

The final six-scenario regression results, in metres (sink in m/s), are:

| Scenario | Result | Along | Cross | Sink |
|---|---|---:|---:|---:|
| sts-live-oms72-entry-20261009 | crash | +1,263.043 | -505.755 | 89.297 |
| sts72km-run9-entry-interface | crash | +39,069.409 | -1,635.159 | 47.315 |
| sts-kerbin-20261022-live-orbit86-mm304-40km | pass | +815.619 | -0.467 | 2.938 |
| sts-kerbin-20261028-live-overhead-mm305 | pass | +1,891.369 | +0.117 | 1.863 |
| sts-kerbin-20261022-live-orbit86-mm305 | pass | +662.171 | -0.009 | 2.948 |
| sts-kerbin-20261021-quicksave-mm305 | pass | +674.383 | -0.116 | 2.672 |

Both old entry scenarios also crashed on the original branch baseline. OMS72
passed in the intermediate corrected-runway run before the cone change; the
final failure is therefore an unresolved regression relative to that run,
although differing replan timing also changes its inlet. Its cause has not
been isolated. No claim that the final branch has no regressions is justified.

A frozen-inlet sensitivity check with both plant lift and drag scaled to 0.90
reached the runway at +856.043 m / +0.719 m and stopped on it, but failed the
3 m/s touchdown sink limit at 3.506 m/s. It is a simple sensitivity check,
not a reconstruction of the recorded flight. The measured live force
residuals described above remain uncorrected; live testing was not performed.
The optional diagnostic traces remain because the requested clean-up was
conditional on successful replay **and** no regressions. Do not merge or
rebuild the flyable binary on the strength of this checkpoint alone.

The user's storage cleanup losslessly compressed completed experiment stderr
and older flight logs; some archived backend logs are now named
`backend.stderr.log.gz`. Replay telemetry and summaries were retained.
