# Conditional touchdown forecasts against closed-loop replay

Unmodified L5/L7 MM305 replays preserve the actual propagated Final inlet,
attitude, commands and learned aerodynamic history. Comparing their existing
`FINAL_TRACE` forecasts with eventual same-flight touchdown shows a large L5
late-forecast error and a nearly accurate L7 late forecast. A common correction
factor is not supported. These are existing surrogate attitude plants, not
qualified evidence for live flare tuning.

Reproduce from the integration checkout:

```sh
KSL_DATA_ROOT=/Users/kyooni18/Code/KSL-integration KSP_LANDER_FINAL_TRACE=1 bin/ksl simulate l5-mm305 --json --label final-forecast-audit
KSL_DATA_ROOT=/Users/kyooni18/Code/KSL-integration KSP_LANDER_FINAL_TRACE=1 bin/ksl simulate l7-mm305 --json --label final-forecast-audit
.venv/bin/python Docs/Experiments/STS-Landing-20261011/Final-Forecast-Replay-Comparison/analyze.py runs/<L5-id>/simulation-result.json runs/<L7-id>/simulation-result.json
```

L7 returns exit code 1 because landing acceptance fails; this is a completed
replay, not an execution error. The retained runs are
`20261011T112746Z-final-forecast-audit` (L5) and
`20261011T112809Z-final-forecast-audit` (L7). Result files, simulator manifests,
binary/input provenance and recording hashes are referenced in `result.json`.
Trace values are rounded to 0.1 m/s. CSVs retain forecasts every five seconds,
plus first/last pre-latch forecasts. Nonfinite report values are JSON null.

The analyzer requires one monotonically advancing guidance trace stream and
matches every finite pre-latch forecast to the actual simulator speed within
0.06 s / 0.15 m/s. This prevents shadow predictions from being confused with
flown states. Actual touchdown speed/sink comes from the simulator's contact
summary, not from its later post-contact velocity sample. No state relocation,
force reset, artificial route adoption or hypothetical arrival was used.

| Observation | L5 | L7 |
|---|---:|---:|
| First Final height above wheels m | 2,526.9 | 1,992.3 |
| First Final speed m/s | 140.7 | 97.3 |
| First touchdown-speed forecast m/s | 87.6 | 70.6 |
| Last finite pre-pull-up forecast m/s | 76.6 | 61.8 |
| Actual touchdown speed m/s | 66.129 | 61.929 |
| First forecast minus actual m/s | +21.471 | +8.671 |
| Last forecast minus actual m/s | +10.471 | −0.129 |
| Last finite forecast wheel height m | 819.2 | 120.5 |
| First pull-up latch wheel height m | 815.6 | 118.4 |
| Actual touchdown sink m/s | 1.844 | 3.353 |
| Actual contact along runway m | +297.613 | −376.232 |
| Complete landing acceptance | Pass | Fail |

L5 exactly reproduces touchdown speed 66.129 m/s, sink 1.844 m/s and stop
1,101.616 m. L7 reaches Final/flare and touches down gear-down without a crash,
but is 376 m short and exceeds the 3 m/s sink gate. This is the current
150 m aim configuration; the historical scenario description's 55.5 m/s /
0.91 m/s / 226 m-short figures describe an earlier configuration, not this run.
No configuration, golden or flight source changed during this comparison.

## What the forecast does and does not predict

`terminal_predict_touchdown_speed` in `guidance/final/sequence.inc` freezes the
current glide FPA until a preflare trigger, then follows a fixed sink profile
using learned lift/drag in a 0.25 s point-mass integration. It selects incidence
to meet lift demand immediately. It does not replay the future Final controller,
pitch-rate/acceleration response, changing aim, or future estimator updates.
The actual controller recomputes its targets while flying. In L5 the recorded
preflare trigger changes from 2,035 m at Final entry to about 816 m at pull-up;
that alone changes the trajectory assumed by earlier forecasts. Input state
and learned history are real, but the forecast's future path is conditional.

The production controller computes this forecast only before its pull-up latch.
The NaN forecasts logged after latching are intentional uncomputed values,
not failed touchdown forecasts. The analyzer therefore excludes them. The last
finite forecast is just before actual pull-up, but differences in instantaneous
lift allocation versus actuator response and flown sink/attitude can still
accumulate through the remaining flare. These data identify a discrepancy;
they do not isolate its cause or establish that rescaling drag fixes it.

Native Final admission also uses preflare energy projections and runway/load
checks. This comparison evaluates the controller's touchdown forecast, not
every admission projection in the preceding synthetic audit. Do not use a
passing conditional forecast or the legacy L5 landing as full plant/route/live
qualification. A useful next experiment freezes the same late state/plan and
compares point-mass integration with native response-limited propagation,
recording drag work, elapsed time and sink history before changing guidance.

No production code, gates, new tests or live KSP flight were changed or run.
Existing landing behavior was checked against the L5 golden and the L7 flare
regression requirements; the latest full suite remains 105 tests / 44 subtests
passing from `d9f6dec`.
