# Third approved MM305 continuation

The user approved loading `ksl-20261011T064053Z-live-85km-10deg-landing-mm305`
and running `bin/ksl land --from mm305 -p sts-85km-10deg`. The current backend
was built at 61fb088; the measured simulator experiments were committed as
aeee7b7 during flight. Experimental body-plant coefficients do not affect live
control. Main checkout files were not modified.

The flight again reached low-energy Final and flare with gear down, then
rolled beyond the 2500 m runway. **FAILED, automatic exit 1**, backend Abort
for `Touchdown occurred outside the runway 09 contact envelope.` No operator
abort was needed. The automatic grounded failure exit is verified again;
landing acceptance remains unsuccessful.

| Event | UT | Speed | Sink | Along |
|---|---:|---:|---:|---:|
| Final | 6931.590 | 94.16 m/s | 26.63 m/s | −7690 m |
| Flare | 7050.210 | 63.67 m/s | 7.78 m/s | 1243 m |
| Last sample before initial ground response | 7055.970 | 50.81 m/s | 3.59 m/s | 1567 m |
| First Rollout, still flying | 7056.050 | 50.12 m/s | upward 0.87 m/s | 1575 m |
| First landed flag, after response | 7056.530 | 49.29 m/s | 0.78 m/s | 1598 m |
| Automatic Abort | 7117.850 | cached 3.08 m/s | 0.06 m/s | 2630 m |
| Fresh paused observer | 7117.990 | 1.36 m/s surface | 0.02 m/s | native last along 2630 m |

Motion and runway geometry are cached at different rates. The pre-response
row is a chronology marker, not an exact wheel-contact sensor. Its speed is
below 63.75 m/s and sink exceeds 3 m/s. The earlier observer sample at UT
7055.010 (52.49 m/s, 2.58 m/s sink) was still descending before this final
pre-response sample. First landed sink is already affected by ground contact.
Do not report it as flare performance.

Touchdown/Rollout/Attitude Recovery phase oscillations and mission-layer
illegal backward-transition warnings recurred after initial response.
The universal terminal backstop was not exercised: low-energy TAEM delivery
owned admission. Adaptive flare remained off; no deorbit/MM304 flight was
performed. Contact was slow and long, with a roughly 130 m overrun. Mass in
observer samples declined by less than 1 kg; no earlier 230/460 kg damage
signature recurred. Parts were not inspected.

All 8595 native samples report SAS and airbrakes off, zero commanded throttle
and actual thrust. All 1663 observer samples report SAS/RCS off, zero
throttle/thrust, speedbrake action group off and zero warp. These are sampled
checks. The final observer confirms KSP paused, STS landed at 1.36 m/s,
brakes and gear on. This is not a full physical stop; the failed-rollout
termination pauses at its low-speed threshold.

## Provenance and setup

`make` passed immediately before flight. The preceding measured-state changes
passed `make test` (104 tests, 44 subtests), preserving historical goldens.
The save settled at UT 6566.790 and was paused at UT 6570.790; RCS was disabled
before engagement. The kRPC paused flag initially remained true while observed
UT advanced under backend control. Explicit guard resume was issued near UT
6717.930. Thus this is the same saved checkpoint but not an identical actuator
or time-step history to earlier continuations. No warp was requested.

Runtime and FlightLogs were directed to the integration worktree with
`KSL_DATA_ROOT`; gate, Final and TAEM traces were enabled. Evidence:
`runs/20261011T092911Z-live-l7-current-verification/`, including backend stderr,
status, telemetry, independent-observer.jsonl and final-probe.json. Native
stream: `FlightLogs/2026-10-11T09-29-11Z-STS-vehicle.jsonl` and paired planner
stream. [analyze.py](analyze.py) records SHA256, phase/contact chronology,
sampled invariants and the final probe in [result.json](result.json).
