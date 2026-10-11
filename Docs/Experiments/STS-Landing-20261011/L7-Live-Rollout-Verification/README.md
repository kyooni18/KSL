# L7 repeat live MM305 verification

The user explicitly approved loading
`ksl-20261011T064053Z-live-85km-10deg-landing-mm305` and flying
`bin/ksl land --from mm305 -p sts-85km-10deg` again. The integration worktree
used native guidance from `d9881a1`, including `9fdfa0a`'s failed-rollout exit.
The uncommitted predictor-parity candidate was removed before building and
flying; it is not part of this result. Main checkout files were not modified.

## Outcome

Flare was exercised, but landing acceptance failed again. The CLI terminated
**automatically**, exit 1, at Rollout with backend Abort and
`Touchdown occurred outside the runway 09 contact envelope.` No operator
abort was sent. This verifies the grounded failure termination fix, not a
successful landing or full physical stop.

| Event | UT | Speed | Along | Evidence |
|---|---:|---:|---:|---|
| Final | 6923.570 | 96.65 m/s | −7725 m | low-energy delivery; diagnostic h 2473 m above runway |
| Flare | 7030.450 | 75.68 m/s | 1126 m | 7.83 m/s sink at entry; gear down |
| Before initial ground response | 7042.450 | 47.85 m/s | 1829 m | 4.03 m/s downward, radar 3.56 m; repeated motion values are cached |
| First Rollout | 7042.610 | 46.28 m/s | 1829 m | upward 1.24 m/s, situation still flying: bounce/contact response |
| First landed flag | 7043.650 | 44.91 m/s | 1883 m | occurs after first ground response; not a pre-contact sink measurement |
| Automatic failure / paused probe | 7091.390 | 1.37 m/s surface speed | 2810 m | landed, brakes/gear on; native cached airspeed 1.61 m/s |

The pre-response sample is a chronology marker, not an exact wheel-contact
sensor measurement. Its sink already exceeds the desired 3 m/s ceiling and
its speed is below the 63.75 m/s acceptance floor. Along/cross geometry also
updates less often than the frame timestamps. Do not use the first landed
flag's near-zero vertical speed as touchdown performance.

The native phase changed repeatedly between Touchdown, Rollout and Attitude
Recovery after the bounce, producing illegal-backward-transition warnings
in the mission layer. Near the runway end it even emitted a still-airborne
long-landing warning after prior landed samples. These are diagnostic issues
to investigate, not evidence that the actual rollout began beyond the runway.
Mass fell from roughly 25123 kg to 24663 kg during the overrun, consistent
with damage; affected parts were not identified.

All 8403 native samples reported SAS/airbrakes off and zero commanded throttle
and actual thrust. The 854 independent observer samples reported RCS/SAS off,
zero throttle/thrust and speedbrake action group off. These are sampled checks,
not proof between samples. The final fresh kRPC probe confirmed STS landed,
paused, brakes and gear on, both warp factors zero, and those control
invariants. The read-only observer was then stopped because the backend's
pause at 1.37 m/s prevented its below-0.3 m/s stopping condition.

The universal terminal backstop was not exercised: low-energy TAEM delivery
owned Final admission. Adaptive flare remained disabled. No deorbit/MM304
route or other TAEM path was verified by this continuation. Delivery differs
from the earlier flight (about 238 m higher and 2.4 m/s faster); the flare
result remains sensitive and the simulator gap remains unresolved.

## Reproduction and provenance

`make` passed before flight. The preceding candidate passed `make test`
(104 Python tests, 44 subtests) and native stack checks, with unchanged L5/L7
replay outcomes. Broader MM305 admission qualification failed 0/5 both with
the candidate and on the unchanged baseline; no gate was weakened. See the
[range-control investigation](../MM304-Range-Control/README.md).

The save was loaded with `Tools/ksp_test_guard.py load-save --name …` and held
paused at UT 6566.930 while CLI direct control engaged. RCS was explicitly
disabled before flight. Once TAEM engagement was confirmed, the guard resumed
KSP, with no warp requested. Runtime/FlightLogs were directed to the worktree:

```sh
KSL_DATA_ROOT=/Users/kyooni18/Code/KSL-integration \
KSP_LANDER_GATE_TRACE=1 KSP_LANDER_FINAL_TRACE=1 KSP_LANDER_TEL_TRACE=1 \
bin/ksl land --from mm305 -p sts-85km-10deg \
  --label live-l7-rollout-verification --no-color
```

Evidence: `runs/20261011T083437Z-live-l7-rollout-verification/`, including
independent-observer.jsonl and final-probe.json; native source
`FlightLogs/2026-10-11T08-34-41Z-STS-vehicle.jsonl` and paired planner stream.
[analyze.py](analyze.py) decodes keyframe/delta frames into [result.json](result.json),
which retains SHA256, transitions, contact chronology, terminal status,
invariants and the final probe. Raw streams are retained locally.
