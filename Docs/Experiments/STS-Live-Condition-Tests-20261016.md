# STS live condition tests — 2026-10-15/16

Scope: live KSP runs of the STS polar profile from several entry points, with no source edits. Each run is recorded in `runs/<run-id>/`.

## Runs in this session

| Run | Start | Result | Failing phase |
| --- | --- | --- | --- |
| `20261015T043321Z-sts-polar90-live-orbit` (label `sts-polar90-live-orbit`) | orbit save `ksl-20261014T165854Z-sts-polar90-full-20261015-orbit` | FAILED | Rollout, abort on attitude recovery |
| `20261016T045121Z-cond-mm305-a` | MM305 save `ksl-20261015T153506Z-sts-incl60-consistent-mm305` | ABORTED (by operator) | MM305 TAEM, attitude recovery |
| `20261016T045116Z-dryrun-test` | same MM305 save | dry run only, holding | — |

### From-orbit run (`sts-polar90-live-orbit`)

- Deorbit: best available plan was unqualified; engaged under warning.
- MM304 Entry at 13:38; at 13:55:41 it warned that the maximum-range drag profile could not reach the required Final alignment path. Lateral displacement was not corrected before MM305.
- MM305 TAEM at 13:58 (25.5 km, 824 m/s, 130.5 km range).
- Rollout at 14:08: radar 0 m, runway along −24.9 km, cross 40.2 km. Backend entered Abort: insufficient height for attitude recovery.
- The vehicle was about 25 km short along-track and 40 km off the centreline at touchdown.

### MM305 start (`cond-mm305-a`)

- Start at MM305 TAEM from the `sts-incl60-consistent-mm305` save with `KSP_LANDER_CONTINUE_UNQUALIFIED=1`.
- Live aero residual vs the certified prior-flight uncertainty: 117% at the start, down to about 50%. The live response stayed outside the band throughout.
- 14:04:57 WARN: MM305 crossed runway elevation before reaching the fixed-HAC exit.
- 14:05:02 attitude recovery began at radar about 0 m, runway along −2.25 km, cross −2.05 km. It exceeded 15 s without stabilizing.
- After that the vehicle sat at 0 km and 0 m/s. No further output for several minutes. Operator interrupt at 14:08:15 ended the run as ABORTED.

## Earlier night runs (same profile, for context)

From `ksl runs`: `sts-night-1` failed at Rollout, `sts-night-2` failed at MM304, `sts-night-3` and `sts-night-4` failed at deorbit. `sts-night-4` ended with a kRPC decode fault (`SpaceCenter.WarpRate`, `Thruster.InitialThrustDirection`).

## Observations

1. Every live run this session that reached guidance ended in a failure or abort. The only completed landing remains the earlier run from the 72 km save (`sts-live-20261015`, see `STS-Polar90-Followup-20261015.md`).
2. The attitude-recovery failure appears in both the from-orbit run and the MM305 test, once radar altitude reaches about zero with the vehicle still in a rollout or TAEM attitude. The 15 s recovery window is not enough at that point.
3. The MM304 warning about the Final alignment path appears again in the from-orbit run, so the lateral correction before MM305 is still not closing the miss.
4. The aero live residual is outside the certified band on MM305 starts. The table-based aero model needs to be checked against this range before more live tests.
5. The web view throttle, retro error, ΔV remaining and guidance-state issues are still open; nothing was changed for them in this session.

## Not done

- The planned condition matrix (final-approach start, orbit start under the sts-live profile) has not been run.
- No source changes were made for these tests.

## Next

- Offline: reproduce the MM305 touchdown-altitude and attitude-recovery failure against the sts-live profile before another live run.
- Live: wait for explicit go-ahead before running another start, since each start moves the vehicle in KSP.

## Fixes and pad-launch verification (2026-10-16 afternoon)

### Fixes

- `CLanding/guidance/guidance.c`: warning recovery now treats `splashed` as surface contact, so a vehicle stopped in water ends in Abort instead of sitting in Attitude Recovery indefinitely (the `cond-mm305-a` hang).
- `CLanding/transport/client/warp.inc`: the warp-state read retries up to 3 times on a transient kRPC decode failure (the `sts-night-3`/`-4` UT/WarpRate faults). Rails warp also unpauses KSP first. A paused game left by an earlier failed run had stalled the deorbit coast: "Rails warp made no measurable UT progress".

### Live verification: pad → 85 km / 10° → KSC

Saved prelaunch `sts-polar90-prelaunch-20261015`, profile `sts-85km-10deg`, `KSP_LANDER_CONTINUE_UNQUALIFIED=1`.

| Run | Segment | Result |
| --- | --- | --- |
| `20261016T083029Z-sts-10deg-pad-verify` | Launch → Ascent → Orbit (trim to 85.00 × 84.96 km, 10.000°) | Orbit OK. Deorbit refused by the profile's `requireStrictPlan` (the plan was burn/entry-qualified only). |
| `…-pad-verify-land` | Deorbit from the same live orbit | Fault: rails warp with KSP paused. Led to the unpause fix. |
| `20261016T084526Z-sts-10deg-pad-verify-land2` | Deorbit → MM304 → MM305 → Final → Touchdown → Rollout | **COMPLETE**, exit 0 |

On the `land2` run:

- Deorbit 30.0 m/s, guarded recovery plan. MM304 again warned that the maximum-range drag profile was energy-short.
- Final was captured at 18:17:00 with airspeed below target, so the flare was firm.
- The vehicle landed on KSC runway 09/27 and stopped on the pavement about 9 m from the centreline. The orbiter ended landed at 25.4 t.

Same vehicle and same flight from the pad, but run under three run IDs because of the two blocks above. The landing used `--set landing.requireStrictPlan=false`.

### Still open

- Polar (90°) returns: the MM304 lateral miss is not addressed.
- MM304 energy-short warning and the low-energy Final, also present on this low-inclination flight.
- The strict full-route gate is not achievable for this profile as configured.
