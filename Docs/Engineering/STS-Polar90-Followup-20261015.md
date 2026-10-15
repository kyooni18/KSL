# STS Polar 90 follow-up — 2026-10-15

Follow-up to `Docs/Experiments/STS-Polar90-Live-Test-20261015.md`. Offline only; KSP was not restarted or commanded.

## Fixed (main)

| Commit | Item | Result |
| --- | --- | --- |
| ca36dd3 | Native build broken (`jet_assist` vs `jet_assist_throttle`) | Transport drives proportional jet throttle; `make test` builds |
| 8b9a640 | L5 golden replay regression (bisected to 9e108c2/79d3c8f) | TAEM station slope capped at configured `finalGlideSlope`; L5 lands 72.4 m/s, 1.53 m/s sink, stops 908 m |
| 8b9a640 | Flare attitude recovery at 8 m (sideslip runaway) | ShuttleSim adapter reports nose heading = track − β (offline only) |
| a016894 | TUI test failures (live-KSP contamination, Launch tab default, click race) | `tests/conftest.py` stubs kRPC; tests fixed |
| f6c55e8 | Vacuum Mach = TAS | Mach uses atmosphere-top speed of sound |
| fe38f89 | `ksl simulate <raw.ini>` silently flew general `default.json` | Uses the profile's backend config and aero profile |
| fe38f89 | Orbit checkpoint replay retargets 90 → 100 km | Mission started at Orbit refuses trim when the established orbit is >5 km off target |
| 0e9e8b4 | Intelligent ascent lost near pad | Aero α cap 35° → 90°; the cap bound pitch to noisy pad prograde; traceback now logged |
| 7258d5b, 7dce60e | Offline reproduction | `sts-polar90-72km.ini` / `-30km.ini`; `benchmark_deorbit_planning --polar90-terminal` |

Offline gate: 217 passed, 0 failed, 56 subtests.

## Polar return

**Fixed in f0c6bd6.** MM304 now measures the lateral miss as the capture point's angular distance from the inertial trajectory plane, with the capture point rotated by planet rotation over time-to-go. This replaces the straight-ground-track `range·sin(course error)`. Over short ranges it reduces to the old value. The calibrated replay (plant `sts_aero_polar90_live20261015.csv`, fitted from the live flight) went from 10 km short / 92 km cross-track at MM305 handoff to touchdown 201 m along / 9 m cross at 65 m/s. Its only remaining failure was a harness scoring bug. The energy law chose RW27, but the harness measured rollout heading against RW09 because no fixed HAC record exists. Fixed in 218a6b3; the replay now passes acceptance (touchdown 65.4 m/s, 1.6 m/s sink, stopped on runway). The fix is in shared MM304 code; `make test` (218 passed) shows no general-guidance regression.

Deorbit ranking (item 1 below) was left unchanged: the replay that now lands flies the live burn, so selection was not the blocking fault.

### Original diagnosis

The 72 km replay (`ksl simulate ShuttleSim/scenarios/sts-polar90-72km.ini --from mm304 -p sts-polar`) reproduces the live miss: MM305 handoff ~147–153 km out at 25.7 km, landing ~50 km off the runway.

1. **Deorbit plan is energy-short by design.** Every inlet candidate is 210–270 kJ/kg short. The secant refinement found a burn placing the inlet at 94.7 km (nominal 96.5), but ranking preferred a 36 km candidate with a smaller deficit. The flown trajectory matches the plan through ~45 km; the reported 492 km "entry range" gap was a definitional mismatch.
2. **MM304 lateral law spends lift on a phantom miss.** `point_miss = alignment_range·sin(course_to_site_error)` assumes a straight ground track. On the polar pass it read an 8–10° error (~100 km) at Mach 8 and banked to 40°, which barely turned the vehicle. With lateral demand suppressed (experiment, not committed): handoff 131 km instead of 153, final cross-track 29 km instead of 52. The prediction should use the rotation-curved ground track, or the deorbit witness's predicted pass.
3. The MM304 live replan correctly reports the shortfall (~88 km short at 0° bank from 69 km). MM304 cannot recover it, so the fix belongs in deorbit selection (1) plus lateral law (2).
4. Not adopted: gating MM304's α drag boost on range surplus (no measurable effect).

## Not addressed

- The overnight Final-only envelope replays used general `default.json` with 40 t STS-N scenarios. They are general-guidance diagnostics and were left unchanged.
- The TAEM qualification stall (native MM305 route planner at ~100% CPU) occurs under general guidance only; the STS energy law does not stall.
- ShuttleSim closed-loop results are wall-clock coupled; exact values differ by checkout.

## Live retest and overshoot fix (2026-10-15, later)

**Live run** `runs/20261015T022111Z-sts-polar90-live-planemiss-cont`, from the 72 km save, with `KSP_LANDER_CONTINUE_UNQUALIFIED=1` because the shadow predicted 60 km short on the drag model:
- The lateral fix held live: MM305 handoff at 25.6 km with 61 km cross-track (previously 85–92 km), and Final lined up on RW27 within 25 m.
- The vehicle **overshot**: it touched down past the RW27 end and stopped 783 m beyond it.

**Aero model (9744130).** The apparent inclination dependence of drag tracked α instead (r = 0.93). `live20261010` is right near α 27° but has bad cells, for example drag ×0.70–0.73 at Mach 4.8–5.7, α 20°.
- `Tools/fit_sts_aero_cells.py` replaces each table cell with live-measured values. The samples are unpowered, gear up, airbrakes stowed, and pooled across all inclinations.
- The result is profile `sts-live-20261015`. It is not yet the mission default.

**Overshoot causes and fixes**

1. **TAEM energy pricing (fbbc425).** The straight-in energy law priced a Dubins path to the Final station pose, while its lateral pursuit captures the centreline one lookahead ahead. Arriving from the side past the station's along-track distance, the law held energy for a roughly 13 km loop. Capture at 5 km left the vehicle 3.3 km high. The energy pricing now ends at the predicted capture point when that point is inside the station. Lateral steering and the vertical glide reference are unchanged.
2. **Split-rudder speedbrake (9fc5c2a, fbbc425).**
   - STS Final now holds its reference speed with the fin, controlled by `guidance.terminalSpeedbrake` in `sts-live.json` only.
   - In the preflare and float, the fin keeps touchdown inside the long-landing limit.
   - The fin's drag area is learned in flight. The opening is capped at 50%.
   - ShuttleSim now models the proportional fin.
   - The transport no longer zeroes the fin under `KSP_LANDER_UNPOWERED_ONLY`.
3. **Harness.** A slow touchdown passes when the sink rate is within limits (operator rule).

**Replays.** `sts-polar90-live-taem.ini` starts at the live TAEM handoff:

| Run | Touchdown speed | Sink | Outcome |
|---|---|---|---|
| Before the fixes | 79 m/s | — | Overran |
| Old guidance model | 66 m/s | 2.2 m/s | On runway, stopped with 237 m left |
| Measured-cell guidance model | 68 m/s | 1.1 m/s | On runway, stopped with 618 m left |
| 72 km polar replay | 59.6 m/s | 1.2 m/s | On runway |

`make test`: 218 passed.

**Open items**
- Switch the STS missions to the `sts-live-20261015` profile after a live check.
- Drag above Mach 7.1 is still unmeasured.
- The split-fin drag area is learned in flight but has not yet been measured live.

## Live verification (2026-10-15 13:06–13:26 KST)

**Run.** `runs/20261015T040616Z-sts-polar90-live-v2c` (label `sts-polar90-live-v2c`) used profile `sts-live-20261015` at a515a09 and flew from the 72 km save. Result: **COMPLETE.** It flew MM304, MM305, Final, Touchdown, Rollout and Complete in sequence, and the vehicle stopped on the runway.

**Approach**
- MM305 handoff: 25.5 km, 838 m/s, 61.6 km cross-track.
- Final: RW27, 113.6 m/s about 366 m above the runway and 456 m before the threshold. The previous live run was at 136 m/s and about 1,180 m up 1 km before the threshold.

**Touchdown and rollout**
- Touchdown about 1.55 km past the RW27 threshold at about 54 m/s.
- Small bounce: the first ground sample reads +2.35 m/s upward, then a second contact at −1.24 m/s.
- Stopped 787 m before the runway end, 18 m off the centreline. The previous live run stopped 783 m past the end.

**Engage gate.** Engage needed `KSP_LANDER_CONTINUE_UNQUALIFIED=1`.
- The closed-loop shadow's MM304 prediction uses the scalar L/D and ballistic-coefficient model, not the aero table. It therefore predicts 60 km short on either profile.
- `KSP_LANDER_SIM_SHADOW=1` now runs that shadow in ShuttleSim, for diagnosis.
- A first attempt engaged right after `--load-save` and was rejected on unsettled telemetry: peak q 0, range error +1,278 km.

**Open items**
- The shadow's MM304 aero model should come from the table.
- Settle telemetry before qualifying after a save load.
- The flare produced a small bounce.
- Touchdown was long: about 1.55 km past the threshold, with 0.79 km of runway left at the stop.

## Orbit-start failure and telemetry repair (2026-10-15, afternoon)

The orbit-start run `20261015T043321Z-sts-polar90-live-orbit` **failed** at
14:08:27 KST. It flew Orbit → Deorbit → MM304 → MM305 → Rollout → Abort;
it never reached Final. At abort the vehicle reported landed, 48.65 km from
KSC, with runway cross-track magnitude 40.23 km. This does not invalidate the
v2c checkpoint landing, but that landing is not evidence of a reliable return
from orbit. KSP was not restarted or commanded during this follow-up.

The engaged plan was already unqualified: burn UT 24907.368, ΔV 34.096 m/s,
normal angle 34.475°, predicted TAEM range error −39.49 km, inlet energy
margin −238.49 kJ/kg, robustness 0/15. The run used the diagnostic unqualified
execution override. Do not classify this whole failure as a shadow-only false
negative: the flight itself ran out of energy before reaching the runway.

Decoded vehicle keyframes/deltas show that the flights diverged before TAEM:

| Descending altitude | v2c latitude / speed | Orbit-start latitude / speed |
| --- | --- | --- |
| 69 km | 68.628° N / 2302.4 m/s | 65.789° N / 2300.9 m/s |
| 60 km | 83.366° N / 2303.9 m/s | 88.223° N / 2304.4 m/s |
| 50 km | 47.813° N / 2193.4 m/s | 55.186° N / 2199.9 m/s |
| 40 km | 24.955° N / 1917.4 m/s | 31.701° N / 1925.7 m/s |
| 30 km | 11.409° N / 1370.9 m/s | 17.523° N / 1354.8 m/s |
| 26 km | 6.382° N / 894.7 m/s | 12.227° N / 867.2 m/s |

TAEM first records: v2c 25.75 km / 864.1 m/s / 6.113° N; orbit-start
25.98 km / 865.2 m/s / 12.209° N. The latter's STS energy log prices a
136.3 km remaining path against an 86.3 km required energy-dissipation path.
These observations locate a substantial deficit at handoff; they do not yet
prove whether burn selection, prediction-model mismatch, or entry execution
is the dominant cause. Flare tuning alone cannot repair this run.

Telemetry repair `b7b124b` preserves the backend command, guidanceState,
autopilotError, statusMessage and warningMessage in mission telemetry and
merges them through the run overlay. Full targetPitch takes priority over the
legacy FPA+AoA reconstruction. Stale records do not supply full command,
alignment or burn-progress state during warp. Older flattened records retain
the fallback path. The frontend uses commanded throttle while engaged and the
backend's alignment error for inertial burns. A follow-up labels these readings
CMD THROTTLE and BURN ALIGN: this plan's normal component means the commanded
burn direction is not pure retrograde. No flight-guidance code was changed.

Validation:
- `make test`: **219 passed, 56 subtests passed**, 2 warnings, no failures
  (62.02 s). Both warnings are test background threads trying to write temporary
  run status after directory cleanup (`ksl-mission`, `ksl-commands`). Full output
  is `/tmp/ksl-make-test.log`.
- The focused overlay test and `node --check WebTelemetry/app.js` passed after
  the final message/label changes.
- Offline browser fixture, deliberately conflicting with observer values:
  **18% command, 4.0° burn alignment, 10.6 m/s remaining** from 23.5 m/s
  delivered against the archived 34.096 m/s plan. This is synthetic regression
  evidence, not a live burn measurement.
- Archived v2c screenshots at MM304 (UT 25943.1), TAEM (26038.5), and Final
  (26291.0) confirm phase/command rendering. The old archive lacks full burn
  guidance state; this is not end-to-end validation of a new live flight.
- Screenshots and the temporary offline fixture script are in
  `work/telemetry-verification-20261015/` (ignored local diagnostics).
- The read-only observer on port 8789 reloaded the repair. Its supervisor
  automatically relaunched it; a redundant manual start hit Address already
  in use and exited. No duplicate observer was left running.

The rebuilt offline `benchmark_deorbit_planning --polar90-terminal` with
`KSP_LANDER_MODEL_PROFILE=sts-live-20261015` still gives **inlet=false,
route=false, final=false, robustness 0/15**, despite entry-stage execution=true.
It selects burn UT 25037.674 and inlet margin −210.9 kJ/kg. This fixture uses an
older frozen orbit/aero envelope, not the exact latest orbit-start state;
it must not be presented as reproduction or validation of a live fix.
Output: `/tmp/ksl-polar90-plan.log`. An initial stale benchmark binary rejected
the newer profile; rebuilding the benchmark resolved that tooling issue.

Remaining work: reproduce the exact latest orbital state and calibration
through the STS planner and table-based shadow, validate a positive inlet
energy reserve and achievable route, then retest from orbit. Long touchdown
and the small bounce remain separate open issues. No reliable orbit-start
landing has been demonstrated by this follow-up.

## Delivery-policy follow-up (2026-10-15 afternoon)

The user requested fixes followed by live reruns, including investigation of a smoother drawn RW09 alternative. These changes are isolated behind `guidance.stsTrajectoryDelivery` and model ID `STS`; `Configuration/sts-live-delivery.json` opts in. The default general program and archived L5/L7 scenarios retain their existing control behavior. Mission aero profiles now use `sts-live-20261015-live-only`; archived scenarios remain on 20261010.

### Source provenance correction

The original cell fitter accepted simulator vehicle logs. Source classification now requires an STS session with TCP transport and reported live applied-control feedback, rejects explicit simulator identities and ambiguous sources, deduplicates content, and groups rotated logs by session. The replacement table has 47 measured cells from 206 verified live sessions. Unmeasured cells come from the explicitly provisional original table, rather than inheriting the later mixed-source fits. Input hashes, classifications, base hash and cell support are retained in `ShuttleSim/reference-model/sts_aero_cells20261015_live_only.json`. This is still provisional evidence, not certification or independent-flight count. The historical mixed-source profile is retained for replay.

Commits: `8fcfb15` source guard, `aef430f` replacement profile. A minimal source-provenance regression prevents matching STS/schema fields from overriding simulator identity or substituting for actual live-source evidence.

### New STS-only route and energy controls

Commit `8a9d741` adds the explicit opt-in delivery policy. It rejects the wide direct option whose Dubins pricing did not match pursuit steering, forecasts spatial delivery with both height and speed, honors the same speed/q/load bank boundaries as live commands, prices compact HACs at their predicted speed instead of a guessed future-speed floor, and derives the maximum-range pressure endpoint from the table's best L/D and banked load. The versioned policy owns the configured STS incidence envelope instead of silently inheriting the STS-N 15-degree terminal cap; explicit overrides remain supported. Runway reconsideration requires a forecast-deliverable alternative, with roll-capture cost and hysteresis, before the energy/capture window closes. None of this certifies a candidate merely because it is the least infeasible option.

The original broad changes regressed the archived L5 and L7 tests. They were isolated rather than changing those tests' requirements; the full suite passed again. A missing `<string.h>` declaration and a null model dereference in a pure-geometry fixture were found and repaired. Fixed-pattern locking and an alternate scoring preference were tested, failed to improve acceptance, and reverted. The proposed mid-drag bank penalty explanation was rejected: its measured penalty was zero in the inspected candidates.

### Confirmed runway-switch initialization bug

Live `20261015T070522Z-sts-polar90-live-delivery-v1` started from the earlier 72 km save at `6533a98`. Early measured/model lift agreed within about 1%, with filtered drag about 6% high. At UT 26015.1, the scorer chose RW09 with score 484 versus RW27 15955. In that frame the vehicle was +33.767 km along, -5.006 km cross, 23.459 km above runway, 768.7 m/s, with heading error +105.8 degrees.

The scorer called `tel_plan` unconditionally. Live initialization instead waited for `tel_plan_open`; its cross-track-only bootstrap gate was already closed, leaving `loop=false`, a wrong-bank direct pursuit, and a 178.918 km displayed path instead of the scored HAC. Resetting near the centreline also disabled the spatial forecast despite the large heading error. Commit `fa13783` makes every new STS policy instance plan on its first tick and always use spatial forecasting, including near-centreline runway switches. A regression reproduces the exact recorded switched-frame state.

The invalid v1 diagnostic was stopped through the owning mission's abort handler (exit 130), and both processes exited before rerun. KSP remained paused, airborne at 11.781 km; it was not restarted. Its automatic 45 km checkpoint is `ksl-20261015T070522Z-sts-polar90-live-delivery-v1-descent-45km` (UT 25828.09).

### Confirmed duplicate flare reference shaping

With initialization fixed, the old-inlet replay reached the runway but touched down at 14.768 m/s sink. `FINAL_TRACE` showed the generic jerk limiter overshooting a relaxing pull, followed by a late arrest: at wheel height 0.9 m, Final's own rate-bounded request was 27.1 degrees while the delivered reference was 15.6 degrees. The resulting balloon and re-descent consumed speed and height.

Commit `263f309` delivers the STS program's already rate-bounded incidence reference once; it retains the shared bank handling and incidence bounds. Legacy/general behavior is unchanged. It also repairs the offline verification parser: the new timestamped runway commitment line had been missed, so a valid RW27 rollout was checked against RW09 and falsely reported 180-degree error. A focused parser regression covers both log formats; no flight gate was relaxed.

### Closed-loop acceptance and next live attempt

`mm304-mm305-polar90-single-flare-verified-20261015T072711Z-kaxpk9hg` passes all existing closed-loop gates from the recorded failed flight's first TAEM inlet:

- Selected RW27; simulator RW09-frame touchdown along 925.601 m (about 1574.4 m past RW27 threshold), cross 0.813 m.
- Speed 56.739 m/s, sink 2.251 m/s; gear down, no bounce, no tail strike.
- Stopped at RW09-frame along 208.509 m, about 209 m remaining in the RW27 direction; rollout heading error 0.0409 degrees.
- This remains one nominal model replay, not live or robustness proof. Touchdown remains long and stopping reserve is small.

Latest full check before live v2: `/tmp/ksl-sts-live-v2-full-test.log`: **221 passed, 56 subtests passed, two pre-existing asynchronous test-cleanup warnings**, 63.82 seconds. Earlier test failures and rejected flight replays remain recorded; they are not successful validation.

Live v2 is `runs/20261015T072920Z-sts-polar90-live-delivery-v2`, executing `263f309` from the saved 45 km checkpoint with `Configuration/sts-live-delivery.json`, source-verified model, and adaptive flare disabled. Console `/tmp/ksl-live-delivery-v2.log`; native stderr in the run directory includes force diagnostics and Final trace. At this note it is in MM304; actual landing outcome is still pending. KSP was not restarted.

### Live v2 failure, approach-speed recovery, and ground-clearance correction

Live v2 did not land. It kept the RW27 pattern after the initialization fix, but the Final-line maximum-range pressure schedule let speed fall below the configured minimum before lateral capture. Bank protection then removed turning authority. The vessel struck the ground and was destroyed before operator-abort cleanup completed. KSP was paused afterward; no restart was performed. Its first TAEM inlet is preserved in `work/taem-red-path-20261015/mm305-live-v2.ini`.

`64998c0` preserves the existing configured final-approach speed on the Final line in both forecasting and live vertical control. It does not prescribe a new speed. The exact v2 inlet replay subsequently passed, while the older inlet contacted 37.432 m before RW27. A proposed minimum of radar/runway-plane clearance initially appeared ineffective. Investigation exposed a simulator data bug: contact used the planar deck, but zero-slope `vertical_m` used spherical altitude. At the old inlet's contact, Final reported about 5.4 m wheel clearance. Nonzero slope already used planar clearance, explaining why its existing collision test missed this case.

`d485677` reports radar clearance using the same finite ground geometry as collision, including zero slope. The opt-in STS policy uses the lower runway-plane/radar clearance before pavement, preserving runway-plane reference above lower terrain. No empirical height or aim offset was introduced. Existing collision coverage was extended at the far threshold. Properly rebuilt closed-loop checks:

- `mm304-mm305-old-inlet-ground-clearance-20261015T075949Z-d13cr5_g`: all gates pass; RW27 touchdown 187.812 m past threshold (RW09 along 2312.188), 55.063 m/s, 1.694 m/s sink, cross 2.619 m; no bounce/tail strike; stopped with 1616.740 m remaining.
- `mm304-mm305-v2-ground-clearance-20261015T075938Z-piw27sp5`: all gates pass; RW27 touchdown 836.810 m past threshold, 55.634 m/s, 1.649 m/s sink; no bounce/tail strike; stopped with 961.178 m remaining.
- These are nominal model replays, not a live landing or robustness proof.

Verification pitfall: `make test` can rebuild native test objects without relinking the flight backend; `ksl run` does not build it. Explicitly run `make backend` (or the isolated build's default target) before flight comparisons. The corrected relinked radar-only experiment still failed 37 m short before simulator telemetry was repaired; its failure is preserved in `/tmp/ksl-sts-old-clearance-relinked.log`.

Full test run `/tmp/ksl-sts-ground-full-test.log` failed 9 fake-controller cases because a real live controller owned the endpoint lock: `Another KSL controller owns this kRPC endpoint; watch or stop its run first`. Production ownership was not bypassed. `e29f477` isolates the offline tests' ownership directory, retaining real flock behavior. Rerun `/tmp/ksl-sts-isolated-pytest.log`: **221 passed, 56 subtests passed, two existing asynchronous cleanup warnings**, 60.32 s. Simulator CTest: **4/4 passed**. Native touchdown quality still passed.

### Live v3 and selected-law reference telemetry

Live v3 is `runs/20261015T075551Z-sts-polar90-live-delivery-v3`, parent PID 19043, from the saved v1 45 km checkpoint, explicit delivery configuration and live-only aero profile. Console `/tmp/ksl-live-delivery-v3.log`. Backend SHA256 at launch: `d5e6397500bcc386179208d11bae83e95c6a4bb2f0b2de66667d8898a41171d6`; this executable already contains the approach-speed and pending STS radar-minimum branch. The source was briefly restored during comparison, but the running executable was not modified. The simulator correction cannot affect KSP. At this note UT 26255, v3 is airborne around 6.1 km, 87 m/s, approaching the RW27 HAC; final result remains pending.

MM304 and TAEM browser screenshots are in `work/live-delivery-v3-20261015/`. Burn/command/progress fixes remain intact. The TAEM panel correctly showed RW27, overhead-loop/downwind, drag level, path-to-go and required path, but no geometric reference was published by the energy law. `fd5e778` samples the existing selected-law spatial continuation at its existing pricing cadence, preserves only fresh references, passes referenceTrajectory through snapshot_summary and the web overlay, and labels it LAW REF. Shadow planning does not publish these samples. It is a model reference, not a certified touchdown prediction. The v2-inlet replay with display sampling (`mm304-mm305-v2-reference-display-20261015T080512Z-qv7g3ohv`) produces identical landing measurements to its predecessor; 89 stored snapshots contain sampled reference points. Targeted geometry and telemetry round-trip checks pass. A later live run must verify this new path display.
