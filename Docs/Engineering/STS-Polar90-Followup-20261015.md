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

### v3 outcome correction and configured-station experiment

The original v3 was unnecessarily aborted at UT 26348.39, 2151.9 m MSL, 112.1 m/s: `FINAL_TRACE` lines at future UT 26386 belonged to a shadow forecast, not the live aircraft. The aircraft was intact (26 parts) and safely paused at 2141 m. This was an investigation mistake, not a flown early flare. `ac24f1f` suppresses live Final/Gate trace labels for diagnostic shadows. An intact aligned checkpoint was saved as `ksl-sts-polar90-v3-aligned-2141m`.

The subsequent actual live approach, `runs/20261015T081020Z-sts-polar90-live-delivery-v3-final-resume`, admitted Final, contacted about 241 m short of RW27 at 62.5 m/s with 3.8 m/s sink, bounced and lost parts. Parent 45181 was then interrupted and exited 130 through safe controls; KSP paused with throttle zero. This is a failed live landing. Browser `work/live-delivery-v3-20261015/final-resume.png` confirms Final's actual status and commands were displayed instead of the TAEM candidate state.

Two Final-only experiments were rejected and their source changes reverted: adding a touchdown-distance forecast to activate low-energy mode had no effect (that mode was already active); updating the aim during the early pull latch moved touchdown onto pavement but caused 36.13 m/s, 7.834 m/s sink and a tail strike. See `/tmp/ksl-sts-final-v3-{range-diag,aim-hold}.log`. Do not restore either as a validated fix.

A separate legacy policy in `final_alignment_distance_for_state` extended the configured 8 km station to 9 km for high/fast inlets, based on an older replay frontier. `560ed1e` makes only the opt-in STS delivery law retain its configured station (no new physical distance constant); general and archived guidance retains the extension. This moves the HAC closer and preserves approach range. Native admission coverage checks the configured station; full `/tmp/ksl-sts-configured-station-tests.log`: **221 passed, 56 subtests passed, two existing cleanup warnings**, 66.23 s.

Closed-loop nominal comparisons with the configured station:

- v3 recorded TAEM inlet: baseline extended-station choice selects RW09 but touches 327.930 m short; forced RW09 produces the same result. With the configured station, `mm304-mm305-v3-configured-station-20261015T081920Z-tsgj4_3g` selects RW09, touches 246.651 m past threshold at 57.552 m/s and 1.756 m/s sink, no bounce/tail strike, stops at along 973.478 m (1526.522 m remaining); all gates pass.
- v2 inlet: `mm304-mm305-v2-configured-station-20261015T081959Z-6715krhu`, RW27, touchdown 600.936 m past threshold, 56.932 m/s, 1.597 m/s sink, no bounce/tail strike, stop with 1172.400 m remaining; all gates pass.
- Older inlet `/tmp/ksl-sts-old-configured-station.log`: RW09, touchdown 157.115 m past threshold, 56.464 m/s, 1.777 m/s sink, no bounce/tail strike, stop at 866.439 m (1633.561 m remaining); all gates pass.

These support feasibility of a closer RW09 pattern resembling the user's drawn alternative under the nominal model; they do not prove the exact red curve or live reliability. Live v4 starts from the intact 45 km checkpoint: `runs/20261015T082139Z-sts-polar90-live-delivery-v4`, console `/tmp/ksl-live-delivery-v4.log`, execution session 94501. It uses explicit delivery configuration, `560ed1e`, latest reference telemetry, and requests an automatic MM305 phase save. At this note it is in MM304; final outcome remains pending. No KSP restart.

Route comparison figure: [PNG](Assets/STS-Polar90-Route-Comparison-20261015.png), [SVG](Assets/STS-Polar90-Route-Comparison-20261015.svg), [sampled coordinates](Assets/STS-Polar90-Route-Comparison-20261015.csv), [source hashes and limitations](Assets/STS-Polar90-Route-Comparison-20261015.sources.json). The blue series is the recorded v3 RW27 route stopped before Final. Dashed gold and solid red are ideal-servo model replays from the same kinematic inlet, with the extended and configured stations respectively. They are not two measured live flights. The nominal inlet mass differs by 17.7 kg from the most recent logged mass, and live force calibration/attitude response differs; no statistical confidence interval is claimed. The configured-station RW09 curve resembles the user's qualitative red alternative; it does not reproduce an exact digitized drawing.

### Live v4 failure and pavement-intercept parity

V4 entered MM305 at UT 26004.15 and selected RW27 (`score09=12242`, `score27=1938`), rather than the RW09 nominal counterfactual. The live calibrated inlet did not support forcing RW09. It saved `ksl-20261015T082139Z-sts-polar90-live-delivery-v4-mm305`. Browser screenshots in `work/live-delivery-v4-20261015/` verify LAW REF with 23 sampled points, actual command bars and energy decisions. The map's terminal autofit was retaining the first two distant entry points regardless of the horizon; `9fe2c14` removes that exception for terminal actual history. `cfabf98` clears inactive HAC metrics in Final; `fce58ea` relabels the energy law's `s_req` as RANGE SCORE, because `tel_solve_k` publishes an available spatial-progress/energy-delivery score there, not a literal required path (negative values are valid penalties).

The configured station improved v4 to a 5 km / 142 m/s HAC, but it still delayed Final admission. At roughly 8 km / 114 m cross offset, the TAEM line law's exponential centreline closure pointed beyond the far pavement edge. The unchanged Final gate correctly rejected this projected track. Only after spending altitude and speed on centreline convergence did it admit Final. Actual contact occurred about **88 m short of RW27**, ~61 m/s with ~2.2 m/s sink; the craft was destroyed. SIGINT parent 63425 reaped backend 63529 with safe-control exit 130; KSP paused, no restart. The uninterrupted v4 is a failed landing, not an offline success.

`14383d2` makes the opt-in STS TAEM line controller and Final controller target a runway-midpoint intercept when **both** projected runway endpoints fit the configured physical width. Larger offsets retain the authority-aware centreline acquisition. The Final admission gate is unchanged. The model continuation uses the same intercept bearing. This avoids commanding a convergence rate that the next phase rejects, without an empirical angle/distance tolerance. Existing Final parity coverage now checks the recorded 114 m / 8 km neighborhood, the unchanged endpoint gate, an offset too large for this intercept, and legacy-policy exclusion.

Nominal latest matching-forecast checks:

- v4 inlet `mm304-mm305-v4-intercept-consistent-20261015T083709Z-hqbr0m2w` (exact ID and artifacts in `/tmp/ksl-sts-v4-intercept-consistent.log`): touchdown RW27 826.890 m past threshold, 56.429 m/s, 1.641 m/s sink, cross 17.020 m, no bounce/tail strike; stopped with 955.301 m remaining; all gates pass.
- v3 RW09 inlet (`/tmp/ksl-sts-v3-intercept-consistent.log`): touchdown 12.888 m past threshold, 54.395 m/s, 1.504 m/s sink, cross 10.978 m, no bounce/tail strike; stopped at along 688.177 m. All gates pass, but threshold margin is only 13 m; do not describe this as robust.
- Full `/tmp/ksl-sts-pavement-intercept-full-test.log`: **221 passed, 56 subtests passed, two known asynchronous cleanup warnings**, 69.14 s. After the subsequent forecast-bearing parity addition, targeted native geometry and Final-parity checks passed in `/tmp/ksl-sts-intercept-final-native.log`. Root backend rebuilt before live.

Live v5 is `runs/20261015T083842Z-sts-polar90-live-delivery-v5`, console `/tmp/ksl-live-delivery-v5.log`, execution session 63617. It resumes the intact v4 MM305 checkpoint with explicit delivery configuration and `14383d2`; no old live controller remains. At UT 26046 it is still airborne on RW27's downwind leg, 17.4 km / 515 m/s. Actual Final handoff and landing outcome remain pending.

### V5 intact landing but false airborne recovery at contact

V5 reached Final earlier, at over 2 km altitude and over 6 km before RW27, with a pavement-valid ~110 m intercept offset. It transitioned toward rollout around 22–27 m before threshold; first coarse `landed` telemetry was 3.35 m inside the threshold. It then stopped intact: all **26 parts**, mass 25222.4 kg, speed ~0.6 m/s, RW27 along about 179.3 m / 2320.7 m remaining, cross ~33.1 m. Surface-frame stopped attitude was pitch -5.66°, roll +10.87°, heading 274.96°. This is marginal early contact and a tilted edge rollout, not a clean centered touchdown. A body-reference-frame pitch of -79° was an inappropriate attitude frame and is not the local stopped pitch. The intact paused vessel is saved as `ksl-sts-polar90-v5-intact-landing`.

Mission status was nevertheless FAILED: `Control departure: insufficient height remains to complete attitude recovery (radar 3 m, runway along 42 m, cross 26 m, phase Touchdown).` Confirmed wheel contact was already available to the FCS, but guidance's delayed contact acquisition allowed its airborne recovery monitor to interpret first-contact strut/terrain roll rates as a flight departure. The warning wrapper retained braking and terminated failed once stopped. V5 exited naturally with code 1; no controller remained.

`b3412ba` gives confirmed main-wheel contact or coarse landed state rollout ownership before the airborne monitor, **only** for the opt-in STS policy after Final capture. It preserves the existing contact envelope and outside-runway failure. A recorded-contact regression proves the 19.4° / 30°/s roll transient at radar 3.3 m cannot request airborne recovery once main contact is confirmed, and that the stopped valid runway state completes without a warning. It also centralizes pavement intercept geometry: prefer the configured touchdown point when both runway-end projections fit, otherwise use the safe midpoint, otherwise retain centerline acquisition. Forecast and control share that geometry. Heading-reference telemetry follows the actual lateral-rate target instead of an unrelated inactive heading cut.

Full `/tmp/ksl-sts-contact-ownership-full-test.log`: **221 passed, 56 subtests passed, two known cleanup warnings**, 66.64 s. A focused native check after adding confirmed-contact pavement validation also passed. Nominal v4 replay with touchdown-point preference still passes (RW27 touchdown 826.942 m past threshold, cross 17.001 m, 56.411 m/s, 1.647 m/s sink, no bounce/tail strike, 955.454 m remaining at stop). The v3 RW09 replay passes at only 6.151 m past threshold, cross 1.466 m; this threshold margin remains inadequate as robustness evidence.

`d067db7` separately latches established nose contact in STS rollout so a momentary nose-wheel flag change cannot regress Rollout→Touchdown or release braking. The existing recorded-contact test now exercises a nose-contact gap and retains Rollout. This focused native check passes; the change is for the next rebuilt live backend, not the already-running v6.

Live v6: `runs/20261015T085709Z-sts-polar90-live-delivery-v6`, console `/tmp/ksl-live-delivery-v6.log`, execution session 75107. It resumes the intact v4 TAEM checkpoint on `b3412ba`. At UT 26123 it is still in TAEM at 9.5 km / 120 m/s. Outcome remains pending. A subsequent uninterrupted entry/orbit test is still required; resumed TAEM completion alone is not proof of a reliable polar return.

### V6 failure, V7 completion, and remaining lateral error

V6 failed and lost five parts (26→21). Before threshold its centre of mass fell below the configured runway deck, followed by a violent roll transient. Contact ownership alone did not cure the deficient approach range. V7 used `3a5fe06`: the existing touchdown-speed integration now also reports ground range; reversible range shortage removes the excess station-speed demand while preserving the configured approach-speed reserve. Both recorded inlet replays passed. The new range-short condition does not latch the general degraded-energy state.

V7 (`runs/20261015T091755Z-sts-polar90-live-delivery-v7`) completed and stopped intact with all 26 parts. Independent kRPC measured 0.151 m/s at UT 26365.97, paused, all three wheels grounded. However, its approximately 34 m stopped cross error, +9.38° roll and 280.19° heading are an edge landing, not reliable runway alignment. Final reduced cross error from 113 m to about 27 m at threshold. Commands around -0.7° bank produced nearly level actual bank and roll inputs around -0.002. That tracking deficit remains under investigation.

`dcd7421` publishes the selected runway's exact reciprocal threshold, heading, dimensions, and configured Final distance/slope through `guidanceState.activeRunway`. The live overlay and replay normalization use it. The map no longer draws an invented hardcoded 8 km/20° funnel; it draws the published Final reference. An encountered full-suite TUI teardown race failed with `NoMatches` on queued selection events after controls unmounted. `c89788f` guards those lifecycle events; focused tests and the full Python rerun passed (221 tests, 56 subtests, two existing asynchronous cleanup warnings).

`9a50e15` corrects another lateral issue: when the touchdown aim's straight track crossed the far runway edge, choosing the runway midpoint unnecessarily retained threshold offset. The shared STS intercept now intersects both endpoint track-slope bounds and chooses the feasible slope closest to the configured touchdown aim. No physical gains or dimensions were added. The existing Final regression covers mirrored offsets and both pavement ends. Its first assertion mistakenly expected under 6 m threshold offset for the 30 m offset / 500 m range fixture; the analytic feasible bound is 19.167 m, and the assertion was corrected to under 20 m.

Recorded v4 RW27 replay: touchdown 914.869 m past threshold, cross 13.181 m (previously 18.275 m), 55.474 m/s, sink 1.716 m/s; no bounce/tail strike/departure, stopped with 885.884 m remaining. Recorded v3 RW09 replay remains touchdown 64.047 m past threshold, cross 0.873 m, 59.32 m/s, sink 2.19 m/s, no bounce or departure. These are nominal model results, not live proof.

V8 began from the intact v4 MM305 save on `9a50e15`, explicit `Configuration/sts-live-delivery.json`, console `/tmp/ksl-live-delivery-v8.log`. Root backend rebuilt before launch. Web observer restarted between runs and the browser shows live RW27 MM305 energy-law decisions and LAW REF. Outcome pending. No KSP restart occurred.

### V8/V9 failures and flare response gap

V8 did not land successfully. It was destroyed at RW27 along approximately -34.84 m, cross +15.49 m, speed 60.50 m/s and sink 1.444 m/s. Console `/tmp/ksl-live-delivery-v8.log`; native trace in `runs/20261015T093619Z-sts-polar90-live-delivery-v8/backend.stderr.log`. Mission continued briefly on ghost last-vessel telemetry; operator SIGINT ended it (exit 130). The new geometry improved lateral offset but did not cure early contact.

`087e8c0` adds STS-only bank trim using the existing roll-loop bandwidth/damping/lag; legacy behavior and confirmed contact clearing are covered in the existing native tests. Full offline tests passed (221 Python tests, 56 subtests, two existing cleanup warnings). V9 resumed the intact v8 Final checkpoint with cold guidance/learning state. It also fell below the runway plane and was destroyed; operator SIGINT was too late to preserve the vessel. Paused independent kRPC state: UT 26364.25, zero parts, altitude 64.10 m. Do not describe its Rollout phase as an intact landing.

A best-glide reserve experiment using max(touchdown speed, minimum safe speed, learned 1-g speed), instead of configured approach speed, was rejected and fully reverted: v3 replay touched down at 47.406 m/s with 4.248 m/s sink. Its v4 nominal pass does not justify the regression.

User correctly prioritised insufficient flare command. V9's trace shows a response inconsistency: the latch reserves about 3 s pitch response but the post-latch arc solves with about 1 s residual lag. At wheel height 178 m, sink 25.2 m/s, the command requested zero vertical acceleration; near 102 m it requested 5.02 m/s² and near 38 m 9.55 m/s². The demand grew only after much of the physical arrest margin was gone. At wheel height 1.1 m, still 427 m before threshold, sink remained 4.9 m/s. Applied elevator and measured incidence must be considered as well; this is not proof that all error resides in the outer law.

Measured cause in the controller: v9 `controlDiagnostics.pitchTrim` sat on the fixed Approach/Flare cap 0.420 from ~105 m altitude to contact while measured AoA lagged target by 2.5-4 deg; achieved sink arrest was ~2.2 m/s^2 against 5-11 m/s^2 requested. `d0d0a51` lets the experimental STS Final pitch trim span full elevator travel (existing total-demand anti-windup is the only limit); other programs keep the cap. v9 was also energy-short from Final entry: forecast touchdown 500-1100 m short from 1.9 km, achieved L/D ~2.5 vs learned ~3.2; no flare change could have reached the threshold from that state.

A response-aware minimum arrest acceleration candidate (preflare, full ~3 s response reserve after latch) was REJECTED: with `--direct-control` both recorded inlets crashed into the runway at ~36 m/s, 18-19 m/s sink (v3 1150 m, v4 281 m). Trim-only direct-control replays: v4 682 m, 2.83 m/s sink, 49.9 m/s (baseline 683 m, 2.92, 50.1); v3 21 m, 1.34 m/s, 53.7 m/s. Patch kept at `work/flare-trim-20261015/rejected-response-arrest.patch`. Full `make test`: 221 Python + 56 subtests passed.

V10 (`runs/20261015T100028Z-sts-polar90-live-delivery-v10-trim`, `/tmp/ksl-live-delivery-v10-trim.log`, from v4 MM305 save, `d0d0a51`): COMPLETE, intact (25.2 t, landed, stopped 0.13 m/s). Main contact about RW27 +331 m, cross -1.8 m, sink 0.3 m/s, but at 47 m/s against the 75 m/s touchdown target. Trim reached 0.546 (exercised beyond the old cap). The sink was arrested at ~6 m wheel height (UT 26353), and the vehicle then floated ~11 s from 77 to 47 m/s. One live success: it is not reliability proof. Next issue is the post-arrest float and speed bleed, not insufficient pull. Final-approach checkpoint: `ksl-20261015T100028Z-sts-polar90-live-delivery-v10-trim-final-approach`.

V11 (`runs/*-sts-polar90-live-delivery-v11-gentle`, `/tmp/ksl-live-delivery-v11-gentle.log`, v4 MM305 save, `6aee4c8`): COMPLETE, intact (25.2 t, stopped 0.31 m/s). The gentle STS arc removed the late step: sink fell steadily from 33 m/s at ~290 m to ~2 m/s at ~78 m altitude, AoA tracked within ~1-1.5 deg without the v10 oscillation, trim peaked ~0.49. It still levelled ~5 m above the runway and floated ~10 s (77 to 48.5 m/s); main contact about RW27 +390 m, cross -1.7 m. The alignment change did not visibly speed lateral capture: cross 24 m at -1800 m closed at ~1.1 m/s with near-zero bank, reaching ~1 m only after the threshold. The capture rate is bounded by the position-loop limit `omega_limit=.35/(2+reversal_time)`, not by the pavement intercept. Replays with the change: v3 165 m, 1.42 m/s, 58.8 m/s, cross -0.08 m; v4 684 m, 2.74 m/s, 49.8 m/s.

V12 (`runs/*-sts-polar90-live-delivery-v12-lateral`, `/tmp/ksl-live-delivery-v12-lateral.log`, v4 MM305 save, `0277b82`): COMPLETE, intact (25.2 t, stopped 0.29 m/s). Final lateral capture with need-sized reversal: cross 31.6 m at -5.8 km, 6.1 m at -4.0 km, 2.7 m at -3.4 km, roll within about +/-2.5 deg; touchdown cross 0.0 m, RW27 +507 m. Touchdown speed 40.8 m/s (sink 0.38 m/s): the post-arrest float/speed bleed is worse than v11 and remains the open issue.

Body flap: since `80fce88` (2026-09-27) the kRPC client latches SEPARATED surface allocation at TAEM ownership for the rest of the flight. The centre body flap is removed from every axis and pitch uses the inboard elevons only, including in Final and flare. That is consistent with the elevator-only trim saturation seen in v9; not yet changed.

V13 (`runs/*-sts-polar90-live-delivery-v13-bodyflap`, v4 MM305 save, `ee34f61`): COMPLETE, intact (25.2 t, stopped 0.18 m/s). The backend log confirms the switch to "separated + body flap pitch" in Final/flare and back afterwards. Pitch trim settled ~0.21 (v10-v12: 0.45-0.55) and AoA tracked within ~0.5 deg. Cross 0.0 m from -600 m to touchdown. This approach arrived high: threshold crossed at ~128 m altitude (~60 m above the runway), the arrest levelled ~13 m above it, and the float lasted ~11 s; main contact RW27 +1025 m at 49.8 m/s. The float/speed bleed is a guidance issue (shallow glide/float law), not pitch authority.
