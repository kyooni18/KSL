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
