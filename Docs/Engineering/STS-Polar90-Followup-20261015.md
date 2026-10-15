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
