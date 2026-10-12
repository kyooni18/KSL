# Polaris diagnosis — STS polar reentry overflight (2026-10-12)

## Scope and source evidence

This is an offline investigation of the fourth live 100 km polar-orbit shuttle mission, not a new KSP flight or a claim of successful runway recovery. The orbiter completed ascent, 100.09 x 99.94 km parking, and a 42.14 m/s commanded deorbit, then passed KSC at excessive speed/height without a viable MM305 Final route.

- Mission: `runs/20261012T124857Z-full-fourth-oms-resumed-20261012/`.
- Deorbit plan: `deorbit-plan.json`; live: `telemetry.jsonl`, `events.jsonl`, `backend.stderr.log`.
- Full-fidelity live vehicle state/attitude/thrust: `FlightLogs/2026-10-12T12-51-12Z-STS-vehicle.jsonl` (keyframes and deltas).
- Configuration: `Configuration/sts-live.json`, runway RW09, target Pe 50 km, MM305 Mach 2.5 +/-0.35, TAEM max radius 96.5 km, HAC 12 km, Final 8 km.
- KSP vessel and game state were not changed for this investigation. A stale `running` run-status record does not establish a live flight-control process; inspect processes separately before future live work.

## 1. The selected trajectory was already an overflight in the planner

The saved deorbit plan's executable MM304 shadow reaches its nominal `TAEM` at **27.03 km, 872 m/s, only 0.485 km from KSC** with a heading approximately 173 degrees (RW09 is 90 degrees). This is not a usable high-energy HAC inlet; there is no distance remaining for a physically constrained 90-degree turn, alignment, and glide. The planner independently recorded `nominalCaptureAchieved=false`, `targetCaptureAchieved=false`, and **0/15** robustness/recovery cases, but marked `executionQualified=true` because its orbital-stage acceptance intentionally certifies only a safe burn and entry, not a landing. Its `predictedTAEMRangeError` was **-96.0 km** against the 96.5 km TAEM reference.

The live run also overflew KSC, and therefore **prediction error alone cannot explain the entire miss**:

| Descending altitude | Saved shadow | Actual live |
| --- | --- | --- |
| 50 km | ~2,192 m/s, latitude ~39.38 deg N, UT ~146957915 | ~2,179 m/s, latitude ~32.93 deg N, UT ~146958077; range ~346 km |
| 35 km | ~1,647 m/s, latitude ~8.71 deg N | ~1,606 m/s, KSC range ~57.6 km |
| 29 km | ~1,126 m/s, latitude ~1.68 deg N | ~1,104 m/s, KSC closest distance ~4.7 km |
| 27 km | ~872 m/s, range ~0.5 km | ~904 m/s, range ~14.5 km and increasing |
| 26.6 km (actual MM305 transfer) | Already ended near KSC | ~870 m/s, range ~16.3 km, no qualified committed route |
| 17 km | No further predicted route | ~324 m/s, KSC range ~58 km, still moving away |

Live MM305 transfer was triggered by crossing the upper Mach/altitude ownership boundaries. The backend explicitly reported `routeQualified=0 routeCommitted=0`; transfer of command ownership **does not guarantee** a feasible runway route. Its subsequent maximum bank/heading control could not recover geometry after the overflight. S-turn and control gains should not be tuned as though this were an otherwise viable 96 km TAEM arrival.

## 2. The shadow erroneously labeled forced transfer as geometric capture

`CLanding/guidance/guidance_terminal.c` supports safety-driven forced MM304-to-MM305 control ownership. In `CLanding/prediction/entry_planning.inc`, however, the executable shadow previously copied `shadow.taem_interface_captured=true` and hardcoded `taem_interface_error=0.0` for such a transfer. The `deorbit_runway_capture_qualified()` predicate then compared the 0.5 km TAEM *range error* of almost -96 km against the **100 km orbital capture tolerance**. A 96 km miss therefore passed that numeric test, even though HAC+Final has no remaining trajectory length.

**Correction applied:** at forced ownership boundaries the witness rechecks the actual measured-state MM305 inlet envelope; forced and nonadmissible transfers are labeled `taem_ownership_boundary_missed` and cannot count as strict runway capture. The interface error now reports a computed positional residual instead of a literal zero. Runway qualification also requires physical TAEM range within the geometric reserve for one HAC and Final (20 km for the present vehicle/mission) and the configured 96.5 km maximum. The orbit-stage `executionQualified` contract is **intentionally unchanged**: it still means burn+entry, not guaranteed landing.

Added a direct regression in `Validation/DeorbitStageAdmissionTests.c`: the fourth flight's nominal false-positive 485 m forced handoff is rejected; a non-overflown 70 km inlet is still admissible to this *geometric* screen when other synthetic fields qualify; a missed forced boundary is rejected. This does **not** substitute for validated terminal route replay.

## 3. The modeled finite burn does not follow the executable throttle law

The saved plan estimated a 118.25 s deorbit burn, from ~UT 146957021.8 to 146957140.0. Live `Deorbit Burn` lasted ~214.7 s, and significant thrust first appeared around UT 146957055.7 (about 34 s after the estimated start) because of alignment/acquisition. The commanded throttle then remained near its 0.18 maximum until ~UT 146957153, tapered for more than 90 s, and the phase ended ~UT 146957245.5. The plan did not price the same slow periapsis governor or late ignition.

Direct Kepler periapsis reconstruction from the recorded inertial position/velocity (mu 3.5316e12 m^3/s^2, Kerbin radius 600 km) confirms the mechanism:

| UT | Observed periapsis | Commanded throttle |
| --- | --- | --- |
| 146957140 | 63.63 km | 0.180 |
| 146957150 | 59.19 km | 0.180 |
| 146957155 | 57.09 km | 0.155 |
| 146957170 | 53.25 km | 0.086 |
| 146957190 | 51.25 km | 0.029 |
| 146957210 | 50.58 km | 0.0096 |
| 146957230 | 50.35 km | 0.0033 |
| 146957245 | 50.29 km | 0.0014 |

At full permitted throttle the actual periapsis rate was about 440 m per second. The previous periapsis taper was `clamp((Pe - Pe_target - 250m) / 8000m, 0, 1)`. Its local first-order time constant is ~8000/440 = 18 s, producing approximately 90 s of slow tail down to the existing 300 m cutoff, as observed. **Correction applied:** shrink the proportional taper window to 1.5 km, retaining the 300 m physical cutoff, maximum throttle, existing burn ramp and measured thrust watchdog. At the same measured full-power response the approximate terminal time constant is ~3.4 s; this is a retrospective engineering calculation, **not a verified live burn duration**. Regression asserts full commanded throttle still available at 2 km Pe error and bounded taper at 700 m.

This correction should substantially reduce the time mismatch, but it does **not** fix an otherwise impossible TAEM ground-track geometry, and the predictor still lacks an exact account of alignment-delay and measured periapsis throttle dynamics. Reconcile the finite-burn predictor with the executable controller in a separate deterministic replay before claiming trajectory prediction parity.

## 4. Recorded-state native energy/route screen

To avoid guessing whether the measured inlet would be recoverable, replayed **four actual fourth-flight keyframe/delta states** against the repository's native STS aerodynamic and path-energy code. A 28 KB frozen fixture with source-log SHA-256 is preserved in `Validation/PolarOverflightEvidence/20261012-FourthFlight-MM304.json`.

| UT / descending altitude | Mach | Native energy margin | MM305 inlet admission |
| --- | --- | --- | --- |
| 146958365 / 31.0 km | 4.09 | -520 kJ/kg | rejected (Mach, altitude, energy) |
| 146958412 / 26.7 km | 2.85 | -280 kJ/kg | rejected (energy veto 16) |
| 146958420 / 26.1 km | 2.70 | -257 kJ/kg | rejected (energy veto 16) |
| 146958455 / 22.0 km | 2.54 | -230 kJ/kg | rejected (energy veto 16; terminal reference unreachable) |

The model's `reference_reachable` flag was still **true** at the 26.7 km inlet: the available specific energy (about 582 kJ/kg) barely covered nominal drag work (about 578 kJ/kg), but **could not cover the roughly 284 kJ/kg model/physics uncertainty reserve**, resulting in -280 kJ/kg certified margin. This is evidence of an *unqualified, fragile route*, not proof that every idealized trajectory is mathematically impossible. At 22 km the nominal reference itself was no longer reachable. These readings agree with the lack of any qualified MM305 route at the live handoff. The root failure is earlier than MM305 gain tuning: the selected deorbit/entry energy and geometry left no certified arrival margin.

## 5. Remaining required engineering work

1. From the captured orbital state and calibrated plant, re-solve deorbit opportunities against the **actual MM305 inlet envelope** and bounded, unpowered Final route, not just the 70 km entry-interface geometry. Do not make TAEM `nominalCaptureAchieved` equivalent to an ownership boundary or KSC proximity.
2. Propagate live finite-burn execution delay and the new terminal governor through timing forecasts, then compare both ground-track and altitude-time errors at 70/50/30/27 km. The original shadow was already an overflight; model calibration alone cannot repair that target selection.
3. Compare MM304 drag-reference forecasts and achieved S-turn bank/AoA using recorded-state fixtures in `Tools/prepare_mm304_energy_replay.py` + `Tools/mm304_energy_replay.c`. Check whether any higher-drag or earlier lateral reversal is physically available at the measured Mach/q/control authority before modifying guidance gains.
4. Require an actual bounded MM305 route+preflare feasible inlet for a **production landing mission** before labeling the flight landing-qualified. If no candidate exists, report unqualified rather than executing a simulated success or weakening the 1 km physical handoff constraint.
5. Validate fixes first through existing C admission tests and recorded-state offline replay. There is **no live landing-success proof** in this audit; a future KSP flight should be separately authorized.

## Follow-up implementation

The immediately following guided deorbit retiming, 60-second retrograde attitude acquisition lead, and strict route-certification/mission gate are documented in [Polar-Deorbit-Guided-Opportunity-20261012.md](Polar-Deorbit-Guided-Opportunity-20261012.md). The frozen fourth-flight replay selects a geometrically improved 71.3 km MM305 inlet while retaining its **negative ~246 kJ/kg energy margin and unqualified 0/15 route status**; no runway-delivery claim is made.

## Reproduction

- C admission regression: `make -C CLanding deorbit-test entry-forecast-admission-test`
- Full regression suite: `make test`
- Recorded inlet audit (offline only): `make -C CLanding build/mm304_energy_replay` then `CLanding/build/mm304_energy_replay Validation/PolarOverflightEvidence/20261012-FourthFlight-MM304.json`.
- Raw source checks: `git diff --check` and review changes in `CLanding/prediction/entry_planning.inc`, `CLanding/guidance/guidance_terminal.c`, `Validation/DeorbitStageAdmissionTests.c`.

All findings above are from the saved live trace and local source at the audit date; no new flight was performed.
