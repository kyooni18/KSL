# STS Polar 90 Live Test — 2026-10-15

**Status: overnight test in progress.** Evidence collected through 03:06 KST; KSP has been restored to the named prelaunch checkpoint and independently verified. No source files were edited by this test session. The checkout already contained six modified C source/header files at test start.

## Test setup

- Profile: `sts-polar`; requested target 90 km circular orbit, 90° inclination.
- Launch state: STS on the pad, 8 clamps, 61 parts, mass 521,003.44 kg. A separate named KSP save was made before launch: `sts-polar90-prelaunch-20261015` (UT 15291.502).
- Checkpoints from the full run:
  - Orbit: `ksl-20261014T165854Z-sts-polar90-full-20261015-orbit`, Ap 89,995.57 m / Pe 89,986.66 m / inclination 90.000202°.
  - Descending 72 km: `...-descent-72km`, measured altitude 71,995.76 m, UT 25348.894.
  - Descending 30 km: `...-descent-30km`, measured altitude 29,992.23 m, UT 25918.514. This was before MM305; the actual MM305 handoff happened near 26.0 km.
- KSP `.sfs` saves for the prelaunch, orbit, 72 km, and 30 km checkpoints were verified in the `shuttletester` save folder.

## Flight result

**Launch and orbit succeeded; return and landing failed.** The full flight reached a 90 × 90 km, 90° orbit, then accepted an explicitly unqualified deorbit candidate under the profile's degraded-test setting. Entry and TAEM did not recover to the runway. The vehicle splashed about 10.08 km from the runway after crossing runway elevation before reaching the fixed-HAC exit; this was not a runway touchdown.

### Ascent and orbit

- Launch azimuth was 355.6° for the 90° target. Booster separation completed near 23.6 km; the orbiter's 26-part inventory remained intact.
- Recorded ascent peaks were 27.88 kPa dynamic pressure at 9.96 km and 3.727 g at 22.05 km. The run completed rather than breaking up.
- MECO left a suborbital orbit. Two OMS trim burns spent 14.12 / 13.86 m/s and 8.01 / 7.83 m/s, with 0.03 m/s residual each; final orbit met the requested target.
- A separate orbit-checkpoint replay without the original CLI overrides defaulted to the profile's 100 km target and proposed two burns totaling about 16.2 m/s. It was stopped before burn execution. Replaying the same save with the 90 km / 90° overrides completed the orbit phase with no burn and retained the target orbit. **Operational finding:** loading a KSP save into a new run does not restore the previous run's CLI overrides; reapply them.

### Deorbit, entry, TAEM

- Deorbit plan: 34.10 m/s at UT 24907.3; strict qualification false, nominal target capture false, predicted TAEM range error −43.6 km, and entry robustness 0/15 passing (4 unsafe cases). The profile allowed the best available unqualified candidate to execute, while retaining downstream gates. The plan predicted 1,180.6 km to entry; the measured range at about 89 km was 1,673.0 km, roughly 492 km farther.
- The 72 km checkpoint was saved while descending at about 2,293 m/s. At 72 km the measured range to site was about 1,249 km. At the 30 km checkpoint it was about 143.8 km, with speed 1,344.9 m/s, sink rate −85.2 m/s, and q 5.18 kPa.
- MM304 warned that its maximum-range drag profile could not reach the required Final alignment path. The flight nevertheless transitioned to MM305 near 26.0 km.
- At the MM305 handoff near 26.04 km, the shuttle was still 88.97 km from the site and about 85.42 km cross-track from the runway. MM305 followed an overhead-loop/HAC profile but descended through runway elevation before the fixed-HAC exit. It then entered Attitude Recovery with insufficient height, did not stabilize within 15 s, and was operator-aborted through the safe-control path after the vessel had splashed. At the first logged splashed sample: UT 26464.254, altitude 0.50 m, speed 14.52 m/s, about 10.08 km from the runway with roughly 2.02 km cross-track error.
- The MM305 descent had already become unrecoverable well above the surface: at 14.98 km, speed was 292.6 m/s with a −157.2 m/s sink rate and −32.5° FPA; at 1.70 km, speed was 86.5 m/s with −25.6 m/s sink and the runway still 14.7 km away; at 501 m, speed was 37.6 m/s with −13.7 m/s sink and 11.2 km remaining. These samples place the miss before the attitude-recovery abort.

## Checkpoint replays

- `--from mm305` from the 30 km save failed before control engagement. Shadow admission reported `entered=true, MM305=true, closest=87.8 km, range error=-8.7 km, q=5.4 kPa, g=1.25`.
- `--from mm304` from the 72 km save also failed before control engagement: `entered=true, MM305=true, closest=45.6 km, range error=-50.9 km, q=7.4 kPa, g=1.73`.
- A distinct `--from mm304` replay from the 30 km save returned the same 30 km shadow rejection as the direct MM305 start. Both starts therefore reject this checkpoint without sending flight controls; the MM305 phase flag in the shadow result is already true while the backend remains Idle.

## Offline validation

- `make test` stopped during native C compilation before running its test targets. The compiler error is `no member named 'jet_assist' in 'GuidanceCommand'` in `CLanding/transport/client/controls.inc:217`. The currently modified `CLanding/include/guidance_types.h` declares `jet_assist_throttle`; those local source changes predated this test. No source edits were made to address it.
- The separate `entry-70km-sts` MM304-to-landing ShuttleSim scenario could not start because its runtime backend build hit the same compile error. Run: `runs/20261014T175611Z-sts-polar90-sim-entry70/`. This is blocked validation, not a trajectory result.
- The Python closed-loop suite ran against the existing `CLanding/build/landing_backend` artifact timestamped 2026-10-14 22:33 KST; the failed native build did not refresh it. Treat the two ShuttleSim landing failures as results from that existing artifact, not a clean build of the currently modified C sources.
- A distinct archived MM304 scenario (`ksp-20261014-mm304-start.ini`) was first refused when paired with the STS profile because the scenario's aerodynamic profile identifies STS-N. After selecting its matching STS-N configuration, it reached TAEM at 26.95 km, 64.70 km from the site and 24.05 km cross-track, then remained in “tracking the displayed provisional TAEM route while qualification runs.” After 2:17 wall time, all 33 guidance snapshots were frozen at the same UT/state while the backend used about 196% CPU. The run was interrupted before its 600 s planner timeout; it is recorded as a TAEM qualification stall, not as a completed landing failure. Evidence: `ShuttleSim/runs/mm304-sts-polar90-recorded-mm304-matched-20261014T180104Z-v_o3d7ew/` (manifest records 583.42 simulated seconds and `runner interrupted: KeyboardInterrupt`). This simulator used the ideal attitude plant and is diagnostic only.
- A separate clean-controller run of `.venv/bin/python -m pytest -q tests Validation` completed with **212 passed, 4 failed, 7 warnings, 56 subtests passed**. Failures:
  - `test_l5_mm305_golden_replay`: phase history stopped at MM305 instead of completing.
  - `test_l7_low_energy_final_still_flares`: no Final Approach handoff.
  - `test_tui_executes_an_external_reviewed_plan`: the Execute Plan button was not clickable in the test driver.
  - `test_mode_and_vessel_context_limit_operations`: the app observed the live KSP vessel in orbit and correctly showed the launch-clamp requirement, while the test expected the disconnected-state “Connect to KSP” text. This case is contaminated by the actual connected KSP test session.
- The first pytest run overlapped a live KSL controller and had 10 failures; it is superseded by the isolated 212/4 result above. It is not counted as an independent regression result.

## Telemetry quality

At near-vacuum altitudes, telemetry reported Mach values equal to speed in m/s (for example about 2,293 at 72 km while dynamic pressure was zero). This is invalid Mach telemetry in that regime and should not be used as a physical Mach measurement. The relevant telemetry path divides true airspeed by a speed-of-sound value floored at 1 m/s.

## Supplemental Final-only replay

Five isolated Final/flare/rollout replays on the matching STS-N model (all ideal-attitude, supplemental simulation only) produced no passing touchdown:

| Initial Final condition | Touchdown result | Failed gate |
| --- | --- | --- |
| 185 m/s, −10° FPA | 69.53 m/s, 3.128 m/s sink, on runway at 1,529.8 m; stopped at 2,494.2 m | Sink |
| 170 m/s, −16° FPA | 58.88 m/s, 2.465 m/s sink, 2,991.1 m along-track; did not stop | Speed and runway contact |
| 180 m/s, −16° FPA | 67.10 m/s, 3.259 m/s sink, on runway at 2,429.5 m; wheel departure at 2,497.5 m, no stop | Sink and runway rollout |
| 180 m/s, −18° FPA | 59.52 m/s, 2.448 m/s sink, 3,057.7 m along-track; did not stop | Speed and runway contact |
| 200 m/s, −20° FPA | 58.31 m/s, 4.239 m/s sink, 3,352.3 m along-track, peak gear load 4.374 g; did not stop | Speed, sink, and runway contact |

The 185 m/s case recorded wheel runway departure near the runway end (along 2490.2 m, cross 0.007 m), then stopped at 2494.2 m; rollout stability and runway rollout flags were true, with no crash. The 180 m/s / −16° case touched down on runway but reached runway departure at 2497.5 m and did not stop. The other three overshot the runway. These are ideal-attitude STS-N simulation results, not live evidence for the custom STS craft. Runs: `final-polar90-terminal-final-isolation-20261014T180531Z-_xtlt184/`, `final-polar90-terminal-fpa16-v170-20261014T180613Z-jypfld0s/`, `final-polar90-terminal-fpa16-v180-20261014T180706Z-yohyn9se/`, `final-polar90-terminal-fpa18-v180-20261014T180629Z-7t4mc1w4/`, and `final-polar90-terminal-fpa20-v200-20261014T180623Z-m47v98cm/`.

## Evidence locations

- Full flight: `runs/20261014T165854Z-sts-polar90-full-20261015/` (`status.json`, `events.jsonl`, `telemetry.jsonl`, `ascent.log`, `orbit-trim.log`, `deorbit-plan.json`, `checkpoint.json`, `descent-checkpoints.json`).
- Checkpoint replays: `runs/20261014T174720Z-sts-polar90-mm305-from-30km/`, `runs/20261014T174800Z-sts-polar90-mm304-from-72km/`, and `runs/20261014T175404Z-sts-polar90-mm304-from-30km/`.
- Orbit replay configuration check: `runs/20261014T175022Z-sts-polar90-orbit-checkpoint-replay/` (aborted before burn); `runs/20261014T175115Z-sts-polar90-orbit-replay-explicit/` (90 km override, passed and held at orbit).
- Current KSP state after tests: prelaunch save `sts-polar90-prelaunch-20261015` reloaded and settled; STS pre-launch, 8 clamps attached, mass 521,003.44 kg.

## Run IDs

```text
20261014T165854Z-sts-polar90-full-20261015
20261014T174720Z-sts-polar90-mm305-from-30km
20261014T174800Z-sts-polar90-mm304-from-72km
20261014T175022Z-sts-polar90-orbit-checkpoint-replay
20261014T175115Z-sts-polar90-orbit-replay-explicit
20261014T175404Z-sts-polar90-mm304-from-30km
mm304-sts-polar90-recorded-mm304-matched-20261014T180104Z-v_o3d7ew
final-polar90-terminal-final-isolation-20261014T180531Z-_xtlt184
final-polar90-terminal-fpa16-v170-20261014T180613Z-jypfld0s
final-polar90-terminal-fpa16-v180-20261014T180706Z-yohyn9se
final-polar90-terminal-fpa18-v180-20261014T180629Z-7t4mc1w4
final-polar90-terminal-fpa20-v200-20261014T180623Z-m47v98cm
```
