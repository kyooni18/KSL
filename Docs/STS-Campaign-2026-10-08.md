# STS Landing Campaign — Session Log, 2026-10-08

Craft: `STS` (unflown STS orbiter, not STS-N). Profile: `Configuration/sts.json`, `vehicle.modelId = "STS"`.
Status at end of session: **planner stage complete; burn stage not run.** No burn, no entry, no landing has been flown in this session.

## Evidence classes

- **Live KSP observation** — read from the running game through kRPC.
- **Offline / planner output** — computed by the native planner, not flown.
- **Validated** — none in this session. Nothing here is a pass.

## Starting state (live observation)

Savegame `Main` (`/Applications/.Misc/LMPKSP/saves/Main`), flight scene.

| Quantity | Value | Source |
|---|---|---|
| Vessel | `STS`, orbiting Kerbin | live |
| Orbit | Pe 86.70 km, Ap 86.75 km (near-circular) | live |
| Mass / dry mass | 26146.85 kg / 20348.0 kg | live |
| LF / Ox / MonoProp / EC | 377.9 / 461.9 / 400 / 210 (of 378 / 462 / 400 / 210) | live |
| SSMEs (×3) | inactive, 1000 kN available each | live |
| OMS AJ10.v2 (×2) | active, 30 kN available each, throttle 0 | live |
| Attitude | pitch −89.8°, heading 229.9°, roll −139.9°, angular rate 0 | live |
| Control | throttle 0, SAS off, RCS off, autopilot off, brakes off, gear off, AG1 on | live |

Warp is zero. The OMS pair is active and the SSMEs are shut down, which matches the OMS-only precondition in `Docs/STS-ADAPTATION.md`.

Mass check, not yet resolved: the live mass exceeds dry mass plus LF and Ox at standard densities by about 1.6 t. Cross-check the mass model before trusting burn timing.

## Checkpoint preservation

Before any action, SHA-256 hashes were recorded for:

- `Main/86km.sfs` — `4ea194a0…662a6`
- `Main/Backup/86km (2026_10_08_03_17_00).sfs` — `24a365fc…6ff`
- `Main/persistent.sfs` — `afebd2ad…9829`

They were re-checked after the planner run and match exactly.

A new baseline save was created with kRPC `space_center.save`: **`STS-campaign-base-20261008`** (UT 1290.08, mass 26146.79 kg). No existing save was overwritten.

## Planner-only stage

Command (no `--live`, no quickload):

```bash
./run_headless.command --config Configuration/sts.json --no-quickload --max-wall-seconds 600
```

Result: exit 0, `plan validated; pass --live to engage the controller`. The KSP physics pause during the search was released afterwards, and the post-run probe showed the vessel unchanged.

Selected plan (offline / planner output):

- Burn UT **7423.0**, Δv **31.35 m/s** (retrograde), estimated OMS burn at the configured 18% throttle cap.
- Post-burn periapsis **50.0 km**; entry interface **1045.3 km** range; entry FPA **−1.72°**.
- Capture gate, shadow, MM304 gate and TAEM all reported `1`, but `strict=False`.
- Miss **40.0 km**, TAEM range **40.0 km**, TAEM range error **−56.5 km**.
- **Strict robust counter `robust 5/15` (33%)**, recovery `5/15`, unsafe count 0.
- Summary line: `Entry-stage qualified. 15/15 burn/entry stress cases pass (100%)`.

### Acceptance-gate discrepancy (blocking)

- `Docs/FLIGHT_TEST.md` Stage 3: automation must stay blocked unless at least **75%** of the deterministic stress cases pass and none crosses a hard unsafe boundary.
- `Docs/ALGORITHM.md`: the strict capture corridor is the "preferred engagement gate", with the same 75% threshold.
- The strict robust counter for the selected plan is **5/15 (33%)**, which fails that criterion.
- The code's engage path sets `execution_qualified = entry_stage_qualified` (`CLanding/prediction/deorbit_planning.inc:816`). The entry-stage counter (15/15) is a separate measure, so the planner reports the plan as executable while the documented strict gate fails.

Decision: **do not engage.** The strict criterion is the one the campaign defines as the acceptance test, and it fails. Documentation and code disagree on which gate governs engagement, and that has to be settled before any burn. This decision is not a validation of the plan.

## What was not done

- No OMS burn, no `--live` engage, no entry, no TAEM, no touchdown.
- The STS aero profile is still the inherited STS-N fallback (L/D 0.4, BC 700, speed/bank limits). It is provisional, and the data book has 0 certified aero cells.
- The `86km` checkpoint and `Main` save were not loaded or overwritten.

## Open items before the next stage

1. Resolve the engage-gate discrepancy: either gate engagement on the strict robust fraction (as the docs say), or correct the docs to the entry-stage gate. Record the choice.
2. Reconcile the live mass with the mass model (≈1.6 t gap).
3. Only after 1–2, decide between a planner re-run with a changed uncertainty setting (still no engage) and a manual OMS-only burn evaluation with a quicksave first.
4. Fallback STS aero values stay provisional until live STS data is collected.

## Live runs (2026-10-08, after the planner-only stage)

The user authorized a live test flight. Both runs used the same pinned backend and the same baseline save, `STS-campaign-base-20261008` (UT 1290.4, 86.7 km, mass 26.147 t). Savegame `Main` and checkpoint `86km` were not overwritten.

Run 1 (`live_flight_20261008_1.log`): launched, coasted to T-15 s, then the process was lost (orphaned when the session ended). A probe afterwards showed UT 9493.5 with no burn. Checkpoints were unchanged.

Run 2 (`live_flight_20261008_2.log`): relaunched from the baseline. The planner reproduced the same plan: burn UT **9510.3**, Δv **31.36 m/s**, Pe 50.0 km, miss 39.7 km, TAEM range error −56.8 km, robust **6/15**, entry-stage **15/15**, `strict=False`.

Guidance engaged at UT 1292.9 ("Guarded recovery execution engaged") on the entry-stage gate. This departs from the planner-only decision above, which did not engage because the strict robust fraction (5/15 at the earlier planner run) fails the 75% criterion. The engage was made under the live test-flight authorization, and the gate discrepancy is still unresolved.

Run 2 was stopped by the user during the coast. The task was killed before the burn. A read-only probe at 04:12 (UT 8278) showed the vessel in its 86.7 km circular orbit, mass 26.147 t, throttle 0. Process check at 04:13: no backend processes running.

Status: **no OMS burn has occurred, no entry, no TAEM, no touchdown.** Nothing has been validated as a success.

Open items:
1. Resolve the engage-gate discrepancy (strict 75% vs entry-stage 15/15), and record the decision in `Docs/ALGORITHM.md` or `Docs/FLIGHT_TEST.md`.
2. Decide whether to relaunch from the baseline save. The relaunch needs the user's go-ahead, because the run was stopped by the user.
3. Reconcile the ≈1.6 t mass gap and the provisional STS aero values.

## Live runs 3–5 (2026-10-08, after the run 2 stop)

Run bq0adnn5y (`live_flight_20261008_3.log`): burn from baseline UT 1290.3 under the STS profile with `timeoutMs: 2000`. Post-burn Pe 52.0 km, Ap 86.8 km, mass 25.819 t. Coast to 72 km saved `STS-campaign-72km-20261008`. Entry was flown on guarded recovery. The fault at UT 10250.0 (55.3 km, range 486.7 km) was `direct control C-Nano batch failed: decoding failed (transport=timeout)`. Control was returned to the pilot and the runner reloaded the baseline.

Live measurements from this run (provisional, STS profile, degraded aero feedback per MM304):
- Entry drag: measured D 0.00 m/s² at 69 km against reference 2.7 m/s²; D 0.56 against reference 1.03 at 55 km. The measured drag was far below the reference model.
- Predicted range minus available range swung from −179 km (69 km) to +541 km (55 km). This is an unstable prediction, not a single scale error.

Changes committed:
- `97abc5b` raised the STS `timeoutMs` from 2000 to 10000, to match the validated MM305 and live-cnano profiles. Connection setting only.
- `6199020` widened the load settle window in `Tools/ksp_test_guard.py` from target+8 s to target+60 s. This was needed because the 72 km save (UT 9930) loads from a baseline scene at UT 1290, and the Flight scene takes longer than 8 s to rebuild.

Run 4 (`live_flight_20261008_4.log`): failed in the load helper (`scene-did-not-settle`). The save had loaded, and KSP was paused at UT 9958.7, 70.3 km.

Run 5 (`live_flight_20261008_5.log`): loaded `STS-campaign-72km-20261008` at UT 9930.2 and settled. Guidance did not engage: "Reentry rejected by closed-loop shadow: entered=true, MM305=false, closest=0.5 km, range error=−62.0 km, q=8.4 kPa, g=3.07." The run then exited with "Timed out waiting for the landing backend" after cleanup restored and froze the quicksave.

Status: **no entry-stage control, no TAEM, no MM305 handoff, no touchdown, no rollout.** The entry-stage shadow gate rejects the STS planning aero (inherited STS-N fallback, LD 0.4, β 700). Nothing here is validated as a success.

Open items:
1. The planning aero is the blocker. Fix it from data, not from the single measured-drag ratio: the MM304 feedback is labelled degraded, and the predicted range is unstable. Compare the live density at entry altitudes against `ShuttleSim/data/kerbin_atmosphere_seed.csv` (the seed is isothermal 170 K above 40 km, with density 0.00175 kg/m³ at 40 km). Then decide whether to refit the drag model or change the atmosphere source.
2. Diagnose the run 5 backend timeout. KSP was responsive afterwards (UT and altitude readable), so the timeout is in the runner, not in KSP.
3. Keep the 86 km, 72 km, and baseline saves unchanged. The quicksave is overwritten by each cleanup.

## Atmosphere and aero source check (offline + read-only probes, 2026-10-08)

Correction: an earlier version of this section said the terminal model used the seed atmosphere and that the live/seed mismatch explained the shadow-gate rejection. That was wrong. The seed CSV (`ShuttleSim/data/kerbin_atmosphere_seed.csv`) is not used on any live or MM305 path. The seed is only used by ShuttleSim's own tests and tools.

Which tables each path uses:
- Terminal model (MM305/TAEM admissibility): `shuttle_sim_model_path` with the default profile `identified` (`CLanding/telemetry/sim_telemetry.c`, `model_profiles[]`). Atmosphere is `ShuttleSim/reference-model/kerbin_atmosphere_identified.csv`. Aero is `stsn_aero_identified.csv`, which is STS-N-era. The force book is `none` for this profile, so only the coefficient tables are loaded.
- Live guidance: the 71-sample table captured from KSP in `CLanding/transport/client/session.inc:297–320`.

Live KSP density (read-only `CelestialBody.density_at`, KSP 1.12.5) against the `identified` table the terminal model uses:

| altitude (km) | live (kg/m³) | identified (kg/m³) | live/identified |
|---|---|---|---|
| 0 | 1.1399 | 1.1486 | 0.992 |
| 10 | 0.2819 | 0.2826 | 0.998 |
| 20 | 0.04003 | 0.04003 | 1.000 |
| 30 | 0.005714 | 0.005719 | 0.999 |
| 40 | 0.0010007 | 0.0010010 | 1.000 |
| 50 | 0.00022172 | 0.00022144 | 1.001 |
| 60 | 3.8179e-5 | 3.8091e-5 | 1.002 |
| 65 | 1.1478e-5 | 1.1538e-5 | 0.995 |

Conclusion: the terminal atmosphere already matches live KSP to within about 1%. The atmosphere source is not the cause of the MM305 rejection or the shadow-gate rejection. The earlier proposal to adopt the live table into the terminal model is dropped and was reverted, since it would change nothing measurable.

MM305 offline rerun with the `identified` profile (the default): same result as the committed baseline, 5 admitted, 0 qualified, 0 violations. The rejection is "no HAC join has an energy-feasible native vertical profile". That points at the aero (STS-N-era coefficient tables) and the energy profile, not at the atmosphere.

Diagnostics (`KSP_LANDER_TAEM_DIAGNOSTICS=1`, run from the repo root): every HAC join station fails in `terminal_solver_generate_profile` (`CLanding/terminal_solver.c:586`, `:681`). The failures split two ways:
- "route is too short to descend within the slope limit" (76 probes at short leads).
- "alignment profile is too slow for Final handoff" (exit speed 113–132 m/s against the 160 m/s `final_alignment_speed`, with the 12% tolerance window 141–179 m/s, `TERMINAL_SMOOTH_SPEED_TOLERANCE` in `terminal_solver.c:234`).
No lead in the bisection range lands inside the window.

Ballistic-coefficient probe (temporary override of `estimated_ballistic_coefficient`, reverted afterward; not a fix):
- BC 700 (config, STS-N-inherited): 5 admitted, 0 qualified.
- BC 250: 2 admitted, 0 qualified.
- BC 120: 0 admitted; every sample vetoed `0x10` (energy or turn margin negative, `CLanding/prediction/decision/taem_capture.inc:153`).
So the drag scale alone does not open a window. The 700 value is not established for STS and must not be tuned to pass the test.

Next, in order (not yet done):
1. The MM305 gate has no STS-valid window with the current STS-N-era aero, across the drag values tried. Resolve it with STS-specific aero data (not available offline) or by stating that MM305 cannot qualify for STS. Do not tune BC or any aero value to pass the test, and do not reuse STS-N calibration as STS validation.
2. Provide an STS-specific aero book, or mark STS terminal aero as unavailable and refuse admissibility.
3. Re-run `make -C CLanding test` before any live relaunch.

## Offline baseline (2026-10-08)

`make -C CLanding test` on the committed source (before any CLanding change) fails at `mm305-feasibility-test`: "no physically admitted state qualifies an MM305 route; the contract would never hand over." 5 samples admitted, 0 qualified, all held in MM304. The failure is in the repository state, not from the run 3–5 changes (`97abc5b` and `6199020` touch only `Configuration/sts.json` and `Tools/ksp_test_guard.py`). The rejection reason is "no HAC join has an energy-feasible native vertical profile". The atmosphere source has been ruled out (see the source check above), so the cause is in the aero and energy profile. Fixing this is the first gate before any further live entry attempt.

## Live attempts after the offline probes (2026-10-08)

- Run 6 and run 8 (`--reentry-only` and `--connect-only`, both from `STS-campaign-72km-20261008`): the load helper returned `scene-did-not-settle` (`Tools/ksp_test_guard.py:265`). Run 7 with the same command settled. KSP load timing is flaky.
- `--reentry-only` is a live continuation mode and is refused without `--live` (run 7). No unguided observation mode exists in the runner.
- Read-only probe after the failed loads: KSP is in the Flight scene, vessel STS at 69.94 km, speed 2099.7 m/s, Mach 7.11, throttle 0, SAS off. UT is frozen at 9964.8 across 80 s of samples, so the game clock is paused. The vessel is parked and not descending. The 72 km save and the 86 km checkpoint are untouched.
- Run 5 reported "0 certified aerodynamic cells" for STS. The shadow gate rejects reentry for this reason, so guidance never engages. Rerunning the live command without a change reproduces the rejection.

Status: no entry, TAEM, touchdown, or rollout has occurred. The only path to guidance engagement is past the MM305 and shadow gate, which is a design decision (see the MM305 section above), not a runner setting.


## Live run 9 (2026-10-08, from `STS-campaign-72km-20261008`)

Live observation, not a pass. Command: `--live --reentry-only` with `KSP_LANDER_UNPOWERED_ONLY=1`.

- Entry Interface at UT 9932.4, alt 71.9 km, range 1092.8 km, TAS 2092.6 m/s. The shadow gate qualified on its own. The log reads "Reentry-only continuation armed ... after shadow qualification". No rejection message was logged.
- Correction from the raw telemetry audit: `FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl`, session `2026-10-07T21-34-08Z-29000-108766028242`, records `event=shadow-gate-bypassed; reason=KSP_LANDER_BYPASS_SHADOW_GATE` at UT 9932.411826 (record sequence 2, tick 1), immediately followed by `event=reentry-continuation-armed; ... shadow-qualified=true` (sequence 3). The earlier statement that no bypass event existed is withdrawn. Run 9's telemetry remains useful for physical trajectory comparison, but it is not evidence that the shadow gate admitted the run. The bypass was later removed; all future attempts must pass the normal shadow qualification path.
- MM304 phases reached: Temperature Control, Equilibrium Glide, Constant Drag, Transition. Predicted range margin ran +2490 km, +160 km, −62 km, −104 km, then settled near −100 km.
- MM305 was never admissible. TAEM was never acquired.
- Abort at UT 10785.4: alt 10.7 km, range 197.6 km, TAS 85.0 m/s, cross-track −43.5 km. Message: "MM304 flight recovery margin was exhausted before TAEM acquisition." The runner restored the 72 km save and froze the quicksave.
- No touchdown, rollout, or landing occurred.

Corrected reading (archived position check below): the vehicle passed the runway at about 39.8 km / Mach 5.37, then exhausted energy east of KSC while turning back. The 198 km quantity is unsigned distance, not an upstream shortfall. The next change must address entry energy prediction and TAEM geometry; the run does not justify changing the shadow gate.

## MM305 root-cause probes (2026-10-08, offline, all reverted)

Temporary single-value probes. Source is restored; `git status` shows no `CLanding` change.

- `finalAlignmentSpeed` 160 → 120 m/s: still 0 of 5 qualified. The rejection is unchanged.
- `estimatedLiftToDrag` 0.4 → 1.0 and → 4.0: the outcome and per-station reason counts are identical. The scalar L/D does not reach the native vertical profile. Only the final preflare planning uses it.
- Per-station reasons, baseline (`KSP_LANDER_TAEM_DIAGNOSTICS=1`): "route is too long for the available energy" (317), "profile exceeded propagation time" (300), "route is too short to descend within the slope limit" (277), "alignment profile is too slow for Final handoff" (96). The lead-length bisection never finds a station between the too-short and too-long bounds.
- The native profile is driven by the identified STS-N aero table (`ShuttleSim/reference-model/stsn_aero_identified.csv`). At alpha 18 its L/D is 0.74 at Mach 2, 0.67 at Mach 3, and 0.54 at Mach 5. Those are the MM305 speeds. The energy and slope bounds cannot both be met at that L/D, so no lead length qualifies.

Conclusion: MM305 cannot qualify for STS with the available aero. The blocker is the aero data, not a gate constant or the handoff speed. Changing the 12% tolerance, the slope limit, or the alignment speed to make the test pass would be tuning to the test, so it was not done. The same limit explains live run 9: MM304 could not close range from 10.7 km (predicted margin about −100 km).

Not resolved, and needing STS-specific aero (lift and drag tables for the orbiter in the MM305 band, Mach 1.5–5, alpha 15–20) before any further MM305 or live entry attempt is meaningful.

## Offline closed-loop reproduction (2026-10-08)

`ShuttleSim/scripts/run_guidance.py --scenario ksp70km-entry-interface.ini --configuration Configuration/sts.json`, closed loop, no KSP.

- Result: `success=false`, `abortReason="MM304 flight recovery margin was exhausted before TAEM acquisition."`, no touchdown, no rollout.
- This matches live run 9 (abort at 10.7 km with the same message). The offline harness reproduces the live failure, so it is a valid bench for entry-profile experiments.
- Existing offline manifests in `ShuttleSim/runs/` (23 runs): none has `rolloutValid`, `touchdownSpeedOk`, or `touchdownGearDown` set true. The offline closed loop has never landed with the current aero.

## Offline entry-AoA probe (2026-10-08, temporary configs, `Configuration/sts.json` unchanged)

- `vehicle.entryAngleOfAttack` 12 and 24 (default 18), offline closed loop from `ksp70km-entry-interface`: both abort with the same message, "MM304 flight recovery margin was exhausted before TAEM acquisition." No touchdown, no rollout.
- The result does not change across entry AoA. The abort is insensitive to this knob, which points at the aero table or the MM304 range/energy plan, not the entry AoA command.
- Not yet tested: whether `entryAngleOfAttack` is read by the offline path at all (`run_guidance.py` may take the initial AoA from the scenario `initial_aoa_deg`). Check before reading this result as a sensitivity.

## Entry-AoA probe: correction (2026-10-08)

The open question from the entry-AoA probe is resolved, and the probe result stands as recorded.

- `vehicle.entryAngleOfAttack` does reach guidance: `CLanding/guidance/entry_alpha.c:58` uses it as the trim, and `CLanding/guidance/entry/contract.inc:35` uses it as the default AoA.
- Several MM304/final paths clamp it to 10–18°: `guidance_terminal.c:183`, `final/recovery.inc:247,254,375`. A value of 24 is therefore clamped to 18 on those paths, so the 24° offline run cannot have tested a higher AoA there.
- The offline plant takes its own initial AoA from `initial_aoa_deg` in the scenario (1.3° in `ksp70km-entry-interface.ini`), not from the vehicle config.
- Result unchanged: 12° and 24° both abort with "MM304 flight recovery margin was exhausted before TAEM acquisition." The abort is not sensitive to the entry-AoA command across this range, given these clamps.

## Trigger of the MM304 abort (2026-10-08)

- Source: `CLanding/guidance/guidance_terminal.c:249`. The abort fires when `!physical_handoff` and `handoff_lost` holds, where `handoff_lost = true_air_speed < minimum_safe_speed || mean_altitude < site.altitude + 100`.
- With `minimum_safe_speed` = 85 m/s (`Configuration/sts.json`), live run 9 ended at TAS 85.0 m/s, and the offline run hits the same floor.
- So the abort is the airspeed floor, reached because the vehicle bleeds speed below 85 m/s before reaching the MM305 handoff window (141–179 m/s, the same window as the offline profile failure). The handoff never happens, so the floor triggers the abort.
- Offline plant effective ballistic coefficient along the 70 km baseline (`m·q/drag`, 40252 kg mass from the scenario): median 456 (range 351–767), not the planner's 700. Drag is higher than the planner assumes at most altitudes.
- Offline run with `estimatedBallisticCoefficient` = 456 (temp config): still aborts at the same floor. BC is not the lever on its own.
- Not done, and not a valid fix: lowering `minimum_safe_speed` to avoid the abort would only let the vehicle continue below the handoff window with no path to the runway. It would be gate tuning, so it was not changed.

## AoA schedule and the drag cap (2026-10-08): correction and result

Correction to the earlier entry-AoA conclusion: the guidance does not fly the configured entry AoA through the hypersonic phase. In the offline 12° run the plant flew about 23–24° from 91 s, reached the 28° `maximumAngleOfAttack` at about 6,300 s, and only dropped to 12° near the end. The MM304 energy controller sets AoA, bounded by `maximumAngleOfAttack`. The earlier "entry AoA is insensitive" reading was therefore not a valid sensitivity test.

Cap test (offline, temp configs, `Configuration/sts.json` unchanged):
- `maximumAngleOfAttack` = 18: the plant flew at most 18° (honored). The run aborted at 5.6 km, 84.8 m/s, 267.6 km along-track to the runway, cross-track 45.4 km. Abort message: "MM304 flight recovery margin was exhausted before TAEM acquisition."
- `maximumAngleOfAttack` = 22: same abort message.

Best L/D available from the identified STS-N aero table (`ShuttleSim/reference-model/stsn_aero_identified.csv`), maximized over alpha at each Mach:
- Mach 1.0: 1.25 (alpha 20)
- Mach 1.5: 0.65 (alpha 25)
- Mach 2.0: 0.75 (alpha 16)
- Mach 3.0: 0.74 (alpha 10)
- Mach 5.0: 0.63 (alpha 6)
- Mach 7.0: 0.53 (alpha 14)
- Mach 10.0: 0.53 (alpha 0)

Corrected result: this table offers at most about 0.75 L/D across the entry Mach range, and the tested AoA schedules abort. The positive along-track value was incorrectly read as an upstream shortfall: positive along-track lies beyond Runway 09. These runs do not establish that materially higher L/D is necessary; energy must be dissipated before the runway passage, and the downstream TAEM route still must qualify. See the archived geometry correction below.

## Plant sensitivity sweep: lift and drag scaling (2026-10-08, offline only)

Purpose: find out whether the existing guidance can reach TAEM if the plant's aero were different, with the guidance model unchanged. These are sensitivity results only. They are not STS aero, not a fix, and not validation. Runs use `ShuttleSim/scripts/run_guidance.py --scenario ksp70km-entry-interface.ini --configuration Configuration/sts.json --max-sim-time 2400`.

| Plant change | Outcome | Abort altitude (m) | TAS (m/s) | Along-track to runway (km) |
|---|---|---|---|---|
| lift ×1.5 | MM304 abort | 7,767 | 84.9 | 274 |
| lift ×2.0 | MM304 abort | 9,913 | 85.0 | 428 |
| lift ×3.0 | MM304 abort | 12,632 | 84.9 | 768 |
| drag ×0.5 | MM304 abort | 5,962 | 85.0 | 679 |
| drag ×0.25 | no abort; timed out at 2400 s sim time, still in MM304 Entry | 11,129 | 146.8 | -1,266 |

Reading:
- Every lift and drag case that aborts does so at the 85 m/s airspeed floor, before the MM305 handoff window (141–179 m/s). Scaling lift up moves the abort to higher altitude but does not carry the vehicle to TAEM.
- Drag ×0.25 is the only case that avoids the floor, but it does not get near the runway. At 2400 s it is still in MM304 Entry at 11.1 km, 147 m/s, with along-track -1,266 km. It is not a landing, and the along-track sign is unexplained. It may mean the guidance steers the wrong way when drag is low, or that the plant drifts. This has not been diagnosed.
- Conclusion: scaling the plant's lift or drag alone does not produce a TAEM handoff with the existing guidance. The blocker is the energy and geometry balance of the guidance on this plant, not one scalar aero term. This agrees with the earlier finding that the identified STS-N aero offers only about 0.5–0.75 L/D across the entry Mach range.

Still open: the drag ×0.25 along-track sign, and whether a combined lift and drag change (matching a full STS aero table) gives a TAEM handoff. Neither is answered by these runs.

### Drag ×0.25 along-track diagnosis (2026-10-08, offline)

Source: `ShuttleSim/runs/mm304-sens-dragx0.25-20261008T005700Z-fhjyxyuk/guidance-events.jsonl.gz`, guidance snapshots (735 samples, phase MM304 Entry).

- The vehicle does not abort, because its speed stays above the 85 m/s floor (146.8 m/s at timeout). It ends at the 2400 s sim limit.
- The guidance commands AoA up to the 28° `maximumAngleOfAttack` and holds it there.
- Roll rises from about 3° to the 40° roll command and stays there for a long stretch.
- Heading drifts from the runway heading of 90° to about 111° and then about 102°. The final along-track is -1,266 km, so the vehicle is flying away from the runway.
- Interpretation: with less drag, the energy controller has excess energy to dump. It dumps it through a large bank and turn, not through range, and the heading error is not corrected in time. This is a steering problem on the low-drag plant, not an abort. The snapshots are coarse (one every few seconds of sim time), so the exact heading history needs a finer trace before any fix is considered.
- Not a fix: nothing in this result justifies changing guidance gains. It only identifies where the low-drag case goes wrong.

## STS archived-force provisional basis (2026-10-08, offline)

Progress: recovered STS-specific aero measurements from the archived **live run 9** vehicle log, `FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl`. No running-KSP access occurred. Earlier statements that STS-specific aero data is unavailable were too broad: this flight contains measured force vectors, but does not provide the requested independent Mach/AoA sweep.

Added reproducible `ShuttleSim/tools/build_sts_provisional_aero.py`, `reference-model/sts_aero_provisional.csv` and its provenance/coverage JSON report. The tool requires exact vessel identity `STS`, deduplicates UT and filters q <100 Pa, powered flight, gear, airbrakes, sideslip >3°, low terrain clearance, and incidence outside 5–30°. Gear/airbrake flags come from measured state, rather than the commanded controls. It retains 10,639 samples. Table regeneration is byte-identical; an STS-N log is refused.

Model assumption, **not validation**: in each Mach bin, median K = CL/(sin(alpha)cos(alpha)), CD0 = CD−CL*tan(alpha); then CL = K*sin(alpha)cos(alpha), CD = CD0+K*sin²(alpha). Coefficient normalization area is 250 m²; it is not a measured STS wing area. Mach interpolation and constant endpoint extension fill the simulator table. Every alpha outside the report's support is extrapolated. There are **zero certified cells**, one source flight, and no independent held-out test. In-sample 95th-percentile relative residuals reach 51.5% in the Mach 0.5–1 bin; transonic/subsonic behavior needs better data.

Requested coverage remains incomplete: Mach 1.5–2 samples cover alpha 17.79–18.27°, Mach 2–3 cover 17.56–22.12°, Mach 3–4 cover 19.54–26.17°, Mach 4–5 cover 26.12–27.62°. The table's Mach 4–5 / alpha 15–20 cells are therefore assumption-based extrapolation, not STS validation.

Added the opt-in `KSP_LANDER_MODEL_PROFILE=sts-provisional` in both Python and native profile registries. Default `identified` is unchanged. This profile uses STS provisional aero, matching Kerbin atmosphere, **borrowed STS-N attitude dynamics**, no force book and no certified prior. Fixed the offline runner's path parser to accept `certified_prior=none`; it previously converted the sentinel into a nonexistent absolute filename. Removing the borrowed prior also changes guidance's available aero knowledge, so these runs are not a pure plant-only comparison.

### Offline verification and changed next hypothesis

- Default `make -C CLanding mm305-feasibility-test` and root diagnostic execution still fail: 5 admitted, 0 qualified, 0 contract violations.
- `KSP_LANDER_MODEL_PROFILE=sts-provisional make -C CLanding mm305-feasibility-test` likewise reports 5 admitted, 0 qualified, 0 violations. This property test uses its existing default configuration, synthetic atmosphere admission samples and 40 t states; it is a solver probe, **not an STS mission validation**.
- Provisional diagnostic replay finds at least one native vertical profile (`length=55859`, exit V=141.6 m/s, FPA −19.4°, slope-limited dive captured exit glide line) that meets the unchanged profile checks. Sample 2 (26 km, M2.5, −60 km along-track) then rejects for `candidate diverged from the MM305 tracking corridor` at 57.5/58.0 s. Thus "aero never permits any native profile" is no longer the full explanation under this provisional basis. Next investigate the replay's lateral/course tracking failure; do not widen the corridor or handoff tolerances.
- Offline closed loop, original `ksp70km-entry-interface.ini`, STS configuration and provisional profile: run `mm304-sts-provisional-baseline-20261008T011109Z-hti6o_sd`, abort at 1,125.82 s, altitude 3,935 m, TAS 84.98 m/s. No TAEM, touchdown or rollout. The vehicle passes the runway and continues east. Live run 9 also overshot (see correction below), though the trajectories and control histories differ.
- The original offline checkpoint mass is 40,252.92 kg, versus archived STS run 9 mass ≈25,816 kg. Added `ShuttleSim/scenarios/sts70km-run9-provisional.ini`, derived from the nearest archived descending 70 km sample (70,000.216 m, UT 9963.712). It uses measured LLA, surface speed, heading, FPA, AoA, bank and mass; heading approximates course at this nearly zero sideslip initial state. Surface coordinates avoid importing the incompatible KSP inertial rotation phase.
- Offline closed loop on that STS checkpoint: `mm304-sts-provisional-run9-checkpoint-20261008T011135Z-u66jnq51`, abort at 1,028.02 s, altitude 7,223 m, TAS 84.86 m/s. No TAEM, touchdown or rollout; again overshoots. Both runs use the ideal AoA/bank servo, not validated STS direct-control dynamics.

Decision: keep this as an explicitly provisional offline experiment profile. Do not use its overshoot as a reason to fly or retime the live burn. Validate archived commanded-control replay and force residuals first, including the lack of certified prior and borrowed attitude model, then inspect the newly exposed MM305 tracking-corridor rejection. All gate thresholds, handoff/slope tolerances, validation constants, BC 700 and live saves remain unchanged. Open issues remain backend timeout, load settling, mass/BC reconciliation and strict deorbit gate versus entry-stage gate.


## Archived geometry and replay correction (2026-10-08, offline investigation)

**Supersedes the earlier upstream-shortfall interpretation.** The previous goal turn made progress by adding a provisional STS aero basis. This turn checked its trajectory fidelity and the sign of the archived geometry, exposing a material campaign interpretation error.

The archived **live run 9** measured state at UT 10515.352 is longitude −74.73025°, altitude 39,805 m, Mach 5.366, speed 1,781.1 m/s, heading 90.21°, along-track −20 m. This is the runway passage, while far too high and fast for MM305. At the abort's last sample (UT 10785.572), longitude is −56.30576°, latitude −4.19378°, heading 282.64°, TAS 84.77 m/s, altitude 10,656 m. KSC Runway 09 is at −74.72833° longitude. The vehicle is **east/downstream** of the runway and turning back, not still upstream. Raw vehicle guidance fields show along-track **+192,921 m**, cross-track **+43,436 m**, unsigned range 197,584 m. The earlier −43.5 km cross-track description should not supersede these measured fields.

The original offline baseline `mm304-sts-offline-70km-baseline-20261007T221843Z-cjlu6_f5` also ended downstream: its manifest records along-track **+129,420 m**, longitude −62.36915°, altitude 4,227 m. The AoA-cap result with +267.6 km along-track and the positive plant sensitivity results likewise represent overshoot. The low-drag ×0.25 case ends at longitude +164.33° after continuing far around Kerbin; its negative projected along-track cannot, by itself, establish that heading turned away before initial runway passage. Previous conclusions that higher L/D was necessary to close an upstream range deficit are withdrawn. Low supersonic L/D still affects MM305 feasibility, but is a separate finding.

Added `ShuttleSim/tools/validate_sts_archive_replay.py`: fully offline actual-attitude and commanded-attitude reconstruction from run 9, aligned at the archived 70 km sample, with source/model/binary hashes and time-matched checkpoints. It disables all simulator network ports, uses the provisional STS coefficient table, and does not involve guidance or any certified prior. Report: `ShuttleSim/reference-model/sts_run9_replay_report.json`. No touchdown or landing is claimed; both replays are limited to the archived flight duration.

With recorded actual AoA/bank, replay altitude errors at live 60/50/40/30/20 km are approximately +65/+781/+1159/+1204/+1120 m. Speed errors at those checkpoints are +2.9/+8.7/+32.3/+108.8/+75.1 m/s. At the live ~10.6 km checkpoint, longitude error is about +2.29°. This is useful trajectory agreement early in entry, but insufficient fidelity to claim a live landing prediction. Command replay with the borrowed STS-N servo ends at about 12.1 km, +221.5 km along-track, compared with actual-attitude replay at about 11.6 km, +216.9 km along-track; neither matches the live terminal state exactly.

Experiment: added optional `--mach-bin-width .25` to the provisional fitter and produced a temporary 28-knot table. Maximum per-bin in-sample 95th-percentile relative force residual dropped from 51.5% to 29.1%, but recorded-attitude replay speed/longitude errors did not improve materially (near the final comparison: +44.6 m/s and +2.35° versus baseline +40.4 m/s and +2.29°). **Rejected as a model replacement**; tracked provisional table remains unchanged. Narrower bins do not supply independent AoA coverage or held-out validation.

MM305 diagnostic execution with `KSP_LANDER_MODEL_PROFILE=sts-provisional KSP_LANDER_TAEM_DIAGNOSTICS=2` still yields 5 admitted, 0 qualified, 0 contract violations. Trace evidence for a native-profile candidate: at t55 s, course 54.6° versus reference 79.8°, bank −19° versus command −11.1°; at t57.5 s, course 53.1° versus reference 86.8°, bank −2.8° versus command +5°. The lead-in curvature is already positive, while measured bank is still negative. Nearby candidates fail unchanged emergency course/cross-track limits; lateral authority shortfalls reach roughly 39–54 m/s². A valid vertical profile alone is not a flyable route. Do not weaken the corridor to pass these candidates.

Next justified experiments: compare reduced DRAGREF predicted range with full recorded-attitude propagation before the first runway passage; inspect authority and curvature evolution along the lead-in instead of raising L/D or changing gate constants. No running KSP action occurred. Backend timeout, scene settling, BC/mass reconciliation and the strict deorbit engagement-gate discrepancy remain unresolved separately.


## Campaign continuation: consumed AoA and MM305 fixture (2026-10-08)

Resumed from `240fef1`. All work in this section is offline. No live KSP connection, save load, or control action occurred. Protected WebTelemetry changes are excluded from campaign commits.

Reproduced both required MM305 invocations (default make target, then root diagnostic binary): 5 admitted, 0 qualified, 0 contract violations. The opt-in provisional STS profile likewise remains 0/5. Complete diagnostic logs are under `/tmp/sts-campaign-resume-20261008/`; the initial `mm305-default.log` is partial because a combined foreground command timed out, so `mm305-default-complete.log` is the authoritative second invocation.

Corrected one fixture/production mismatch in `Validation/MM305FeasibilityPropertyTests.c`: after setting measured AoA/bank, initialize both command and requested attitude to those measurements, as production `terminal_live_state` does. Previously the fixture retained model-file zero commands while actual AoA was 18 degrees, so the first replay command was 2.2 degrees. The corrected first command is 20.2 degrees. No physical sample, tolerance, or validation constant changed. Provisional rerun still reports 0/5, with sample 2's selected failed candidates at 59.0/58.0 seconds instead of 57.5/58.0. This removes an artificial initial transient; it does not solve route tracking or validate a landing.

`run_guidance.py` sends the entire JSON configuration to the backend; the scenario's `initial_aoa_deg` separately initializes the plant. `entryAngleOfAttack` is not an open-loop fixed AoA command: the phase schedule, modulation and drag boost remain bounded by `vehicle.maximumAngleOfAttack`. Repeated the probe using an ignored configuration copy that changes only that cap from 28 to 18 degrees, retaining the archived STS 70 km state and provisional model:

- Baseline `mm304-resume-sts-control-baseline-20261008T012408Z-roan1k0n`: requested AoA peaks at 25.5059 degrees, actual at 25.4468; first runway passage at 41,155.9 m and 1,816.65 m/s; abort at elapsed 1,028.02 s, altitude 7,223.325 m, speed 84.864 m/s, along +278,613.212 m.
- Cap probe `mm304-resume-sts-aoa-cap18-20261008T012509Z-8ljf14v3`: both requested and actual AoA peak at exactly 18 degrees, proving the changed input is consumed; first runway passage at 39,842.1 m and 1,819.44 m/s; abort at elapsed 988.52 s, altitude 7,927.279 m, speed 84.969 m/s, along +261,649.152 m. It is not a fix.

Both runs have no TAEM, touchdown, or valid rollout. The provisional STS table, borrowed STS-N servo, no certified prior, unchanged gates and no-mirror simulator transport are recorded in their manifests. Tracked `Configuration/sts.json` remains unchanged.

New entry observation to investigate: both provisional closed loops command essentially zero bank throughout entry. Their feedback has authority confidence zero despite finite measured lift and drag. At the baseline 40 km checkpoint, measured lift is about 2.11 m/s^2 and required vertical lift about 1.99 m/s^2; nevertheless commanded bank is zero. `entry_lateral.c:bank_magnitude` multiplies current lift by the minimum of model and longitudinal confidence, so removal of the borrowed certified prior can erase measured local authority. Do not restore an STS-N prior or invent STS certification to mask this. Isolate measured-force provenance from extrapolation confidence and regression-test any change before further closed loops.


### Measured lift versus model confidence: implemented and isolated

Added an explicit `has_measured_lift` input to the entry lateral allocator. The production caller sets it only for a valid physics sample with finite positive mass and lift, matching `live_lift_accel` provenance. Direct local lift is no longer multiplied by extrapolation/model confidence; model-only lift retains the original confidence discount. This does not set model confidence, load a certified prior, or alter gates, bank limits, roll shaping or sideslip protection.

Regression `measured_lift_is_not_model_confidence` failed before the implementation and passes afterward. It proves the measured-force bank allocation, unchanged rate bound, zero-confidence model-only fallback, rejection of zero/negative/nonfinite force, and wings-level behavior when measured lift cannot meet the vertical demand. `make -C CLanding -j2 entry-lateral-feedback-test entry-energy-law-test` passes.

Offline black-box run `mm304-resume-sts-measured-lift-20261008T013252Z-qhr55eqp` uses the same STS checkpoint, configuration and provisional model. It now commands the existing 40-degree bank ceiling and 28-degree AoA cap. The first runway passage remains 41,155.9 m and 1,816.65 m/s: bank authority becomes useful only around that passage. It aborts at elapsed 957.32 s, altitude 7,335.957 m, speed 84.998 m/s, along +201,800.504 m, cross +44,295.113 m. No TAEM, touchdown, or rollout. The fix changes downstream control but does not repair the upstream overshoot.

Important hypothesis correction: the reduced range predictor already flags excess required distance well before runway passage (baseline at 50 km: predicted achievable range about 496.7 km versus 128.3 km required). The controller still demands about 1.16 m/s^2 of vertical lift when only 0.64 m/s^2 is measured, so wings level is the bounded allocator's answer. Investigate the unreachable longitudinal demand and entry-state reachability rather than claiming the predictor simply failed to notice the overshoot.


### MM305 saturated lift allocation: route tracking improves, speed still rejects

Replaced the tracker's weighted absolute-error score with the Euclidean acceleration-vector error for bounded AoA/bank allocation. The former `vertical_error + 0.35*lateral_error` preferred almost zero lift when vertical demand was zero and every available turn was saturated: at bank 70 degrees, `cos(bank) > 0.35*sin(bank)`. This is an allocator defect, not evidence that a lower lift command makes the requested turn physically feasible. Feasible lateral solutions retain precedence; infeasibility flags and all flight/gate limits are unchanged.

Added `Validation/TAEMTrackerAllocationTests.c` and `make -C CLanding taem-tracker-allocation-test`. The regression independently evaluates an available bounded force vector at three saturated demands and both turn signs. Before the fix, the provisional case selected AoA 0.1 degrees and acceleration error 6.2290 m/s^2 when an allowed vector achieved 2.2078 m/s^2. Afterward the error is 2.2078, while `lateral_authority_ok` remains false. Both default and provisional model tests pass. The broader `taem-native-stack-test` passes before and after the allocator change; logs are `native-stack-before-allocation.log` and `native-stack-after-allocation.log` in the continuation scratch directory.

Full provisional MM305 rerun (`mm305-vector-allocation.log`) still reports 5 admitted, 0 qualified, 0 contract violations. Sample 2's best candidates now reach the Final alignment area instead of failing the tracking corridor near 58 seconds. Left/right selected failures occur at 192.5/188.5 seconds and are correctly rejected for missing the simultaneous Final contract. Near the left alignment station: along -8197.7 m, cross +8.7 m, heading error -1.93 degrees, bank +2.97 degrees, bank rate +1.08 degrees/s, height error +94.7 m, FPA error +0.50 degrees, but speed 142.9 m/s against 160 +/-10. The right candidate similarly reaches along -8196.9 m with acceptable position/attitude/height/FPA, but speed 145.5 m/s. Do not lower the target or relax the speed tolerance; improve the reachable profile and its delivery energy.

No STS mission landing is established by this synthetic 40 t/default-configuration solver probe. The 70 km closed-loop entry overshoot remains a separate blocker. Next inspect native-profile versus replay control/integration fidelity and energy preservation: native replay currently steps the same semi-implicit airborne integrator at 0.5 s, versus the plant's 0.02 s cadence, and traces show alternating AoA around a constant command. This is a hypothesis requiring a convergence experiment, not yet an established cause.


## Campaign continuation: exact run-9 checkpoint replay and allocator A/B (2026-10-08)

All work here is offline. The live KSP remained paused at the named 72 km checkpoint; no scene load, control command, or live flight was issued. The reconstructed scenario is `ShuttleSim/scenarios/sts72km-run9-entry-interface.ini`, initialized from archived run 9 keyframe 1 at UT 9932.411826 (altitude 71,851.542 m, TAS 2,092.573 m/s, FPA -1.6072°, mass 25,816.105 kg). This is the first archived post-load keyframe, 2.34 s after the save's UT 9930.071826, altitude 71,992.616 m. Its lat/lon, heading, attitude, and Runway 09 geometry are recorded in the scenario. The saved checkpoint and tracked `Configuration/sts.json` were not changed.

The exact-state downstream controller replay with `Configuration/sts.json`, `sts-provisional`, and no mirror transport is `mm304-sts72km-baseline-provisional-20261008T120451Z-zjh5ba8s`. It aborted at UT 10909.032, altitude 7,316.376 m, TAS 84.988 m/s, along +180,645 m, cross -36,555 m, before TAEM. At the first runway-longitude passage it was at 40,002 m and 1,784.7 m/s, with +0.6 km along and +3.88 km cross. The archived live run passed at UT 10513.87, 39,944 m and 1,785.3 m/s, with -2.5 km along and -0.31 km cross. Early range/time/speed are close; the lateral path then diverges. No touchdown or rollout occurred.

To isolate commit `cec6a79`'s measured-lift bank-authority change, a temporary backend was built with only `entry_lateral.c` restored to its pre-`cec6a79` version; the current terminal propagator and the rest of the source remained the same. The identical replay (`mm304-sts72km-premeasured-lift-20261008T122128Z-672lqsy7`) held bank at 0° through the observed descent and also aborted before TAEM, at UT 10991.332, altitude 7,234.235 m, TAS 84.916 m/s, along +278,588 m, cross -418 m. It overshot much farther downrange than the current allocator run.

At 50 km, current code commanded +32.2° bank, while the old allocator and archived live run were near 0°. By 40 km, current code commanded -40°, while the old allocator and live run remained near 0°. At 35/30 km, the archived live run commanded +34/+40° and rolled +46/+40°; current code commanded -40°, while the old allocator remained wings-level. This confirms that using valid measured lift independently of model confidence enables earlier bank commands. It does **not** explain the later live positive-bank direction: current and archived live commands have opposite signs, while the old allocator suppresses the turn entirely. Treat the bank-sign/timing difference as unresolved; do not revert the measured-force fix or infer that its A/B is a successful flight result.

The 12°, 18° baseline, and 24° `entryAngleOfAttack` configurations were all echoed by the backend and exercised from this same checkpoint in downstream-controller diagnostics. All three reached the unchanged 28° maximum AoA at some point and aborted before TAEM at the 85 m/s recovery floor. The 12° and 24° variants did not repair the trajectory. Their complete copies are `Configuration/experiments/sts-entry-aoa-12.json` and `sts-entry-aoa-24.json`; production `sts.json` remains at 18°.

The terminal propagation convergence check (`make -C CLanding terminal-propagation-convergence-test`) passes against the native airborne kernel at the 50 Hz plant cadence for 0.02, 0.1, and 0.5 s outer control intervals. The propagator subdivides held controls into 0.02 s steps; observed state differences are zero in the parity fixture. This establishes integration-step parity for that kernel fixture, not STS aerodynamic accuracy or landing prediction.

The vehicle-correct MM305 fixture remains 0/5 qualified with the same rejection: no HAC join has an energy-feasible native vertical profile. Using the 50 Hz subdivided propagator increased mean solver wall time about 2.1× versus a detached HEAD build using one 0.5 s propagation step, with no change to admission or rejection. It is not a feasibility fix. No shadow or Final gate, speed limit, bank limit, or other flight limit was relaxed. STS-specific aero/actuator evidence and the MM304-to-MM305 energy/geometry delivery remain unresolved.

### Shadow-gate fast-path correction

Source audit found that `landing_controller_start_reentry_only` sets `simulator_fast_qualification` whenever its kRPC session is a simulator. That branch seeds the prediction from the current state and accepts the continuation without calling `predictor_simulate_entry_guidance_shadow` or `reentry_guidance_shadow_recovery_qualified`. `run_guidance.py --engage engageReentry` uses this offline path. Therefore the exact-state replay, its 12°/24° probes, and the allocator A/B above are downstream controller diagnostics only; their `shadow-qualified` status text is not a shadow-gate result. Unsetting `KSP_LANDER_BYPASS_SHADOW_GATE` does not change this simulator fast path. The archived live run 9 separately contains the explicit `KSP_LANDER_BYPASS_SHADOW_GATE` event already recorded above. Neither source provides legitimate shadow admission. Before any live continuation, run the production shadow predictor from the checkpoint and require its normal recovery qualifier to pass; no bypass has been used on the current live KSP.

An isolated backend copy with only that simulator fast-path condition disabled ran the actual production shadow predictor/qualifier from the exact checkpoint. With `estimatedLiftToDrag=0.4`, it admitted an **MM305 advisory opportunity** at UT 10529.3, altitude 24,363 m, TAS 860.1 m/s, FPA -10.59°, course 88.13°, and energy margin +78.6 kJ/kg. The shadow terminal-policy result was invalid/unavailable; this was an entry-start gate pass, not a route or landing qualification. The ensuing lockstep replay still aborted before MM305 at the same 85 m/s floor. Its full trajectory does not realize that nominal advisory capture.

The difference is consistent with an entry-model lift mismatch. In the `estimatedLiftToDrag=0.4` shadow, model-only lift at 50/40/35 km is 0.351/1.238/2.302 m/s², with bank commands 0/+0.5/+39.3°. The lockstep STS plant at those bands reports measured lift 0.643/2.461/4.348 m/s² and current controller commands +32.4/-40/-40°. The predictor's scalar lift is especially low at 50 km and sends the wrong bank sequence by 35 km.

Added `Configuration/experiments/sts-shadow-entry-ld075.json`, a complete `sts.json` copy changing only `vehicle.estimatedLiftToDrag` from 0.4 to 0.75. The value is an offline sensitivity fit that makes the shadow's model-only lift approximate the measured lockstep lift; it is not a held-out STS fit or a production recommendation. With this one change the shadow bank commands at 50/40/35 km become +32.1/-40/-40°, closely matching the lockstep plant. The normal shadow gate then rejects the checkpoint before physics advances: its prediction aborts at 11,814 m, minimum speed 84.953 m/s, with no MM305 opportunity. The matching sensitivity run is `mm304-sts72km-shadow-ld075-20261008T123902Z-_icjd7e1` (`recordedSimTime=0`). This supports the diagnosis that the 0.4 scalar causes an optimistic shadow; it does not establish a safe aero model. Keep production `Configuration/sts.json` unchanged and do not start a live flight from this unvalidated scalar.


## STS repair pass: runner, vehicle identity, and frame consistency

This pass repairs demonstrated defects; it does **not** establish a successful STS entry, MM305 acquisition, touchdown, or rollout. No KSP scene was loaded and no new live flight was started. All new execution evidence is offline. Scratch evidence is under `/tmp/sts-repair-20261008`.

### Committed mechanisms

`4872131` fixes backend engagement acknowledgement ordering and strict-runner semantics. Native actions can publish their decision snapshot before their JSON response. The runner previously discarded that already-received decision and waited for another snapshot: a paused rejection or a blocking coast warp then appeared to time out. `BackendProcess.engage` consumes a fresh decision, preserves rejection/fault reasons, rejects stale snapshots, and does not mistake an `ok` response for armed automation. All three headless engagement paths use it. `--require-strict` now refuses failed strict qualification rather than printing a warning and continuing; failed post-burn strict verification also stops. Entry-stage qualification and strict full-route capture remain distinct. Planner-only output explicitly says that no live flight was validated.

`00cd5c9` fixes checkpoint-load false positives. The old loop could accept an unchanged paused scene, an unchanged target clock without a load event, or a rewind to a different checkpoint. The repaired loop requires the requested clock window, a flight scene without warp, and positive load evidence (acknowledgement, an observed scene transition, or a qualifying clock discontinuity). A network error alone is not load evidence. This is covered by mocked load-loop tests, not a newly attempted KSP load.

`1a7129a` fixes the current MM304 vertical-support frame. DRAGREF used airspeed in `g - v^2/r`; radial support actually depends on body-centered inertial **tangential** speed. Production now supplies the measured state, excludes radial velocity from curvature, and retains airspeed for aerodynamic/range-profile calculations. An eastbound fixture failed at 3.597842 m/s^2 instead of the independently expected 2.493948 m/s^2 before the repair. Reduced callers without a state retain their old explicit approximation. The existing support bounds and all flight limits remain unchanged.

`3f4f2fa` separates named model-profile identity from vehicle identity and fixes the STS experiment fixture. `identified`, `fitted-legacy`, and `reference-b` describe STS-N; `sts-provisional` describes STS. Native engagement/model capture refuses a mismatch, and `run_guidance.py` rejects it before allocating ports, creating run artifacts, or spawning a backend. This prevents silent use of the default STS-N profile for an STS configuration. It does not certify a matching profile or validate arbitrary operator-supplied override files. The default remains STS-N; no profile is silently changed to make a run pass.

`ebf55b6` fixes the terminal tracker's corresponding rotating-atmosphere gamma-rate equation. It now projects gravity, Coriolis, and centrifugal acceleration onto the normal to air-relative velocity before computing lift demand. A central difference of independent gravity-only inertial propagation verifies the demand in 54 rotating/non-rotating, latitude, direction, speed, and descent cases. The old equation already failed the 160 m/s eastbound equatorial case: 8.971139850 versus 8.824618644 m/s^2. Existing turn-allocation tests still pass. No handoff speed, slope, bank, load, or structural limit was relaxed.

### Experiment scope and mass reconciliation

The earlier profile-only MM305 probes used a default STS-N vehicle configuration, 40,000 kg mass, and a synthetic envelope fixture, even when selecting the provisional STS aerodynamic CSV. They are not vehicle-correct STS experiments. The extended native fixture accepts `--configuration` and requires an explicit `--mass-kg`; it verifies model identity, uses that configuration's site, and derives atmosphere and measured-state forces from the selected native plant. Default behavior remains a separately labeled legacy fixture. A failed finite set of samples is not a proof that every state is unreachable; the diagnostic now states the narrower failed non-vacuity condition.

The explicit STS fixture uses `Configuration/sts.json`, `sts-provisional`, and the run-9 checkpoint mass of **25,816.072266 kg**. The pre-terminal-frame-fix run in `mm305-sts-configured.log` completed with 5 physically admitted, 0 qualified, and 0 handoff violations. The preceding native default baseline in `mm305-native-default.log` likewise completed at 0/5. These failures remain failures; they were not converted to skips or passes.

The separate approximately 1.6-tonne live mass discrepancy was missing monopropellant accounting, not demonstrated corrupt mass physics. Installed `GameData/Squad/Resources/ResourcesGeneric.cfg` gives LiquidFuel/Oxidizer 0.005 t/unit, MonoPropellant 0.004 t/unit, and ElectricCharge zero density. The rounded snapshot reconstructs `20348 + 5*(377.9+461.9) + 4*400 = 26147.0 kg`, versus measured 26146.85 kg. Do not retune vehicle mass to remove that explained difference.

### Recorded STS checkpoint result and reproducible checks

Post-MM304-frame run `mm304-sts-repair-inertial-frame-20261008T020456Z-dm7oi_ha` used the measured STS 70 km scenario and the provisional plant, not the generic 40-tonne checkpoint. It first passed the runway at elapsed 550.52 s, altitude 40,034.620 m, and airspeed 1,785.747 m/s. It still aborted before MM305 at elapsed 945.12 s, altitude 7,323.845 m, airspeed 84.999 m/s, along +180,664.806 m, cross -36,532.949 m. This is an **overshoot**, not an upstream shortfall and not a repaired landing. The later terminal-frame change requires its own final verification record below.

The fast regression overlay avoids modifying the pre-existing Makefile work:

```sh
make -C CLanding -f Makefile -f ../Validation/STSRepairTests.mk sts-repair-test
```

It covers 33 Python cases, native engagement/profile checks, MM304 frame checks, and 54 independent TAEM frame cases. Native backend protocol checks are separate. The broader suite and the explicit STS experiment are:

```sh
make -k -C CLanding -f Makefile -f ../Validation/STSRepairTests.mk test sts-repair-test
make -C CLanding -f Makefile -f ../Validation/STSRepairTests.mk sts-run9-mm305-test
```

The STS provisional table still has no independently certified cells and borrows STS-N attitude dynamics. Neither model-name matching nor structural `replay_validated` flags replace held-out aerodynamic/actuator validation. The unresolved work is qualifying the entry/terminal trajectory and its energy delivery with that uncertainty visible, not bypassing shadow or Final gates. Pre-existing `terminal_propagator.c`, its convergence test/Makefile work, and the WebTelemetry changes remain separate; their existing tracked-file hashes were preserved during this repair pass.

### Offline simulator qualification labels and missing forecasts

The simulator fast path in `landing_controller_engage_reentry` now arms downstream diagnostics without marking them as shadow-qualified, robust, or execution-qualified. Its status, warning, plan note, and structured snapshot event say that shadow admission was skipped. TAEM, miss-distance, entry, and worst-case forecasts are absent (`null`) instead of being seeded from the entry state or serialized as zero. This path remains useful for controller replay only; it cannot establish any campaign gate.

The backend builds with `make -C CLanding -j2`. Offline smoke run `mm304-sts72km-diagnostic-null-check-20261008T124629Z-1sqsxbl_` confirms all qualification flags are false, the diagnostic warning/note are present, and forecast fields are null. It intentionally advances only 2 s and records no landing result. No KSP controls or scene changes were issued.
