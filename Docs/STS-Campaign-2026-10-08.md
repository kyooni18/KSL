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
- An opt-in `KSP_LANDER_BYPASS_SHADOW_GATE` bypass was added and then reverted before this run's conclusions were written. It was never exercised: no `shadow-gate-bypassed` event appears in the run log or the state logs. Run 9 did not depend on it. Do not cite it as the reason run 9 engaged.
- MM304 phases reached: Temperature Control, Equilibrium Glide, Constant Drag, Transition. Predicted range margin ran +2490 km, +160 km, −62 km, −104 km, then settled near −100 km.
- MM305 was never admissible. TAEM was never acquired.
- Abort at UT 10785.4: alt 10.7 km, range 197.6 km, TAS 85.0 m/s, cross-track −43.5 km. Message: "MM304 flight recovery margin was exhausted before TAEM acquisition." The runner restored the 72 km save and froze the quicksave.
- No touchdown, rollout, or landing occurred.

Reading: the dominant failure is energy and range. At 10.7 km the vehicle is still about 198 km out with a predicted range margin near −100 km, so MM304 cannot bring it to the HAC. This matches the offline MM305 finding that the STS-N-era aero cannot produce an energy-feasible native profile. The run does not show a gate problem. The next change must address aero or energy, not the shadow gate.

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

Result: with this aero table, no AoA schedule reaches a glide L/D above about 0.75 in the hypersonic-to-supersonic range where the vehicle must bleed energy. The offline closed loop covers roughly 830 km and then runs out of speed about 267 km short of the runway. This is a property of the aero data, not of a gate constant or the AoA command. Landing from this entry state needs STS-specific aero with a materially higher hypersonic and supersonic L/D. Until that exists, every guidance variant tested here aborts in the same way.

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
