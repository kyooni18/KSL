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

## Atmosphere and aero source diagnosis (offline + read-only probes, 2026-10-08)

Live KSP density (read-only `CelestialBody.density_at`, KSP 1.12.5, this install) against `ShuttleSim/data/kerbin_atmosphere_seed.csv`:

| altitude (km) | live density (kg/m³) | seed density (kg/m³) | live/seed |
|---|---|---|---|
| 0 | 1.140 | 1.225 | 0.93 |
| 10 | 0.282 | 0.414 | 0.68 |
| 20 | 0.0400 | 0.098 | 0.41 |
| 30 | 0.00571 | 0.0150 | 0.38 |
| 40 | 0.00100 | 0.00175 | 0.57 |
| 50 | 0.000222 | 0.000170 | 1.31 |
| 55 | 9.63e-5 | 5.0e-5 | 1.93 |
| 60 | 3.82e-5 | 1.4e-5 | 2.73 |
| 65 | 1.15e-5 | 3.4e-6 | 3.38 |
| 70 | 0 | 0 | — |

The seed file is documented as an approximation to be replaced by exported KSP data. It is not this install.

Which tables each path uses:
- Live guidance and planning: the 71-sample live table captured in `CLanding/transport/client/session.inc:297–320` into `PlanetModel`.
- Terminal model (MM305/TAEM admissibility, `CLanding/app/controller/api.inc:10–22`): atmosphere, aero, and force book come from the ShuttleSim default model files (`shuttle_sim_model_path`, `SHUTTLE_SIM_MODEL_ATMOSPHERE/AERO/FORCE_BOOK`) unless overridden by environment variables. These defaults are not STS-specific and not live-derived.
- `CLanding/guidance/guidance_final.c:50–68` also copies the terminal atmosphere into `PlanetModel` for native terminal delivery admissibility.

Conclusion: the shadow gate and terminal admissibility can see a different atmosphere than the one KSP actually applies, and both use STS-N-era aero. The −62 km range error and the unstable predicted range are consistent with this. The measured-drag ratio in the earlier notes does not reconcile with a single density scale and should not be used to fit the drag model.

Next, in order (not yet done):
1. Make the terminal path take atmosphere from the live capture when a live capture is present, and keep the seed as the offline fallback. Record the source in the run manifest.
2. Provide an STS-specific aero book, or mark the STS terminal aero as unavailable and refuse admissibility. Do not reuse the STS-N book as validation.
3. Re-run offline validation on the new source before any live relaunch.
