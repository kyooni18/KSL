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
