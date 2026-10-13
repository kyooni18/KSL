# Polar reentry undershoot: root causes and fixes (2026-10-13)

Branch `mm305-campaign-unpaused`. Fix commit `2d9e14e`, offline tool commit `b5b9392`.

## Symptom

Every polar (90° inclination, 100 km) STS return arrived at MM305 hundreds of kilometres short
of KSC. In the 09:40 KST run (from the 50 km checkpoint) the vehicle was 674 km from the site at
50 km, 362 km at 26 km, and still 276 km short at 3.4 km when that run was aborted by the operator
(KSP paused, vehicle intact). The deorbit planner had forecast the MM305 inlet at 67 km.

## Method

1. Same-altitude and same-UT comparison of the 06:40 deorbit plan trajectory against the 06:40
   flight (`Tools/polar_reentry_audit.py`, then a same-UT CSV).
2. Measured lift/drag tabulated by Mach and AoA from the vehicle logs of the 06:40, 08:07 and
   09:40 flights.
3. Burn Δv reconstruction: integrate (Δv − gravity) between vehicle-log samples during the burn
   and project onto along-track/radial/normal axes.
4. Closed-loop shadow (`predictor_simulate_guidance_shadow_core`, i.e. the production guidance
   flying the predictor plant) from the recorded 50 km state reproduced the live MM305 handoff
   range within ~17 km (345 km vs 362 km), so it is a trustworthy offline test bench.
5. New `plan` mode in `Tools/polar_reentry_replay.c` runs the full native deorbit planner from the
   recorded pre-plan orbit and then the closed-loop executable witness for the selected burn.

## Root cause 1: MM304 flew a 27° alpha the vehicle cannot glide at

Measured L/D of the 22-part Benjee orbiter in KSP (median per bin):

| Mach | AoA 12–15 | 15–18 | 18–21 | 21–24 | 24–27 | 27–30 |
|---|---|---|---|---|---|---|
| 4–5 | 0.71 | 0.70 | 0.67 | 0.41 | 0.36 | |
| 5–6 | | | | | 0.39 | |
| 6–8 | | | | | 0.33 | 0.30 |
| 2–3 | 0.89 | 0.86 | 0.79 | | | |

The live alpha schedule (`entry_alpha_schedule_default`) ramped from trim (18°) to
trim + 0.9·(28 − 18) ≈ 27° above 1.85 × 1300 m/s. The predictor used by the deorbit planner
already flies trim incidence (18° + drag-deficit boost). So the planner selected a burn for a
vehicle with roughly twice the L/D that MM304 then actually flew.

Closed-loop shadow from the *planned* post-burn state of the 06:40 plan:

| Alpha schedule | 50 km gate range | MM305 handoff range | Closest approach |
|---|---|---|---|
| Former 27° ramp | 550 km | 236 km | 135 km |
| Trim 18° | 407 km | 29 km | 4.4 km |

**Fix:** hold trim alpha at every velocity. Drag modulation, the bank-saturated drag boost and the
q/g/stall protections still raise incidence when energy must be shed. Lower incidence also leaves
more rudder authority (the 08:07 roll departure happened at AoA ≈ 24°).

## Root cause 2: the deorbit burn thrust was 22° off because the OMS cant was ignored

Reconstructed 06:40 burn: Δv along −41.70 m/s, **radial −16.80 m/s**, normal +18.69 m/s, i.e. the
thrust was pitched 22° toward the belly while the nose held the commanded direction. The planned
and achieved post-burn orbits had the same plane and periapsis height, but the line of apsides was
rotated **10.8°**. The shuttle therefore descended ~55 s (~125 km) early at every altitude above
the atmosphere. This was the vacuum part of the miss.

The backend already supports thrust-axis alignment (`thrust_alignment_attitude`), but kRPC
`Thruster.InitialThrustDirection` / `ThrustDirection` throw (null transform) for the Benjee
`benjee10.AJ10.v2` OMS, so `refresh_burn_geometry` reported `geometry=unavailable` and fell back to
the body Y (nose) axis. `Part.Direction` of the two OMS parts is (±0.163, 0.925, 0.342); their sum
is (0, 0.938, 0.347), exactly the 20.3° axis measured in earlier sessions when thrusters were
readable. For the SSMEs, `Part.Direction` and the thruster direction agree to 1e-4.

**Fix:** per-engine fallback to `Part.Direction`/`Part.Position` when the thruster query fails;
logged as `geometry=part-orientation`.

## Root cause 3: MM305 Final-line cross-track closure too slow for Final's alignment gate

With the two fixes above, the planner witness flies a complete MM305 overhead loop and HAC and
rolls out on the runway 09 line ~10 km out with 120–250 m cross-track. Final's pavement-projection
gate (`terminal_preflare_alignment_valid`) requires the projected track at the threshold *and* at
the runway end to lie within ±35 m. The line law's rate-closure cap
`omega_limit = 0.25/(τ + reversal_time)` gave a ~28 s time constant, so alignment arrived only
below ~200 m and the witness flew into the ground without a flare.

Witness sweep of the cap numerator: 0.25 → no Final; 0.4, 0.5, 0.6, 0.7, 0.8 → Final captured,
flare, touchdown on runway; 1.0 → aligned early but Final's specific-energy contract then failed.
**Fix:** 0.5. Final admission still requires every existing gate; no gate was relaxed. The L5
MM305 golden replay still lands on the runway with gear down (re-baselined 64.182 m/s,
1.675 m/s sink, stop 1519.9 m, 2.3 m off centreline).

## Root cause 4: open payload bay halved L/D and contaminated the planner history

Hypersonic L/D measured at Mach 6, AoA 27–30: 0.30 with the bay open (earlier flights) against
0.47 with it closed (10:30 flight); 0.56 at AoA 18–21. All earlier history was recorded with the
bay open, so the deorbit planner assumed a much draggier vehicle and the 10:30 flight (bay closed,
burn 146969766.4) overshot: 19 km from the site at 35 km altitude and Mach 5.

**Fix (`0e31480`):** the physics history is keyed to the configuration. `ksl land` sets
`KSP_LANDER_PHYSICS_VARIANT=payload-bay-closed` and the structure id becomes
`model:STS#payload-bay-closed`; the two bay-closed sessions were migrated to it (backup
`Runtime/Physics/observations.sqlite3.bak-20261013T1150`).

## Root cause 5: MM304 alpha did not follow the predictor's drag-deficit law

The predictor raises alpha in proportion to the drag deficit; live MM304 only boosted alpha when
bank was saturated. **Fix (`3e5bcb9`, `4e444fd`):** live MM304 uses the same proportional deficit,
low-pass filtered (8 s) with a filtered alpha ceiling, which also removed a 20.9↔22.9° target AoA
oscillation seen live.

## Root cause 6: coarse deorbit timing grid

The ±2 × 43.3 s candidate grid moves the MM305 inlet by 40–80 km per step. **Fix (`4e444fd`):**
after the scan, up to three secant refinements on a bracketing pair of the 96.5 km inlet target,
judged by the same candidate comparison (`DEORBIT_INLET_REFINE` trace).

## Model cross-check: flight-history cells vs a fitted 4-regime plant

A 4-regime envelope fitted to the 10:30 flight (L/D ×1.4) and the planner's AoA-resolved
bay-closed history cells both reproduce the 10:30 flight, but disagree by 70–100 km for earlier
burns that need range stretch at trim alpha (Mach 7→3), because the fitted plant has no AoA
dependence (10:30 flew AoA ≈ 28° there). Replaying the actual 10:30 burn on the history cells
matched the flown range within 3 km at 50 km altitude and 27 km at 40 km. The history cells are
therefore treated as the better model; the fitted plant is pessimistic for stretch. Below Mach 3
no bay-closed cells exist and the envelope default (L/D 0.4 near Mach 1) is conservative, i.e.
it biases towards arriving at MM305 with excess energy.

## Safety limits and gates

No AoA/bank/q/g/stall limit, admission gate or Final contract was relaxed. The improved RCS
pre-entry control (`2495309`) and the payload-bay preflight (`b882aeb`) are unchanged.

## Live verification

See the live-run section appended below.
