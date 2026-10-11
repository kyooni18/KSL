# Flare improvement plan (after live L7, 2026-10-11)

## What happened in L7

Live mission `20261011T062628Z-live-85km-10deg` / `…-landing`: 85 × 85 km at
10°, deorbit 30.1 m/s, entry, MM304 and TAEM all ran. **No flare.**

| UT | State |
|---|---|
| 6566 | TAEM entry, Mach 2.85, 26.9 km. Energy law at drag level `k = 0` (maximum range) from the first sample. |
| 6566–6900 | `k = 0` throughout. Initially `loop = 0`; at UT 6605.4 the law selects `loop = 1`, `leg = 2` (direct HAC intercept/downwind). Speed decays to ~100 m/s by 3 km. |
| 6900–7004 | On the Final line: cross-track 3–13 m, FPA −17.5°, 96 → 85 m/s. |
| 7005 | "MM305 crossed runway elevation before reaching the fixed-HAC exit." Contact ~75 m short, ~78 m/s, ~25 m/s sink. |

**Cause chain** (reproduced offline with `ksl simulate l7-mm305`):

1. TAEM delivered the orbiter to the Final line ~60 m/s below
   `finalAlignmentSpeed` (160 m/s). The energy law was saturated at maximum
   range for all of TAEM.
2. Final admission (`terminal_outer_capture_admissible`) needs a feasible
   preflare plan. Every candidate pull-up predicted an exit below
   `minimumSafeSpeed` (85 m/s): `preflare-speed-aoa-or-stall-gate` on every
   tick.
3. The energy law's Final-line handoff had no other exit. TAEM kept the
   vehicle and flew the min-drag glide into the ground. TAEM has no flare.

## What was fixed (d52a719)

- **Low-energy Final delivery.** On the Final line, if the vehicle is
  runway-aligned, the law is at `k ≤ 0.05`, and the nominal contract has
  failed for 3 s, TAEM hands off to Final through a logged
  `TAEM_TRANSITION_LOW_ENERGY_DELIVERY`.
- **Final, low-energy mode only:**
  - The aim is seeded on the energy-sized aim, not a geometric 20° seed (which
    pointed 3.6 km short).
  - The 160 m/s station-speed schedule is dropped from the speed closure. It
    was steepening the glide by up to 12°.
  - The glide is blended toward the learned best-glide angle while speed
    still covers a flare.
- **Data root.** Live landings now write to the canonical evidence store and
  FlightLogs (`KSP_LANDER_ROOT` = main checkout), even from a worktree.

Offline L7, same initial state:

| | Phase path | Touchdown | Sink | Gear | Along |
|---|---|---|---|---|---|
| before | TAEM → impact | 70.8 m/s | 13.2 m/s | up | −960 m |
| after | TAEM → Final → flare → rollout | 63.3 m/s | **1.25 m/s** | down | −962 m |

The L5 golden replay is unchanged and `make test` passes. The vehicle still
lands short: from that energy state the runway is out of reach at the learned
L/D (~3.8). **This fix makes the flare happen. It does not make L7 reach the
runway.** That needs the TAEM energy work below.

**Live follow-up:** the approved MM305 continuation on 2026-10-11 verified
low-energy Final delivery and flare, but landed slow/long and stopped beyond
the configured runway. See [live verification](Experiments/STS-Landing-20261011/L7-Live-Flare-Verification/README.md).

## Evidence on flare quality

| Flight | Source | Touchdown speed | Sink | Along (target 375 m) |
|---|---|---|---|---|
| L5 | live | 55.7 m/s | 1.13 m/s | 621 m |
| L6 | live | 52.8 m/s | 1.57 m/s | 640 m |
| L5 golden | ShuttleSim | 60.7 m/s | 4.94 m/s | on runway |
| L7 | live | no flare | ~25 m/s | −75 m |
| L7 (fixed) | ShuttleSim | 63.3 m/s | 1.25 m/s | −962 m |
| L7 (fixed, MM305 continuation) | live | 52.77 m/s before contact | 2.11 m/s before contact | 1531 m; stopped 2646 m, beyond runway |

The touchdown gate is 75 ± 15 % (63.75–86.25 m/s), so live L5 and L6 both
landed **slow and long**: the float bleeds ~20 m/s past the aim point. The
simulator disagrees with live on sink (4.94 vs 1.13 m/s for the same L5
state). Offline flare tuning is therefore not trustworthy until the plant's
pitch response matches live.

## Status after 8a42d22

| Plan item | State |
|---|---|
| 1. Flare ownership backstop + visible block reason | **Done** (all MM305 paths; status shows `Final blocked: …`) |
| 2. TAEM energy | **Partly.** The executed law no longer dives for the 160 m/s station speed at k=0. Root cause found upstream: the deorbit plan flew unqualified (`robustnessQualified false`, predicted TAEM range error −20.5 km, worst case 40.6 km), and MM304 handed over 87 km out (L5: ~60 km) at the same Mach/altitude. Open. |
| 3. Preflare floor | **Done** as a fallback floor (nominal flights unchanged) |
| 4. Touchdown speed / float | Open. Low-speed flare sink is sensitive: 0.8–4.5 m/s across small stretch-threshold changes on L7. |
| 5. Sim/live gap | Open. Blocks trustworthy flare tuning. |

ShuttleSim now: L5 passes every acceptance gate (66.1 m/s, 1.84 m/s sink).
L7 lands with a flare (55.5 m/s, 0.91 m/s sink) but 226 m short. The subsequent
live MM305 check verified Final delivery and flare but landed 1531 m along
and stopped 146 m beyond the configured runway, with a 230 kg mass loss during
overrun. The universal backstop and full deorbit/MM304 route remain unverified
live. Grounded warning recovery also failed to terminate; operator abort was
needed after stopping. Offline follow-up `9fdfa0a` now terminates a failed
grounded landing with Abort after braking below 1.5 m/s, while preserving
simulator contact evidence on termination. The regression and unchanged L5
golden pass; this termination fix has not flown live. No landing acceptance
gates were weakened.

## Plan, in priority order

### 1. Guarantee flare ownership (safety backstop)
- Add one terminal backstop that applies to every TAEM path (energy law,
  native HAC route, direct). Below a height set by time-to-impact (for
  example `h < max(300 m, 10 s × sink)`), Final owns the vehicle. The native
  HAC path can still abort with "crossed runway elevation before reaching
  the fixed-HAC exit". That is the same failure on a different path.
- Publish the Final-admission block reason (preflare reject, terminal block)
  in the snapshot status and the vehicle log. In L7 it was visible only with
  `KSP_LANDER_GATE_TRACE`.
- Acceptance: no scenario reaches runway elevation in TAEM. A
  `ksl simulate` matrix (L5, L6, L7, plus a high-energy case) asserts the
  Final → Flare phase path.

### 2. Fix TAEM energy delivery (root cause of L7)
- Investigate why the law sat at `k = 0` from TAEM entry and still selected
  a HAC intercept. `loop = 1` alone does not prove an overhead circuit or
  excess energy: `tel_plan` also sets it on direct `TEL_LEG_DOWN` candidates.
  The original live trace first selects that mode at UT 6605.4, at `k = 0`.
  `tel_plan_open` permits replanning on the downwind leg while the tangent
  remains more than 5 km away; the 8 km freeze applies to straight-in selection.
  - Check `tel_solve_k` and the loop selection against the L7 entry state,
    which is now a scenario.
  - Check whether MM304 handed over short. TAEM entry was at −80 km along,
    −33 km cross, Mach 2.85.
- Reconcile the energy targets. The subsonic dynamic-pressure band
  (`TEL_Q_LO`/`TEL_Q_HI` = 3.5/11 kPa, ≲135 m/s near the ground) cannot
  deliver `finalAlignmentSpeed` = 160 m/s. Either the station energy target
  should come from Final's speed schedule, or `finalAlignmentSpeed` should
  match what this vehicle can hold.
- Allow a mode change while unfrozen. If energy turns short after the loop
  decision, shorten or abandon the loop (straight-in).

### 3. Make the preflare planner model the actual vehicle
- `minimumSafeSpeed` = 85 m/s is used as the pull-up exit floor, but live
  touchdowns at 53–56 m/s were controlled. Derive the floor from the learned
  1-g speed at the incidence limit (`speed_1g`) with a margin. Then a slow
  but flyable state gets a real preflare plan instead of degraded mode.
- Keep low-energy mode as the fallback, not the normal path.

### 4. Touchdown speed and float length
- L5 and L6 floated ~250 m past the aim and lost ~20 m/s. Candidates:
  - start the arc later;
  - shorten the shallow-glide segment (`SGS_HEIGHT` = 12 m);
  - size `AIM_BIAS` from live data.
- Evaluate the adaptive flare (`adaptiveFlare`, off in `sts-live.json`). It
  predicts touchdown speed and along-track from the identified incidence
  response, which is the missing feedback. Enable it only after it beats the
  fixed flare on the matrix and on replays of the L5/L6 live data.

### 5. Close the sim/live gap first
- [L4/L5/L7 cross-check](Experiments/STS-Landing-20261011/Pitch-Plant-Crosscheck/README.md)
  rejects direct adoption of the existing pitch regression: held-out L7 R²
  is negative for both body and AoA coordinates, including a 0–400 ms delay
  sweep. The active direct plant integrates AoA using body-pitch coefficients;
  address that coordinate/kinematic mismatch before accepting a refit.
- Refit the ShuttleSim attitude plant (`att-live.ini`) from the L4–L7 pitch
  responses (`L6-MM305-Final/fit-pitch-response.py`,
  `pitch-response-L4-L5.json`). The acceptance check is that the L5 replay
  reproduces the live touchdown (≈56 m/s, ≈1.1 m/s sink, ≈620 m) within
  tolerance before any flare tuning depends on the simulator.

### Verification for each step
1. Run the `ksl simulate` matrix (L5 golden, L7, plus new L6 and
   high-energy scenarios). Phase path and touchdown gates are recorded per
   scenario.
2. Run `make test`, plus `make -C CLanding qualification` for any TAEM change.
3. Then a live flight from an MM305 save (`ksl-…-mm305` saves exist for L7)
   before a full mission.
