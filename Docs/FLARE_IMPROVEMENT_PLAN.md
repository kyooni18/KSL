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
| 2. TAEM energy | **Partly.** The executed law no longer dives for the 160 m/s station speed at k=0. Upstream targeting and lateral geometry remain under investigation: the plan had `robustnessQualified false`, and L7 reached TAEM much farther laterally displaced than L5. Scalar range and `k=0` alone do not establish the root cause. Open. |
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
golden pass; the repeat approved flight below now verifies automatic low-speed failure termination live. No landing acceptance
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
- [Measured entry lateral history](Experiments/STS-Landing-20261011/Entry-Lateral-History/README.md)
  shows L7 reversing bank near centreline crossing but needing about 150 s
  to arrest its lateral velocity. An [experimental arrest projection](Experiments/STS-Landing-20261011/Entry-Lateral-Arrest/README.md)
  improves the provisional TAEM crossrange by 4–5 km, but its full chain still
  misses Final/flare and contacts short with gear up. It remains off by
  default and is not accepted as a production energy/entry fix.
- [Corrected MM304 inlet comparison](Experiments/STS-Landing-20261011/MM304-Measured-Inlet/README.md)
  finds less than 0.1° heading/course difference at entry. The large L7
  lateral handoff error survives corrected geometry and both provisional
  coordinate plants, narrowing the inquiry toward lateral/range prediction
  and model/history error. No targeting change is justified yet.
- [Current MM304 replay investigation](Experiments/STS-Landing-20261011/MM304-Range-Control/README.md)
  rejects lowering the coupled acquisition/drag-sizing range from 96.5 km to
  60 km: L7 then misses the Mach/altitude handoff window. L5 also begins its
  golden MM305 replay at `k=0`, so saturation alone does not establish L7's
  cause. Compare lateral path and longitudinal model error. A candidate aligning the range predictor
  with execution preserved L5/L7 touchdown outcomes, but was set aside after
  the broader MM305 admission qualification failed. The unchanged baseline
  also fails the same 0/5 non-vacuity assertion; admission coverage remains open.
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
- [Measured pitch authority/inertia cross-check](Experiments/STS-Landing-20261011/Pitch-Authority-Crosscheck/README.md)
  expands held-out checks to L6 and the third L7 continuation. Replacing
  capped Q control authority with native torque/inertia still gives negative
  held-out R² on every flight. Maximum available torque is not validated as
  net aerodynamic moment; no coefficients are adopted.
- [Measured-state restarts and matrix](Experiments/STS-Landing-20261011/Measured-State-Replay/README.md)
  correct course/body-heading confusion and preserve measured body pose/rates.
  Independent L6 and a synthetic high-energy inlet now join L5/L7. The pinned
  L4-fit CLI matrix reaches Final and flare with gear down in all four cases,
  but every landing fails acceptance. No plant is qualified; earlier MM304
  comparisons also need corrected inlet geometry before targeting changes.
- [Body-coordinate plant experiment](Experiments/STS-Landing-20261011/Body-Coordinate-Plant/README.md)
  adds opt-in body moment/quaternion state with incidence derived from the
  changing flight frame, plus actual body-rate telemetry. Conservation/sign
  checks and unchanged baseline goldens pass.
  [Corrected coordinate-rate decoding](Experiments/STS-Landing-20261011/Body-Coordinate-Rate-Fix/README.md)
  removes false recovery triggers: all six body-model replays now flare, and
  the L4 fit passes L5 acceptance. The L7 fit isolated Final case is close to
  live L5 speed/sink/position, but full-route and held-out response validation
  remain incomplete. Roll/yaw identification and predictor/control parity
  remain open. No production model switch or flare tuning was accepted.
- [Integral identification and kinematic check](Experiments/STS-Landing-20261011/Pitch-Integral-Crosscheck/README.md)
  includes both L7 continuations. Longer intervals and sqrt(Q) damping still
  do not produce a transferable L4–L7 fit. Subtracting measured FPA rotation
  improves near-level AoA-increment RMSE by 55–76% on every flight, supporting
  preserving body rotational state and deriving incidence from the changing
  flight frame before fitting moments. No new plant coefficients were adopted.
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

### Repeat live verification, 2026-10-11

[The approved repeat MM305 flight](Experiments/STS-Landing-20261011/L7-Live-Rollout-Verification/README.md)
exercised flare and automatically ended FAILED after runway overrun. Initial
ground response followed telemetry near 47.85 m/s and 4.03 m/s downward;
first landed flag followed a bounce. KSP paused landed at about 2810 m along
and 1.37 m/s, brakes on. Grounded failure termination is live-verified; slow,
long delivery, phase oscillations, flare sensitivity and the plant gap remain
open. Universal terminal backstop and full deorbit/MM304 route were not tested.

### Third approved continuation, 2026-10-11

[Current-backend live verification](Experiments/STS-Landing-20261011/L7-Live-Current-Verification/README.md)
again reached Final and flare, then automatically failed after a 130 m overrun.
The last pre-ground-response sample was 50.81 m/s with 3.59 m/s sink,
1567 m along. STS is paused landed at 1.36 m/s, gear/brakes on. Phase
oscillations recur; no backstop, adaptive flare or full entry route was tested.

### User-requested earlier aim and drogue

[Earlier aim and drogue deployment](Experiments/STS-Landing-20261011/Earlier-Aim-Drogue/README.md)
sets the STS touchdown aim to 150 m (previously 375 m) and enables ground-only
parachute deployment. The actual Mk12-R drogue was found stowed after the
third flight. The measured L7 replay moves contact only 31 m earlier and
still overruns; drogue braking is not modeled offline. Both changes await
approved live verification; no live controls were changed for this work.
