# STS TAEM on a Heading Alignment Cone (2026-10-21)

## Why

Live run `runs/20261021T103239Z-deorbit` overran RW27.

- MM304 targeted the TAEM energy law's straight-in reach at full drag (38.7 km site distance). It handed over at the Mach 2.85 band edge, 41 km west of RW27 and heading east (180° heading error).
- The energy law priced that inlet as a 459 km path, flew at k = 0 with no HAC, and reached 1 km out still 1096 m high.

Replays:
- `ShuttleSim/scenarios/sts-kerbin-20261021-mm305-handoff.ini` (MM305 handoff)
- `ShuttleSim/scenarios/sts-kerbin-20261021-mm304-45km.ini` (MM304 at 45 km)

## What changed

`taemHeadingAlignmentCone` (on in `Configuration/sts-live.json`; it takes precedence over `taemEnergyLaw`) runs `CLanding/guidance/taem/hac_law.inc`. Its structure follows Moore, NASA TM-104744.

- **Phases.** S-turn → acquisition → heading alignment → prefinal (TGTRAN).
- **Spiral HAC.** R(ψ) = RF + 93 m·ψ² (R2 = 0.093 ft/deg²). The HAC is entered when the vehicle is within 1.1·R of the tangent turn angle.
- **Range-to-go.** Measured along acquisition arc + tangent + spiral arc + final.
- **Energy reference.** Integrated backward from the Final station on the polar, with per-Mach measured lift and drag ratios (EMA, frozen while the speedbrake is out).
  - Speed schedule: equilibrium glide at the front-side incidence where L/D is 0.9 of its maximum.
  - Within 1.5 km of the station, the altitude reference is the outer glide slope; it blends back to the schedule by 5 km.
  - Turn cost is priced from the polar at the banked lift.
- **Side and RF.** Chosen to null predicted energy error. In the turn, the spiral adjustment moves RF at ≤ 15 m/s and never tightens below what the measured speed can turn at 55° bank.
- **Speedbrake.** Subsonic PI on the energy error.
- **Final handoff** reuses the energy law's admission (Final's capture gate held for 1 s).
- **Fixes required for Final to own the vehicle:**
  - the TAEM executive treats the HAC law's runway line as the selected path;
  - the pavement-intercept line capture is enabled under the HAC law.

MM304 under the HAC law budgets its handoff on `mm305_hac_handoff_state`: the point of MM305's own reference where the band-centre Mach is reached. MM304 also:
- delivers to the approach side of the runway end;
- feeds the live aero observer;
- has the entry-policy bank solver hand off at the band centre.

## Findings

1. **The KSP STS cannot hold 160 m/s on a 20° glide.**
   - Wing loading is ~990 Pa, so at 3 km altitude and 160 m/s CL ≈ 0.09, L/D ≈ 1.5, and the steady glide is ~31–38°.
   - A steady 20° glide happens only at ~85–105 m/s.
   - Delivering 160 m/s at −33° (the TAEM equilibrium slope) made Final pull up and land 1.2 km short.
   - Delivering at the 108 m/s steady-glide speed left Final no preflare energy ("specific-energy envelope missed").
   - The working delivery: on the 20° line at the configured station speed plus a 300 m energy surplus (~180 m/s). A shortfall at the station is unrecoverable; a surplus is Final's speedbrake trim.
2. **The HAC must be flyable at the measured speed.**
   - Entering at 270–330 m/s with RF 2.2–3 km (60° bank needs 4.3+ km) cut inside the cone; ψ wound up by whole turns and the energy reference jumped tens of km.
   - Fixes: ψ is monotone in the HAC, and the RF floor from measured speed is approached at the rate limit.
3. **MM305 flies the reference closely once on it.**
   - From ψ ≈ 110° to the station, energy error stayed within about ±150 m.
   - Station delivery: h 3286 m vs 3276 m reference.

## Best MM304→MM305 handoff (from the C reference, mass 25.2 t)

| Path to 9 km station | Mach | h above runway | V | E = h + V²/2g |
|---|---|---|---|---|
| 24 km | 1.47 | 15.9 km | 435 m/s | 25.6 km |
| **40 km** | **2.53** | **17.6 km** | **751 m/s** | **46.4 km** |
| 56 km | 3.19 | 23.2 km | 959 m/s | 70.1 km |

Recommended handoff:
- **Energy:** Mach 2.5 at E ≈ 46 km. MM304 at its entry incidence reaches Mach 2.5 nearer 25–26 km altitude. Energy height, not altitude, is the contract: 26 km at ~600 m/s is equivalent.
- **Range:** ~40 km of path to the station, i.e. ~49–52 km from the threshold for a straight-in.
- **Geometry:** on the approach side of the selected end, so the HAC turn is small (straight-in, ψ ≲ 90°). An overhead HAC from a supersonic inlet needs RF far above the 2.2–3 km band.

## Replay results (commit b9842dd)

| Scenario | Before | After |
|---|---|---|
| MM305 handoff (live failure state) | lands off runway (RW27 overhead, no HAC) | **COMPLETE**: RW09 straight-in with S-turns, touchdown 64.5 m/s, sink 2.7 m/s, stopped on runway |
| MM304 at 45 km | — | on runway, centreline, 49 m/s, sink 4.4 m/s (fails the 3.0 m/s sink gate) |

## Open items

- **Final flare.** On this airframe Final starts "Preflare" at ~2 km height and bleeds speed toward the stall before touchdown (45–60 m/s). The hard touchdowns are there, not in TAEM.
- **MM304 bank in ShuttleSim.** MM304 holds 0° bank in ShuttleSim (no worker bank plan), so the new handoff target is only exercised live. Live MM304 S-turned with ±10–40° bank.
- **Deorbit ranking.** `deorbit_sts_handoff_target` still uses the energy-law reach, and only under `stsTrajectoryDelivery`.
- **Runway-end choice.** MM305 re-chooses by energy cost rather than inheriting MM304's approach-side choice. That is correct when MM304 misses its target, but it can choose an overhead HAC from a supersonic inlet.
