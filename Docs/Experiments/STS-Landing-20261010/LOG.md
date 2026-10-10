# STS terminal landing campaign — 2026-10-10 experiment log

Goal: repeatable STS landings on KSC Runway 09 (entry → MM304 → MM305/TAEM → Final → touchdown → rollout), modelId `STS`.

## 1. Measured baseline (live, 2026-10-09, checkpoint STS-OMS-72km-20261009-v2)

Seven live runs (FlightLogs 10-46-41, 11-27-40, 12-33-31, 13-02-48, 13-27-33, 13-52-53, 14-21-19):

- MM304 is repeatable: TAEM handoff at ~27 km, 870–880 m/s (M2.9), 59 km before the site, cross ≈ −2 km, course ~112°. Energy height ≈ 66 km against 59 km of range — large *excess* energy for a straight-in.
- TAEM fails every time, differently:
  - v7/v8/12-33: straight-in dive (FPA to −34°), over the threshold at ~12 km altitude, late overhead turn; v7 then mushes at 45–60 m/s, AoA 15, wings level, drifting off (abort at 1.2 km alt); v8 reaches Final from a bad state and splashes 7.7 km off.
  - 13-02/13-27/13-52: *direct runway fallback* opened at ~21 km/M2.4 and commanded AoA 0°, bank −45°: Mach-2 dive to 8 km → attitude recovery → abort.
  - 14-21: AoA 18° command at TAEM start → roll departure (−166°) → abort.
- Offline full-orbit sim (`dbgfull-20261009T114631Z`) crashed nose-first 51 m off the runway edge at 125 m/s sink.

## 2. Why the sim could not reproduce quickly

Lockstep ShuttleSim waits up to 180 s wall per MM305 plan request and TAEM re-requests every 2 s sim; the terminal shadow ensemble replays native routes inside acquisition. A 700 s TAEM run did not advance past 0.1 s sim time in 5 min. Live flies on while plans are pending, so the barrier also makes sim≠live.

## 3. Aero model vs live (Tools/fit_sts_terminal_polar_live.py)

Live CL/CD (S=250 m²) from the seven flights vs `sts_aero_live20261009.csv`:
- Mach 0.4–0.9, AoA 0–6: live CD ≈ 0.05–0.07, table 0.08–0.15 (table 1.5–2.5× too draggy).
- Lift Mach 0.3–0.9: live 0.7–0.9× table; Mach 0.1–0.2 higher (KSP lift-Mach curve).
- Fitted per-Mach polar: CD0 ≈ 0.052–0.061 to M0.9; L/Dmax ≈ 3.9 (M0.2), 3.3 (M0.4), 2.8 (M0.5), 2.0 (M0.7), 1.5 (M0.9), 0.75–0.9 (M1.1–1.5).
- New table `ShuttleSim/reference-model/sts_aero_live20261010.csv` (+report), profile `sts-live-20261010` (sim plant + guidance). Provisional, uncertified.

## 4. Final/flare findings (engageFinalTest from an aligned 8 km / 2912 m / 160 m/s / −20° station)

| change | result |
|---|---|
| baseline, old table | admission rejected: predicted touchdown 57.7 m/s (< 75) |
| arrest cap 0.15 g → 0.5 g | predicted 70.7, still rejected |
| + live-fitted table | admitted; touchdown on runway at 47.5 m/s (preflare arc ended ~150 m high → 30 s float) |
| arc as sink-vs-height profile with push-back | phugoid balloons, 39 m/s, 12 m/s sink — **rejected** |
| arc room uses residual lag (≥1 s) instead of full latch response | arc ends at SGS height; balloon at 8 m → 46 m/s |
| mains-first pitch floor capped at 1.1 g incidence | **landed: 64.9 m/s, 0.36 m/s sink, 323 m along, 0.01 m cross, stopped 1130 m** |

Physics (Tools/sts_terminal_bench.py point mass): on a 20° glide the vehicle settles to ~90 m/s (table drag) / ~105 m/s (live drag) regardless of station speed; with a tight flare touchdown ≥75 m/s is reachable; steeper OGS (26–28°) adds ~10–20 m/s.

Live v8 Final showed the same float signature (preflare ~600 m, 48 m/s at 160 m).

## 5. TAEM energy law (commits a2d254b..1707db8)

`guidance.taemEnergyLaw` (CLanding/guidance/taem/energy_law.inc) replaces MM305 acquisition with a
predictive drag-modulation law: level k in [0,1] (supersonic AoA 15→8, subsonic q 3500→11000 Pa) solved
by bisection on a point-mass predictor over the planned segments; pattern chosen among straight-in,
direct HAC (either side, R 2.2–10 km, flyable-radius floor) and overhead loop; HAC spiral resizes R in
flight; handover to Final after the outer-gate + delivery contract holds 1 s.

Full chain (engageReentry, profile sts-live-20261010) from the seven live MM305 handoffs + v7: all on the
runway after 1707db8, but touchdown 59.8–66.9 m/s (below the 63.75 band).

## 6. Flare: short shallow glide, no-push arc (b2f6140)

| change | handoff cases (7) | note |
|---|---|---|
| baseline (SGS 25 m) | 59.8–66.9 m/s | ~7 s at 25 m bleeds 88→66 m/s |
| SGS 12 m (guidance + admission plan) | 66–67, but 11-27-40 44 m/s / 8.3 m/s sink | late hard pull (5.4 s incidence lag) overshoots, arc ends 130 m high, push back, second balloon |
| + arc closed on sink one residual lag ahead | 10-46-41 also balloons | root cause: on-arc test `sink<.92 glide_sink` chatters; arc pushes −0.3 g toward the design sink, builds a 45–50 m/s dive |
| + never push on the arc (arc_accel ≥ 0 above SGS sink) | all clean, 61–66 m/s, sink ≤ 2.9 | slower: approaches arrive at latch ~110 m/s |
| + outer-glide speed floor 1.3→1.5 v_td | 67.0–68.7, sink ≤ 1.44, but 54–104 m along | too close to threshold |
| + aim bias 250 m | 63.7–66.5, 257–329 m along | float costs speed |
| **aim bias 150 m (committed)** | **66.0–66.7 m/s, sink ≤ 1.75, 199–222 m along, cross ≤ 5.2 m** | Final-only stations 66.1–70.9 m/s |
| arrest cap .35→.5 g | worse (64–67 Final-only) | rejected |

Runner: with `taemEnergyLaw` the HAC requirement is committed path + HAC completion (law sizes its own HAC);
`Configuration/sts.json` now enables the law (637d5ee). **7/7 handoff cases succeed on all runner criteria.**

## 7. Open issues
- v7 (pre-TAEM 30 km, M3.65 — more energetic than the current M2.9/27 km MM304 handoff): k=0 supersonic plan
  is an overhead loop with a near-full-circle HAC (40.8 km to go, predicted 6.6 km short); at M1 it flips to
  a direct HAC (13.7 km, 4.6 km excess). No intermediate pattern; spiral (±250 m/2 s) cannot absorb it; Final
  admitted at 4.4 km / 1.9 km high → lands 1669 m along, 30 m cross (rollout off the centre band).
  Faster spiral (±500) made it worse (2342 m) — rejected.
- `make test`: pre-existing failures unchanged (mm305_admission link error, mm305-feasibility non-vacuity,
  touchdown-quality). touchdown-quality improved from a gear assertion abort to an on-runway 1.58 m/s
  touchdown at 91.75 m/s (default model, limit 75).
- Live: KSP running but kRPC server not listening (ports 50000/50001 closed) and screen control of KSP was
  not granted, so no live flight with the energy law yet.

## 8. TAEM robustness and dispersions (f609663..61f8b30)

12 handoff dispersions around 12-33-31 (±1.5 km alt, ±60 m/s, ±8° heading, ±11 km range, ±5 km cross, ±2° FPA):
- baseline 9/12: lo / steep overflew the runway by 13 km (HAC met at its exit took the ~0 arc, not a circle); slow
  reached the station 2.2 km high, dove 30°+, never admitted to Final, crashed at 121 m/s.
- f609663 full-circle decision on exit entry; a8b989e S-turns when k=1 with excess range; a5fe47c added circle may
  tighten to 1600 m (global 1600 m floor rejected: ~32 m touchdown offsets); e981c17 size from airspeed;
  61f8b30 Final lateral position loop 0.25→0.35 (touchdown cross roughly halved, no overshoot).
- now 11/12 (lo 61.6 m/s), live handoffs 7/7 with |cross| ≤ 2.7 m, Final-only stations unchanged.

0.5° dive-AoA experiment (terminal_solver.c): MM305 feasibility A/B identical (0/5 qualified either way), full chain
bit-identical → reverted; patch kept as rejected-dive-aoa-0p5.patch.

## 9. Entry-interface full chain (offline)

- sts72km-run9 entry interface: MM304 range margin positive throughout (+559 km at 10202), bank saturated at the 40°
  restriction → over the runway at M4.3, lands 60–65 km long. Not representative of the current OMS checkpoint.
- New scenario sts-live-oms72-entry-20261009 (the 2026-10-09 checkpoint): sim MM304 hands off 27 km out (live 46–59 km).
  Live/table aero at AoA 18–30: CD ×1.51 (M2), ×1.16 (M3), ×1.09 (M4), ≈1 (M5–7); CL ×0.90–0.96 (M2–4).
  Provisional profile sts-live-20261011 scales M1.6–7.1 rows; handoff moved only 29→27 km, so the gap is MM304 targeting.
- Close-in supersonic handoff exposes a TAEM bug: a 10 km HAC met at its exit at M2.3 (flyable radius 35–48 km) falls
  through to the Final line and dives over the runway. Overhead-loop fallback drafted (not committed, not regression-tested).

## 10. Live (2026-10-10)
kRPC came up; L1 started from STS-OMS-72km-20261009-v2 (Steam save, SHA identical to the certified profile),
taemEnergyLaw on, runway 09 only, profile sts-live-20261010, KSP_LANDER_UNPOWERED_ONLY=1.

### L1 result (FlightLogs/2026-10-10T04-39-07Z)
Entry → MM304 → TAEM (handoff UT 2164, 27.1 km, 873 m/s, 58 km out) → Final (UT 2425, 5.7 km, cross −20 m,
121 m/s) → flare → touchdown ≈UT 2488, ~59 m/s, ~3.4 m/s sink, ~552 m along → rollout stopped on the runway
(1831 m along, 6.4 m cross). Physically landed, but outside the runner band (speed < 63.75, sink > 3).

Flare, live vs sim replay of the same Final state (sim: 67.4 m/s, 1.5 m/s sink, 426 m):
- Live preflare latched ~350–440 m wheel height (sim direct-control: trigger 817 m, arc starts ~250 m).
- Live pitch loop tracks AoA ~2.5° below command throughout the pull (FCS diagnostics pitchError 2.4–2.7°,
  elevator 0.2–0.5, not saturated; approach trim integrator ki 0.004 climbs 0.12→0.31). Sim lag ≤0.5°.
- Late arc → 1.8 g pull at 20 m → vs +1.8 m/s balloon at 14 m radar; then the mains-first attitude floor
  (lift capped at 1 g) holds the level float ~8 s at 12 m radar ("sink −0.2 / −3.5") and bleeds 78→59 m/s.
- Sim with a stiffer pitch plant (stiffness 0.92/kPa, trim AoA 1.6°; live elevator-vs-AoA slope ≈0.07/°)
  reproduces the float only with faster trim (ki 0.025: 62.5 m/s/401 m; 0.2: 57.8/519 m); nominal ki 0.004
  lands 73.8/102 m. identify_stsn_attitude.py on STS logs gives pitch R² ≈ 0 (unusable).
- Rejected: floor lift cap reduced by the sink deficit when floating — off-runway/hard (77 m/s at −2 m,
  66 m/s at 5.6 m/s sink, 48 m/s), nominal sink 1.5→3.9. The floor carries the last metres.

### L3 result (FlightLogs/2026-10-10T05-12-03Z) and flare fix
- MM304 → TAEM handoff UT 2163, 27.1 km, 872 m/s, 59.3 km (L1: 2164/27.1/873/58 — repeatable); overhead loop;
  Final at UT 2413, 6.2 km, cross +9 m.
- Same pitch-lag signature as L1 (AoA 2–3° below command through the pull, trim 0.12→0.24), pull from ~240 m at
  38 m/s sink, 1.6 g only at 34 m → first main contact 221 m short of the threshold at 87 m/s, bounces (one at
  23° pitch / 11.6 m/s sink), rollout stopped on the runway ~1.2 km along. Run flags "touchdown outside runway
  envelope" and never completes.
- Sim with a live-like direct plant (att-live: pitch ctrl 23.3°/s²/kPa, stiffness 1.63/kPa, trim AoA 1.6°)
  reproduces it (L3 replay lands 23 m short at ki 0.004).
- Approach/flare pitch-trim ki 0.004 → 0.025 (TAEM's live value): L1 replay 70.3 m/s/0.73/320 m, L3 69.9/1.27/269 m;
  plant bracket (stiffness 1.2/1.63/2.2) 5/6 vs 3/8 at 0.004; 7 handoff chains (direct FCS) 6/7, 12-33-31 short
  (−382 m; −838 m at 0.004 — pre-existing): admission-trigger latch at 1357 m freezes an unconverged aim (−1154 m).
  Unfreezing the aim while latched rejected (speed loop is off when latched: 49 m/s, 5 m/s sink).
  The STS-N borrowed plant is not representative (baseline also fails).

## 11. Adaptive flare (guidance.adaptiveFlare, CLanding/guidance/final/adaptive_flare.inc)
Predictive flare replacing latch/arc/SGS/floor when the flag is on (default off; flag off is bit-identical):
- Response ID every Final cycle: decayed (5 s) least squares of d(aoa)/dt = (cmd + bias − aoa)/tau, firm prior
  tau 0.8 s, weak offset prior. A gain term was dropped: over a flare it is collinear with the offset and both ran
  to their clamps (chattering inverted command). An offset/lag split must lean on physics: a steady
  command/incidence gap with the incidence still is trim/stiffness, not lag.
- One law shared by predictor and execution: sink-vs-height profile (constant-decel arc to the shallow-glide sink,
  then sink proportional to height to 1 m/s at 1.5 m), read 0.6·tau ahead (only building deceleration), loop
  TC = tau, capped at the sink when the flare started, never a push below the shallow glide except out of a real
  float, offset-compensated command, mains-first floor ≤ 1 g.
- Solver: 6 arcs × shallow-glide heights {10,14,18} m on the forward predictor, scored on the identified response
  and on a weaker one (tau×1.3, gain 0.8, offset −0.3), worst counts; touchdown before the threshold near-hard;
  speed below 66 m/s weighted. Onset when the chosen plan's lead-read sink reaches its arc, below 600 m; plan locked
  once engaged. Outer-glide aim corrected by integrating the forecast touchdown error before onset.
- Rejected along the way: overdrive lag compensation and a 1 s sink loop behind a 2-3 s lag (limit cycle);
  two-sided tracking up to the profile (dives from 900 m); full-pull lead extrapolation (unloads, sink returns);
  short shallow-glide heights 5/8 m (stiff plants trail a rising command by 3°: hits at arc sink); an incidence
  integral on the command (fights the online offset estimate: 10-19 m/s sink); a −0.08 g push below the
  shallow glide (8/12).

Sim (direct FCS, live-like plant att-live and ±stiffness, trim ki 0.025 and the old 0.004):
| set | fixed flare | adaptive |
|---|---|---|
| L1/L3 live Final states × 3 plants × 2 trim gains | 7/12 | 10/12 (L1 nominal 62.4 m/s; L1 stiff/ki0.004 102 m short) |
| 7 live handoff chains | 6/7 | 5/7 (13-02-48 4.9 m/s sink; 12-33-31 51 m/s) |

### 11a. Incidence surge (second-order pitch momentum)
The trace of the L1 nominal float (62.4 m/s) shows the incidence still rising ~0.5 s after the command drops at the
shallow glide (5.2 -> 5.9 deg with the command at 4.2): pitch-rate momentum that the first-order lag model does not
carry. 13-02-48 shows the mirror (incidence still falling after the command rises: sink rebounds to 4.6 m/s).
Change: the law reads the pull at aoa + (measured aoa rate - model rate)*0.5 s (signed, ±2 deg), only below
SGS+5 m, and below the shallow glide may unload to -0.08 g in proportion to a positive surge.
| variant | matrix | ki0.025 subset | chains |
|---|---|---|---|
| m9 (before) | 10/12 | 5/6 | 5/7 |
| surge 0.5 s everywhere | 9/12 | | |
| surge 0.25 s everywhere | 10/12 | | |
| surge near SGS only | 9/12 | | |
| + unload below SGS on surge | 8/12 | 6/6 | 5/7 |
| + signed surge (kept) | 11/12 | 6/6 | 5/7 (12-33-31 51 m/s; 13-02-48 4.3 m/s sink) |
Applying the surge on the whole arc weakens the main pull (short landings on soft plants).

## 12. Codex offline continuation: predictor/execution consistency

User requires offline FLARE refinement before L4, and stopping before its launch.
Foundation recall succeeded; Jev was not exposed. Another session committed the signed
surge experiment as b3dfb34; its remaining uncommitted AF_SLEW=3 edit was preserved.
All experiments here used an isolated source copy and backend, without touching live KSP,
protected saves, shared backend builds, existing flight restrictions, or unrelated edits.

The pending slew edit changed only the predictor: execution still used FLARE_SLEW=5.
Tested shared slew rates, then targeted the observed 13-02-48 command drop and sink rebound.
The predictor discards measured pitch momentum after its first step; execution uses it.
Also, the sink profile has a slope discontinuity at SGS, causing a deceleration reduction.

| isolated variant | matrix | chains | decision |
|---|---:|---:|---|
| shared slew 3°/s | 10/12 | 4/7 | reject; 13-02-48 improves to 2.65 m/s, other cases float |
| shared slew 4°/s | 11/12 | 4/7 | reject; 13-02-48 still 4.32 m/s |
| shared slew up 5/down 3°/s | 9/12 | 5/7 | reject; 13-02-48 1.57, 14-21-19 3.18 m/s |
| predictor momentum residual decays over 0.5 s | 9/12 | 5/7 | reject; 13-02-48 1.85, short/slow matrix regressions |
| C1 Hermite profile join over SGS…SGS+10 m | 10/12 | 4/7 | reject; 13-02-48 2.62, two other chains too slow |
| falling-incidence surge ×2, existing ±2° cap | 10/12 | 5/7 | reject; 13-02-48 improves to 3.53, still hard |

Each score is the runner's unchanged complete success check, including runway contact and rollout;
a 64 m/s pass does not meet the planner's stricter 66 m/s objective. No code variant has yet
met both ≥10/12 and ≥6/7. Adaptive remains off by default. Source patches, portable inputs,
exact run IDs, outcome fields, and provenance are retained in `Codex-Flare-Offline/`.

### 12a. Further response-model experiments (all rejected)

| isolated variant | matrix | chains |
|---|---:|---:|
| falling-incidence surge ×3, ±2° cap | 11/12 | 5/7 |
| above, falling correction cap −3° | 11/12 | 5/7 |
| height look-ahead 0.6·tau, sink lead unchanged | 9/12 | 2/7 |
| pessimistic tau floor 1.5 s | 10/12 | 3/7 |
| pessimistic aircraft response, nominal controller compensation | 0/12 | 0/7 |
| above, gain fixed at 1; lag and offset uncertainty only | 10/12 | 5/7 |

The 13-02-48 onset already identifies tau=1.62 s, so the pessimistic 1.5 s floor
cannot address that case. Enlarging the falling correction cap changes no outcomes.
The current pessimistic predictor alters both response and controller compensation;
splitting them reveals that permanent 20% gain loss with frozen controller estimates
is excessively pessimistic. Removing gain uncertainty recovers results but still fails
the chain threshold. Neither model change is adopted. All 228 offline replays are retained.
Shared source/builds remain unchanged, including the other session's pending AF_SLEW=3 edit.

User subsequently authorized L4 from the full 86 km orbit, replacing the earlier stop-before-L4
instruction. L4 will use the existing fixed-flare/ki0.025 live backend with adaptive off,
normal orbital planning/deorbit admission, and unchanged flight restrictions.

## 13. L4 full 86 km orbit launch (user authorized)

Initial save is an exact new copy `STS-L4-full86-20261010T062914Z` of the Steam `86km.sfs`.
Protected original SHA-256: `4ea194a07422eece54135f2f7586dc92b556b2c9380b785c4754ef868eb662a6`. The `/Applications/.Misc/LMPKSP`
protected path was absent at preflight; it was not created or modified. Exactly one KSP
instance (PID 6313) and no prior headless runner were found before launch.
Backend SHA-256: `8beafeebca9aa957a167469a413d0553e255204fb229802f72f8fbbbb3530c27`; existing fixed flare + trim ki0.025,
model profile sts-live-20261010, modelId STS, runway 09 only, adaptive explicitly false.
Full orbital mode uses normal createPlan → qualification → engage, rather than reentry-only.
No unqualified-continuation override is inherited. Original campaign no-crash-guard policy,
1× preentry physics warp, unpowered approach restriction, and 3600 s wall limit retained.
Runner PID 59125; immutable backend/config pin enabled; launch details in
`L4-full86/launch.json`, live progress in `L4-full86/live.log`. Launch is not landing evidence.

L4 startup confirmed: copied save loads at UT81.2, orbit86.7km, vesselSTS.
Orbital admission passes executionQualified=true; selected burnUT1161.6, deltaV31.49m/s,
strict=false, nominal=true, recovery7/15, downstream shadow TAEMrange12.9km. Normal
entry-stage policy admits execution and guidance engagement is logged. Touchdown/rollout
are not yet verified. Immutable campaign identity:
`ddcb9a0d4ff48e57-8beafeebca9aa957-1ddd6e9ced45dd2b`.
