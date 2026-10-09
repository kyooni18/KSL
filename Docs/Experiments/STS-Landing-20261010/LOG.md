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

## 5. Next steps
- Touchdown speed 65 < 75 target: shorten shallow-glide segment / steeper OGS energy.
- TAEM: replace provisional acquisition + Mach-2 direct fallback with an energy-managed law that dissipates the measured excess (S-turns/HAC sizing) and holds AoA ≤ 15°.
- Repeatability across station dispersions, then full chain from live-derived 30 km state, then live.
