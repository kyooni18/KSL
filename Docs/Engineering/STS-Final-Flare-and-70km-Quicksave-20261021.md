# STS Final flare fix and the 70 km quicksave tests (2026-10-21)

Commits 2c61cca (Final) and aa18ca9 (scenario, pacing, diagnostic). This follows
[STS-TAEM-Heading-Alignment-Cone-20261021.md](STS-TAEM-Heading-Alignment-Cone-20261021.md).

## 70 km quicksave as a scenario

KSP was not running, so the live checkpoint was flown in ShuttleSim instead:

- **Source.** `saves/shuttletester/quicksave.sfs`: STS sub-orbital at 70,278.8 m, lat 0.1344, lon 170.1174, UT 146852.5347.
- **Scenario.** `ShuttleSim/scenarios/sts-kerbin-20261021-quicksave-70km.ini`.
- **State.** Taken from the save's orbit elements and placed at the save's body longitude. It matches the saved altitude to 0.3 m.
- **Mass.** 25.47 t, from the live flight. The save's part-mass sum is not usable.

**Pacing matters.** At `--rate max` (about 114x) MM304's wall-clock prediction worker replanned only 3 times in 650 s. The handoff then landed about 20 km from the field instead of 50 km. `run_guidance.py` now honours `SHUTTLESIM_RATE`. At 2x, MM304 replans about 147 times and hands off on target: 49.3 km before RW09, 553 m off the centreline, Mach 2.5.

```bash
SHUTTLESIM_RATE=2 .venv/bin/python -m ksl simulate ShuttleSim/scenarios/sts-kerbin-20261021-quicksave-70km.ini --from mm304 -p sts-kerbin
```

## Final flare

| Finding | Fix |
|---|---|
| The admission preflare trigger includes an energy term: start before the projected speed bleed reaches the 85 m/s floor. That term can put it kilometres up (2535 m here; 1820 m in live flight 4, see commit 6db8a16). Latching froze the aim point and the outer-glide speed loop, so Final dived 2 km down a stale aim. | The latch only arms the arc. Until the measured sink reaches the arc onset (`terminal_arc_started`), the outer glide keeps the aim and the speed. |
| Under the HAC law, the outer glide dived to hold the precomputed station speed schedule (112 m/s) that this airframe cannot afford. The live touchdown forecast already said "short" from the start of Final. | Under `taemHeadingAlignmentCone`, `range_short` (live forecast) is enabled and drops the schedule. The forecast error is also integrated into the aim (`terminal_aim_shift`). |
| 45 km replay: arrested at 4 m, floated from 69 to 60 m/s, touched down at 3.4–3.6 m/s. | The final flare closes on the sink 0.5 s ahead, using the measured lift. A full 1 s lead eased the MM305 replay's pull at 11 m (4.1 m/s), and 0.7 s gave 3.2 m/s; 0.35–0.5 s passes both. A settle floor from the measured drag and the learned 1-g speed caps the float, at most 2 m/s. |

### Results

| Scenario | Before | After |
|---|---|---|
| MM304 45 km | on runway, sink 4.4 (fail) | **pass**: 60.0 m/s, sink 1.51 |
| MM305 handoff (live failure state) | pass, sink 2.7 | **pass**: 67.2 m/s, sink 2.63 |
| L5 golden | pass, 56.3 / 1.60 | pass, 56.0 / 1.71 |
| L7 golden | pass, 82.0 / 1.01 | pass, 76.5 / 1.59 |
| final-aligned | fail, sink 3.14 | fail, sink 3.09 (pre-existing) |

`make -C CLanding test` passes; pytest: 157 passed.

## Open: TAEM cannot fly a real supersonic handoff

From the 70 km quicksave at 2x pacing, every run crashed in MM305:

- **Arrival.** MM304 reaches Mach 2.5 at 26.3 km, not the HAC reference's 17.4 km. That is 6.8 km of energy over the reference at the measured path.
- **Why the reference can't be met.** The table's supersonic best L/D is only 0.8–0.9. The reference is an equilibrium glide at that L/D and sheds about 1.3 m of energy per metre. MM304 flies a lofted, low-q path that sheds about 0.5 m per metre, so the surplus grows with time:

  | Handoff Mach | Surplus over reference |
  |---|---|
  | 2.85 | +8.5 km |
  | 2.5 | +6.8 km |
  | 2.15 | +13.3 km |

  Gating the handoff on energy, in either direction, did not help, so the gate was removed. Only the logged excess remains (`HAC energy excess` on the handoff line).
- **What MM305 does.** It ends the S-turn at its 15 km range-to-go floor within 40 s; at 700 m/s the turn radius is about 50 km. It then flies the 1° minimum AoA, which dives instead of dissipating. The surplus grows to 20–30 km and the vehicle overflies the runway.
- **Max-rate replays.** These sometimes reach Final, but by luck: TAEM releases the vehicle late and low. One run released it 4.6 km out at 1.5 km altitude and touched down 416 m short.

The supersonic part of MM305 needs its own design for a vehicle with L/D below 1:
- dissipate with high AoA and bank, as entry does, rather than diving to a reference altitude;
- or continue MM304-style flight to subsonic and start the HAC there.
