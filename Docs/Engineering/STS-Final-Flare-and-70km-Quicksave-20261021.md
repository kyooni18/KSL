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

## Supersonic TAEM redesign: the drag-energy phase

MM305 under `taemHeadingAlignmentCone` now starts in a **drag-energy phase** when it takes over above Mach 1, and hands to the HAC acquisition law at Mach 1 (or once energy is down to the gate's).

| Piece | What it does |
|---|---|
| Gate | Where the HAC reference first reaches Mach 0.95, about 17 km of path from the Final station at about 16 km of energy. |
| Reference beyond the gate | Kept as the **subsonic** glide: the shallowest subsonic slope (about 0.49 m/m), with the surplus held as altitude. The equilibrium reference climbed at the transonic drag rise (1.2 to 2+ m/m) and read a vehicle just through Mach 1 as 7 to 17 km short. |
| Equilibrium profile | Still kept (`h_eq`, `v_eq`), but only for MM304's handoff point (the reference's Mach 2.5 point). Without it MM304 found no handoff target and delivered Mach 2.5 overhead the field. |
| Target | The energy the subsonic law needs for the path from here. That path is the turn to the nearer HAC's tangent, priced at the gate speed rather than the current supersonic one, plus the line and the HAC, plus the HAC bank cost. |
| Incidence | Sheds the surplus over the path to the gate, up to the vehicle's supersonic limit (`maximumAngleOfAttack`, 28 deg) and 2.5 g. The trim ceiling starts at the subsonic 15 deg, where supersonic flight sheds only about 1.1 m/m. Slewed from MM304's attitude at 2 deg/s, and bank at the TAEM roll rate. |
| Bank | The lift the descent to the gate altitude does not need. Steers along the centreline ahead (past the gate, for the HAC tangent), with +-45 deg S-turns while even full drag cannot shed the surplus. |
| Runway end | Held. A supersonic start keeps the end being approached, because the HAC prediction prices each end at a 25-50 km turn radius. After the drag phase, end selection does not reopen. |
| HAC capture | Now also requires the heading to be within 60 deg of the HAC tangent direction. After the drag phase a vehicle within 10 % of the radius but flying round the wrong way was "captured", and held wings level for 80 s. |
| Map | The HAC law publishes the line to the tangent and the spiral to the station as the reference trajectory, not just the remaining arc (empty at psi 0). |

### Results

| Scenario | Before | After |
|---|---|---|
| 70 km quicksave, 2x pacing | crash (loop beyond the field, then RW27) | **pass**: RW09 straight-in, Mach 1 about 18 km out at +1.2 km over the path's need, Final at 5.9 km; touchdown 62.9 m/s, sink 1.58, 387 m past the threshold. Before the acquisition turn was priced into the drag path: 736 m short. |
| MM305 from that handoff (`sts-kerbin-20261021-quicksave-mm305`) | - | **pass**: 65.5 m/s, sink 2.62, 197 m past the threshold |
| MM305 live-failure replay (Mach 2.85, 38.7 km out) | pass by luck (RW27) | fail, 3.7 km short: the field is overflown supersonic, and the turn-back costs more than is left |
| MM304 45 km, max rate | pass 1.51 | fail, 2.4 km short (sink 2.48): the max-rate handoff is late (32 km), same overflight |
| L5 golden | pass 56.0 / 1.71 | pass 56.0 / 1.71 |
| L7 golden | pass 76.5 / 1.59 | **fail**, 1.77 km short |
| final-aligned | fail | fail (pre-existing) |

**L7.** The scenario's plant collapses from the logged 27.5 deg AoA to -1 deg in the first second, and attitude recovery triggers at 0.4 s in both versions. Whether recovery releases within 15 s depends chaotically on the first tick's roll target (0.6 deg bank difference). The committed code released at 11 s; this version times out, and bounded recovery flies wings level to Mach 0.34. The drag law's own commands (stepped or slewed) do not change the outcome.

**Open.** On an earlier paced 70 km run, Final was held by `terminal_preflare_alignment_valid` ("runway capture gate") from the 9.5 km station to 4.4 km. The prefinal weaves a few hundred metres across the centreline, and Final then flew a 30 deg dive into a short preflare. The final run handed over at 5.9 km.

## Attitude-recovery release: trim is not saturation (L7)

L7's start transient (the plant drops from 27.5 deg to -1 deg AoA in a second) triggers recovery in every version. `KSP_LANDER_RECOVERY_TRACE=1` shows the vehicle settled from UT 6580: roll < 1 deg, roll rate 0.05 deg/s, AoA 8-8.5 deg for the 10 deg recovery target. But holding that at Mach 2.5 takes a steady 0.36-0.49 elevator. Release counted |elevator| >= 0.35 as saturation, and the braking exemption flickered with a +-1.8 deg/s pitch wobble. So the 2 s stable timer never held, and recovery timed out at 15 s. The committed code escaped it by chance.

Release now measures the elevator trim during recovery (3 s mean). Pitch saturation counts only as deflection 0.35 or more away from that trim, or at the stop (0.95). The trigger logic is unchanged.

| Scenario | Before | After |
|---|---|---|
| L7 | recovery timeout, 1.77 km short | recovery releases at 14 s; on runway, 70.7 m/s, **sink 3.31 (fails the 3.0 gate)** |
| L5 | pass 56.0 / 1.71 | pass 67.4 / 1.61 |
| quicksave MM305, 45 km, MM305 replay, final-aligned | as above | unchanged |

L7's remaining failure is the late Final handoff. The HAC is captured from 860 m inside the spiral at 16 deg to go, and flown wings level to an exit 470 m off the line. The runway-line law then closes that offset with a ~25 s time constant, so Final's pavement-projection gate holds until 5.2 km. Final then dives steeply (87 m/s sink) into the flare.
