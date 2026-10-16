# MM304 energy and late Final handoff — 2026-10-16

Baseline: `runs/20261016T084526Z-sts-10deg-pad-verify-land2` (85 km / 10°; it landed, but the Final handoff came late and slow).
The scenario `.ini` files here are surface-state starts taken from that flight's telemetry. ShuttleSim aero is optimistic compared with live: from the same MM305 state the sim arrives with surplus energy and lands long. These scenarios are therefore regression and trace tools, not acceptance gates.

## Findings (baseline)

1. **MM304:** the live replan reported "short even at 0°" for the whole entry, so the vertical demand was all of the available lift. The lateral allocator (`entry_lateral.c`, binding branch `atan2(lateral, vertical)`) still flew 40° of purely lateral bank from 57 km to 44 km. It nulled 100 km of crossrange to the MM304 aimpoint and overshot to +22 km in the runway frame. MM305 started at drag level k = 0, right at the edge of maximum range.
2. **MM305 final line:** inside the 9 km station, path-to-go is 0, so `tel_solve_k` always returned k = 1 (maximum drag). The vehicle joined the line energy-balanced (4020 m energy height against a 4021 m reference). It then bled energy for about 90 s while Final was gated ("runway capture gate", then "specific-energy envelope missed"), and handed off at 265 m above the runway at 85 m/s against a 112 m/s target.

## Changes

- `guidance/taem/energy_law.inc`: on the runway line inside the station, k = 0.5 + (E − E_ref(d)) / 400 m, where E_ref(d) is the energy on the 20° glide at the vehicle's distance plus the Final alignment speed.
- `guidance/entry/contract.inc`: while the live replan is short at 0° bank, MM304 accepts a 15 km crossrange (`MM304_ENERGY_SHORT_CROSSRANGE_TOLERANCE_M`) instead of nulling it.
- `guidance/guidance_terminal.c`: STS wheel contact is latched after Final capture under every STS profile. A touchdown bounce previously re-entered the airborne departure monitor and aborted a landing that had stopped on the runway (`runs/20261016T092811Z-sts-10deg-energy-fix`).

## Live result after the fixes: `runs/*-sts-10deg-energy-fix2` (same orbit save)

| | Baseline | After |
| --- | --- | --- |
| MM304 bank at 50 / 30 km | 42° / 25° | 37° / 21° |
| MM305 start drag level k | 0.00 | 0.77 (surplus) |
| Final handoff height / speed | 333 m / 85 m/s | 746 m / 127 m/s |
| Touchdown sink | firm | 3.5 m/s |
| Stop | on runway, 9 m off | **52 m off the centreline (runway is 70 m wide): Abort, outside runway bounds** |

Energy is now positive through MM305 and Final. The new failure is lateral. This time MM305 flew straight-in and joined the runway line only 3.6 km out, 454 m off the centreline with a 13° heading error. Final was gated until 746 m, inherited a 115 m offset, and closed it only to about 50 m.

## Open

- Straight-in runway-line capture: the energy law's centreline pursuit joins too close to the threshold.
- Final's lateral closure rate is too low for a 100 m offset.

## Straight-in line capture fix (`0cc3faf`): `runs/*-sts-10deg-hac-fix`

- **Cause.** Without `stsTrajectoryDelivery`, `tel_plan` kept a straight-in pursuit for a vehicle 44 km past the threshold, 27 km abeam and flying away from the runway heading. It priced the cheap chord and required a 2 km margin before switching. The pursuit reversed at the station and joined the line 3.6 km out.
- **Fix.** While outbound (|course error| > 90°), the plan prices only HAC patterns, which matches the delivery-policy rule.
- **Live result from the same orbit save.** The vehicle flew a downwind and a 4.2 km HAC and was on the line at the 9 km station, 57 m off with a 2° heading error. Final took over at 1.4 km altitude, 6.4 km out, 3 m off the centreline. Touchdown sink was about 3.2 m/s, and the vehicle stopped on the runway 10 m off the centreline. Result: **COMPLETE**.
- **Rejected.** Ungating Final's STS reversal sizing for sts-live. For a 115 m offset the needed bank already equals the 10° cap, so it does not help. For small offsets it raises the gain about 5× and cost the L7 low-energy regression its runway (5.8 m/s sink, 1.2 km short).
- **Still open.** On the HAC path MM305 runs at k = 0 and reaches Final slow (100 m/s against 112 m/s), so the flare is firm. The remaining energy margin is thin.
