# Early preflare latch and the `on_arc` shortcut (scratch variants, NOT applied)

## Live evidence (flight 4, `runs/20261011T143513Z-live-aim150-drogue-verification`)

The live admission plan stored a preflare trigger of **1820 m**, so Final latched the pull at
~1700 m height, about 10 s into Final (offline L5/L7 triggers are 118-820 m). After the latch,
`on_arc` was true because `sink < 0.92 * glide_sink` (the reference glide was clamped to -25 deg,
the vehicle sat at -13.6 deg). In the arc branch vertical acceleration is floored at 0 ("never a
push while on the arc"), so `vertical_accel` was exactly 0.00 from UT 6957 to ~7040. The vehicle
held a constant ~-13.6 deg path at 5 deg incidence and 17-19 m/s sink for ~90 s instead of
steepening to the geometric glide, arrived ~1 km high over the threshold and flared ~1.5 km past
the 150 m aim point (contact ~1674 m along, stop ~56 m past the runway end).

## Scratch experiment (`run.py`; production untouched)

`early-latch` forces the planned trigger floor to 1820 m. `fix` replaces
`on_arc = sink < .92*glide_sink || onset_sink < sink` with `on_arc = onset_sink < sink`.
Speed m/s / sink m/s / along m (L5 golden 66.129 / 1.844 / 297.6 accepted; L7 61.929 / 3.353 / -376.2):

| Variant | L5 | L7 |
|---|---|---|
| baseline | 66.13 / 1.84 / 297.6, success | 61.93 / 3.35 / -376.2, fails |
| early-latch | 82.45 / 4.24 / -531.6, fails | 52.00 / 3.34 / 209.0 on runway, fails |
| early-latch + fix | 82.45 / 4.24 / -531.6, fails | 56.05 / 1.42 / 38.6 on runway, fails speed |
| fix only | 70.91 / 1.19 / 32.4 on runway, **success** | unchanged from baseline |

Reading: the shortcut lets a vehicle that is merely *too high* be classified as already on the arc,
which suppresses glide pursuit. Removing it makes L7 under the live-like early latch land 38.6 m
along (aim 150 m) with 1.4 m/s sink, and moves the accepted L5 touchdown from 297.6 to 32.4 m.
Forcing 1820 m on L5 is not realistic (L5's own trigger is ~816 m) and fails regardless.

## Status

Not adopted. "Fix only" changes the accepted L5 golden (touchdown 297.6 to 32.4 m, 66.1 to 70.9 m/s,
stop 1101.6 to 913.4 m), which needs a deliberate decision. The four-scenario matrix check
(`EARLYLATCH_MATRIX=1 run.py`) was started and interrupted by the user, so **robustness across
L6/high-energy is untested**. The early-latch cause (why the live planner stored 1820 m) is also
unexplained. No gate, golden or production code changed. Live-flight results here are single
samples in an unqualified simulated plant.

## Why the live planner stored ~1820 m (read-only analysis of the flight-4 trace)

From `FINAL_TRACE` (guidance instance `0x7732c006d0`, UT 6929.7-6960): at Final entry the plan was
valid with trigger 181 m and `planSpeed` 85 m/s (the configured `minimumSafeSpeed`). Within 1.5 s the
plan flipped to `planSpeed` 111-118 m/s (the glide-mode reference, `terminal_final_speed_target`, instead of
the 85 m/s floor) with trigger 1516-1830 m, flipped back (6934.0: 89 m/s / 594 m), and from UT 6941.1 held
trigger = 1820 m, `planSpeed` = 118.4 m/s, response 2.8 s, constant for 100 s while the vehicle fell from
1971 m to ~140 m and slowed from 90 to 70 m/s. The plan stayed `valid` the whole time, so
`planned_preflare_trigger` (1820 m) floored the latch (`preflare_latch_height = fmax(preflare_height +
response_height, planned_preflare_trigger)`) and the pull latched at 1813 m (UT 6948.4), 7 s after the trigger
saturated.

In `terminal_preflare_plan_floor` (`final/planning.inc`) the trigger is `flare_altitude + required_height` of the
latest-initiation candidate whose predicted exit speed stays above `minimum_speed`. From 94 m/s with a 27 m/s
sink and an 85 m/s exit floor, only very gentle pulls keep the exit above the floor, and the vertical-recovery
envelope then needs ~1700 m. The trigger is therefore set by the exit-speed floor (plan item 3 in
`FLARE_IMPROVEMENT_PLAN.md`: the 85 m/s floor sits above the 53-56 m/s controlled live touchdowns) rather
than by any real pull-up need, and flips discontinuously between candidates. The latch floor consumes it as a
hard requirement even though the vehicle is 1.8 km up.

Candidate remedies (none applied, all need the matrix and golden review): (1) plausibility-cap the planned trigger
used as the latch floor (e.g. the response-and-pull height the present sink actually needs); (2) fix the `on_arc`
shortcut above so an early latch cannot stop glide pursuit; (3) derive the exit floor from `speed_1g`
(plan item 3). Offline L5/L7 triggers (118-820 m) never reach this regime, which is why the simulator did not
show it.
