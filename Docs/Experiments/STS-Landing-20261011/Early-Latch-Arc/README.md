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
