# Booster-separation attitude handoff — 2026-10-21

The restored legacy run `20261021T083401Z-launch` records nose error increasing
from 3.49 degrees at UT 145108.80 (20.9 km) to 35.00 degrees at UT 145112.62
(23.2 km), immediately after booster separation. The first detached sample
still has an SSME limiter of 0.741, rather than the 0.30 clearance cap. The nose
then swings from 63.7 to 43.0 to 34.6 degrees. The upright roll does not start
until later, below 800 Pa, so this initial disturbance precedes that maneuver.

Three discontinuities in the legacy handoff are corrected:

- `_separate_boosters` caps STS/SRB core limiters at 0.30 before detachment. A
  general throttle slew limit must not leave high canted thrust on the lighter
  vehicle after the balancing SRB moment disappears. Liquid-booster and nuclear
  vehicle staging behavior is unchanged.
- The guidance loop requests attitude dynamics invalidation immediately after
  the vehicle update stages, instead of resetting integral/dynamics on the next
  guidance tick. A thread event makes the FCS reset its integral and refresh
  inertia, available torque and thrust-line moment before its next control step.
  An event arriving during refresh survives for the next step.
- Thrust-axis correction now blends in over four seconds after separation,
  instead of immediately introducing the roughly 15-degree SSME cant offset.
  Existing four-second roll blend-out and six-second blend-in remain continuous,
  including when an upright roll starts before the initial blend completes.

Validation: 95 ascent tests pass, including three focused regression cases for
pre-detachment cap ordering despite a slow limiter slew, dynamics invalidation,
and continuous compensation. This is an offline correction based on recorded
flight evidence. No live validation, kRPC client, launch, warp, or save change
was performed. Jev was not available. The legacy gravity-turn/PEG path and the
original elapsed-UT slew fix remain; calculated-ascent optimization is not added.
