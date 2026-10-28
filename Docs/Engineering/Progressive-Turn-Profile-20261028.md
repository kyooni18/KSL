# Progressive shuttle turn profile

The second feedback run (`runs/20261028T040505Z-feedback-test-2/ascent.csv`)
records 34.70 degrees actual nose pitch and 38.63 degrees commanded pitch at
9.565 km, with 41.73 kPa dynamic pressure. At 4.852 km, actual/commanded pitch
were already 50.77/55.09 degrees. The early trajectory was too shallow.

The previous fractional-power altitude schedule dropped rapidly immediately
after release from the roll target. Corridor/forecast downward corrections
could deepen the turn, while the prograde +1 degree upper bound prevented
useful upward correction toward the reference. Repeatedly following a shallow
velocity vector could therefore perpetuate the shallow climb.

## Reference for a 90 km orbit

| Altitude | Reference nose pitch |
| --- | --- |
| 0.5 km | 78 degrees |
| 5 km | 75 degrees |
| 10 km | 68 degrees |
| 20 km | 55 degrees |
| 30 km | 42 degrees |
| 40 km | 28 degrees |
| 55 km | 12 degrees |
| 65 km | 5 degrees |

This is an engineering starting profile for this Kerbin shuttle, not a claim
to reproduce a historical Earth shuttle trajectory. Monotone cubic interpolation
provides continuous pitch and altitude slope, with flat endpoint tangents. The
reference crosses 45 degrees near 28 km. Altitudes scale with the reference
orbit altitude; transfer missions retain their parking-orbit reference.

The velocity-gated initial roll still targets 78 degrees and the gravity turn
can progress while roll correction continues. A five-simulation-second command
response remains; no fixed pitch-rate cap is introduced.

Corridor and planner corrections are restricted to +/-3 degrees around the
reference through 10 km, widening smoothly to +/-15 degrees at 45 km. The same
envelope is applied after the live executive's booster-phase corrections and
inside the continuous booster forecast. Thus planner bias cannot erase the
early profile. Post-separation insertion guidance still owns the final orbit;
separation attitude capture, MECO settling and SSME trim are unchanged.

The existing q*alpha bound takes priority over the profile when the flight is
already off-course. Positive and negative AoA use that same bound, replacing
the old +1 degree recovery restriction. At 25 kPa and 35 degrees prograde,
the command can recover only to 37 degrees, not jump to 68 degrees. These are
reference targets, not guaranteed actual pitch or a license to exceed wing loads.

## Offline verification

Ascent plus operation regression suite: 146 passed and 3 subtests passed.
After adding the second rejected-seed case, the profile and full-plan suites
were rerun: 9 passed. `git diff --check` passed.

Tests cover waypoint values, monotonicity and slope continuity, the delayed
45-degree crossing, target-altitude scaling, combined downward corridor/planner
corrections, and load-bounded recovery. The stage-to-MECO forecast succeeds from
a 10 km climb seed at 68 degrees commanded / 66 degrees flown, 300 m/s radial,
and 150 m/s surface-horizontal speed, without loosening feasibility tolerances.

The initial regression run found one failed forecast test seeded at 45 degrees
commanded / 42 degrees flown, 300 m/s radial and 350 m/s surface-horizontal at
10 km. That already-shallow seed is now explicitly tested as unreachable under
the revised model, alongside the prior rejected seed. The success case uses
the new climb seed above. Neither result qualifies an entire pad-to-orbit flight.

No KSP connection, launch or quickload was attempted. Live stability and actual
pitch tracking remain unvalidated. Existing unrelated work is excluded from
this change.
