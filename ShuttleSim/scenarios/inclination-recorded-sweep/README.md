# Recorded-energy inclination sweep

These cases rotate only the tangential direction of the recorded KSP 86 km post-deorbit state from `../ksp86km-postburn.ini`. Position, UT, mass, AoA, bank, inertial speed (2238.467958 m/s), and radial velocity (-6.814570 m/s) are held fixed. This isolates orbital-plane / ground-track direction from deorbit-energy changes.

The matrix covers 45, 60, 75, 85, 90, 95, 105, 120, 135, 150, 165, and near-180 degree inclination. The 180-degree case is 179.997122 degrees because the recorded starting position is slightly off the equator.

Initial 2026-09-27 runs used the production atmospheric guidance via `run_guidance.py --engage engageReentry`. Every case reached MM304 and then aborted with `MM304 flight recovery margin was exhausted before TAEM acquisition.` No case reached MM305/HAC. The current near-equatorial recorded control also has this failure, so the baseline MM304 regression must be repaired before the matrix can qualify MM305/final behavior. See `initial-results.tsv` for terminal geometry.
