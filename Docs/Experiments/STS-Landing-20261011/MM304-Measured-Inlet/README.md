# MM304 measured-inlet cross-check

The measured MM305 fixtures exposed body-heading/course confusion. This
follow-up checks whether correcting that same approximation at MM304 explains
the earlier range-control replay discrepancy. It does not.

`prepare.py` reconstructs surface velocity from native inertial velocity minus
planet rotation and verifies speed/FPA/AoA/sideslip against the source frame.
It preserves measured body pose/rates. L5 UT 1542.810 heading minus course is
−0.0048°; L7 UT 5986.950 is +0.0857°. Both first MM304 frames pass coherence
checks. Full native fields and source hashes are in `sources.json`.

`run.py` uses the same corrected inlet for each paired run, the snapshotted
current STS configuration (96.5 km acquisition range), existing direct
coefficients and either legacy AoA integration or opt-in body-coordinate
integration. Both are provisional plants. Runs deliberately stop on TAEM;
exit 2 means an unfinished landing, not failed handoff. No deorbit burn or
live flight is included. `result.json` retains input/binary provenance and
final backend telemetry separately from the slightly later simulator endpoint.

| Inlet / plant | TAEM altitude | Speed | Along | Cross |
|---|---:|---:|---:|---:|
| L5 / legacy | 24.450 km | 860.726 m/s | −34.209 km | −2.340 km |
| L5 / body | 24.427 km | 859.863 m/s | −34.164 km | −2.200 km |
| L7 / legacy | 24.752 km | 861.243 m/s | −79.067 km | −41.662 km |
| L7 / body | 24.731 km | 860.654 m/s | −79.212 km | −43.179 km |

The large L7 lateral displacement survives both inlet correction and body
kinematics. MM304's initial course approximation is too small to explain it.
The original live L7 handoff was −32.5 km cross-track, so neither provisional
plant reproduces that lateral history. L5 also still reaches TAEM much closer
than its original 59.8 km range, about 24.4 km high rather than 27.1 km.

This narrows the investigation toward entry lateral/range prediction and
model/history error. It does not justify changing the deorbit target or the
coupled acquisition-range boundary. In particular, keep the prior rejection
of a simple 60 km acquisition-range change: it prevented L7 admission in that
experiment. No production coefficients, guidance laws or acceptance gates
were changed here. Ground parachute deployment is not reached at this stop.
