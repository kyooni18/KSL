# Ascent turn timing correction — 2026-10-21

The legacy ascent in `runs/20261021T052318Z-launch` targeted 90 km but
reached Ap 117.2 km by altitude 59.9 km, with about 888 m/s vertical and
354 m/s horizontal speed. This is an upright trajectory before insertion.

The guidance loop used `1/args.hz` for pitch, heading, and engine limiter
slew, although synchronous RPC work made the actual simulation interval
substantially longer. Recorded pitch-command rates between 3 and 20 km were
mostly 0.03–0.26 deg/s rather than the configured 0.8 deg/s. This also
invalidated the insertion planner's assumed attitude slew and limiter response.

The loop now measures elapsed KSP UT once per update and uses that interval
for all four slew call sites. The first update, paused simulation, and clock
rewinds advance commands by zero. `--hz` remains the polling frequency cap.
The gravity-turn reference, roll phase, q-alpha constraints, and MECO orbit
requirements retain their existing settings.

A replay on recorded pre-separation states, beginning when the recorded
pitch first leaves 89 degrees, applies the current scheduled/corridor pitch
and 0.8 deg/s slew with elapsed UT. At 24.9 km it commands 64.2 degrees
(the reference) versus the recorded 70.3 degrees. This replay verifies command
timing only: recorded states are held fixed, so it does not predict the new
trajectory or final orbit.

Validation: `.venv/bin/python -m pytest tests/ascent -q` — 92 passed.
One regression test compares regular and irregular/paused control intervals
and checks pitch and engine throttle rates. Jev was unavailable. No live
flight or vessel control was performed; final apoapsis needs flight validation.
