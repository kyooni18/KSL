# Simulation-time audit — 2026-10-28

Scope: offline review of the pending timing changes after `0ee5569`. No KSP
connection, launch, quickload, or live validation was attempted. The target
remains a stable 90 km orbit at 45 degrees. This is a timing checkpoint, not
flight qualification.

## Reviewed and changed

* Attitude integration and dynamics refresh use measured KSP UT. Pause gives
  zero integral increment; rewind clears the integral and refreshes dynamics.
  Idle/disengagement clears the previous step timestamp.
* SSME feedback consumes the full measured interval, removing the one-second
  truncation. It remains gated by pointing/rate and bounded by maximum bias.
  A three-second interval now gives the same accumulated correction as three
  one-second intervals for constant input. This does not establish that a
  sparse sample represents the attitude throughout a long interval.
* Pad settle, ignition, SSME lead, calibration waits, OMS/RCS alignment and
  deadlines, rendezvous approach, intelligent ascent/insertion, and landing
  flight timeout now use simulation time.
* Intelligent ascent no longer truncates dt at two seconds. A backward UT step
  rebases its ascent clock/deadline, skips integration at the discontinuity, and
  resumes on the next advancing sample. Propulsion depletion dwell restarts
  after a rewind. Coast telemetry now includes UT.
* Pad settle and landing timeout restart their intervals on a rewind. Legacy
  ascent command smoothing already uses nonnegative full UT differences.

## Wall time that remains intentional

`time.sleep`, loop pacing, child termination escalation, backend request/pipe
timeouts, connection/scene availability, UI freshness, command acknowledgement,
and warning rate limiting are polling or process supervision, not physical
flight time. Python event telemetry is throttled to one wall second and landing
snapshot logging to half a wall second. Native flight-log emission/keyframes
also use wall time. These are delivery limits; flight analysis must use sample
UT, not log arrival timestamps. The ascent/attitude CSVs contain UT.

## Open findings

1. **Native landing dt is not yet compliant everywhere.**
   `CLanding/transport/krpc.c` clamps atmospheric control dt to 0.01–2 seconds
   and substitutes nominal dt on a repeated or backward UT sample.
   `CLanding/control/flight_control.c` also falls back to nominal dt when no
   positive interval is available. A repeated sample can therefore advance
   internal control state if it reaches that path. The transport's fresh-sample
   gate reduces ordinary exposure but is not a substitute for correct dt.
2. **Native telemetry derivatives distort irregular intervals.**
   `CLanding/transport/client/telemetry.inc` clamps attitude derivative dt to
   0.01–1 second; it can manufacture rates on a rewind and exaggerate rates
   after a long interval. Course derivatives reject intervals outside that
   range. `CLanding/telemetry/sim_telemetry.c` instead uses positive full UT dt.
3. **Native guidance still truncates physical integration.**
   `guidance_update` in `CLanding/guidance/guidance_terminal.c` caps UT dt at
   one second, including delivered delta-v and burn-active elapsed time.
   Native timing fixes need focused behavioral coverage and a separate patch;
   the pre-existing dirty native files were not edited here.
4. **A UT rewind is not a restored flight session.** Absolute UT deadlines in
   ignition, calibration, trim, rendezvous, and intelligent insertion retain
   their old endpoints; a backward jump extends the wait. Existing nodes,
   vessel handles, separation state, and propellant accounting are not rebuilt.
   OMS thrust-axis estimation also needs to discard its velocity window on
   nonpositive dt. No quickload/resume capability is claimed by this patch.
5. **Optional generic guidance differs from shuttle requirements.** Intelligent
   ascent retains a hard 2 deg/s ceiling on its authority-derived pitch slew
   and exception-driven flight stops. These predate the timing patch. The
   native deorbit missed-alignment path also calls `guidance_abort`. They need
   a separate manual-abort/continuous-guidance review before those paths are
   considered compliant. The primary shuttle ascent uses warning-and-continue
   flight-envelope checks; no new automatic abort policy was added here.

## Recorded flight evidence

`runs/20261028T034252Z-feedback-test-1` reached MECO near 91.4 x 60.4 km at
45.02 degrees. Its OMS log reports end-of-burn aim errors of 89.3 and 134.3
degrees. The final 90.96 x 89.73 km / 45.012 degree orbit does not demonstrate
stable trim attitude.

`runs/20261028T040505Z-feedback-test-2` reports a partial booster batch failure,
a 46.6 degree nose-error warning, and a premature MECO attitude hold followed
by propellant-depleted cutoff at 152.2 x -410.9 km. Later fixes address detached
part forecasts, separation attitude, governed insertion acceleration and hold
release, but these logs precede those fixes and cannot validate them.

Preserve the 78-degree initial-roll target, velocity-gated roll start,
continuing gravity turn, measured separation hold, TVC upright roll, settled
MECO/ET separation, gentle trim and one-degree gimbal reserve for 3 G limiting.
No tuning changes to those behaviors were made in this timing review.

## Verification

Focused ascent, operations, controls and process-lifecycle suite: 151 passed,
3 subtests passed. Tests exercise pause, slow UT increments, a multi-second
update, rewind, separation dwell, and SSME feedback interval propagation.
Full working-tree offline regression (`.venv/bin/python -m pytest -q tests
Validation`): 267 passed, 59 subtests passed in 43.83 seconds, no failures.
Two `PytestUnhandledThreadExceptionWarning` warnings came from `ksl-mission`
and `ksl-commands` writing status after the TUI test's temporary run directory
was removed. This repeats the earlier teardown race; it is not a clean,
warning-free run. The previous landing-simulation failures did not recur.
No separate native C gate was rerun because this patch changes Python only;
native findings above are static review, not newly validated native fixes.

The unrelated `lock_ssme_gimbals` edits, `test_flight.py`, CLanding edits,
ShuttleSim edits/data, and earlier failure-analysis document remain uncommitted.
