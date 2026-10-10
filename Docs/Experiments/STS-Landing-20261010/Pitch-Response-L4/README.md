# L4 pitch-response investigation

L4 was monitored only through files. It completed touchdown and rollout naturally;
no control, stop, launch, or save reload commands were issued during this investigation.
The final stopped position was 1404.3 m along and −2.0 m cross at 1.50 m/s. Touchdown
was around 49 m/s following a balloon/float, so it does not satisfy the campaign's
63.75 m/s touchdown acceptance band. The prior-flight aero residual was reported as 77%.

`L4-pitch-trace.csv.gz`, `L4-response-summary.json`, and the first Final snapshot/station
retain the measured command/response evidence. The scenario uses logged true airspeed
as its surface-speed approximation; this is an offline restart fixture, not a certified save.

## Adopted correction

Native direct-control simulator packets publish the plant's actual normalized pitch,
roll, and yaw inputs. The backend previously never set `Telemetry.has_controls` from
simulator data, preventing passive trim/response observations that live kRPC can make.
Decoding is optional and rejects missing, nonfinite, out-of-range, and high-level-mode
feedback. One round-trip regression test covers publication, decoding, and fallback.

Run the test in a separate build directory:

```sh
make -C CLanding -f Makefile -f ../Validation/SimulatorControlFeedbackChecks.mk \
  BUILD=build-pitch-verification simulator-control-feedback-test
```

Build the native simulator separately with CMake to use the new feedback in replays.
No live pitch gains have been changed. Correcting replay feedback does not itself
prove stronger live pitch response. Feedback-only replay keeps 6/7 complete handoff
passes; 12-33-31 remains short/slow. Replays use fixed flare, STS modelId, the existing
sts-live-20261010 aero profile, and the three live-like direct-control plant fixtures
when running the matrix. Never use these experimental backends to hot-swap a live run.

## Rejected candidates

- Restoring energy-law command bookkeeping and trim feedforward passes its focused
  regression test, but real input feedback reveals 5/7 handoff passes (13-52-53 slow).
- Restricting trim samples during rolling/turning worsens that candidate to 3/7.
- Increasing pitch wn from 1.05 to 1.20 gives 1/7 handoff passes and 4/12 L1/L3
  plant/gain matrix passes, with short and hard landings. It is rejected.
- An initial settled-sampling build failed on a nonexistent Telemetry.yaw_rate field;
  the subsequent runs reused the earlier binary and are excluded from comparison.
  Corrected heading-rate sampling was then rebuilt and replayed as trim-settled-v2.

`rejected-energy-trim.patch` and `TAEMEnergyTrimTests.c` retain the rejected study;
the test file must be restored under Validation to resolve its relative source include.
It is not part of active checks because its proposed source change is rejected.
`replay-results.json` retains exact outcomes/provenance for 28 completed comparisons.
The stronger live pitch correction remains unresolved and must be validated before
another launch. Existing flight restrictions and safety/admission gates are unchanged.
