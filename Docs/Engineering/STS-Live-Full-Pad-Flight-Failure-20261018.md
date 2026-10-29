# STS live full-pad flight failure report — 2026-10-18

## Summary

The live full-pad flight did not produce a qualified deorbit plan. The planner selected a tier-3 candidate, but its executable-policy witness failed capture and robustness qualification. It reached the entry/TAEM stages yet missed the ownership boundary; this is a planning/selection failure, not evidence that the entire vehicle or flight controller failed.

## Evidence

Source log: `runs/20261018T031133Z-live-full-pad-flight/backend.stderr.log`.

- The planner reports its 50 km policy witness as reduced along/cross `-447.7/+20.9 km`, versus executable `-412.3/+15.3 km` (difference `+35.4/-5.6 km`; speed difference `-1.3 m/s`). The reduced prediction is not a faithful substitute for the executable witness in this case.
- Selected candidate: tier 3, burn UT `147844327.6`, Δv `30.0 m/s`, periapsis `50.0 km`, entry range `1132.7 km`, FPA `-1.63°`.
- Reported entry/TAEM diagnostics: alignment `25.7 km`, course debt `13.9°`, altitude `24.9 km`, speed `861 m/s`, TAEM range `30.8 km`, range error `-65.7 km`.
- Qualification flags: `capture=0`, `boundaryMiss=1`, nominal `0`, robust `0/15`, recovery `0/15`; energy margin is marked valid, but that alone does not establish a viable capture.

## Failure assessment

The planner chose the best available tier-3 candidate despite the candidate not satisfying capture and not being robust in any of the 15 assessed stress scenarios. The positive reported impact margin (`+90.6/-67.7 km`) does not override the explicit capture failure and boundary miss. The logged prediction discrepancy between reduced and executable witnesses is a further warning that reduced-model evidence should not be treated as executable-policy qualification.

## Impact and limits

This report documents the failure indicated by the available backend log. The log excerpt does not establish touchdown outcome, hardware damage, or root cause beyond the planning/witness qualification discrepancy. No corrective change or live vehicle command is asserted here.

## Recommended follow-up

1. Reproduce the run with the same mission/configuration and retain the complete planner candidate and stress-scenario traces.
2. Investigate why selection permits a tier-3 candidate with `capture=0` and `boundaryMiss=1`; ensure failed executable capture is clearly surfaced and cannot be represented as qualified.
3. Compare reduced and executable witness propagation around entry/TAEM, especially the approximately 35 km along-track discrepancy.
4. Re-run offline planner tests and the full-pad scenario before another live attempt; require explicit capture and robustness acceptance.
