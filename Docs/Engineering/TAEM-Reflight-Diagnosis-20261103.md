# Reflight diagnosis — 2026-11-03

Source run: `runs/20261103T052636Z-deorbit-coverage-reflight`, immutable executable revision `c20514c69f5767d3781f3b610b03473cd8378d38`. Analysis reads the flown source, configuration, recorded proposals, commands and measured telemetry. No new live attempt, control intervention or simulation was performed for this diagnosis.

## Outcome

This was an off-runway ground stop, not a successful landing. Selected runway is Dessert 36, heading 0.94276554 degrees, length 1,750 m, width 42.6 m, elevation 820.95 m ASL. Along is measured from its selected threshold; crossrange is positive right. At first main-gear contact UT 19089.560376, along was −892.5 m, cross −1,605.2 m, airspeed 49.4 m/s, radial vertical speed −0.65 m/s and bank 7.4 degrees. Measured gear contact was present. At UT 19130.080376, contact was stopped, along −967.6 m, cross −1,945.5 m and speed approximately zero. Fresh continuous telemetry still showed stopped contact and 26 parts at UT 19168.900376. This corroborates an intact off-runway stop, not runway capture.

The requested 72 km checkpoint was not created. The separate watcher incorrectly used vessel-relative Flight vertical speed, which reported zero. Its corrected body-fixed watcher has not flown; the preserved orbital save was not overwritten.

## What improved

Actual first atmospheric entry UT 18053.840376 was at 69,495.27 m ASL, 2,101.52 m/s, threshold range 1,020,197.28 m. The new selected plan predicted 1,030,065.09 m entry range, against the 1,030,000 m configured aim. This is consistent with the deorbit selection fix taking effect. It is not downstream landing qualification.

## Remaining failure chain

| Event | UT | Height above runway | Speed | Along / cross |
| --- | ---: | ---: | ---: | --- |
| MM305 acquisition | 18628.320376 | 27,002 m | 877.8 m/s | −15,340 / −15,007 m |
| Twenty seconds later | 18648.280376 | 25,176 m | 670.5 m/s | −16,474 / −393 m |
| Fifty seconds after acquisition | 18678.360376 | 20,600 m | 413.5 m/s | −14,960 / +14,453 m |
| Hundred seconds after acquisition | 18728.300376 | 12,156 m | 194.7 m/s | −7,668 / +22,448 m |
| Last-chance approach | 19046.640376 | 959 m | 100.6 m/s | +1,931 / +460 m |

All 4,033 MM305 guidance samples used `local-advisory-path-no-complete-forecast`, reference plan 186251. Its path error grew from 101 m at takeover to about 22.4 km after 100 seconds. The first large turn was already marked physically infeasible in the recorded advisory assessment: unavailable lift/trim, excessive curvature/bank, actuator-rate failures and incompatible vertical demands. An assessment's `predictionComplete` flag here is not a native controller landing certificate.

At UT 18648.280376, the local fitter requested normal-force components `(8.214, −49.880)` m/s² and modeled `(2.158, −5.929)` m/s². Commanded bank was −58.0 degrees, measured bank −55.16 degrees. Measured lift/mass was only 6.109 m/s², with bank-projected vertical lift 3.490 m/s². Thus the major error was not a disconnected bank command: the reference asked for an unattainable turn, while the jointly fitted command also lacked the requested support. By +50 seconds the measured sink was 185.6 m/s. This is evidence of force/energy loss; it does not by itself prove that any particular replacement route would have landed.

The selected advisory was created at UT 18625.060376, with 42 stations and length 35,944.51 m. Its acquisition includes curvature around −0.00044 /m at station 18.32 km. The tracker eventually advanced to its final station and remained there while far off the path. The curve was never regenerated during this MM305 segment.

Code explains the frozen fallback: `MissionPlanningWorker.plan` returns only `TAEMRecoveryPlanner` output when `taemRecovery` is true; no refreshed terminal advisory is returned. The 83 recorded recovery proposals contained 747 trials: 664 ended `unsupportedDynamics`, 83 ended `scheduleExhausted`; none supplied a selected complete-accounting forecast. `LandingMissionController` therefore continued fitting horizontal demands from the single old advisory against current vertical recovery. A pointwise fit bounds the command but does not make the old curve attainable or restore the energy spent missing it.

At approach takeover the course was approximately 156.2 degrees from the selected runway heading. This was explicitly last-chance recovery at the Final inlet height, not measured NEP capture. It reached the runway vicinity but was still heading away from the selected landing direction, then touched down left of and before the threshold.

## Next engineering change

Replace persistent tracking of the old infeasible advisory with continuously regenerated recovery geometry consistent with measured course, winding, roll response and attainable coupled force/energy. Keep flying the best recovery when the predictor is unsupported, but do not treat a bounded pointwise command as proof that its reference can be followed. Restore enough predictor coverage to compare complete continuations without bypassing moment-support guards. Preserve the configured 40-degree AoA/70-degree bank limits and RCS policy. This diagnosis does not establish the corrected trajectory or justify another flight by itself.

Evidence is in `Validation/TAEM-Reflight-Diagnosis-20261103`: exact matched telemetry samples, final pre-MM305 advisory proposal, first command-recovery proposal, trajectory summary and provenance with artifact hashes. No regression tests were added because production code was not changed.
