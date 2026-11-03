# Recovery refresh and support constraint — 2026-11-03

The c20514c live flight retained one pre-MM305 curve throughout 418 seconds of recovery. Its local turn demand exceeded 49 m/s² when the model could produce only a small fraction, and the Euclidean force fit spent required normal support chasing that unpriced lateral demand. Matched telemetry and the full diagnosis are retained in `TAEM-Reflight-Diagnosis-20261103.md`.

## Changes

The default MM305 worker now returns regenerated advisory geometry alongside command-recovery proposals from the same measured snapshot and calibrated model. It supplies the existing winding, course, force, trim and response context to HACPlanningEngine. It does not replace admission with a favorable preview: complete command schedules retain their independent accounting/ownership requirements, and fresh advisory geometry remains explicitly unqualified. Nil geometry removes the old advisory instead of preserving a stale join.

For local unqualified recovery and its protection seed, TAEMNormalForceFitter now accepts an explicit minimum vertical wind-normal lift component. The caller supplies its current pressure/glide recovery request. Each sampled real polar incidence determines its own permissible bank from `L cos(bank) >= min(requestedSupport, maximumAttainableLift)`. If the support request itself cannot be attained, the fit uses maximum attainable lift upright. The original requested force, residual and attainable support reserve remain visible. No additional lift, actuator authority or vehicle limit is invented. Unconstrained joint fitting remains available for priced candidate comparison; complete command forecasts are not silently modified by this fallback policy.

This is an explicit physical support constraint for an unpriced geometric request, not proof of optimal mission recovery. It can delay a turn when the vehicle cannot support it. Regenerating a local advisory also does not resolve the missing aerodynamic moment evidence that prevents complete native forecasts. These limitations remain visible and must be assessed in the authorized flight.

## Decisive checks

Two focused checks use genuine states from the failed flight: (1) the exact failed turn request retains the attainable portion of its requested support with forward-polar verification, and (2) two measured epochs produce fresh geometry at the measured inlet, accepted into the normal mission without claiming a complete controller forecast. Using the unchanged model at the failed state, old joint normal support was 2.672 m/s²; the constrained fit yields 7.813 m/s² (requested 8.214). These numbers are model results, not achieved flight forces. The test does not invent weakened/strengthened STS capability.

Four existing checks also pass, covering advisory ownership, unconstrained joint fitting and default command-search/cursor wiring. The earlier integration assertion requiring nil terminal geometry was updated to require refreshed advice; a nonzero-bank assertion was removed because a physically unsupported turn may correctly require upright recovery. The new force regression verifies this substantive behavior instead.

The corrected mission-specific 72km watcher uses explicit body-fixed Flight data and a descending ASL crossing determined by consecutive altitude/UT samples, without a vertical-speed sign gate. Replaying its exact trigger against the previous recorded descent detects one crossing, from 72,005.618 m to 71,998.720 m. This replay does not call save or change KSP. The authorized live rerun must verify the actual save file and its altitude separately.

Evidence: `Validation/TAEM-Recovery-Refresh-20261103`. Six focused tests passed. Live landing readiness remains unqualified before the rerun.

## Authorized live checkpoint

Run `runs/20261103T063436Z-recovery-refresh-live` uses immutable source `06a41f465c97a32ebc9e55f239747a7d77fb30a9`. The previous controller was stopped before the preserved original orbital save was loaded. Fresh preflight was UT 634.360024, approximately 85 km orbit, 26 parts and zero throttle.

The descending 72 km save exists in the current Steam `shuttletester` save directory as `ksl-20261103T063436Z-recovery-refresh-live-descent-72km.sfs`, 320,384 bytes, SHA256 `d05907070408b705d31335dc4fd2de63aee0d2650c936226f579ca10ef04000a`. Save UT is 18011.022133800518; STS persistent ID 961889008 is suborbital, with saved header altitude 72,001.819857 m ASL and header epoch 18011.002133800517. The original watcher's strict saved-header-altitude-below-72000 assertion flagged this two-metre offset. That was a verification false negative, not a missing save: the file was independently read and verified. The watcher now preserves exact file altitude and checks time bracketing instead of assuming separate KSP header/Flight snapshots are simultaneous. The original error artifact remains in the run directory. No pause or flight-control change was made for the save.

The flight was still in atmospheric entry when this checkpoint record was added. This establishes the replay checkpoint, not a landing outcome.
