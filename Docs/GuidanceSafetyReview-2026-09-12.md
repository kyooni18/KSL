# Guidance safety review — 2026-09-12

This is an offline software review, not flight qualification. No KSP or live kRPC
connection was opened.

## Corrected issues

- **Persistent Entry plans bypassed dynamic-pressure protection.** Requesting a
  replan did not ensure that ALPHA's live protective command reached execution:
  propagated state may be unavailable, the solve may fail, and cached supervision
  may lower AoA afterward. Execution now applies the live q-protection floor after
  plan/supervision selection. Stall and g-load unloading retain priority. Ordinary
  command shaping remains in place.
- **Command shaping could exceed live limits.** AoA reference inertia could
  overshoot maximum incidence, and a bank reference could remain outside a newly
  reduced q/g/stall limit. The shaped outputs now respect hard bounds and reset
  the corresponding limiter state when clipped.
- **The ALPHA modulation band could erase thermal protection.** A valid thermal
  floor above nominal AoA plus the modulation allowance was clipped away. The
  drag-tracking band now applies before safety protections, with g-load and stall
  protection still able to override the thermal floor.
- **Malformed lateral limits were accepted.** Nonfinite limits could be masked
  by `fmin`/`fmax`, silently changing dwell or command-rate behavior. Invalid
  limit sets are now rejected before capture/reversal state is changed.

- **Missed preflare initiation could continue steep descent.** Both trajectory
  capture and outer final now abort before consuming the modeled pull-up reserve,
  even when a valid plan exists but alignment/stabilization has not been achieved.
  An invalid live plan retains the previous height budget for this guard; absence
  of any feasible budget fails closed. This releases automation, not an automatic
  recovery or proof of a survivable landing.
- **Pull-up capability was overstated.** The preflare model no longer raises weak
  positive lift authority to 0.6 m/s² or learned slow AoA response to 1 degree/s.
  Lower capability now increases the required pull-up height instead of being
  hidden by a numerical floor.
- **Preflare airbrakes alternated each frame.** The simultaneous deploy/retract
  request is replaced with explicit retraction throughout sink arrest, preserving
  the maneuver's energy reserve.
- **TAEM checkpoint delivery bypassed recovery.** Restart classification may
  retain final-intercept phase, but final-approach delivery remains pending until
  recovery clears and the current terminal contract is re-evaluated.

## Regression coverage

`Validation/GuidanceIntegrationContractTests.c`, `Validation/EntryAlphaTests.c`,
`Validation/EntryLateralTests.c`, `Validation/ApproachSequenceContractTests.c`,
and `Validation/TAEMExecTests.c` cover the corrections. The new persistent-plan,
thermal-floor, malformed-limit, weak-authority, and recovery-restart tests were
observed failing before their fixes; the focused corrected tests pass.

The updated focused guidance, approach, TAEM, and sanitized offline landing
regressions pass. A clean full sanitizer qualification was not completed: the
make-level run stopped at an existing link failure for `entry_bank_authority_limit`
in the offline landing target. The earlier offline-acceptance attempt also used
the pre-fix 364 m fixture and aborted as expected after the new reserve guard;
the corrected sanitized offline landing target passes.

Physical Entry/TAEM trajectory performance and landing success still require a
separately authorized live campaign; offline contracts do not establish them.
