# KSL: Best-Effort Mission Admission and Recoverable Guidance

**Date:** 2026-10-13 (KST)  
**Scope:** Unified KSL Python mission controller, headless operator, and C native guidance regression. No live KSP controls were issued.

## Intent

**A failed forecast, threshold, physical-energy qualification, or missing terminal route is not by itself an executable-program failure.** KSL must attempt the best available physical trajectory where there is a usable plan and a working guidance/telemetry connection. The software reports uncertainty and unsuccessful outcomes honestly rather than reclassifying them as validated successes. Replanning and bounded control remain active during the attempt.

## Implemented behavior

- `Configuration/missions/sts-polar.json`: `landing.requireStrictPlan=false` by default. A separately requested strict mission (`--set landing.requireStrictPlan=true`) still explicitly opts into preflight certification blocking. This does not change the C planner's physical envelope, HAC energy accounting, or stress thresholds.
- `ksl/backend.py`: `validate_plan_qualification()` now issues nonfatal warnings for an unqualified **existing** candidate. Both failed entry qualification and failed MM305/Final qualification are visible. It does not mutate `executionQualified`, `targetCaptureAchieved`, or robustness counters. Explicit opt-in strict admission, an empty/nonexistent plan, or a disabled warning policy can still reject execution.
- `ksl/programs.py`: a degradable candidate is logged as a `guidance-degraded` event with its qualification status and terminal energy margin, followed by the actual `engage` command. The independent STS structural-inventory probe now reports a warning on failure and no longer automatically cancels a connected flight attempt.
- `Tools/headless_flight.py`: warning-only admission is the default for non-strict execution; `KSP_LANDER_CONTINUE_UNQUALIFIED=0` explicitly restores blocking of entry-unqualified plans. The CLI's diagnostic print reflects actual qualification instead of assuming entry-stage pass.
- The native `guidance_update_with_terminal_model()` recovery layer already converts internally rejected terminal solutions to **bounded control with a warning** while the vehicle is airborne. It releases invalid MM305 route commitment for replanning; it does not automatically disengage or report success. Added a regression with two consecutive ticks of missing native terminal route to preserve this contract.

## The state machine distinction

| Situation | Action | Reported outcome |
| --- | --- | --- |
| Planner proposes a candidate, 0/15 strict stress passes | Attempt with warning | Not certified |
| Entry qualification is false but a real plan exists | Attempt with warning | Degraded |
| MM305 route invalid/unavailable in flight | Bounded runway-directed control and retry | Degraded, not a successful Final |
| Structural probe finds missing STS components | Warn and attempt with connected native backend | Degraded vehicle |
| Operator explicitly requests strict qualification | Reject uncertified mission before execution | Intentional policy stop |
| No candidate plan, no control transport, explicit operator abort, actual crash/ground stop | Cannot generate valid flight commands, or finish physically | Failure/abort, not reclassified |

## Offline verification

- `tests/test_mission.py`: simulated live mission from **Deorbit** to Complete with an unqualified route and an unqualified Entry plan. Both cases triggered actual backend `engage`, preserved a `0/15` failure record, and logged warnings even when the independent structural check failed. This is a scripted backend test, **not** proof of KSP touchdown.
- `Validation/test_headless_backend.py`: default nonfatal warnings, optional explicit hard fail, and persistent strict policy checks.
- `Validation/MM305RecoveryMonitorTests.c`: missing native MM305 route while airborne returns a bounded TAEM command, zero throttle, warning and continued automation over two successive ticks. Actual ground contact still triggers the usual braking and failed-stop terminal outcome when appropriate.
- `make test`: the project's native and Python regression suite (see final test results in the tool log).

## Constraints

Best effort **does not** mean commanding undefined controls from nonfinite state, unvalidated live kRPC transport, inventing thrust, silently ignoring operator aborts, or relabeling physically impossible landing attempts as successful. Aerodynamic load, bank, AoA, gear, throttle and attitude limits remain enforced by the native controller. The user can run explicit experiments without being blocked by pessimistic model confidence, but the software must preserve diagnostic evidence for the next iteration.
