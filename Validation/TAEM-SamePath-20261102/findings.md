# Same-profile route admission and downmode — 2026-11-02

## Scope and cause
Terra's report established that at 5.1 seconds after the recorded handoff, legacy q-line EMIN refreshed from about 36 km to 54.37 km energy-height. Available energy was 51.22 km. This triggered replacement of the 48.707 km / 6.313 rad committed route with a 29.851 km / 0.035 rad reference. The candidate admission check only bounded geometric slope, although the newly commanded Hermite profile differed from the trajectory priced by EMIN.

## Change
Initial selection and downmode now use the same-profile necessary force/energy check. It integrates drag work forward from measured energy at the Hermite height, slope and curvature; uses the candidate's HAC curvature (arclength inverted) and acquisition arc curvature; solves lift and incidence at that state; checks positive normal lift, trim/lift with configured uncertainty, bank, structural load and q. No clipping of an impossible demand into a fictitiously affordable route is allowed. Gear deployment is included. Nonzero wind is explicitly unpriced by this still-air check. Legacy EN/EMIN are retained as advisories and for the existing S-turn law, but cannot admit or reject a route for selection/downmode.

Downmode requires the alternative to complete that same-profile check with at least the NEP energy. Failure of the current route is not evidence that a shorter route is attainable. When none passes, guidance keeps an unqualified overhead recovery and exposes the physical failure in TAEM status and mission diagnostics. It does not abandon control.

This is a necessary point-mass screening model, NOT a full actuator/closed-loop certificate. It uses the current supported trim ceiling, the existing acquisition geometry approximation, and a local atmospheric reference. It does not propagate actuator/trim history internally, price wind, or prove tracking response. The matched-state continuation below does preserve controller/trim history. Passing the screen must not be represented as a landing certificate.

## Decisive results
- The original seven-second same-state replay confirms no downmode and no 19 km route collapse at 5.1 s. Inputs, controls and controller warmup match Terra's harness.
- Four focused regressions pass, including the actual recorded shortened reference: geometric slope fit passes, but its required upward normal acceleration is negative. TAEM's positive-lift/bank envelope cannot fly it.
- The new check also rejects the original overhead Hermite profile for negative normal lift. Thus retaining the original route alone cannot solve the descent. At 40.1 s the retained-route continuation is still near the old ballistic descent (741.5 m/s, 17.31 km height); the invalid shortened route was not the sole cause of that descent.
- One full continuation after the decision change ends in unqualified Final recovery at 350.3 s, height 961.23 m, along -4331.28 m, cross +5344.69 m, airspeed 41.32 m/s. No downmode occurred. This FAILS terminal delivery; no landing/rollout success is claimed.
- The later HAC acquisition/progress issue remains: reported remaining turn grows above its committed entry value around acquisition, then range closes faster than flown distance near the center. This is separate from the removed 5.1 s downmode.

## Next actual engineering work
Generate an attainable descent reference rather than fitting only endpoint height and slopes with a cubic that demands negative normal acceleration. Couple altitude/slope evolution to available normal and lateral force, energy and response; keep candidates consistently priced. Also trace the actual HAC join and phase/progress geometry before claiming planned flight. Do not mask these failures with an altitude gate, speed cap, relaxed bank/q limits, or by declaring the retained route feasible.

## Artifacts
`switch-test.txt` and `switch.json`: first focused seven-second decision replay.
`selection-test.txt`: same replay after applying the common check to initial selection as well.
`continuation-test.txt` and `continuation.json`: four regression checks plus the single full continuation; a passing scratch assertion is NOT a landing pass.
`ZZSamePathDiagnosticTests.swift.archived`: exact full continuation harness, removed from test target afterward.
`release-build.txt`: release build.

Source seed: da7cfff. Configuration and recorded handoff are unchanged from `runs/20261102T040424-mm304-shooting-live-72km/configuration.json` and `Validation/TAEM-Planned-20261102/handoff.json`. No live commands, loads, monitoring, or new run occurred. Unrelated working-tree changes were preserved.
