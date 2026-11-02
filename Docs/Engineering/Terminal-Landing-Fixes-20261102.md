# Mainstream terminal landing corrections

## Evidence and changes

Source evidence: `runs/20261102T042609Z-deorbit` and native session
`67DFCE0A-B86B-4F1E-A4C1-165E06B74D2B`. The regression fixture contains
unmodified recorded vehicle states and commands, not simulated continuations.

- TAEM now inherits the configured outer glide envelope when no separate TAEM
  envelope is supplied, rather than admitting a vertical descent. Recovery
  retains normal-force support before spending remaining lift on turning.
  A rejected descent reference no longer makes the existing steep flight path
  its recovery target. The configured 24-degree envelope is not a guarantee
  that the vehicle can instantaneously recover to it.
- Terminal bank targets use the configured/identified response horizon and
  entry-rate fallback instead of the generic 18-degree/second slew allowance.
  Maximum bank and target no-overshoot protections are retained.
- The normal mission controller owns speedbrakes through TAEM and final.
  Feedback uses measured energy and an independently enforced trim/lift and
  retraction-energy reserve. Unknown deployment drag is not invented or
  credited to the planner. The former terminalSpeedbrake compatibility setting
  cannot disable this normal control path.
- Flare float guidance closes the actual touchdown objective within the
  configured touchdown sink limit, rather than preserving a late float.
- Rollout requires current gear support for wheel braking, uses observed
  stopping work rather than nominal friction as a stopping-distance promise,
  and defaults to automatic rollout parachutes.
- STS mission defaults select Configuration/sts.json. No experimental profile
  or opt-in flag is required for these fixes.

## Validation and integration

Release build passed. Sixteen focused checks passed: five recorded-flight
regressions, six TAEM transition tests, four operator configuration tests, and
the target-limiter no-overshoot regression. The ordinary STS CLI dry run selected
the Swift backend and Configuration/sts.json; nothing was flown.

All local and remote-tracking branch tips were checked with
`git merge-base --is-ancestor` and are contained in main. No branch deletion,
remote push, save loading, or live flight was performed. Existing unrelated
working-tree changes were preserved.

## CLI/TUI rebuild and wiring

Re-ran `make swift ascent` after checking no native flight executable was
active. The release products are current (incremental build passed).
Corrected `swift/KSL-Landing/bin/landing_backend`'s standalone fallback from
`sts-taem.json` to `sts.json`; explicit per-run configuration is still preserved.
`bin/ksl validate -p sts-kerbin --json` passed and its MM304 dry run selected
this wrapper and the mainstream configuration. A headless Textual mount
verified both quick-Landing and Console configuration fields and the entry
operation's resolved Swift-backend request. Vessel probing was disabled for
that check; no connection or flight was initiated.

Flight readiness remains unproven: historical-state checks and a successful
build do not establish a successful full-chain landing or runway stop. Final
validation requires an explicitly authorized orbit-to-sustained-stop live run
with intact vehicle and continuous fresh telemetry.
