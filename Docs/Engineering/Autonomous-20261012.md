# KSL reliability campaign — 2026-10-12

## Scope and baseline

Objective: make the existing unified mission application reliable across launch,
orbit, entry, landing and rollout, with usable CLI/TUI, reproducible evidence,
and maintainable boundaries. Work is authorized for live KSP flights; preserve
saves and historical evidence. No subagents.

Starting commit: `305a296`. Existing untracked simulator `.smoke` / `.replay`
artifacts are retained. The repository already integrates the ascent and landing
programs behind `MissionController`; retain that architecture and validated laws.

Initial inspection:

- Native build and fast C regression targets pass. Root `make test` stops because
  the default Python lacks pytest; `ksl status --live` similarly lacks krpc.
  Running the documented `make setup` to establish the project environment.
- Mission subprocess handling has no finally cleanup: an event callback error or
  second interrupt can leave a live controller running in a detached session.
  EOF also leads to an unbounded wait without cancellation. Fix and verify with
  real child processes before further live operation.
- Profile validation coerces values without type checks, does not reject NaN or
  nested unknown keys, and can crash before producing actionable diagnostics.
- Foundation recalled prior live and simulated landing failures; neither the
  unified architecture nor a passing build establishes full-flight reliability.
- No Jev executable or callable Jev tool was found in the initial environment.
  Use direct focused experiments and preserve their evidence.

## Next steps

1. Establish Python and simulation baselines; harden process ownership and abort.
2. Validate configuration and continuation reproducibility.
3. Probe live KSP read-only, preserve its current state, then select useful live
   and offline flight experiments from actual available checkpoints.
4. Investigate mission-chain failures without weakening acceptance gates.

This log is updated at implementation/validation checkpoints.

## Process ownership checkpoint

- `make setup` succeeded. Read-only live probe: STS, Kerbin, paused, landed,
  UT 7103.8301, mass 25122.7637 kg. No flight controls changed.
- Child owners now receive group SIGINT and bounded escalation on cancellation
  or callback failure; EOF no longer bypasses cancellation. Controller records
  unexpected exceptions as failed and honors abort before starting any program.
- Four real-process regression cases pass (1.32 s): normal output cancellation,
  closed-output cancellation, callback exception with cleanup marker and PID
  reaping, and pre-start cancellation.
- Full gate: native tests pass; Python 108 passed + 44 subtests, one L5 golden
  fails (66.735 m/s versus 66.129). Original HEAD mission Python replay in an
  isolated package reproduces the current result, so this is not introduced by
  process supervision. Evidence: `runs/20261011T160713Z-baseline-305a296/`.
  Preserve the expected values while investigating the baseline discrepancy.

## Configuration checkpoint

- Mission profile validation now checks structure/types, finite numbers, unknown
  nested keys, port integers, positive timeouts and rates, and orbit consistency.
  Invalid configurations return field-specific diagnostics rather than tracebacks
  or coercing strings like `"false"` into true. Backend JSON shape is checked.
- `continue` restores the resolved mission configuration saved with the run,
  preserving original overrides; explicit new profiles and overrides still work.
- Config/CLI tests: 8 passed + 7 subtests (including actual continue/dry-run).
  Mission/lifecycle subset before the continuation test: 23 passed + 7 subtests.
- Investigating the L5 golden with a forced native rebuild; initial make reported
  the existing backend up to date, so stale build artifacts remain a hypothesis.

## Runway acceptance and fresh-baseline findings

- Native rollout now requires a finite, nonnegative stopped speed and a stop
  inside the configured runway's actual along/cross bounds. The wider contact
  envelope remains for acquiring touchdown and braking; its +80 m margin no
  longer qualifies a completed landing. The 2026-10-11 live stop at 2556 m on
  a 2500 m runway must therefore fail while braking remains available.
- Six native completion cases (inside, 56 m beyond, 20 m short, 40 m wide,
  NaN speed, negative speed) pass in `mm305-recovery-monitor-test`.
- Full `make test` after this change: native tests passed; Python 111 passed,
  51 subtests passed, one L5 exact golden failure at 66.735 vs expected 66.129
  m/s. A forced native rebuild and a fresh CMake simulator build reproduce
  66.735/2.055/1068.915. Isolated rollback of the two later native changes
  (`terminal_propagator` cadence and disabled adaptive-flare slew) still gives
  the same result. No acceptance gate or assertion was relaxed.
- Saved the current paused/landed STS scene as
  `ksl-autonomous-20261012-pretest-landed` before loading the existing MM305
  checkpoint. Guard confirmed the checkpoint settled at UT 6566.83, 26.9 km.
  Live MM305 continuation with strict runway completion is in progress; its run
  record is `runs/20261011T161341Z-live-runway-bound/`.

## Subprocess orphan bound

- Added a five-second post-exit output-pipe deadline. If a child flight program
  exits while a descendant keeps its stdout pipe open, the mission fails and
  kills the owned process group rather than waiting forever. A real spawned
  descendant test passes; all five process-lifecycle tests pass (6.97 s).

## Connection and checkpoint handoff

- Python kRPC connections now obey `connection.timeoutSeconds` for the handshake.
  A timeout returns an actionable connection failure and closes any client that
  arrives late. The stalled-connection test uses a delayed fake kRPC client and
  verifies cleanup; five subprocess tests and this test pass.
- `ksl run/continue --load-save` now calls the independent guard's `load-save`
  mode and requires its settled-flight confirmation. That guard checks scene,
  warp, target save UT and independent load evidence. The CLI validates profile
  and phase range before any save load. Ten targeted CLI/guard/connection cases
  plus seven subtests pass. The earlier direct `space_center.load` call could
  report a disconnect mid-load and proceed without confirming the target scene.

## Golden replay provenance lead

- The archived forecast study pins source commit `dc12311` and exact backend /
  simulator SHA-256 `83b74e87…` / `985b0d56…`. Those **exact binaries still
  exist** in `/Users/kyooni18/Code/KSL-integration/`. The model, attitude,
  scenario and configuration file digests are identical to the current fresh
  replay. This enables a binary A/B after the live flight; the discrepancy is
  now narrowed to executable builds rather than inputs or mission Python.

## TUI abort ownership

- TUI quit now yields to the UI event loop while waiting for a running mission
  to abort. After 30 s it remains open if the controller still owns flight;
  it no longer exits and kills its daemon mission thread mid-flight. TUI
  targeted tests pass (3 cases), including abort-to-exit behavior.
