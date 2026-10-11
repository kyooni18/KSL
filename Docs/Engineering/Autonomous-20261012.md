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
