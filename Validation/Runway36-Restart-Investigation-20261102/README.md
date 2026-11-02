Recorded-state diagnostics only; no live actuation.

- `provenance.json`: original restart log/plan/configuration identities.
- `timeline.json`, `summary.json`, `entry-proposals.json`: compact evidence extracted from the original restart tape.
- `handoff-record.json`, `previous-record.json`, `range-sample.json`: genuine measured snapshots.
- `replay-check.json`: frozen release replayer, 4,240 records, zero mismatches.
- `probe.swift`, `handoff-probe.log`, `range-probe.log`: temporary diagnostic test source and results. Source copies the flown TAEM engine and range reference with print instrumentation. It is deliberately outside the test target. Raw TAEM target precedes the mission target limiter; compare status/geometry here, not raw target directly with limited flight target.

To recreate the untracked 81 MB replay prefix, run `python3 Validation/Runway36-Restart-Investigation-20261102/extract-prefix.py`. This reads the preserved original run, stops at first acquisition and verifies 4,240 records. The diagnostic source reconstructs the actual authority resets from recorded dynamics changes. The full prefix replay separately establishes exact command reconstruction.

Replay command used:

```
swift/KSL-Landing/.build/release/ksl-landing replay --input Validation/Runway36-Restart-Investigation-20261102/entry-through-handoff.jsonl --configuration Validation/Runway36-Restart-Investigation-20261102/configuration.json --verify-commands
```

Temporary probes were individually run with `swift test --package-path swift/KSL-Landing --filter ZZRunway36InvestigationTests` and `--filter ZZRunway36RangeInvestigationTests`, then removed from the build target. Passing means the diagnostics executed, not that a landing is qualified. No vehicle, atmospheric or actuator capability was altered.
