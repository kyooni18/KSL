# Terminal path contract screen — 2026-09-20

## Purpose

The simulator's terminal curriculum can begin inside the local MM304 handoff
set. That local contract is not proof that MM305 has a flyable runway path.
This screen verifies that candidate selection, commitment, and RL accounting
keep those contracts separate.

## Production preview correction

The terminal selector now rejects a response-propagated future origin when the
shared bounded-curvature path from the live state cannot reach that origin in
the candidate's forecast horizon. The commitment gate already made the same
check; applying it during selection prevents an unreachable preview from being
retained by refinement hysteresis. This change does not relax geometry,
energy, response, or vertical gates.

## RL contract

The adapter exports the nested production diagnostics as a terminal-path
contract:

- a candidate must be valid and non-degraded;
- its live-energy envelope must be valid with a finite non-negative margin; and
- terminal path readiness requires `path_committed=true`.

`initial_handoff` and `curriculum_handoff` remain local MM304 diagnostics. A
terminal-corridor reset that starts inside the handoff tube is not counted as a
handoff event or terminal-path progress. Validation promotion reports
`terminal_path_unproven` when terminal-corridor episodes never observe a
committed path.

## Results

Fresh terminal-corridor seeds 21, 22, 121, and 122 all ended in
`unsafe_or_abort` with no touchdown. Each had:

```text
handoff_event_seen=false
initial_terminal_path_ready=false
terminal_path_candidate_seen=false
terminal_path_committed=false
```

The final blockers were `candidate-invalid`, `energy-invalid`, and
`path-not-committed`; this is a fail-closed diagnostic, not a claim that the
airframe reached a valid terminal path.

## Verification

Passing after the correction and accounting changes:

- `make -C CLanding -j1 all`
- `cmake --build ShuttleSim/build-offline -j2`
- `make -C CLanding -j1 native-component-tests`
- `make -C CLanding -j1 terminal-preview-scheduling-test`
- `python3 -m unittest discover -s ShuttleSim/rl -p 'test_*.py'` — 36 passed,
  9 skipped
- `git diff --check`

The broader qualification target remains red at the pre-existing
`MM304OfflineTests.c:348` synthetic AoA-extension assertion. No live-KSP
flight was started and no touchdown or rollout was observed.
