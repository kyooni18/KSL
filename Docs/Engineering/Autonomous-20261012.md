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

## L5 golden replay resolved

- Executable A/B with identical scenario/config/model hashes isolates the exact
  golden change to ShuttleSim. Archived backend + archived simulator reproduced
  66.129 m/s touchdown, 1.844 m/s sink, 1101.616 m stop. Archived backend +
  newly built simulator produced 66.735 / 2.055 / 1068.915. Newly built backend
  + archived simulator returned the archived numbers. All variants passed the
  simulator acceptance gates. Archived simulator SHA-256 `985b0d56…` was built
  2026-10-11 18:20, before the 18:24 measured body-state restart commit
  `61fb088`; it was an out-of-date executable at the study's later commit.
- Updated the L5 exact regression and named scenario baseline to the current
  clean-build values; retained the same tight numerical tolerances and all
  runway/contact/gear acceptance gates. The archived golden remains in the
  experiment documents with its original binary hashes. This change reflects
  tested source behavior, not a relaxation of a failing constraint.

## Full offline gate after integration changes

`nice -n 15 make test` passed: native C gates plus 117 Python tests and 51
subtests in 24.22 s. This includes the current clean-build L5 exact replay,
L7 low-energy replay, mission/CLI/TUI integration, guard load verification,
connection deadline, and process cleanup. It is offline evidence only; the
ongoing live flight and end-to-end orbital mission remain separate gates.

## Parallel run records

- Run IDs created in the same second now reserve their directories with an
  exclusive `mkdir`; concurrent CLI/TUI runs cannot select the same path and
  intermingle telemetry or overwrite status. A 16-run concurrent creation test
  passes with 16 distinct directories.

## Live MM305 continuation: runway overrun

- The guarded live continuation from the existing 26.9 km MM305 checkpoint
  finished **failed**, with the STS safely landed and paused. Run evidence is
  `runs/20261011T161341Z-live-runway-bound/`; the postflight scene is preserved
  as `ksl-autonomous-20261012-overrun`. The prior landed scene is preserved as
  `ksl-autonomous-20261012-pretest-landed`. Neither existing checkpoint was
  overwritten.
- Native TAEM delivered Final at UT 6926.5, 2307 m height, 94.6 m/s,
  7730 m before the threshold, and reported low energy. Final warned of a
  37-38% aerodynamic-model residual and a long landing. The first landed
  telemetry appeared at UT 7053.77, about 1720 m down runway 09 and
  55.6 m/s. Rollout began near 1736 m; the backend requested parachute
  deployment at UT 7053.990, 55.27 m/s. It later aborted outside the
  2500 m runway's +80 m contact envelope. Last telemetry at UT 7100.77
  showed 1.49 m/s and 2770.93 m along, 12.09 m cross. This is an overrun,
  not a completed runway landing. The earlier live flight from the same save
  touched near 1674 m at 51.0 m/s and stopped at 2556 m; it had incorrectly
  qualified as complete under the wider contact envelope. The new strict
  completion check is covered by native regression, but this flight hit the
  pre-existing contact-envelope abort first.
- Read-only save comparison shows 67 parts in the pretest landed scene and
  66 after this overrun. The missing part is `benjee10.shuttle.bodyFlap`;
  dry mass is 230 kg lower. The trace does not establish when or why it
  separated. The radial drogue offered `Cut Parachute` after flight and the
  backend logged a deploy request, but kRPC's parachute state RPC raised a
  null-reference error, so inflation is not independently confirmed.
- The backend briefly reported Touchdown again after Rollout, producing
  backward-transition warnings in the mission event stream. This may reflect
  contact oscillation near the runway end; do not silently treat it as an
  ordinary phase progression.

## Early-flare hypothesis matrix

- Re-ran the scratch `Early-Latch-Arc/run.py` four-variant experiment against
  four measured-state replay cases with freshly built source executables.
  Evidence: `Docs/Experiments/STS-Landing-20261011/Early-Latch-Arc/matrix-20261012.json`.
  Baseline succeeds in 0/4; the `on_arc` shortcut removal alone succeeds in
  1/4 (L5), leaving L6/L7/high unqualified. Forcing the live 1820 m early
  latch drives L5, L6 and high off the approach end and L7 off the far end;
  combining it with the shortcut change does not recover the matrix. These
  are unqualified replay plants, not live predictions. No experimental law
  change is applied. Next investigation must address TAEM delivery energy,
  preflare timing and live aerodynamic residual together, then recheck all
  cases and a guarded live continuation.

## Orbit checkpoint: load/physics race isolated

- The first guarded orbit-to-rollout attempt from
  `ksl-20261011T062628Z-live-85km-10deg-orbit` selected a strict deorbit
  plan (13/15 stress cases, burn UT 5586.7, 30.02 m/s), then stopped safely
  before burn: the planner had read 22,000 kg, while the live vessel read
  25,521 kg before burn, beyond the unchanged ±3% mass envelope. Run record:
  `runs/20261011T163547Z-live-orbit-chain/`. A paused postabort scene was
  saved separately as `ksl-autonomous-20261012-orbit-mass-mismatch`.
- Repeated load of the same orbit save reproduced KSP's transient value:
  independent kRPC samples reported 22,000 kg from UT 5248.759 through
  5249.599, then 25,521.25 kg at UT 5249.879. The existing load guard had
  declared the flight scene settled around UT 5248.66, before resource/part
  mass initialization completed. This is a checkpoint handoff race, not a
  16% physical mass change during coast.
- Orbital save loads now wait five simulation seconds after the loaded clock
  before allowing a plan. Descending saves retain the previous quick handoff.
  A focused regression checks rejection of the 22 t early sample and
  acceptance of the later finite mass; all 14 guard tests pass. The same
  checkpoint loaded with the new guard at UT 5253.659 and 25,521.25 kg.
  Next: exercise the integrated CLI path and actual deorbit burn.

## Slow native qualification gate

`nice -n 15 make -C CLanding qualification` failed at the existing MM305
feasibility non-vacuity check. Five fixture states were physically admitted,
zero found a qualified handoff route, and five stayed under MM304 ownership;
there were zero contract violations. Individual rejections include insufficient
energy for the route, propagation timeout, and slow Final alignment. The fast
native and Python gates passed earlier, but this slow gate remains open. Its
full output is preserved as `Docs/Engineering/qualification-20261012.log.gz`
and the command is reproducible from the source checkout. No constraint or
assertion was weakened.

## Full offline gate after orbital load fix

`nice -n 15 make test` passed: native C gates plus 119 Python tests and 51
subtests in 25.56 s. This includes the orbital post-load regression and the
parallel run-directory reservation test. The slow MM305 qualification failure
above remains independent of this fast gate.
