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

## Live orbit-to-runway completion

- The integrated CLI retry `runs/20261011T164103Z-live-orbit-retry/` completed
  **Orbit → Deorbit → MM304 → MM305 → Final → Rollout → Complete** with exit
  code 0. It loaded the original 85 km / 10° orbit save through the corrected
  guard, planned at the settled 25.5 t mass, and flew a 30.06 m/s deorbit
  burn. The selected plan was entry-stage executable but not strict full-route
  robust (10/15 stress cases); the profile explicitly allowed guarded
  recovery execution. All runtime handoff and cutoff checks remained active.
- MM304 admitted MM305 at UT 6575.33, 26.705 km, 867.75 m/s, 63.19 km
  before and 32.85 km cross from the threshold. The automatic MM305 save
  `ksl-20261011T164103Z-live-orbit-retry-mm305` was preserved. TAEM reached
  Final late, near UT 6838 / 3 km before the threshold; the backend warned
  that live aerodynamic response exceeded its certified uncertainty. The
  flare nevertheless arrested the steep sink before the threshold.
- First landed/rollout telemetry was UT 6876.667, 579.51 m along runway 09,
  5.58 m left of centerline, 52.37 m/s, vertical speed -0.36 m/s. The backend
  requested drogue deployment at UT 6876.607 / 52.38 m/s; module events
  afterward offered `Cut Parachute`, but inflation was not independently
  confirmed. Completion at UT 6914.187 was 1.545 m/s, 1466.01 m along and
  3.46 m left of centerline, within the configured 2500 m runway bounds.
- Independent kRPC probe after completion found STS paused and landed at
  UT 6914.487, throttle 0, gear and brakes on, SAS off, 22 orbiter parts
  including the body flap, and dry mass 20160.01 kg. The scene is preserved
  as `ksl-autonomous-20261012-orbit-runway-success`. This is one successful
  orbit-to-rollout live flight, not a qualification of every initial state,
  the slow MM305 fixture, or pad launch/ascent.

## Replay clock correction

The live `ksl replay` command initially mixed its first native UT=0
placeholder with wall-clock time, displaying a huge negative duration and
timeline. Replay now chooses one clock for the entire recording, skips that
placeholder for elapsed time, retains its entry phase at T+0, and appends
the recorded Complete event after the last telemetry frame. The same live
run now reports 1659 s and a monotonic Orbit → Complete timeline. Four
focused replay tests pass, including the clock regression.

## Full pad-to-runway flight: structural abort

- A separate `quicksave` held a 41-part STS stack on the launch pad at
  UT 2682.93. The completed orbit-to-runway scene was already saved, so
  `bin/ksl run -p sts-85km-10deg --load-save quicksave --json --label
  live-pad-chain` began a full mission without overwriting either save.
  Run record: `runs/20261011T171716Z-live-pad-chain/`.
- Preflight measured a 520.7 t stack and identified three SSMEs, two
  Clydesdale SRBs and two AJ10 OMS engines. Liftoff, heads-down roll and
  SRB separation completed. PEG MECO yielded Ap 86.9 km / Pe 53.2 km;
  ET separated and the orbiter transferred to OMS trim.
- OMS trim corrected the orbit in two burns to Ap 85.001 km / Pe 84.975 km /
  inclination 9.99993°. The first burn over-raised apoapsis to 94.59 km;
  closed-loop replanning corrected it with a 7.88 m/s second burn. The
  `aim error` log values (17.7° and 86.1°) require separate diagnosis because
  the measured final orbit is on target. The automatic orbit save is
  `ksl-20261011T171716Z-live-pad-chain-orbit`.
- The deorbit search selected a strict 13/15-stress-case plan, burn UT
  3933.12, 30.04 m/s. Burn completed and MM304 entry began. At about
  UT 4697 / 50 km the damaged orbiter entered attitude recovery; roll
  approached 180° and heading diverged. KSP was paused at UT 4729 / 47 km,
  the controller was interrupted, and safe control was independently
  confirmed (zero throttle, SAS off). This failed scene is preserved as
  `ksl-autonomous-20261012-pad-entry-departure`. No touchdown is claimed.
- Save-file comparison found the pad save had 41 parts, including both
  `benjee10.shuttle.deltaWing` assemblies, four elevons, and two main gear;
  the new orbit checkpoint had only 14 orbiter parts and none of those eight.
  The earlier successful orbit-start and landed saves each retained all 22
  orbiter parts. KSP.log at 02:17:45 records both deltaWing assemblies
  becoming separate four-part vessels at clamp release; at 02:17:50 Collision
  Enhancer exploded a detached main gear. The wing loss preceded SRB separation
  and was not an entry-guidance-only failure. The existing ascent part-loss
  check began after liftoff, so it missed the loss at staging. The wings had
  node parents on the mid fuselage with Heaviest autostruts in `quicksave`,
  while the clamps attached to the aft fuselage and ET. Physical joint failure
  at release is the leading hypothesis; the exact joint stressor remains to
  be tested. The new orbit save is unsuitable as a reusable flight checkpoint.

## Clamp-release integrity guard and joint experiment

- Ascent now compares the pre-staging part count with the count immediately
  after clamp release, allowing only the released clamp parts to disappear;
  it cuts throttle and aborts before climb if anything else is lost. Benjee STS
  wing/elevon/gear inventory is also required at ascent preflight and before
  the landing backend can plan or engage. The failed live scene itself
  exercised the latter guard and was rejected with the missing eight parts.
  Offline mission tests (28) and ascent tests (32) passed after the change.
- A derived copy of `quicksave` with both wing `rigidAttachment` flags enabled,
  `ksl-autonomous-20261012-pad-rigidwings`, was launched to an orbit hold.
  It still lost one complete four-part wing assembly at clamp release:
  expected 37 parts after four clamps, found 33. The new guard stopped the
  launch at UT ~2698 / ~88 m, and the scene was saved as
  `ksl-autonomous-20261012-rigidwing-liftoff-failure` with throttle zero.
  Rigid wing joints alone are therefore rejected as a fix.
- In the original pad scene the two aft-fuselage clamps are attached near
  x=±2.41 m, y=-12.91 m; wing roots are near x=±3.41 m, y=-10.57 m. A
  controlled release of a single aft-fuselage clamp *without engine thrust*
  removed one entire four-part wing assembly. This failed scene is
  `ksl-autonomous-20261012-pad-aft-clamps-released`. Clamp/wing interference
  is now a much stronger explanation than launch thrust or entry control.
- A second derived copy of the original save removed only the two aft-fuselage
  clamps while preserving the two ET clamps. Its serialized vessel graph
  validates at 39 parts, two wings, two ET clamps; KSP loaded it at the expected
  520.7 t and the wings stayed attached. After 25 seconds of pad settling KSP
  marked the clamped stack `LANDED` rather than `PRELAUNCH`, so the first ascent
  attempt stopped in preflight. The launch validator now accepts a nearly
  motionless landed STS still held by at least two launch clamps. This is
  bounded to pad-like conditions and does not permit a free landed craft.
  The second no-aft-clamp ascent ran from
  `ksl-autonomous-20261012-pad-no-aft-clamps`. The full offline gate passed
  again (122 Python tests, 51 subtests, native checks).
- This ascent cleared liftoff with 37 parts and retained both wings and both
  main gear at 1.1 km. It reached SRB separation at about UT 2793 / 25 km.
  An independent check at UT 2805 / 35 km found 21 parts with zero wings,
  elevons, or main gear. KSP.log records a deltaWing vessel split at
  03:04:46, coincident with booster separation, then the second split about
  0.3 s later. The mission was paused and aborted before orbit. Throttle had
  remained at 1.0 despite the abort while paused, so an independent guard
  briefly resumed physics, commanded zero throttle, paused again, and saved
  `ksl-autonomous-20261012-srb-wing-loss-safe` at UT 2807.21.
- Removing aft clamps therefore resolves only the liftoff wing collision;
  the booster separation is a second structural failure. The booster-mounted
  Mite separator motors, booster trajectory, and wing `Heaviest` autostruts
  are candidates; none is yet proven causal. Ascent now checks the essential
  orbiter inventory one simulation second after booster separation and again
  after ET clearance, so neither damaged vehicle can be certified as an
  orbital checkpoint.

## Root-autostrut SRB replay

- A derived ET-clamp-only pad save changed only the two wing autostruts from
  `Heaviest` to `Root`. It retained both wings through liftoff. A clean
  pre-SRB checkpoint was saved at UT 2787.05 / 20.9 km with 37 intact parts:
  `ksl-autonomous-20261012-pre-srb-rootwing`.
- The unchanged booster-separation program still detached one four-part wing
  assembly. The new one-second post-SRB inventory guard aborted automatically
  at about UT 2794 / 26 km; a paused, zero-throttle scene is preserved as
  `ksl-autonomous-20261012-rootwing-srb-failure`. Root autostruts improved the
  loss from two assemblies to one in this trial but did not solve it. The
  pre-SRB checkpoint now allows targeted separation tests without replaying
  the entire climb.
- Two direct kRPC separation experiments from that checkpoint moved separation
  earlier to about UT 2788.5–2788.8 (versus automated UT ~2793). With Mites
  inhibited, both wings and all four elevons remained after four simulation
  seconds; with Mites enabled, the same inventory also survived four seconds.
  These paired trials do **not** identify the Mite motors as the cause. The
  changed separation timing and absence of the active ascent guidance remain
  confounders. Both experimental scenes were paused, commanded to zero
  throttle, and saved separately.
- A third direct separation waited for both Clydesdales to report no fuel,
  then separated at UT 2793.55 / 25.7 km with Mites enabled. Both wings also
  survived four subsequent simulation seconds. However, without the ascent
  attitude controller this replay had drifted into a very different attitude
  (at UT 2798: pitch -23°, heading 268°, roll -15°). This does not exonerate
  the Mites under the nominal controlled geometry. The next controlled test
  must retain the normal ascent program and inhibit Mite thrust in the pad
  save, changing only that variable.
- That controlled no-Mite-pad trial did not reach launch. During the 25-second
  settle, an ET clamp and a booster branch detached from the two-clamp pad
  stack; preflight correctly rejected the moving `LANDED` vessel. The scene
  was paused and saved as `ksl-autonomous-20261012-no-mite-pad-failure`.
  The only save-file difference was two Mite `SolidFuel` amounts changed from
  40 to 0, so this result indicates the ET-only pad support is not reliably
  stable across reloads; it does not assess Mite influence at separation.
- The next controlled full ascent uses the original four-clamp quicksave with
  both wing joints set to `rigidAttachment=True` and `autostrutMode=Root` in a
  new derivative save, `ksl-autonomous-20261012-pad-fourclamp-rootrigid`.
  Neither original quicksave nor prior experiment is overwritten.
- Four-clamp Root+Rigid still lost both wings at clamp release (29 versus
  expected 37 parts); its scene is `ksl-autonomous-20261012-fourclamp-rootrigid-failure`.
  Moving the aft clamps inboard from x=±2.41 to ±1.0 m still lost one wing
  (33 parts), saved as `ksl-autonomous-20261012-inboard-clamp-failure`.
- Moving both aft clamps onto the ET avoids attaching clamp hardware to the
  orbiter, but the first serialized copy was invalid: the moved child PART
  records still appeared before their new ET parent and KSP loaded only a
  13-part fragment. This trial never ignited. A corrected derived save reorders
  those two records after the ET and remaps all top-level part references.
  KSP then loaded all 41 parts, 520.7 t, two wings, and four ET-parented
  clamps; an independent probe at UT 2688 confirmed the complete graph.
  The corrected save is `ksl-autonomous-20261012-pad-four-et-clamps-ordered`.
- The corrected four-ET stack passed the 25-second pad settle and released
  all four clamps without part loss. An independent probe just after liftoff
  (UT 2703 / ~125 m) found the expected 37 parts, two wings and both main
  gear. The ascent is continuing to SRB separation under the existing
  guidance. This identifies clamp attachment/order as a viable liftoff fix,
  but does not yet establish an intact orbital vehicle.
