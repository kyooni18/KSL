# Decision-Architecture Engineering Record

## Scope and preservation

This work reviewed and locally refactored decision logic in the native landing stack and its offline simulator clients. The repository has no commits and most production source is untracked, so no Git diff can reliably identify ownership of pre-existing code. Baseline source hashes are retained in `baseline-source-sha256.json`; all work remains local and uncommitted. Existing flight logs and simulator artifacts were preserved. No cleanup, training jobs, alternate project copies, Skyline, or subagents were used.

## Baseline

Before editing, the source, build targets, audit tooling, tests, active processes, and disk capacity were inspected. Approximately 111 GiB was free.

The decision-literal audit reported 2,522 findings, of which 2,501 were active. The largest concentrations were `guidance.c` (1,025) and `predictor.c` (429). This inventory mixes materially different kinds of literals and is not, by itself, a measure of architectural correctness.

The four deterministic cases labeled as terminal-corridor tests did not exercise TAEM at baseline. They remained in `ENTRY_ENERGY`, never obtained a valid TAEM target, held zero commanded bank, moved away from the handoff region, and eventually crossed the minimum-speed boundary. Their outcomes are therefore not evidence of an MM305 path-tracking defect.

## Assumptions and literal classification

The audit has not been fully classified. The working classification used during this change was:

1. **Physical/model constants** — planetary, atmospheric, vehicle, actuator, or geometric quantities that belong in a named model or configuration.
2. **Numerical implementation constants** — integration tolerances, finite guards, iteration limits, serialization bounds, and logging tolerances. These require safety and convergence justification, but are not necessarily behavioral policy.
3. **Mission constraints** — runway geometry, structural/q/G limits, minimum speeds, and delivery/contact requirements. These should be explicit contracts or configuration.
4. **Behavioral policy** — phase gates, speed multipliers, altitude shells, special-case margins, and fallback choices that select a trajectory or executive outcome. These require candidate-specific physical justification and are the principal architecture concern.
5. **Diagnostic/test constants** — deterministic seeds, fixture values, and output thresholds. These must not silently become flight policy.

No claim is made that all 2,522 findings have been assigned to these classes. Remaining Entry and terminal special cases—including speed multipliers, altitude-dependent policies, and seed-history comments—still need review.

The recovery work assumes current-state local dynamics over a short response/braking interval, finite positive model inputs, a valid atmosphere table, and conservative available lift after accounting for bank and flight-path angle. It does not assume that a preferred trajectory is feasible merely because one deterministic seed progresses farther.

## Changes

- Lateral capture now includes displacement accumulated during actuator response delay; previously delay was added to elapsed time without its associated drift.
- A shared vertical-recovery envelope now reports response height, braking height, total required height, recovery time, and signed height margin.
- Pitch capture timing moved into the shared decision-envelope module. Recovery uses modeled pitch/roll response rather than instantaneous response or a historical phase-delay proxy.
- Historical sink-acceleration EMA is no longer treated as current recovery authority. Vertical support accounts for bank and flight-path angle.
- Final-delivery qualification now includes preflare height margin.
- The contradictory preflare boundaries were replaced by one shared recovery margin and a one-controller-sample forecast. Outer-final ownership and runway alignment must already be delivered; acquisition cannot use preflare as an alignment shortcut.
- One shared atmosphere loader was added to the existing simulator adapter and used by the backend, offline expert, and UDP runner.
- Offline expert diagnostics now expose executive ownership, target validity, path commitment, delivery blocks, and preflare state.
- MM304 handoff timing now evaluates the explicit 1 km spatial and perpendicular-heading capture set. An exact-point Dubins query is retained only as a path diagnostic; it cannot turn a sub-degree residual into a spurious full-revolution veto.
- Direct terminal splines now carry the measured-to-forecast response segment before their future-origin cubic. The forecast tangent cannot command the current frame before the vehicle reaches the propagated origin.
- Candidate live-energy work uses the same projected future AoA that selected the candidate, avoiding a fixed-HAC-AoA re-evaluation of a direct spline.
- The RL residual projector handles one-sided physical headroom without discarding an approved expert baseline. Simulator curricula now record an initial handoff explicitly instead of letting the first physics tick erase a valid 1 km-set state.
- Configuration-dependent simulator guidance metadata is now prepared by one shared post-decode adapter used by the offline expert and UDP runner. This covers physics/aerodynamic confidence, configured vehicle fallbacks, measured response authority, site-relative geometry, and the simulator vessel identity; each consumer retains its own transport-specific rate and ground-track reconstruction.
- Production reentry rejection now reports topology failure flags, selected bank/AoA, terminal-turn timing and geometry, capture timing, position error, and course error. The simulator campaign records this synchronous engagement rejection as `rejected` with the warning and phase instead of waiting for a generic wall timeout.
- Focused recovery/model tests and a deterministic native-command/telemetry smoke runner were added. The smoke path does not apply RL observation normalization.
- Missing dependencies in affected test and simulator-runner targets were repaired.

## Demonstrated defects and evidence

### Offline atmosphere initialization

The simulator loaded the fitted atmosphere CSV, but the offline guidance expert initialized planetary constants without atmosphere samples. Production density evaluation requires the table and returns invalid data without it, preventing construction of a valid TAEM speed-envelope target. This is a demonstrated model-initialization defect, not a complete explanation of landing failure.

After the atmosphere fix, all four deterministic runs acquired valid targets, generated bank commands, and entered attitude recovery. None reached TAEM ownership or touchdown:

| Seed | Baseline stop | After fix | Final altitude | Final speed | Final phase | Touchdown |
|---:|---:|---:|---:|---:|---|---|
| 0 | 268 s | 217 s | 6,095.020 m | 84.933 m/s | `ENTRY_ENERGY` | No |
| 1 | 272 s | 217 s | 6,050.673 m | 84.745 m/s | `ENTRY_ENERGY` | No |
| 2 | 256 s | 223 s | 6,096.578 m | 84.704 m/s | `ATTITUDE_RECOVERY` | No |
| 3 | 260 s | 218 s | 6,525.178 m | 84.899 m/s | `ENTRY_ENERGY` | No |

These changed trajectories demonstrate usable model plumbing, not improved landing performance.

### Handoff and terminal-path follow-up

After the atmosphere initialization fix, a deterministic state at the MM304
handoff point (`along=-8 km`, `cross=0`, runway-perpendicular course) acquired
the target with veto `0` and a positive handoff turn margin. The previous
exact-point Dubins calculation reported a full-turn path for the same nearly
zero residual heading error; using the explicit handoff set removed that
geometric singularity while preserving the late-arrival regression.

The next trace exposed two separate downstream facts. First, a direct spline
was initially allowed to use its response-propagated future tangent in the
current frame. Adding the current-to-future lead segment changed the requested
bank from a nonphysical immediate reversal to a response-compatible lead and
removed that command-generation defect. Second, the same state still had a
negative candidate live-energy margin (approximately `-8.6 kJ/kg` at
`h=21.7 km`, `V=500 m/s`), so its terminal candidate was not commit-ready.
The path preview subsequently lost its vertical reserve as the vehicle moved
away from the lead. This is an infeasible terminal-path sample, not evidence
that the airframe lacks roll authority.

The current `terminal-corridor` curriculum therefore proves an interface
condition, not a landing condition: being inside the MM304 handoff set does
not certify that an unpowered MM305 path has positive candidate-specific
energy margin. No terminal-corridor probe has produced touchdown or rollout.

### Shared simulator telemetry preparation

The offline expert and UDP guidance runner previously repeated the same
configuration-dependent post-decode assignments. They now call
`shuttle_sim_prepare_guidance_telemetry()` from the shared simulator adapter.
The refactor was deliberately limited to common metadata: published and
derived rate behavior remains owned by the respective consumer, and the UDP
runner still reconstructs surface course from successive vehicle states before
the shared site-relative geometry is refreshed. The standalone telemetry
semantics test and the full native component target pass after the change.

This closes a real duplication/parity seam, but it does not establish full
predictor/runtime agreement or live-flight success. The terminal-path energy
shortfall identified before this metadata-only refactor remains the active
feasibility issue and continues to require an upstream-causal investigation.

The retained post-lead four-seed smoke (`lead-native-smoke/`, 120 s horizon)
made that boundary reproducible: all four seeds entered `TAEM`, then aborted
after 18--27 simulated seconds with no touchdown. Final cross-track errors
were 9.24--13.51 km and the recorded diagnostic reason was that the terminal
path optimizer had exhausted maneuver margin without a finite regular
geometry. This is a valid failure record and not a landing result.

### Current entry-topology qualification

The current-build postburn campaign now reaches synchronous production
qualification and fails closed with an explicit engagement rejection. The
latest retained manifest is
`ShuttleSim/runs/current-rejection-diagnostic-20260920-b/manifest.json`.
The vehicle enters the atmosphere and passes near the site (`0.3 km` closest
approach), but never obtains TAEM ownership. The topology result is invalid
with failure flags `0x2c102`; its selected fallback has initial/turn bank
`14.0/49.0` degrees and AoA `18.0/19.6` degrees, reaches a terminal-turn
event at approximately `5.7 km` and `90 m/s`, and still has `180.0 km`
terminal geometry error and `183.2 km` position error. The measured q and
acceleration remain bounded (`7.0 kPa`, `1.19 g`), so this is not a structural
or dynamic-limit abort.

The topology timing and grid diagnostics explain the failure boundary. Early
reversal candidates retain energy but arrive near the runway station with
terminal-turn radii of roughly `1.6--6.3 million m`; late candidates make the
turn only after descending to the altitude/speed floor and arrive roughly
`183 km` beyond the interface. The current postburn checkpoint therefore does
not provide an executable strict-entry topology under the present vehicle and
route model. This is a scenario/model-feasibility result, not evidence that a
production gate should be weakened.

The RL terminal-corridor states have the same limitation at a different seam:
they can satisfy the explicit MM304 handoff set, but the downstream MM305
candidate remains geometry- or live-energy-degraded. A shallower diagnostic
path removed the geometric violation while leaving the live energy margin
approximately `-8.4 kJ/kg`; lowering that gate would therefore publish a
non-executable path. No sampled terminal state has produced touchdown or
rollout.

### Feedback-coupled terminal forecast

The terminal future-state predictor now rebuilds the projected aerodynamic
sample at each forecast step and recomputes force-derived AoA from the same
bounded pitch-response model used by the live terminal law. The committed-path
command uses that force-derived AoA directly instead of converting flight-path
angle plus nominal trim into a separate AoA demand. This closes a forecast/live
law mismatch in which a steepening flight path could be hidden behind one AoA
target held for the entire horizon.

The representative `22.0 km / 540 m/s / -16 deg` direct-corridor state changed
from a false-positive approximately `+2.8 kJ/kg` live-energy margin with a
committed path to approximately `-7.8 kJ/kg` with no path commitment. A bounded
screen over `20--23 km`, `450--600 m/s`, and `-8--16 deg` found no executable
committed terminal path: some states never obtained MM304 ownership, while the
middle-band states either had negative live-energy margin or failed the commit
contract. The retained diagnostic is
`terminal-feedback-screen-20260920.md`.

This is a safety correction and a curriculum-feasibility finding, not a
landing result. The current terminal-corridor curriculum must not be used as
evidence of RL landing progress until its route, energy, and topology contract
produces a physically executable post-handoff path.

### Forecast-origin reachability and terminal RL accounting

The terminal selector now applies the same bounded-curvature path-time check
used at commitment while selecting a response-propagated future origin. A
preview whose origin cannot be reached inside its advertised forecast horizon
is discarded instead of being retained by candidate-refinement hysteresis.
This is a fail-closed consistency correction; it does not widen any path,
energy, or control gate. Focused native, offline-build, and Python checks
remain green after the change.

The RL adapter now reports the MM305 terminal-path contract independently from
MM304 handoff debt. `initial_handoff` can remain true for a terminal-corridor
reset, but it is not counted as a handoff event or terminal progress. A
terminal candidate requires valid, non-degraded geometry and a finite
non-negative live-energy margin; terminal path readiness additionally requires
the production path to be committed. Promotion reports
`terminal_path_unproven` when terminal-corridor validation never observes that
commit. Four fresh terminal-corridor seeds (21, 22, 121, 122) still aborted
without touchdown, with no candidate-ready interval and no committed path.

One deliberately steep diagnostic inlet (`22.5 km / 400 m/s / -45 deg`) did
start with a committed candidate after the reachability correction. It was not
used as a curriculum state: after approximately 50 seconds the live energy
margin became negative, the path was no longer committed, and a 120-second
run timed out near `7.7 km` altitude and `21.5 km` cross-track with no
touchdown. A committed preview is therefore not being counted as terminal
flight evidence.

### Contradictory preflare boundaries

The former logic approximately used:

```text
pull-up trigger = required recovery height + configured flare altitude
abort boundary  = required recovery height
                  + max(100 m, 2 × configured flare altitude)
```

Because the abort boundary was higher, a descending vehicle could abort before reaching its own pull-up trigger. The replacement evaluates one signed recovery-height margin and forecasts whether the next controller sample consumes it.

### Recovery invariants

Focused tests demonstrated that slower response and increased bank increase required recovery height; mirrored banks are symmetric; historical acceleration cannot manufacture current authority; negative preflare margin blocks final delivery; invalid numerical inputs fail qualification; outer final can enter preflare with positive reserve; and acquisition cannot bypass alignment at the recovery boundary.

A synthetic case produced:

| Condition | Required recovery height |
|---|---:|
| Baseline | 132.678 m |
| Slower pitch authority | 179.832 m |
| 60° bank | 312.922 m |

These are model/invariant results, not flight-success evidence.

## Validation status

Passing checks recorded in the retained logs include:

- native production build;
- offline simulator/expert build;
- UDP guidance-runner build;
- decision-envelope tests;
- terminal-recovery contract tests;
- shared atmosphere-model tests;
- simulator telemetry-semantics tests;
- TAEM executive tests;
- terminal preview scheduling tests;
- nine simulator physics tests; and
- fifteen decision-audit tests.

The current focused Python suite passes 36 tests with 9 environment-dependent
tests skipped. The offline physics suite passes 9 tests. The native component
target passes, including flight-control, atmosphere/model, C-Nano, terminal
recovery, and terminal preview scheduling coverage.

The recovery-only refactor left all four exercised Entry trajectories telemetry-identical to baseline. This establishes no regression in those Entry paths; it does not validate terminal flight.

Full qualification remains **red**. The focused native component target is green, but the broader `qualification` target currently stops in the older MM304 offline seam at:

```text
MM304OfflineTests.c:348
isfinite(extended) && extended > cfg.vehicle.maximum_angle_of_attack && extended <= 35.0
```

The test fixture now supplies a valid atmosphere profile. The current energy/authority model does not produce the expected synthetic AoA extension for that fixture, so I have not rewritten the assertion into a tuned replacement. Other known stale tests reference removed preflare fields and signatures, airbrake-assisted terminal delivery, a removed RL `normalize` function, and an obsolete policy-vector size.

## Limitations and unresolved risks

- The vertical envelope is a local constant-acceleration model, recalculated from current state; it is not a proof over a changing aerodynamic trajectory.
- Lift and drag evolve during pitch response and deceleration.
- Combined lateral and vertical authority still needs a coupled feasibility proof.
- Recovery uncertainty is not comprehensively represented.
- The offline expert and UDP runner now share post-decode authority,
  confidence, configured fallback, and site-geometry preparation. Their
  transport-specific rate/ground-track paths and predictor/runtime agreement
  still need end-to-end review.
- Predictor/runtime agreement over a complete trajectory is unverified.
- Preflare, inner-final, flare, contact, and rollout still contain behavioral thresholds outside the shared contract.
- The deterministic terminal-corridor initial states have not yet been shown
  physically feasible as complete MM305 paths; several are valid handoff
  states with negative candidate live-energy margin.
- The established 50 km checkpoint has not been attempted.
- No unpowered runway touchdown and rollout has been demonstrated.

## Next causal investigation

With the common simulator post-decode metadata consolidated, reconcile the
entry checkpoint and terminal route model at the earliest valid topology
boundary. In particular, compare the TAEM target station/range contract with
the terminal route endpoint, then compare commanded and actual bank/AoA,
attitude-recovery entry, lift authority, target selection, energy loss, and
every handoff veto. For each initial state, determine feasibility before
interpreting outcome quality. Replace stale tests with physical invariants
while retaining mission constraints and regression evidence. Broader
checkpoint validation is appropriate only after model inputs, handoff
decisions, and forecast-versus-measured response agree.

The next meaningful milestone is a valid initial state, coherent model inputs, an explainable handoff decision, and agreement between forecast and measured response—not merely one seed traveling farther.

## Evidence index

- `baseline-audit.json` — baseline decision-literal inventory.
- `baseline-source-sha256.json` — source hashes captured before edits.
- `baseline-native-smoke/` — deterministic baseline smoke evidence.
- `recovery-native-smoke/` — recovery-only smoke evidence.
- `model-native-smoke/summary.json` and per-seed JSONL — latest deterministic outcomes.
- `lead-native-smoke/summary.json` and per-seed JSONL — post-response-lead four-seed terminal screen; all aborted without touchdown.
- `current-topology-grid-detail-20260920/` — topology delay/bank grid showing early high-energy and late energy-depleted invalid branches.
- `current-topology-detail-20260920-b/` — current-build terminal topology detail trace.
- `current-rejection-diagnostic-20260920-b/manifest.json` — explicit current-build production engagement rejection with topology diagnostics.
- `terminal-feedback-screen-20260920.md` — feedback-coupled terminal forecast correction and bounded state-space screen.
- `terminal-path-contract-20260920.md` — forecast-origin reachability guard and explicit RL terminal-path accounting.
- `model-qualification.log` — latest qualification attempt.
- `model-build-tests.log`, `model-simulator-build.log`, `physics-tests.log`, and `taem-exec-tests.log` — retained build/test evidence.
