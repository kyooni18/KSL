# TAEM native command propagation checkpoint

2026-11-03, Asia/Seoul. Based on `TAEM-MM305-Replacement-Research-20261103.md`
from the supplied research archive, already preserved in revision `2617147`.

## Implemented behavior

Entry propagation and terminal closed-loop qualification now share
`NativeCommandRollout`: a branch owns its controller, target limiter and
finite-response plant. Each step returns the requested solution, limited target,
issued actuator command and before/after physical states. No target overwrites
physical attitude. `NativeCommandSeed` makes its state/controller/limiter inputs
immutable value copies. The caller retains the calibrated atmosphere/aero model
and configuration and remains responsible for termination and qualification.

The live mission now passes a value copy of the **entire** target limiter through
entry and terminal planning, asynchronous worker splits, and predicted entry to
terminal handoff. Previously these boundaries passed only the last target and
recreated the limiter with zero bank/alpha reference rates. Legacy callers that
provide only `previousTarget` retain their cold-history behavior; a supplied
limiter takes precedence. This is a concrete prediction mismatch, independent of
whether any alternative to the recorded zero-bank recovery can succeed.

Controller snapshots now retain response diagnostics (used by the first
predicted limiter step), main-contact latching, and copied rollout controller
history. Rollout snapshots are values, so prediction branches do not share a live
rollout controller. Native plant observations now retain actual speedbrake and
body-flap actuator positions in `appliedControls`, allowing a branch to initialize
those surfaces without silently stowing them.

## Decisive verification

The engineering question was whether a warmed prediction branch preserves the
native pipeline's next target/command and finite-response step. Disagreement
would block further command-schedule planning; agreement permits using the common
kernel for subsequent model-closure work.

Command:

```
swift test --package-path swift/KSL-Landing --filter 'NativeCommandRolloutTests|GuidanceAuthorityRegressionTests|MissionPlanningSplitTests'
```

Seven checks passed (two newly added tests, one existing split test extended).
Evidence: `Validation/TAEM-CommandRollout-20261103/focused-tests.txt`.

- A limiter copy retains reference momentum; recreating it from the last target
  produces a different next bank. Existing zero-authority/no-overshoot checks pass.
- Both worker splits preserve the next limiter result and dynamics revision.
- From the existing genuine Dessert STS configuration/state fixture, a three-tick
  software check warms one tick then compares two ticks of the shared kernel
  against the direct native limiter/FCS/plant pipeline. It also checks independent
  branches, force/state agreement, response diagnostics and maximum bank. The
  small alpha/bank and split-rudder requests exercise command history, without
  changing physical capability. This is software equivalence, not a replay of
  the research flight's historical commands or validation of the plant.

A development build passed. No release executable was rebuilt, no KSP state was
loaded or unpaused, and no controller, kRPC connection or live flight was started.
No broad suite, trajectory campaign, or synthetic capability perturbation ran.
Pre-existing work outside these changes is excluded from the commit.

## Remaining work and interpretation

This implements foundations from research steps 1–2, not the full replacement.
The current vertical-first MM305 recovery policy, geometric candidate family,
qualification and early-exit behavior remain. This checkpoint does **not** fix or
claim to fix the research flight's zero-bank failure.

The seed is not yet a complete archived replay contract: pending-command latency,
TAEM/estimator progress, model/configuration/runway identities and full physical
plant/authority-transition history still need explicit representation. The
archive's `flight-seed.json` explicitly contains recorded rows rather than a
complete private controller snapshot. No exact warmed reconstruction is claimed.

Next, reconstruct the recorded command history and close force/attitude prediction
over an observed response interval. Only then interpret one paired old/new
recovery comparison. Event-complete outcome pricing, topology-seeded shooting,
reachable spiral joins, all-bad recovery ranking, measured-state rebasing and
proposal/selection/tracking display identities remain to be implemented. No
alternative landing or flight readiness has been established.
