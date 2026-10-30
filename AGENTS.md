# KSL agent principles

These are durable engineering and flight-behavior requirements, not a run-status
report. The Swift backend is under `swift/KSL-Landing`. Keep experiment status,
command examples, and handoff details in investigation reports or run artifacts.

## Evidence, models, and limits

- Establish a failure's cause from telemetry, the actually flown source and
  configuration, and code before changing guidance. A passing build, accepted
  plan, favorable forecast, or phase label is not evidence of flight success.
- Prefer general logic and physically consistent models over hardcoded values,
  arbitrary caps/floors, tuned gates, or scenario-specific branches. Do not hide
  a modeling, planning, or control error with a magic threshold or gain.
- Derive operating envelopes, turn radii, force allocation, energy reserves,
  horizons, and response times from vehicle/aero/plant models, measured state,
  identified actuator dynamics, or existing justified configuration. Preserve
  units, coordinate frames, altitude datums, and uncertainty explicitly.
- Numerical guards are allowed for real numerical singularities; document their
  purpose and scale. Safety/vehicle/mission limits are also allowed when they
  have a physical or explicit configuration basis. Examples include MM304's
  configured 40-degree AoA and 70-degree bank limits, configured minimum HAC
  radius, structural loads, and actuator limits. These are not permission to
  invent new limits. If modeled turning radius is larger than a configured
  search ceiling, report and handle it rather than clipping away the problem.
- Do not hardcode a measured run's timing, coordinates, speed, checkpoint name,
  or failure signature into production guidance. Put experimental settings in
  explicit configuration; distinguish them from validated defaults.
- Validate predictors against measured forces, energy loss, and attitude/turn
  response using identical initial states, commands, actuator/trim history, and
  configured RCS availability. Separate identified evidence from provisional
  priors. A model finding is only as reliable as its validation and uncertainty.

## Guidance behavior

- Use the selected runway's actual threshold, heading, length, width, elevation,
  and reciprocal geometry. In `RunwayFrame`, crossrange is positive RIGHT of
  the selected runway. Distinguish site distance, threshold distance, capture
  objective distance, runway-heading error, and target-bearing error.
- MM304's configured five-kilometre delivery objective is before the selected
  threshold on its centerline. It is an aim, not a radius gate for MM305 search
  or an absolute prohibition on passing. A return HAC is acceptable when the
  actual altitude, speed, energy, and turning authority can support it.
- Separate search permission, candidate ranking, plan admission, and actual
  control ownership. Feasibility assessments are advisories, not plan vetoes.
  Guidance must keep flying the best available recovery rather than abandon
  control because its own assessment says the situation is infeasible.
- Prefer consistently priced, attainable references; compare candidates on the
  same physical basis and keep an advisory fallback when all are bad. Do not
  mistake an incompletely priced path for a better physically realizable path.
- MM305 should acquire briefly and accomplish most of its work on a regenerated
  spiral adapted continuously to measured state. Preserve winding, command/force
  continuity, and actual join geometry; relabeling acquisition as spiral does
  not make a long acquisition acceptable. Fit attainable lift/bank/drag vectors
  rather than independently demanding incompatible commands.
- Preserve configured `maximumBank`, target-limiter no-overshoot behavior,
  configured RCS availability, and the ten-second RPC timeout with reconnect
  recovery. Do not disable protections or increase limits to make a run pass.

## Experiment integrity and completion

- Preserve pre-existing uncommitted work. A run's `source.patch` may be essential
  to reproduce its binary; HEAD alone is not sufficient provenance. Record the
  source revision/diff, exact configuration, executable identity, and logs for
  each useful live experiment.
- Stop the existing flight controller before starting another or loading a
  save. Never rebuild or replace a release executable while a process uses it.
  Keep one guidance owner and avoid unlogged interventions in comparison runs.
- Preserve original user saves and name derived checkpoints separately. A
  checkpoint is a replay tool, not a substitute for final full-chain validation
  from orbit through deorbit, entry, terminal guidance, landing, and rollout.
- Claim landing success only after corroborated touchdown on the selected
  runway and rollout through a sustained full stop with the intended vehicle
  intact. Check runway-relative geometry, contact/gear state, touchdown dynamics,
  ground speed and stability, and fresh continuous telemetry. An airborne phase
  transition, stopped controller, frozen log, or off-runway stop is not success.
- Add only necessary regression tests for meaningful demonstrated behavior.
  Use matched-state replay and focused live experiments for trajectory claims;
  avoid large collections of compile-time tests that cannot establish them.
- Commit useful, coherent checkpoints without sweeping unrelated changes into
  a commit. Search Foundation Memory for **KSL TAEM capture** before this line
  of work; record durable findings and decisions with appropriate project tags.
  Never store credentials in memory, reports, prompts, or Git.
