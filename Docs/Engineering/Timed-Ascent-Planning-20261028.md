# Timed ascent planning and execution

The previous booster forecast selected a constant pitch bias, which the
executive added to an altitude/prograde course. It did not supply an executable
pitch/thrust/mass timeline. The new path supplies and follows that timeline.

## Planner contract

`full_plan.plan_ascent` starts from measured radius, radial and inertial
horizontal velocity, surface horizontal velocity, mass, pitch, thrust, mass
flow, booster burn time, attached booster dry mass, and aerodynamic loads.
It searches smooth early/late steering offsets within the progressive-turn
envelope, upper-stage acceleration at 80% or 100% of the allowed upper G, and
four candidate insertion radii. Selection retains the existing orbital-error,
peak-apoapsis and propellant score and acceptance tolerances.

Its output includes:

* A booster trajectory and an insertion trajectory: relative simulation time,
  radius, radial/horizontal velocity, mass, raw and smoothed pitch, modeled
  flown pitch, thrust and mass flow. Insertion also includes throttle fraction.
* Absolute-UT separation and MECO milestones, including altitude, velocity,
  mass and the predicted initial handoff apsides.
* Pitch-up, pitch-down and hold intervals with start/end UT and pitch.
* The selected upper-stage acceleration, core propellant consumption and
  whether a measured fuel budget was provided and checked.

During SRB flight, measured trim-balanced SSME thrust and measured SRB thrust
are propagated, not freely optimized. A planner without future CoM/gimbal
authority cannot authorize arbitrary SSME reductions. The actual pitch trim
and one-degree gimbal-reserve governor remain authoritative. After separation,
the planner chooses the upper acceleration and predicts the limiter hold/ramp,
declining mass, and thrust schedule together.

Only usable ET liquid propellant is budgeted; SRB propellant loss and detached
dry mass are accounted separately. The final mass cannot cross the seed's
non-consumable mass floor. OMS reserves are not counted as SSME fuel.

## Execution

Planning starts once the velocity-gated initial roll permits the progressive
turn, even if roll correction continues. It covers the remaining booster burn
and insertion; it is not yet a pad-to-orbit six-degree-of-freedom optimizer.
The progressive altitude profile now bounds the search and remains the explicit
fallback when no feasible booster plan is available.

The live executive samples the selected booster trajectory by UT rather than
adding its bias to the old course. Position and radial/horizontal velocity
errors relative to the planned state provide bounded tracking corrections.
Zero state error produces exactly the planned command. Five-simulation-second
command smoothing remains; there is no new fixed pitch-rate cap.

At separation, the measured attitude hold wins and stale booster-state plans
are discarded. Insertion replans from the new mass and motion, then follows
its saved pitch and thrust timeline. Its chosen upper acceleration carries
forward from the full ascent plan. The live thrust request comes from planned
thrust/current mass, subject to phase caps, the apoapsis governor, and MECO
settling/taper. A plan contains the separation pitch blend already, so the
executive does not apply that blend twice.

Plans refresh on approximately ten-second UT intervals and at stage changes;
insertion also refreshes on UT rewind or horizon exhaustion. Sampling does
not extrapolate outside a stage's horizon or interpolate across its mass drop.
Pauses do not consume plan time. This is not full quickload/session recovery.

Forecast MECO time is a target, not a timer that commands cutoff. Existing
measured-orbit and settled-attitude conditions still determine actual cutoff.
An unqualified insertion prediction is labeled a recovery trajectory with a
warning. Planning failure does not add an automatic abort decision.

The latest detailed plans are saved alongside ascent CSV telemetry as
`.booster-plan.json` and `.insertion-plan.json`. Milestone summaries are also
emitted as `ascent-plan` events.

## Reviewable offline example

See `Timed-Ascent-Plan-Example-20261028.json`. It includes all inputs and the
complete trajectories, generated from a **synthetic regression seed**, not a
live vessel: UT 1000, altitude 10 km, mass 300 tonnes, radial speed 300 m/s,
surface horizontal speed 150 m/s, nose command 68 degrees, 110 tonnes of
usable core propellant, and 35 seconds of booster burn remaining.

| Planned event | Time after seed | Altitude | Mass |
| --- | ---: | ---: | ---: |
| Booster separation | 35 s | 27.24 km | 189.00 t after separation |
| Initial handoff / MECO | 205 s | 82.01 km | 109.78 t |

The chosen upper acceleration is 1.12 G. Predicted handoff is 90.45 x 60.13 km,
with 89.72 tonnes of core propellant consumed from the seed. This atmospheric-
periapsis handoff still needs OMS to reach the final 90 km circular orbit.
The pitch/thrust solver is planar; the existing heading controller targets
45-degree inclination. That inclination is not independently qualified by
this planar forecast.

## Verification and limits

Focused ascent/operations/controls/process tests: 157 passed, 3 subtests passed.
Full offline suite: 273 passed, 59 subtests passed, with two known TUI temporary-
directory teardown thread warnings. An initial expectation that the 260 m/s
radial shallow seed was unreachable failed because the new lower-thrust search
made it reachable. The old fixed-thrust rejection remains tested; a new test
checks the lower-thrust solution without relaxing orbit tolerances.

Tests cover UT interpolation through pause/irregular updates, refusal to
extrapolate, nominal feedforward and tracking response, integrated propellant
accounting, insufficient fuel, timed milestones, and trajectory-following
insertion in a separate 0.2-second point-mass integration versus the planner's
two-second insertion steps. The final target-feasibility diagnostics were
checked with another 18-test planner/tracker/insertion run.

This remains bounded candidate shooting, not global optimal control. The
booster atmosphere extrapolates measured loads, thrust/flow are held at their
measured booster values, and rotational/gimbal dynamics are approximated by
attitude response and phase holds. Predictions do not prove live stability,
especially around separation and MECO. No KSP connection, launch or quickload
was attempted. Pre-existing unrelated edits remain uncommitted.
