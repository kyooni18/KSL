# Native Final checkpoint audit

Source audit after `7994039`; no flight experiment, checkpoint implementation,
production change, or new qualification is claimed.

The existing guidance shadow cannot serve as a complete native Final replay.
`CLanding/prediction/entry_planning.inc` copies `GuidanceMachine`, but directly
advances bank/AoA response with `predictor_attitude_step`, then RK4 aerodynamic
translation. It never executes the direct flight-control law. Its synthetic
telemetry deliberately disables body-rate availability and measured controls,
sets sideslip to zero, and reports `flying`. Its airborne termination checks
altitude below site minus 20 m or speed below a floor, rather than main-wheel
contact. The Entry wrapper stops at TAEM; the terminal wrapper can continue,
but that does not make its response or touchdown event native.

The native direct replay instead calls `flight_control_step` in
`CLanding/transport/krpc.c` and sends its outputs to ShuttleSim. The persistent
`FlightControlState` belongs to `KRPCSession`, separately from the application
controller's guidance. It contains learned authority, trim, response, timing,
and contact history. `LandingController` also owns physics, adaptive and
trajectory calibrators, the latest command/sample, and terminal-model state.
Copying guidance alone omits these owners. A telemetry-only scenario restart
cannot restore them either.

ShuttleSim's `Simulation` owns full `SimState` (including `AttitudeModel`,
quaternion, commands and contact history), world/aerodynamic tables, geometry,
and ground parameters. `flight_physics_airborne_step` evaluates force before
advancing attitude, translates with the native discrete integrator, then
recomputes forces for observation. This differs from the reduced shadow's
attitude-before-RK4 sequence. `ShuttleSim/src/main.c` additionally owns the
telemetry publication deadline and lockstep command/advance boundary.

## Required comparison

Use the existing continuous L5/L7 replay as the warmed reference. Preserve the
late checkpoints at L5 UT 2462.649863 and L7 UT 7021.450098, with the exact
pipeline stage stated. The prior frozen experiment captured a point inside
Final guidance, not a clean controller-tick boundary; capturing after command
application would include the remainder of that tick and requires a distinct
checkpoint label.

Before testing a changed forecast, an unchanged resumed tail must reproduce
the continuous run's guidance commands, actuator outputs, state and contact
events. Preserve or explicitly clone:

- Controller guidance, physics/model and estimator history, effective vehicle
  configuration, previous command and sample timing, and terminal-plan state.
- Session flight-control state, latest sample, previous control UT, and command
  latches. Rebind transport handles rather than copying active sockets.
- Simulator state, complete actuator state and command inputs, world/aero
  inputs, physics cadence, and the publication/advance boundary.

Do not serialize whole controller/session structs as raw bytes: they contain
thread primitives, pointers, files and transport handles. Dynamic trajectory
and physics/model storage require ownership-aware copying and pointer rebinding.
Use immutable model inputs with recorded hashes, and exclude background
planning races at the checkpoint. Whether Final can safely omit any of these
owners must be demonstrated by unchanged-tail parity, not assumed.

An experiment that reexecutes the original prefix to this boundary is also a
valid warmed-state comparison and avoids claiming unsupported restart support.
It costs runtime but preserves the complete native pipeline. A later isolated
forecast can reuse that preserved state only after parity is established.

This audit rejects reusing the reduced terminal shadow as proof of native
Final response. It does not reject its existing advisory purpose, change gates,
validate the live plant, or resolve the L5 forecast error.
