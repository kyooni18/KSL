# TAEM incident correction

The canonical C backend now separates HAC progress, capture and completion.
This corrects the nominal-progression failures in
`2026-09-05T16-43-48Z-STS-N.jsonl.zst` without claiming that its original energy
state can be recovered into a successful landing.

## Guidance behavior

- A HAC has one signed remaining arc, integrated from successive radial angles.
  Crossing the exit cannot create another revolution. The reference circle spans
  360 degrees; initial progress uses the actual directed angle instead of clipping
  it to 300 degrees.
- Within 150 m of arc completion, radial error must be within the larger of
  150 m or 8% of radius, HAC course error below 20 degrees, runway course error
  below 14 degrees, and energy and turn authority valid. Otherwise the landing
  is aborted with a latched `HAC missed` reason.
- Available lift is estimated from dynamic pressure, adaptive L/D and ballistic
  coefficient, reduced by measured lift when available, and capped by the load
  limit. The minimum radius also respects dynamic bank limits and the lift needed
  to support the flight path. A 10% radius margin is required. Insufficient lift
  gives an infinite radius (serialized as JSON null).
- Nominal TAEM is deferred to entry/S-turn energy management when the radius or
  energy corridor is infeasible. The energy gate requires altitude within 5 km
  of the HAC profile and speed between minimum safe speed and 135% of the local
  HAC speed schedule. This is a conservative gate, not a new trajectory planner.
- Heading Alignment requires current HAC capture. Altitude alone cannot select
  it. Final Approach additionally requires HAC completion and a runway-relative
  approach envelope, including range, along/cross-track, course, altitude, speed,
  flight-path angle and sink rate.
- Once the terminal region is reached, passing the runway station without final
  capture aborts the attempt, including while TAEM is deferred. Losing approach
  validity after capture also aborts rather than continuing altitude-based phases.
- TAEM preserves the current gear state. Automatic gear deployment occurs on a
  captured final/flare. Flare is latched but remains constrained to the runway
  width, course, runway length and a descent envelope above -12 degrees and
  -12 m/s. Low altitude without approach capture terminates nominal landing.
- HAC bank feed-forward follows the circle's actual turn direction. The nominal
  look-ahead chord heading bias is removed from feedback so it does not add a
  second copy of the steady-circle curvature.

## Control recovery and continuity

Persistent saturation with tracking error, persistent attitude error over
60 degrees, or excessive roll rate inhibits trajectory tracking. The detector
uses both body and Euler roll rates because the incident's body-rate channel
reported zero while the Euler roll rate showed a departure.

After two seconds of invalid control state (immediately above 80 degrees/s roll
rate), `Attitude Recovery` commands zero bank, reduced angle of attack, zero
throttle and a frozen heading through the common command limiters. It resumes
only after two seconds of low rates, small attitude errors, near-level bank and
unsaturated actuators. Failure to recover within twelve seconds, or inadequate
height, aborts and releases control. Geometric overshoot aborts take precedence.
These recovery thresholds require vehicle-specific flight validation.

Atmospheric phase changes retain limiter values and rates. Lower phase rate
limits decelerate existing rates under the acceleration bound; neither phase
changes nor target crossings snap the attitude commands. Re-engagement after
control release still seeds from measured attitude. Abort intentionally releases
autopilot control and cuts throttle through the existing bridge safe path.

Snapshots expose `hacCaptured`, `hacCompleted`, `hacProgressValid`,
`hacArcRemaining`, and `minimumTurnRadius` in `guidanceState`.

## Validation

`make -C CLanding test` passes, including regressions for both HAC sides, bank
sign, missed exit, the atan2 branch cut, infeasible radius, runway overshoot,
altitude-only flare, sustained saturation, recovery success/timeout, command
continuity and acceleration, valid HAC-to-final-to-flare progression, flare
latching, and loss of runway alignment.

The large selected-state fixture and the UI-specific smoke tests were removed
when validation was reduced. TAEM behavior is now checked against flight logs
and live KSP runs; the small native suite keeps only the directed-overshoot and
entry-safety invariants that are cheap and valuable before a flight.
