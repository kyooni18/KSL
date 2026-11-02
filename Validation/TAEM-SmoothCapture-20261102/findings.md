# TAEM command continuity and runway rollout — 2026-11-02

Evidence: archived 37f2fd8 flight
`runs/20261102T031637Z-bounce-ownership-72km`, its exact configuration and
full-rate recorded states/controls. No KSP RPC, load, unpause or flight was
performed for this change. Source baseline is 4178a1a (unchanged TAEM relative
to that flight).

## Causes and narrow changes

- A rejected altitude reference could later pass a force check and resume
  tracking after the aircraft had flown another descent. Regenerate from
  measured height and slope before reacquiring an unfollowed reference.
- Height is parameterized against horizontal ground distance. Converting its
  rate with asin(rate/total airspeed) exaggerated steep corrections and could
  demand a vertical dive. Use the ground-track slope angle instead.
- Minimum pressure priced sec(measured bank), even when force allocation was
  unloading that bank. Price the attainable commanded normal/lateral lift
  vector; retain load, trim, bank and maximum-q protection. No new glide cap.
- Acquisition-to-HAC switched lateral laws instantaneously. Blend acceleration
  over the existing measured/configured roll-response horizon.
- The spiral's exit is tangent to the runway but still has curvature. Blend
  toward critically damped runway cross-track/heading correction during the
  last roll-response distance, so Final does not inherit full HAC bank.
- A completed winding only metres above the NEP started a new full loop.
  Permit the measured descent to reach NEP height within the roll response;
  high-altitude crossings still require recovery planning.
- Capture telemetry preferred the frozen advisory NEP over the actual active
  TAEM NEP. Report the active NEP first. This discrepancy initially prevented
  replay from reproducing the live reference switches.

No route-selection redesign, vehicle capability changes, new tuned flight
limits, or inner attitude-controller gains were introduced.

## Checks and limits

Matched-state replay reconstructs observed lift/drag gains, incidence history,
previous control feedback and active NEP refreshes for both old and new laws.
The old law reproduces the recorded -77.79/-90 degree target events. Neither
law controls a new simulated or live trajectory in this check.

Across the same 2,213 recorded TAEM states:

| Raw guidance quantity | Old | Revised |
|---|---:|---:|
| Steepest descent target | -90.0 deg | -55.93 deg |
| Largest consecutive descent-target step | 40.58 deg | 5.09 deg |
| Largest consecutive bank-target step | 70.0 deg | 9.57 deg |

The revised steepest target occurs on a recorded -52.55 degree descent. This
is evidence that artificial dive amplification is reduced, **not** proof that
a new trajectory stays shallow. Existing outer-loop target slew protection
still applies after these raw targets.

At the last recorded TAEM state, revised bank is about 5.12 deg and remaining
HAC range is zero; old guidance had reset to an 18.8 km winding. The recorded
vehicle's actual alignment is unchanged by replay. Better live alignment and
absence of closed-loop overshoot require a subsequent authorized flight.

Thirteen focused TAEM transition, reference-recovery and control-ownership
checks pass, including one new steep-reference endpoint regression. Release
build passes. Local command outputs are saved beside this report.

To repeat the read-only comparison with the archived run present:
`python3 Validation/TAEM-SmoothCapture-20261102/replay.py`.
The helper temporarily installs two test sources, runs one matched-history
check and removes those sources. It uses no synthetic vehicle perturbations.
