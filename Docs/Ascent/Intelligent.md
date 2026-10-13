# Optional intelligent ascent

The legacy STS launch process remains the default. Intelligent ascent runs in a
separate process and does not use STS engine names, pad limiter settings, launch
roll schedules, or the OMS trim program. Landing guidance is unchanged.

## Select and save

```sh
# Read-only discovery; load the spacecraft in KSP first.
bin/ksl launch --ascent-mode intelligent --to prelaunch

# Launch and insert into a 100 km orbit at 10 degrees inclination.
bin/ksl launch --ascent-mode intelligent \
  --set ascent.targetAltitudeKm=100 --set ascent.inclinationDeg=10

# Elliptical orbit: apoapsis 200 km, periapsis 90 km.
bin/ksl launch --ascent-mode intelligent \
  --set ascent.targetAltitudeKm=200 --set intelligentAscent.targetPeriapsisKm=90

# Explicit legacy selection.
bin/ksl launch --ascent-mode legacy
```

In the TUI, select **Mission → Ascent guidance**. This changes the next mission's
configuration, not an active process. Use **Save mission setup** to save the mode,
orbital target, vehicle configuration path, and other preferences in a reusable
JSON profile. Reopen it with `bin/ksl tui -p <path>` or launch it with
`bin/ksl launch -p <path>`. Merely inspecting a vehicle does not enable guidance.
Existing profiles and recorded configuration snapshots without the new fields
resolve to legacy mode.

## Vehicle editor

**Vehicle → Inspect active vessel** reads the attachment tree. Each component
shows its structural path, role, position in the vessel frame, activation and
separation stages, and (for engines) thrust magnitude and direction. Positions are
relative to the live centre of mass; +Y is the control reference's nose.

Choose a component to correct its role. Save the vehicle and then save the mission
setup to reuse it. Vehicle files carry a fingerprint of part names, attachment
paths, and stages. A different craft is rejected before ignition; it is never
silently given another craft's staging permissions. Keep the saved file available
alongside profiles that reference it.

A decoupler's branch is eligible for automatic release only when every engine in
that branch is active, positively out of fuel, and producing no thrust, with no
protected payload. Decouplers in the same separation stage wait for the entire
group and a depletion confirmation interval. The stock staging stack is never
activated. Engine-less tanks require explicit separator approval and measured
resource depletion. Clamps are released directly after measured ignition thrust
settles. Ambiguous branches remain attached.

## Guidance and evidence

The ascent planner integrates a spherical point-mass model including gravity,
curvature, mass flow, measured aerodynamic drag, and unthrottleable thrust. It
scores bounded candidate pitches, predicts atmospheric coast losses before
cutoff, and recomputes from live state. Surface velocity drives aerodynamic
steering; inertial velocity drives orbital prediction. Live rotational authority
and inertia bound pitch command slew. The attitude target compensates the engine
resultant, including canted mounts and changes in the active engine set.

Insertion searches burn opportunities with two-body propagation and calibrates
maneuver-node coordinates against kRPC. Burns taper near completion, and each
correction is replanned from the measured orbit. A numerical planning failure
triggers a bounded energy-building recovery and another search. Thrust alignment
is checked against the force vector rather than total attitude error, which can
include an irrelevant roll error. Cleanup cuts liquid throttle, releases the
controller, enables SAS, and stops rails warp.

Success requires measured apoapsis, periapsis, and inclination within the configured
tolerances. Defaults are 1 km per apsis and 0.2 degrees inclination: an initial
qualification envelope allowing finite-burn and control-cycle error across engine
sizes, not a claim of exact insertion. Configure tighter tolerances through
`intelligentAscent.altitudeToleranceM` and `inclinationToleranceDeg`. An exhausted
correction or time budget reports failure; predicted success alone is insufficient.

The independent RK4 test plant uses a separate integrator and atmosphere from the
planner. Recorded cases are in [IntelligentEvidence](IntelligentEvidence/): nine
mass/thrust/target combinations (10–80 tonnes, 80–200 km), plus staged and elliptical
cases. These validate trajectory and finite-burn mechanics with ideal attitude;
they do not establish KSP structural, aerodynamic, or asymmetric-engine reliability.

```sh
.venv/bin/python -m pytest -q tests/ascent/test_intelligent.py
.venv/bin/python -m ksl.ascent.intelligent.simulation
```

## Current boundaries

- Use the `circular` mission profile kind for direct inclination/apsis targeting;
  a nonzero intelligent periapsis setting selects an ellipse. Intelligent ascent
  does not yet implement the legacy plane-launch window, GEO longitude phasing,
  or rendezvous workflows. Those remain available in legacy mode.
- The pressure and acceleration governors are feedback limits. Fixed solid thrust,
  aerodynamic transients, and limited authority can exceed them.
- No guidance can make an underpowered, structurally unsound, or uncontrollable
  vehicle feasible. Component identification and attitude authority must be checked
  against actual flights. General-purpose differential engine allocation and
  detailed six-degree-of-freedom aero prediction are not implemented.
- Vehicle profiles authorize topology, not arbitrary scripts or staging actions.
  Save after deliberately changing attachment topology or stage assignments.
