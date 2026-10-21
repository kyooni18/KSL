# Vehicle-specific preplanned ascent

The default legacy STS/SRB launch now calculates a complete pitch and core-engine
limiter reference before ignition. It searches burn duration and six pitch knots
for the least propellant use it can find while reaching the configured ET
insertion apoapsis and periapsis. This replaces the altitude-only gravity-turn
schedule for this vehicle. It is a numerical best-found solution within the
model and bounded pitch family, not proof of global physical optimality.

## Inputs and prediction

Read-only discovery captures part masses and positions, liquid and solid
propellant pools, engine positions and mounting directions, vacuum thrust,
sea-level/vacuum Isp, usable gimbal travel, pitch inertia, body gravity,
atmospheric density/pressure tables, and the launch plane's rotation projection.
Launch clamps are excluded from flying mass; separation-motor fuel is not
consumed as main-booster propellant. The configured core ignition lead is
included as nominal prelaunch fuel consumption.

During prediction, propellant depletion and booster separation change mass, CoM
and estimated pitch inertia. Engine moments about that CoM determine the SSME
trim limiter and remaining gimbal authority. Prediction also includes fixed SRB
thrust, pressure-dependent engine performance, liquid mass flow, post-separation
limiter caps, pitch/throttle slew, attitude lag, roll delays, the projected cant
during the upright roll, q-alpha limits, drag, gravity and spherical curvature.

SciPy differential evolution searches candidates; local simplex refinement
reduces orbit error and propellant. A smaller-step refinement is replayed at
half its step size before acceptance. Both endpoint apsides, peak apoapsis,
dynamic pressure, G, attained angle of attack, and moment/control authority must
pass. A failed search saves its result and stops before ignition; it does not
silently substitute an upright schedule or claim physical infeasibility.

## Execution and saved evidence

The run saves `ascent.csv.trajectory.json` beside its ascent telemetry. This
contains the actual model inputs, model provenance, optimized parameters,
predicted states, pitch, core limiter, mass/CoM, apsides, constraint violations,
and the feasibility decision. Read-only preflight also calculates and saves it.

During flight, elapsed KSP UT selects the reference, with bounded altitude and
velocity feedback. The physical roll sequence, live q-alpha limit, actual
moment trim, slew limits, G/q governors, and measured MECO requirements still
apply. When a burn runs beyond the reference, the existing closed-loop insertion
planner handles the residual. Predicted cutoff alone never releases the ET.
Heading guidance continues to target the requested inclination.

The optimizer is enabled by `--turn-profile optimized` (the default). Restore
the historical schedule explicitly with `--turn-profile scheduled`. Add these
to `ascent.extraArgs` when using a mission profile. Other legacy vehicle layouts
and GTO/GEO transfer ascents currently require `scheduled`; the optional
`intelligent` process retains its existing separate planner.

`--plan-drag-area <Cd*A in m2>` supplies an identified drag area. Without it,
the model records a geometry-based estimate (`0.3 * frontal bounding-box area`).
This is not a measured drag coefficient. Install the updated requirements to
obtain SciPy:

```sh
.venv/bin/pip install -r requirements.txt
```

An existing saved model can be optimized again without KSP or control writes:

```sh
.venv/bin/python -m ksl.ascent.preplan runs/<run>/ascent.csv.trajectory.json \
  --apoapsis 90000 --periapsis 60000 --output ascent-trajectory.json
```

## Evidence and limits

See `Docs/Engineering/Ascent-Preplanning-20261021.md` for retained experiment
results. The regression fixture is synthetic, not a fitted STS identification.

This is a planar engineering model. It does not reproduce full aerodynamic
lift/torque, structural flexibility, cross-plane motion, engine thrust curves,
or detailed six-degree-of-freedom roll dynamics. Tank contents are depleted
proportionally within each propulsion pool; the model does not know every KSP
resource-flow priority. Gimbal authority is a geometry-based estimate, and
fuel removal uses a point-mass approximation for inertia. Drag calibration,
attitude response, predicted propellant savings, and orbit accuracy therefore
need flight verification on the actual vehicle. A search failure means no
qualified candidate was found, not that no physical ascent exists.
