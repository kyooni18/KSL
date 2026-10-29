# Physics, units, coordinate frames, and evidence

## Evidence boundary

This is a reusable native simulation and prediction model, not a claim of KSP parity. It uses the supplied archive's stock Kerbin curves, latest STS polar, direct-input pitch parameters, and explicitly provisional roll/yaw parameters. The archive's own polar metadata says `provisional; not certified`. The quantitative audit is in [reference-audit.json](../Validation/reference-audit.json); original data hashes are in [source-manifest.json](Evidence/source-manifest.json).

The polar contains 294 grid cells. The update report identifies 53 observed cells and lists 19 live sessions and 283 simulator files. The 19 raw live files named there are absent at their referenced archive paths. The included observations database fails a read-only integrity access with `database disk image is malformed`. Consequently this delivery cannot independently establish a held-out live force/moment error bound from those records. It does not repair or rewrite that database.

The update ratios relative to an older polar vary substantially. They are calibration-change ratios, not validation residuals. They must not be presented as the new model's accuracy. Likewise, the configured ±20% force stress cases are engineering sensitivity tests, not a statistically certified uncertainty interval.

## Unit contract

| Quantity | Internal unit and convention |
| --- | --- |
| Position, altitude, route distance, runway dimensions | metres |
| Velocity | metres per second |
| Time and UT | seconds |
| Mass | kilograms |
| Force and moment | newtons and newton metres |
| Inertia | kilogram metres squared |
| Pressure and dynamic pressure | pascals; q-dependent moment coefficients explicitly convert to kilopascals |
| Temperature | kelvin |
| Angles and angular rates | radians and radians per second |
| Aerodynamic coefficients | dimensionless with 250 m² reference area |
| CLI scenario angles and legacy telemetry angles | degrees at that boundary, converted immediately |

The reference area is the polar's normalization area. It is not an independently measured wing planform area.

## Canonical frames

Planet-fixed Cartesian axes are right handed: X crosses the equator at longitude zero, Y crosses the equator at 90° east, and Z points north. A `VehicleState.velocity` is the derivative of this rotating-frame position, not inertial velocity. Latitude/longitude are spherical geocentric coordinates; Kerbin's configured radius is 600,000 m.

Local coordinates use north, east, up. The shuttle body frame uses forward, right, down (FRD). The stored quaternion maps FRD vectors into planet-fixed Cartesian vectors. Positive roll lowers the right wing, positive pitch raises the nose, and positive yaw turns right. Angular velocity is the body's rotation relative to the planet-fixed frame, expressed in body axes.

The runway chart uses along-runway X and right-of-runway Y. Its origin is the configured Runway 09 threshold. The default heading is 90°, so positive cross-track is approximately south. `RunwayFrame` uses a spherical azimuthal-equidistant map and inverse, not a single distant tangent-plane approximation. Height is radial altitude minus surveyed runway elevation. Long-range distance/bearing use the sphere. Terminal horizontal curvature and distance pricing use the runway chart; this remains a local approximation, with growing metric distortion on the longest allowed terminal routes.

kRPC's surface vector is `(up,north,east)`, and its vessel axes are `(right,forward,down)`. Both are left-handed conventions. The adapter transforms basis vectors on both sides before constructing a canonical quaternion. It does not reorder quaternion components as if they were ordinary vectors. Body rates are derived from consecutive coherent canonical quaternion measurements, avoiding an unverified handedness convention for kRPC axial vectors.

Legacy INI inertial states use the archival rotation phase `pi/2 + omega*UT`; the loader rotates position and velocity and subtracts `Omega × r`. Schema-5 historical JSONL instead supplies both inertial and geographic positions, so the importer recovers the rotation phase from those observations and checks their agreement. It never assumes every historical session shares the same epoch.

## Translational dynamics

In planet-fixed coordinates,

\[
\dot r=v,\qquad
\dot v=-\frac{\mu r}{\lVert r\rVert^3}+\frac{F_a}{m}
-2\Omega\times v-\Omega\times(\Omega\times r).
\]

The implementation includes spherical gravity, Coriolis acceleration, centrifugal acceleration, and a configurable atmospheric wind. There is no thrust term or simulated SAS/RCS assistance. RK4 integrates translation using the shared atmospheric and aerodynamic models. The coupled body attitude is evaluated at a midpoint within the small physics step.

Stock FloatCurve keys and both tangents are preserved with cubic Hermite interpolation. Pressure is converted from archival kPa to Pa. Temperature includes altitude, latitude bias, sunlight weighting, and the archival solar phase. Density follows the gas law and sound speed follows the configured gas properties. Pressure and density are zero at and above the 70 km atmospheric boundary. The atmosphere is therefore not an altitude-only fitted density table.

Lift and drag are bilinearly interpolated in Mach/angle of attack. The grid spans Mach 0–25 and angle of attack −10°–45°. Values outside its domain clamp to the boundary; they are not an extrapolation guarantee. Air-relative velocity determines q, Mach, angle of attack, sideslip, and bank. Drag opposes air-relative velocity. Lift is perpendicular to it and follows the body lateral axis. A provisional linear sideslip force is separate from the lift/drag table.

The inverse lift solver examines interpolation knots rather than assuming the polar is globally monotone through stall. It reports whether the requested acceleration is achievable, along with available lift and the selected alpha.

## Rotational dynamics

The force model and moment model are separate. Surface commands are rate-limited actuator states. Pitch moment includes q-dependent control authority, aerodynamic stiffness about the archival trim alpha, and configured damping. Roll and yaw authority, damping, and weathercock terms are explicit provisional inputs. Effective angular-acceleration coefficients are converted to moments using the configured diagonal inertia tensor.

Euler's equation is evaluated with inertial angular velocity. If `w` is the stored body rate relative to the fixed planet and `Omega_B` is planetary rotation expressed in body axes, then

\[
\omega_I=w+\Omega_B,\qquad
\dot w=I^{-1}(M-\omega_I\times I\omega_I)+w\times\Omega_B.
\]

A midpoint update integrates angular rate and a normalized exponential quaternion update advances attitude. The angular-momentum test evaluates momentum in the inertial frame, so a missing rotating-frame term cannot be hidden by testing only planet-fixed coordinates. In vacuum, aerodynamic authority vanishes; surface commands do not create artificial rotational control.

The pitch-trim envelope is derived from available control acceleration divided by aerodynamic stiffness, with an authority reserve. A commanded 40° alpha is therefore not assumed attainable merely because a polar row exists. The low-q identified pitch model supports roughly a 21° nominal operating trim ceiling with this reserve; increasing q can reduce that ceiling when control authority caps.

## Observation correction

`AerodynamicAuthorityEstimator` compares observed lift/drag with the nominal force table when q and force are informative. A two-second exponential filter bounds each scale to 0.5–1.5. Guidance uses the lift effectiveness for lift inversion. This improves response to force mismatch without silently rewriting the underlying polar or a committed plan. The simulator supplies plant observations through the same state contract that receives measured kRPC force data in live operation. The two sources remain distinguishable in telemetry.

This estimator is not system identification of all missing dynamics. It cannot certify the roll/yaw plant, predict part damage, infer wheel friction, resolve large sideslip effects, or replace held-out flight validation.

## Touchdown and rollout

Contact requires crossing the modeled gear-height surface; control commands do not declare a landing. Acceptance checks runway containment, gear command state, sink, horizontal speed, bank, and pitch. The live monitor also retains the immediately preceding airborne sample because KSP may zero impact velocity after contact. A pre-existing parked vessel cannot count as a successful landing.

The nominal contact limits are 115 m/s horizontal speed, 3 m/s sink, 7° bank, and pitch between −3° and 14°. Full success also requires remaining on the runway and staying below 0.5 m/s for two seconds.

Ground dynamics account for aerodynamic unloading of wheel normal force, rolling resistance, wheel braking, aerodynamic drag, and bounded steering. The simulated wheel brake is quantized at the same command threshold as the kRPC boolean brakes, avoiding an offline-only continuously variable brake. The ground model places the landed vehicle on a flat surveyed runway with a simplified nose-down transition. Suspension, tire slip, detailed gear geometry/extension time, brake fade, terrain slopes, and structural damage are not identified. Outside the runway, terrain is simplified to the datum surface. These limitations are material to live rollout qualification.

## Deliberately separate forecast fidelity

The online entry predictor reuses the same atmosphere, polar, rotating-frame translation, trim envelope, and rate/acceleration limits. It approximates attitude response with bounded alpha/bank dynamics instead of running every low-level surface/moment interaction. This is a computational tradeoff, not a second guidance law and not a six-degree-of-freedom qualification claim. Offline closed-loop validation uses the surface controller and full coupled plant. Differences between the two explain why a nominal predicted feasible route can still fail under model or tracking mismatch.

Aerothermal limits, atmospheric heating, part failures, FAR/custom aero, terrain avoidance, and navigation sensor uncertainty are outside the present model. The live adapter explicitly checks Kerbin identity and configured physical constants. Changing a name in configuration does not qualify a different aircraft or world.
