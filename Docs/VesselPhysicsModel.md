# Flight-derived aerodynamic data book and unified vessel physics

`LandingController` owns one `VesselPhysicsModel`. Every successful telemetry
read updates transient vehicle state and current-flight diagnostics before any
prediction. The aerodynamic predictor itself is deliberately frozen for the
session: it is seeded only from observations attached to the user-selected
`vehicle.modelId` and the matching environment from previous flights.
Current-flight observations are archived as flight-test data and may become
part of the data book only after a later session reopens them.

This lifecycle follows the operational lesson from the Space Shuttle more
closely than online coefficient replacement: fly against a reviewed aerodynamic
database, monitor residuals and control response in flight, stress the plan
against known uncertainty, and use reconstructed flight data to improve a later
database revision. The old Mach-binned adaptive calibrator remains as a shadow
estimator and recommendation surface; it no longer rewrites nominal entry
aerodynamics during the flight that generated the evidence.

## Frames and units

* Bridge `lift` and `drag` are full signed vectors in newtons, explicitly read
  from a Flight object in the body's **non-rotating** reference frame. The
  ordinary Flight object still supplies surface flight telemetry.
* `position` and `centerOfMass` are the same sampled inertial CoM position in
  metres. This follows [kRPC's Vessel.position definition](https://krpc.github.io/krpc/0.6.0/python/api/space-center/vessel.html).
  C maps polar vectors `(x,y,z)` to its established canonical `(x,z,y)` basis.
  The log identifies this as `body-non-rotating-canonical-xzy`.
* Optional `centerOfMassRoot` is CoM relative to the root part's reference frame,
  in metres and **native kRPC root-part axes**, without the inertial permutation.
  `centerOfMassRootReferenceFrame` identifies that distinct convention. Missing
  data is null, never an invented zero offset. See the
  [reference-frame documentation](https://krpc.github.io/krpc/0.6.0/tutorials/reference-frames.html).
* Torque/inertia arrays stay in pitch, roll, yaw order, in N m / kg m². Canonical
  control-aligned body rates arrive in radians/s and are converted to degrees/s
  in C. Authority diagnostics and integration use degrees/s². Failed body-rate
  transformations are excluded from authority identification.
* Bridge `physicsSampleUT`, telemetry UT and vehicle-state UT label one packet.
  kRPC streams are asynchronous; this is packet consistency, not an atomic KSP
  physics-tick transaction. Mass and authority streams now run at the fast rate.

## Certified data-book lifecycle

A SQLite flight-test archive retains raw observations of q, Mach, signed AoA,
sideslip, mass, force vectors, device configuration and a structural witness.
The durable vessel-model key is explicitly chosen by the operator in
`vehicle.modelId`; the part manifest no longer silently creates or changes that
identity. A new bridge session selects only observations from earlier flights
with that model ID and the same physical environment identity. Client kRPC
version and inertial coordinate epoch are not part of that physical identity.
The current bridge
session is explicitly excluded from its own `physicsContext.history`.

For backward compatibility, the first use of a manual model ID may import old
SHA-256 structure-keyed rows whose structural witness and environment match the
current vehicle. Once manual IDs exist, two different manual IDs are never
merged merely because their part manifests match. Conversely, deliberately
reusing one model ID keeps one data book even if the structural witness changes;
that is an explicit operator decision rather than an automatic identity split.

For the native predictor, those prior observations are reduced to a bounded
192-cell data book. Each local aerodynamic neighborhood first keeps one clean
representative from each independent KSP flight, then chooses the cross-flight
medoid in normalized lift/drag response as the cell value. A
single clean-looking outlier therefore cannot inherit the confidence of many
agreeing flights. Confidence reflects independent repeatability, not the number
of correlated control-rate packets, reconnects or quickload branches collected
during one launch. A flight is grouped by its launch epoch (`UT - MET`) when
available.

Signed total force is projected into unbanked wind coordinates (drag opposed to
velocity, lift normal to velocity, and side force), then divided by observed q.
Queries interpolate local force/q and multiply by predicted q / predicted mass.
This preserves inverse-mass scaling without putting a fixed vehicle lift or drag
coefficient inside the direct-force model.

The compact inverse-distance kernel uses log(q), Mach, AoA and sideslip. Its
support is at most a factor two in q, 0.5 Mach and ten degrees of each incidence
coordinate, with a combined ellipsoidal distance gate. These are locality bounds,
not vehicle aerodynamic coefficients. Gear, wheel-brake and speedbrake
configurations are separate data-book dimensions. Unknown legacy speedbrake
state remains in the raw archive but is not a certified prior. Unobserved
conditions use the configured compatibility L/D and ballistic fallback at low
confidence.

Clean, speedbrake-retracted prior cells also derive a four-regime operational
envelope used by planning when a local direct-force cell is unavailable. L/D is
averaged linearly and ballistic coefficient in log space with weights derived
from sample quality and independent-flight support. Weighted scatter across the
same cells becomes `physicsCertifiedUncertainty`; uncertainty feeds explicit
stress cases rather than modifying the nominal coefficients.

The engine supplies atmosphere, gravity, aerodynamic force construction and
angular response to the predictor. Density and sound-speed corrections remain
environmental measurements. Current-flight force vectors never overwrite the
certified data-book cells. Instead, independently spaced live force samples are
compared with the matching certified prediction and contribute to a
`physicsModelResidual` health signal. A sustained high-confidence mismatch
widens the aerodynamic robustness stress envelope and raises an operator warning
while nominal coefficients remain frozen.

Direct force predictions bypass legacy lift/drag scales. The fallback path
retains learned bank/turn effectiveness whenever a forecast leaves direct-force
coverage. Density, local speed of sound and effective bank response may adapt in
flight because they describe environment/control response rather than replacing
the vehicle force database. Roll and AoA use acceleration-
limited stopping-distance response from measured authority, substepped for
numerical stability. Guidance's configured rates remain command limits. Torque /
inertia seeds each axis at low confidence; control/body-rate observations refine
effective authority. Yaw authority is exposed by the same engine interface.

The direct controller uses those measured aerodynamic-authority fractions to
blend reaction-control assistance by axis. RCS fills only the portion of pitch,
roll or yaw authority not yet owned aerodynamically. A final transonic latch
prevents late re-arming after the aerodynamic handoff.

Burns use requested thrust and inferred propellant consumed per unit impulse,
when measured consumption and dry mass are available. This estimate survives
engine-off coasting. Mass never increases or falls below dry mass. Entry forecasts
remain unpowered; a historical live throttle is not projected indefinitely into
future entry. Unknown consumption retains the historical constant-mass burn.

Backward UT and significant mass discontinuities reset derivative, mass-flow and
actuator-identification state without destroying the aerodynamic sample bank.
Wet mass is not treated as a vessel-model identity signal. Reconnect selects a
durable prior by the user-selected model ID plus environment identity, while the UI calibration reset
now resets only shadow/live reconstruction and restores that same certified
prior. Explicitly applying a reviewed calibration profile remains possible but
marks the current deorbit plan stale and requires replanning.
Snapshot reads go through `read_live_sample`, which installs telemetry, vehicle
state and attitude together. `landing_snapshot_set_sample` rejects a mismatched
vehicle-state timestamp instead of logging a stale pair.

The Python bridge tags each packet with `physicsSampleValid`. Physics learning,
legacy fallback calibration, trajectory/environment calibration and persistence
all reject degraded startup/scene-load packets. Stream failures retain their last
good value for display continuity, retry with bounded backoff, and no longer
permanently pin a stream to a synthetic fallback after one transient exception.

The deliberate glide-calibration sweep is now a flight-test maneuver rather than
an online certification mechanism. It samples the clean AoA sweep and, at the
middle AoA with generous altitude/q margin, briefly commands one speedbrake-on
sample window. Those force measurements are archived for the next session; the
current session's nominal entry model remains unchanged.

## TAEM configuration prediction and robustness

The predictor carries gear, wheel-brake and speedbrake state explicitly. Entry
keeps the KSP binary speedbrake retracted because a full action-group deployment
is not a faithful analogue of the Shuttle's continuously scheduled high-Mach
trim surface. After TAEM capture, both live guidance and prediction use the same
q-bar PI plus specific-energy speedbrake controller. The predictor also applies
the configured gear-deployment altitude, so direct-force lookup follows the
predicted vehicle configuration rather than the present configuration forever.

Automatic airborne speedbrake deployment is additionally gated on local
prior-flight coverage for the deployed configuration. If a structure has never
flown that q/Mach/AoA neighborhood with the speedbrake deployed, both live
guidance and prediction keep it retracted rather than commanding an actuator
whose drag increment the operational model cannot predict. The deliberate
calibration pulse exists specifically to gather that missing configuration data;
it becomes eligible only on a later bridge session. Rollout remains free to use
the speedbrake because no airborne trajectory prediction remains to certify.

Plan certification includes explicit drag-high/lift-low and drag-low/lift-high
cases. Their magnitude comes from prior-flight data-book scatter and can be
widened by a confident current-flight model residual. Stress multipliers exist
only in the scenario copy of the model; they never mutate the certified nominal
data book.

## Diagnostics and validation

Telemetry/log records contain signed lift/drag vectors, both CoM conventions,
dry mass, learned mass flow, support size/confidence, and pitch/roll/yaw authority
and confidence. They also expose certified uncertainty, current-flight model
residual/confidence, current-flight shadow sample count, and whether a locally
certified deployed-speedbrake model is available. `predictedPhysicsObservedSeconds` and
`predictedPhysicsFallbackSeconds` report the latest cached atmospheric forecast's
coverage (q > 1 Pa); these are not smoothed with trajectory outcomes.

`Validation/VesselPhysicsTests.c` covers certified-vs-live separation, local
interpolation, configuration-specific lookup, data-book envelope derivation,
aerodynamic stress multipliers, the TAEM speedbrake controller, the deliberate
speedbrake flight-test pulse, mass scaling/consumption, authority, CoM ingestion,
reset boundaries, snapshot JSON timestamps and the mock Python-to-C decoder.
Python bridge tests cover current-session exclusion from the data book,
independent-timeline support, vector/frame validity, recoverable stream fallbacks
and axis-specific RCS blending.

## Physical limitations

This is a reduced-order translational/attitude engine, not a six-degree-of-freedom
rigid-body solver. Forecast sideslip is zero; yaw authority is exposed but a yaw /
sideslip evolution equation, inertia cross-products, aerodynamic moment curves,
trim, actuator delay and damping identification remain future work. AoA response
approximates pitch response rather than integrating a full body quaternion.
Effective authority may include trim, damping and axis coupling; it is deliberately
low confidence initially and is not extrapolated upward with q. It cannot yet
separate aerodynamic authority from reaction wheels/RCS at different q.

CoM offsets are retained for diagnostics; no unmeasured force application point
is invented to turn them into moments. Atmosphere assumes co-rotation, without
wind. Gear/brake/speedbrake partitioning does not yet identify arbitrary flap,
damage or an unannounced mid-session topology change. Speedbrake prediction is a
binary KSP approximation of a continuously positioned real control surface.
Structure-matched observations are persisted between sessions and the bounded
native data book necessarily compresses that archive. Outside prior-flight
coverage, fallback accuracy remains limited. Sparse
stream timing and sensor quantization limit acceleration and fuel-flow learning.
No live KSP flight validation was performed by these deterministic tests.
