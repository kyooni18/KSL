# TAEM actuator evidence and exact reference history

2026-11-03 (Asia/Seoul), following `d0c306e`.

## What the recorded sources establish

`Validation/TAEM-Recovery-20261103/audit_recorded_support.py` checks the same
first-TAEM response interval and four selected independent recorded sources.
It performs no new flight or trajectory simulation.

The first interval's quaternion-derived body rates agree with recorded FRD rates:
RMS residuals are 0.000526 roll, 0.000226 pitch and 0.000104 yaw rad/s. Reversing
the signs instead produces 0.00886, 0.03316 and 0.00242 rad/s. Getter timing and
midpoint/body-coordinate transport contribute to the small residuals; no
measurement uncertainty limit is inferred. A simple body-rate sign error does
not explain the predictor inversion.

The four inspected earlier flights have maximum RCS-off TAEM Mach values of
2.404, no RCS-off TAEM samples, 2.144 and 2.449 respectively. They do not provide
independent RCS-off coverage at the research seed's Mach 2.848. Their hashes,
measured configuration fields and marginal ranges are in `recorded-support.json`.
Marginal ranges alone do not establish joint support or configuration compatibility.
No claim is made that every historical source has been exhausted.

The older hypothetical MIMO torque grid is not a drop-in current-craft model:
it records 22 parts versus the current 26, explicitly labels its kinematics as
shadow predictions and its cells as unqualified hypothetical API results. It
also lacks the actual surface history needed for this measured replay. No grid
coefficients were adopted.

## Correct measurement boundary

The current official [kRPC development Parts reference](https://krpc.github.io/krpc/dev/csharp/api/space-center/parts.html)
distinguishes the `Deflection` override command from actual `DeflectionAngle`.
It documents actual angle, control range, actuator speed, deployment angle,
authority limiter and axis-enabled properties. This is a development API, not
a claim about the installed server's capabilities. Older versioned API pages do
not expose all these measurements. The native client therefore checks the
server's existing GetServices response before adding any optional angle calls.
No live service introspection or connection was performed in this work.

`KRPCSurfaceTelemetry` now discovers standard ControlSurface handles read-only
and records actual angle plus advertised range/rate/limiter/deployment/axis
properties in the same bounded snapshot batch. Unsupported servers incur no
new surface telemetry calls and retain nil observations. Invalid optional sensor
samples are omitted with telemetry issues; they are never replaced with zero,
a command value or a stale angle. Handles are rediscovered on connection and
topology changes. The existing snapshot-span/age and ten-second RPC checks remain.
Additional getter cost has not been measured on a live server.

`ControlSurfaceObservation` stores hinge angle/range in radians and actuator
speed in radians/second. Authority is a fraction, not a percent. Its sign is the
surface API's local hinge convention, not an identified nose-up moment sign.
The standard handle covers only a part's first ModuleControlSurface; it does not
claim to measure both modules of the split rudder. Historical records decode
with nil observations. The native aggregate-actuator plant drops actual per-part
sensor samples when advancing into a predicted future, because it cannot
truthfully produce their modeled evolution yet.

The old `systems.speedbrakeFraction` is explicitly documented as the deployment
request/readback it actually represents. Existing actuation behavior is unchanged.
Per-part measured angle is a separate field; setting a deploy angle is not
relabelled as achieved motion. No per-part moment law or incidence support is
inferred simply from the existence of these future measurements.

## Exact limiter evidence

New airborne guidance solutions include `targetHistory`: the requested target
after drag-device allocation but before limiting, and value snapshots of the
limiter before and after its update. The snapshots serialize internal bank/alpha
rates as well as the last target. The limited target remains `solution.target`;
achieved attitude remains in measured state, and the issued actuator command
remains the FlightRecord command. New replay records compare the trace when
present; legacy records remain readable without inventing missing history.

This closes the report's missing raw-reference/slew-history recording boundary.
It does not manufacture missing history for the old flight, and does not add a
second command owner, reset a limiter, alter flight limits or change requests.
Full TAEM/estimator progress and pending-command reconciliation still require
work before an arbitrary full executive can restart from a single saved seed.

## Focused verification and limits

Four selected checks pass:

```
swift test --package-path swift/KSL-Landing --filter 'ControlSurfaceTelemetryTests|NativeCommandRolloutTests|DessertTAEMRegressionTests|testPredictionCopyPreservesSlewMomentumWithoutOvershoot'
```

One compact new test covers optional capability discovery, read-only call names,
unit conversions, actual-versus-request separation, missing/invalid evidence and
serialization. Existing tests check serialized limiter momentum, actual mission
trace emission on a genuine recorded state, and native continuation. None changes
physical vehicle capability. Evidence: `Validation/TAEM-Actuator-Evidence-20261103/focused-tests.txt`.
No live API compatibility/latency, actual actuator tracking, moment closure or
landing validation is claimed. No KSP load/unpause, live command, kRPC connection,
release rebuild, original-save write or broad campaign was performed.

The next model-identification step must retain actual actuator modes and surface
history and use genuinely supported recorded states. The known first-handoff
moment-model failure and zero-bank recovery policy remain unresolved. The
requested dynamically propagated recovery planner is still outstanding; flight
readiness and full goal completion remain unestablished.
