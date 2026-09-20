# Unified Vessel Physics Engine — 2026-09-08

## Why the old forecast missed real flight

The September 8 flight-log audit covered 68 logs and 58,031 snapshots. At a 60 second forecast horizon the aggregate median actual-minus-forecast error was about +221 m altitude and +8.7 m/s speed, while a representative bad run reached about +923 m and +18.8 m/s. In the newest 13:14 run the legacy force law reproduced only roughly half of measured lift through much of the terminal supersonic regime. Drag was also generally underrepresented instantaneously, although the later speed error cannot be reduced to a single drag multiplier because the forecast attitude and altitude history diverged first.

The previous system did not have one vessel model. Entry prediction used L/D, ballistic coefficient, a synthetic AoA polar and synthetic attitude rates. Passive calibration fitted another Mach-binned L/D/beta view. Trajectory calibration adjusted density/drag/lift scales from forecast residuals. Guidance often preferred measured forces directly. The Python direct controller independently identified torque/inertia response. Terminal guidance contained several additional energy and motion surrogates. These models used different definitions, different validity gates and different state histories.

Fifty-one of the 68 reviewed runs ended with zero accepted passive aerodynamic samples. The calibration gates rejected exactly the high-sink/high-incidence terminal regime in which the forecast was worst. The trajectory residual fields also substantially understated actual forecast error because they followed a continuously replaced forecast rather than an independent prediction cohort.

The logs additionally exposed a state-coherency defect: some event records contained telemetry at a newer UT while retaining an older `vehicleState`. Such records are now prevented from claiming a coherent vehicle state unless their UTs match.

## Unified runtime physics

`CLanding/vessel_physics.c` is now the shared physical-data boundary. It owns direct measured aerodynamic evidence, gravity evaluation, mass-flow/propellant-per-impulse observation, center-of-mass state, torque/inertia authority and the short-horizon axis response used by prediction.

Aerodynamic learning is based on measured kRPC lift and drag vectors rather than an assumed wing area. Each accepted observation is normalized as signed wind-axis force divided by dynamic pressure. The resulting effective-force-area sample is indexed locally by dynamic pressure, Mach, AoA, sideslip and discrete gear/brake configuration, then converted to acceleration using current vessel mass. The legacy L/D/ballistic-coefficient polar remains only a low-confidence uncovered-region fallback.

The predictor now asks the same vessel-physics layer for environment, aerodynamic forces, gravity, rotational response and observed burn mass consumption. Measured force samples are not multiplied again by the old trajectory lift/drag scales. This removes the former double-correction path in which fitted drag scale could also multiply lift.

kRPC telemetry now preserves lift and drag vectors in the non-rotating frame, body-rate validity, dry mass, current mass at control rate, torque/inertia, and CoM both as inertial vessel position and as an airframe-relative offset in the root-part reference frame. The latter is useful for identifying fuel-distribution/configuration effects rather than merely restating world position.

## Operator-selected vessel model identity

No operator-entered L/D or ballistic coefficient is required for runtime calibration. The PyQt vehicle editor no longer exposes those two fields as calibration inputs. They remain serialized internally only as compatibility priors for conditions with no measured or historical support. The durable aerodynamic-model identity itself is intentionally manual through `vehicle.modelId`.

At kRPC connection time the bridge still constructs a structural manifest from the actual assembled vessel, but that manifest is a witness and legacy-migration aid rather than the database key. The operator-selected `vehicle.modelId` is authoritative. This avoids accidental model splits caused by harmless part/geometry changes and lets the operator intentionally decide which shuttle configurations share one flight data book.

The environment identity separately includes physical body parameters and a digest of the sampled atmosphere profile and remains a SHA-256 hash of a canonical manifest. The client kRPC version and inertial coordinate epoch are deliberately excluded because they do not change the atmosphere or gravity. The structural witness is also hashed, but changing that hash does not change the manual vessel model ID. Geometry is included only if the full geometry pass succeeds; a partial RPC failure therefore cannot silently rename the model.

## Persistent observations

Durable physics observations are stored inside the project as requested:

`Runtime/Physics/observations.sqlite3`

The database stores the canonical structure/environment manifests, session lineage and sparse raw aerodynamic observations. Raw non-rotating lift/drag, position and velocity vectors are retained in addition to flight condition, mass, root-relative CoM and inertia. This is deliberate: if a future normalization or frame bug is found, historical measurements can be reprocessed rather than being trapped in old fitted coefficients.

The live C model keeps full control-rate evidence, while persistence samples sparsely for coverage. A UT rewind creates a new timeline epoch so quickloads never bridge derivative/calibration windows. Older branches remain usable as physical observations but cannot inflate one continuous time history.

On a later connection the bridge loads a bounded coverage-preserving set only when the user-selected vessel model ID and environment identity match. Legacy SHA-256 structure-keyed rows can be imported once when their witness matches the current craft. Current-flight observations remain shadow flight-test evidence and never overwrite the active predictor during the same descent.

This makes the ownership rule explicit instead of guessing it from parts: changing `vehicle.modelId` starts a separate vessel data book; keeping the same ID intentionally reuses it. Environment changes still split the model automatically because atmosphere/planet physics are not an operator-defined airframe identity.

## Validation

`Validation/VesselPhysicsTests.c` covers direct force normalization, gravity, burn mass evolution, CoM, torque/inertia authority, coherent snapshot UT, provisional historical sample import and live promotion, plus Python-protocol history parsing and C seeding.

`Validation/PythonBridgeTests.py` covers project-local storage, manual model-ID reuse across structural-witness changes, isolation between different manual IDs, quickload/reconnect grouping by flight identity, durable history reuse, coverage-preserving reduction and historical trust semantics.

The complete C test suite passes, including smoke tests, TAEM forecast transition tests, HAC join tracking and HAC planning-state invariants. Python bridge smoke tests and syntax checks pass. No live landing flight was restarted as part of this change, so long-horizon forecast improvement still needs an instrumented flight replay or new KSP flight for quantitative validation.
