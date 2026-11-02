# Dessert Airfield TAEM failure — 2026-11-02

## Finding

The latest attempt never ran the normal TAEM guidance law. Its Final nominal energy point (NEP) was rejected because the configured runway failed the existing modeled braking-distance check. The mission controller then spent all 2,513 TAEM samples commanding a best-glide, zero-bank fallback instead of steering toward the runway. This is a demonstrated coupling defect: an advisory rollout feasibility failure prevents the airborne TAEM controller from obtaining its reference.

Entry also delivered a poor terminal inlet. Fixing the NEP dependency alone does not establish that this trajectory could have landed.

## Run identity and evidence

- Operation: `runs/20261102T080307Z-deorbit`, started 17:03 KST; operator abort recorded at 17:28:24 KST.
- Native control-rate record: `runs/native-swift-B090715F-7F7C-4F5E-AE19-05E4FC473E85/flight.jsonl` (14,045 samples).
- Requested target was Dessert Airfield Runway 36; the native configuration selected reciprocal **Runway 18**, heading approximately 180.943 degrees. This is the actual frame used below. The selected runway has length 1,750 m, width 42.6 m and elevation 820.95 m ASL.
- The deorbit review explicitly recorded `nominalPassed=false`, `targetCaptureAchieved=false`, and `fullRouteQualified=false`. It proceeded in degraded guidance mode. The failure was not an unexpected loss of a certified nominal landing forecast.
- Evidence and offline diagnostic output: `Validation/Dessert-TAEM-20261102/`. No live flight, save loading, or unpausing was performed for this investigation.

The native manifest's `sourceReference=f1eebdc...` is **hardcoded** in `SessionTelemetryRecorder.swift:23`; it is not reliable flown-build provenance. The release executable remaining on disk was built at 17:02:10 KST, immediately before this attempt, and has SHA-256 `262b1c7543317d7842d07bded200aea54d0fc2af7880dec89c68f2a4f91d584c`. Current Swift Sources have no uncommitted changes; observed HEAD is `224ff98`. No run-bound source patch or executable hash was recorded before flight, so exact binary-to-source identity cannot be proven retrospectively. Configuration, raw-log hashes, and the observed executable identity are preserved in `provenance.json`. The logged diagnostic strings and flown behavior match the inspected code.

## Decisive offline check: a rollout verdict deletes the airborne reference

Observed failure: every TAEM sample reports `No solved NEP; retaining TAEM best-glide recovery`. The smallest decisive check was to run the existing NEP algorithm on the first recorded TAEM state and exact native configuration, adding diagnostic prints at its speed endpoints and final root. No vehicle capability, atmosphere, actuator or propulsion values were changed. This check distinguishes a failed airborne Final solution from a downstream feasibility veto.

`NominalEnergyPointSolver.solve` builds the configured Final profile and bisects inlet speed to obtain the configured 85 m/s touchdown target. It then requires the entire `TerminalAssessment.feasible` verdict, which includes runway braking distance (`NominalEnergyPoint.swift`, final guard; `EnergyManagementEngine.swift:346–349`).

The replay found:

| Quantity | Result |
| --- | ---: |
| Final inlet along / height above runway | -2,286 m / 961.511 m |
| Solved Final inlet speed | 119.169877 m/s |
| Predicted touchdown speed | 85.000010 m/s |
| Minimum lift margin | +16.0267 m/s² |
| Maximum modeled q / load | 6,916 Pa / 1.697 g |
| Braking distance, v²/(2 × 0.28 × 9.80665) | 1,315.616 m |
| Available distance: 1,750 - 400 touchdown aim - 100 allowance | 1,250 m |
| Sole root assessment violation | Insufficient runway braking distance |
| Returned NEP | nil |

The root is numerically solved and the airborne profile passes its modeled checks. The downstream braking deficit is 65.616 m. With this touchdown aim, allowance and braking model, the implied required runway length is about 1,815.616 m. This is a statement about the existing model and configuration, **not proof that the real shuttle cannot stop on Dessert**: runway survey and ground dynamics are provisional, and this simple assessment does not price the configured rollout parachutes.

The exact solver copy used for diagnostics is retained as `NEP-probe.swift`, with output in `nep-probe.log`. It was temporarily compiled as one selected diagnostic XCTest, then removed from the test target. The harness passing means only that the diagnostic executed; its result is `SOLVED nil`, not flight qualification. No permanent compile-time test was added.

## How that failure became the flown trajectory

`LandingMissionController.update` at lines 419–467 refreshes the NEP during acquisition. It calls `taem.update` only when an NEP exists. Otherwise, above Final recovery height it selects best-glide incidence with **raw bank = 0**. The normal target limiter preserves command continuity, so the first logged TAEM target still has 8.31 degrees of bank before settling toward zero. This was not an instantaneous control dropout.

Every one of the 2,513 acquisition records has the missing-NEP violation, admission false, and no normal TAEM diagnostic. There is no heading-alignment stage. The admission verdict alone is not the immediate cause of zero bank: the controller's missing-NEP branch independently dictates that command.

| Event (UT) | Height above runway | Speed | Along / cross, positive right | Threshold range |
| --- | ---: | ---: | --- | ---: |
| TAEM handoff, 219045.194 | 23,082.7 m | 648.5 m/s | -57,153.0 / +32,047.8 m | 65,525.0 m |
| Extended centerline crossed, 219149.834 | 11,405.3 m | 195.3 m/s | -54,909.1 / -3.3 m | 54,909.1 m |
| Closest logged point, 219184.414 | 9,620.6 m | 159.2 m/s | -54,366.7 / -5,708.9 m | 54,665.6 m |
| Last fresh native sample, 219249.234 | 7,630.5 m | 108.6 m/s | -53,385.4 / -13,422.5 m | 55,046.9 m |

The fallback persisted for 204.04 simulation seconds. The shuttle crossed the extended centerline about 54.9 km before the threshold while almost wings level (0.317 degrees), rather than turning onto it. Bank tracking RMS error during TAEM was only 0.926 degrees. The available evidence therefore points to the commanded path, not a gross failure to track a requested bank. All those samples remain airborne with 26 parts; there is no touchdown or rollout in this record.

The later paused-time/disconnection messages and operator abort do not explain the preceding 204 seconds of continuous failed recovery. The wrapper's final message that guidance was not engaged is misleading for this run: the native control-rate log and earlier engaged wrapper telemetry demonstrate actual guidance execution.

## Separate entry-delivery deficit

At handoff the vehicle was 57.15 km before the threshold, 32.05 km right of centerline, and descending at 106.38 m/s. The configured native entry objective uses `terminalTargetRange=35,023 m`, `terminalTargetAltitude=25,000 m ASL`, and Mach 2.5. The objective seed derives its along coordinate from that range (`EntryCommandLaw.swift:31–32`); the separate `handoffAlong=-8,000 m` field does not replace that seed. This run was not configured for the durable five-kilometre delivery aim described in AGENTS.md.

The entry law reached its lower Mach handoff deadline up-range. Its range reference repeatedly reported `range-energy-unbracketed`, and late entry commanded nearly zero bank while the lateral offset remained large. At the handoff the latest MM305 candidate reported lift/trim and actuator violations and no priced NEP solution. Those are model advisories, not proof of physical impossibility. The record does not establish whether a correctly steered, regenerated TAEM reference could have recovered this inlet. A single-entry-point offline probe cannot answer that trajectory question.

## Recommended repair order

1. Preserve a physically priced airborne Final/NEP reference and its separate rollout advisory. A braking-distance rejection must not erase the reference required to steer the airborne recovery. Keep the stopping deficit visible; do not remove the allowance, invent stronger brakes, or claim qualification.
2. Give missing-NEP recovery actual model-based lateral guidance toward a useful runway/Final objective. A planning failure must not imply indefinite zero bank. Preserve bank limits and target-limiter continuity.
3. Diagnose entry's unbracketed range/energy solution and lateral force allocation using recorded states, commands and actuator history. Reconcile the actual 35 km configuration with the intended delivery objective before changing guidance.
4. Repair provenance recording so each native session binds to the executable and source/configuration actually used; eliminate the literal source revision. Separately correct the wrapper's engaged-state reporting after a running deorbit flight is aborted.

No production guidance was changed in this investigation. The next engineering check should target the first repair with this recorded state/configuration and a narrow regression. Flight readiness remains unknown; a new live attempt is not needed to establish the demonstrated NEP/rollout coupling defect.
