# Independent normal OMS correction — 2026-11-02

The native deorbit planner now considers a normal-first orbital correction followed by retrograde deorbit. Its two independent solve variables are burn epoch and signed normal impulse; each evaluation separately solves retrograde impulse to the configured periapsis with the finite engine/propellant model. The existing native controlled-execution and MM304/TAEM evaluation still decide candidate ranking and qualification. A better orbital screen is not promoted to a successful landing.

## Units correction

Historical `DeorbitWindow.deltaV` is **retrograde-projected impulse**, not total thrust impulse. The previous conversational decomposition of the Dessert burn as 31.46 m/s total was wrong. At 23.8403°, its 31.4614 m/s retrograde target implies approximately 13.9026 m/s normal and 34.3962 m/s total thrust integral. The investigation report has been corrected. New plans carry `normalDeltaV` explicitly. For the sequential normal-first maneuver, total commanded thrust integral is approximately abs(normalDeltaV) + deltaV, not their Euclidean norm.

## Implementation

- `DeorbitWindowSearch.normalCorrection` uses a damped two-coordinate finite solve of signed incoming ground-track miss and entry-range error. It retains the configured planning lead/search horizon, measured propellant budget, timing uncertainty and impulse resolution. Derivative widths account for finite-burn numerical resolution. A floating-point Jacobian singularity ends the solve; unresolved candidates remain advisory.
- Normal-first candidates share the existing candidate budget with legacy windows. Their actual orbital-probe evaluations are counted. The maneuver is not hidden behind a larger fixed-angle cap: the new finite model explicitly delivers the normal leg before lowering periapsis. This avoids entering atmosphere partway through a long simultaneous normal/retrograde burn.
- Fuel depletion, mass consumption, finite duration, atmosphere interception and the configured throttle remain in the price. Sequential legs consume the sum of thrust integrals. An unsupported periapsis bracket is rejected.
- `DeorbitBurnController` closes measured normal and retrograde impulse separately. Periapsis closure cannot finish a remaining normal correction; off-axis retrograde thrust cannot silently erase that correction. Actual engine-axis alignment, thrust-loss detection, periapsis taper, telemetry-gap impulse uncertainty and cleanup remain active. No bank, AoA, RCS or RPC limit was increased.
- Plans serialize the explicit normal target; diagnostics record its measured delivery. Plan review and mission logs disclose the normal-first correction separately from retrograde delta-v.
- Legacy records decode without the optional field and retain their old controller. The experimental ShuttleSim stream explicitly requests legacy candidates because its external thrust-axis servo does not implement the native normal-first controller. Its existing operation is preserved; the native planner enables the new candidate family by default.

## Focused evidence

One new regression uses the unmodified orbital state/configuration/propulsion of the original Dessert attempt, retained in `swift/KSL-Landing/Fixtures/dessert-deorbit-20261102.json`.

| Quantity | Recorded-state alternative |
| --- | ---: |
| Burn start UT | 217562.108 |
| Normal impulse target | +360.4453 m/s |
| Retrograde impulse target | 31.4500 m/s |
| Ideal finite duration | 457.75 s |
| Ideal post-burn mass | 23,428.94 kg |
| Ideal periapsis ASL | 49,999.42 m |
| Ideal entry range / incoming ground-track miss | 1,030,087.56 m / 10.81 m |
| Controlled normal delivery | 360.4472 m/s |
| Controlled entry FPA | -1.7029° |
| Controlled incoming ground-track miss | 684.56 m |
| Original plan's controlled incoming ground-track miss | 67,228.09 m |

These are model results with the measured plant inputs. They validate independently commanded lateral work and controlled orbital execution, not a live landing or MM304 delivery.

The new regression also checks component delivery tolerance, propellant remaining, planning lead, serialization, closing geometry and improvement after controlled attitude/burn execution. It passes alongside three existing selected safeguards: periapsis taper/no premature cutoff, measured versus unknown telemetry-gap impulse, and late-pointing/thrust-loss handling. No broad suite or altered-vehicle campaign was run. Python plan-review edits passed syntax checks.

A single additional diagnostic ran the **same entry forecaster used by the native planner** from the alternative's controlled entry state. It remains infeasible: terminal along -14,869.10 m, cross +220,320.33 m, altitude 25,730.55 m above the runway, Mach 2.14975, deficit `terminalRegionNotReached`. The actual post-burn mass and unchanged entry law are included. This demonstrates why an almost perfect orbital ground track is insufficient: the downstream result still has to be priced and can rank below another candidate. It does not isolate a unique atmospheric failure cause or establish that a different normal impulse/epoch could not help.

Evidence is in `Validation/Normal-OMS-20261102/`: `focused-checks.log`, `selected-window.json`, `controlled-entry.json`, `selected-entry-forecast.json`, `selected-entry-check.log`, and `release-build.log`. The extra forecast was a temporary diagnostic, removed from the permanent regression to keep routine checks narrow. Reproduction: decode the fixture configuration and retained controlled entry, construct the configured `AtmosphericModelEngine` and `AerodynamicsEngine`, then call `EntryCaptureForecaster.forecast(from:)`; strip its plan only when serializing compact evidence.

The release build succeeded after confirming that no landing executable was running. Flight readiness remains unqualified. The user subsequently authorized one live KSP test from their existing orbital checkpoint; its source/binary/configuration and outcomes must be recorded separately. No save is to be overwritten or loaded as a side effect of preparation.
