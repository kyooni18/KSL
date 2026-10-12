# Native MM305/Final certification in the KSL deorbit planner

**Date:** 2026-10-13 (KST)  
**Repository:** `/Users/kyooni18/Code/KSL`  
**Scope:** Offline engineering implementation and recorded-state verification; **no KSP live operation**.

## Decision and scope

The 2026-10-12 Polaris overflight investigation established that KSL's original polar deorbit opportunity search could pass an entry-stage boundary while leaving MM305 with no certified HAC/Final route. The previous change (commit `65cbe69`) retimed the nominal fourth-flight deorbit and reduced the predicted MM305 distance from 153.5 km to 71.3 km, but left its certified inlet energy near -246 kJ/kg.

This change wires the *existing* production closed-loop MM304 -> native MM305/HAC -> Final replay into the orbital deorbit planner. It does **not** invent a favorable route, bypass the MM305 1 km physical capture contract, or relax the energy and safety criteria. An irreversible deorbit remains blocked in the strict STS polar mission until the whole route is justified by measured-state native evidence and the configured robustness cases.

## Changes

### Source of truth and physical frame

`CLanding/terminal_model.c` and `CLanding/include/terminal_model.h` now provide `terminal_model_align_planet_frame()`. A model imported from ShuttleSim has atmosphere/aero/attitude tables, but its UT=0 inertial prime meridian cannot be assumed to match KSP. Given an observed `PlanetModel`, alignment converts the measured epoch-based prime meridian to the native `rotation_phase_rad_at_ut0` representation:

`phase_at_UT0 = atan2(prime_y, prime_x) - omega * planet_epoch_UT`.

The binder validates native world radius, gravitational parameter, rotation rate and +Z axis compatibility, and explicitly marks `planet_frame_aligned`; a mismatch clears the flag. `landing_controller_connect()` captures then binds the terminal plant to the live planet. A model with valid source tables but no planet-frame binding cannot certify an orbit-to-runway solution. At plan creation, `terminal_model_planet_matches()` rechecks the actual epoch/prime-meridian phase (modulo a full revolution) and dynamic constants against the captured native model, preventing silent reuse of a valid-looking snapshot from a different KSP session.
Before each live deorbit plan, the controller checks the full immutable runway, vehicle, and guidance policy snapshot. If any policy input differs (for example, an adaptive speed envelope), the native model is recaptured and rebound; a stale model is not reused for strict route certification. The dedicated frame test checks stale runway, vehicle and guidance inputs as negative cases.

### Active guidance/policy identity

The new `terminal_model_policy_matches()` gate compares the captured runway/vehicle/guidance snapshot against the currently effective configuration. In-flight identification can change effective minimum/final/touchdown speeds, so the controller re-captures a new immutable native model if the effective policy differs before evaluating a deorbit. The native planner itself also fails closed if called with a stale or different policy; it never treats an entry-only fallback as full terminal certification. `Validation/TerminalPlanetFrameTests.c` verifies that a changed vehicle speed, runway bearing, or HAC radius rejects the stale terminal model.

### End-to-end planning integration

`deorbit_plan_create_with_terminal_model()` accepts an immutable captured `TerminalModel` and the measured control-authority evidence. Existing `deorbit_plan_create()` and `deorbit_plan_create_with_telemetry()` remain compatibility entrypoints and **do not** falsely represent an entry-only witness as native runway certification.

The planner still uses the cheap orbital search to select a nearby pass and a bounded MM304 inlet search. When an inlet is geometrically plausible and has nonnegative certified energy, the candidate is also checked through the native MM305/Final executive so that full-route evidence can influence ranking. The final selected candidate always receives an end-to-end native witness if a frame-aligned model is available. The witness runs production guidance over the predicted post-burn trajectory, including terminal model route planning and final-approach transitions, rather than stopping when MM305 takes control.

If (and only if) the nominal full-route result qualifies, each of the 15 existing burn/thrust/aerodynamics robustness cases runs the same native executive before being counted as a strict runway pass. Unqualified nominal trajectories still receive their entry-stage stress diagnostics, without declaring them runway successes.

The strict runway predicate now requires all of: acceptable entry loading; physically captured MM305 inlet; nonnegative certified inlet margin; actual committed native terminal route **and** successful Final acquisition in the continued witness; no aborted or missed boundary; and valid terminal geometry/speed. A route being merely committed is not evidence that the vehicle ever reached Final.

### Inspectable native evidence

Deorbit plan JSON and persistent planner JSONL now include:

- `terminalModelSnapshotId`
- `terminalReplayEvaluated`
- `terminalInletQualified`
- `terminalNativePathCommitted`
- `terminalFinalApproachCaptured`
- `terminalGuidanceAborted`
- `terminalInletEnergyMargin`

`DEORBIT_NATIVE_TERMINAL` trace prints the corresponding snapshot identity, physical inlet status, route commitment, Final capture, abort status, specific-energy margin, and nominal strict capture decision. The recorded-orbit benchmark exposes `--fourth-polar-terminal` and `--polar-terminal` without connecting to kRPC.

## Offline evidence

Both native and legacy entry-only variants were run using saved polar orbital states and the same ShuttleSim aerodynamic world.

| Frozen fixture | Native terminal replay | MM305 physical inlet | Route committed | Final acquired | Terminal energy | Strict recovery |
| --- | --- | --- | --- | --- | --- | --- |
| Fourth polar, native | Yes, snapshot 11 | No | Yes | No | -246.4 kJ/kg | No; 0/15 |
| Other polar, native | Yes, snapshot 11 | No | Yes | No | -252.2 kJ/kg | No; 0/15 |
| Fourth polar, entry-only | No | Not certifiable | Not evaluated | Not evaluated | -246.4 kJ/kg (diagnostic) | No; 0/15 |

These results expose an important counterexample: **the native MM305 planner can commit a route even when the MM304 outlet has not passed the physical inlet test, and the continued flight can still fail to acquire Final.** The stricter certification therefore correctly rejects route commitment alone. In the fourth-flight replay, native continuation recorded a KSC closest distance near 21.4 km; this does not alter the original 71.3 km *handoff* distance. The numbers measure different events.

The remaining blocker is not missing integration: it is a physically unqualified MM305 inlet and unsuccessful Final transfer with the sampled deorbit energy and geometry. Neither replay constitutes a successful unpowered landing or validates the revised retrograde burn taper in live KSP.

### Reconstructed physical failure at the MM305 boundary

Additional diagnostics using `KSP_LANDER_TRACE_ENTRY_WITNESS=1` and `KSP_LANDER_TRACE_DEORBIT_SEARCH=1` isolate why the geographically improved fourth-flight inlet remains inadequate:

| Native MM305 admission term | Fourth polar, 71.3 km inlet |
| --- | ---: |
| Inlet speed / course | 863 m/s / 185.2 degrees |
| Runway heading | 90 degrees |
| Estimated instantaneous minimum turn radius | 72.4 km |
| Straight-line distance to the Final reference | 72.0 km |
| Specific energy available before drag | 566,925 J/kg |
| Modeled straight-reference drag work | 560,919 J/kg |
| Certified uncertainty charge | 252,413 J/kg |
| Certified inlet margin | -246,415 J/kg |
| Native continuation endpoint | 124 m altitude, 207 m/s, 32.7 km from KSC |

The instantaneous turn radius is already comparable to the entire available approach range while the initial heading differs by about 95 degrees from RW09. The nominal energy budget has only about **6 kJ/kg** left after predicted drag on its minimal reference, even before accounting for a real curved path or uncertainty. The radius is an *instantaneous* bound, not a constant-radius impossibility proof after deceleration; nevertheless the native replay independently confirms failure to deliver the shuttle to Final. The defect is upstream MM304 inlet shaping and energy conservation, not a missing HAC-route object or a benign telemetry labeling error. Do not relax the certified energy veto merely to allow a deorbit.

## Tests

- Native terminal planet frame regression verifies longitude at the recorded UT and at +300/+600/+900/+1200 s, along with rejection of wrong pole or gravity. See `Validation/TerminalPlanetFrameTests.c` and `Validation/TAEMNativeStackTests.c`.
- Strict full-route admission unit test rejects a committed HAC route that has **not** acquired Final. See `Validation/DeorbitStageAdmissionTests.c`.
- `make test`: fast native C suite passed; Python 142 tests and 51 subtests passed.
- The complete, slower `taem-native-stack-test` gate was not verified to completion in this iteration; do not infer qualification from the fast suite or the recorded-orbit witness alone.
- Frozen offline benchmarks:

```sh
make -C CLanding build/benchmark_deorbit_planning
CLanding/build/benchmark_deorbit_planning --fourth-polar-terminal
CLanding/build/benchmark_deorbit_planning --polar-terminal
CLanding/build/benchmark_deorbit_planning --fourth-polar-plan
```

## Next engineering frontier

1. Compare the native MM305 **committed-route-but-no-Final** cases against the actual replayed trajectory, determine the violated Final delivery contract, and repair only the established mechanism.
2. Improve MM304-specific energy management or targeting from feasible high-altitude states. The sampled 50 km post-burn periapsis gives a roughly -246 kJ/kg terminal inlet margin; no amount of relabeling the capture state removes that deficit.
3. On an adequately qualified orbital fixture, verify that the positive native route is genuinely delivered through Final and stress-evaluate the same committed route under relevant aerodynamic variations before enabling strict autonomous deorbit.
4. Only after offline qualification, run an explicitly authorized live KSP mission and compare the whole 70/50/30/25 km profile, propulsion timing, HAC acquisition and runway delivery.

**Operational interpretation:** The native deorbit planner leaves `targetCaptureAchieved=false` for this orbit and never calls the recorded fourth-flight overflight a certified runway landing. The separate KSL orchestrator may be configured for strict admission (`landing.requireStrictPlan=true`) or a warning-only experimental attempt. A warning-only launch is **not a qualified production landing**; check the active mission policy explicitly before any live KSP operation.
