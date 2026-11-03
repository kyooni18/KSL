# Runway Capture Investigation

Date: 2026-10-31  
Repository: /Users/kyooni18/Code/KSL  
Starting HEAD: `678f653` — Reject early immediate terminal ownership after water impact  
Status: source corrections implemented; executable validation deliberately not performed.

Follow-up: section 13 analyzes the subsequently archived `live-runway-capture-20261031-1241` record, corrects remaining range-allocation, ranking, search-coverage, and tracking defects, and preserves the no-execution restriction.

## 1. Outcome and scope

The evidence supports a failure chain that starts in MM304, crosses the MM304/MM305 planning and ownership boundary, and is amplified by the terminal reference and its force allocation. It does not support attributing the failure solely to MM305, reversing a coordinate sign, or increasing flight-control gains.

MM304 was trying to intercept a runway-relative point while arriving approximately reciprocal to the landing direction. Its lateral demand grew far beyond the lift left by its longitudinal/AoA selection. Prospective terminal exploration was delayed by operating-envelope gates, so a large remaining turn was not being managed as a terminal maneuver early enough. MM305 then accepted a selected advisory even when its own assessment rejected the path. An aggressive vertical reference could unload the lift vector while commanding a large bank. The recorded vehicle often followed those particular attitude commands reasonably closely; it consequently banked without producing the lateral acceleration the ground track required. Energy-floor continuation and overstated predicted attitude authority made the numerical evidence more optimistic or less physically interpretable than it appeared.

The corrections address that chain without replacing the existing aerodynamic tables, atmospheric model, translational integrator, FCS gains, or CLI/telemetry protocols. They separate advisory exploration, modeled feasibility, and control ownership; couple entry AoA selection to turning work; enforce a shared force budget; and stop physically exhausted trajectory predictions.

**No tests, builds, compilation, benchmarks, simulations, flight replays, experimental trajectories, KSP interaction, kRPC connections, controller execution, or background monitoring were performed. No logs or saves were modified. No commit was made.** This report is not a landing, capture, robustness, numerical-convergence, or compilation claim.

### Answers to the principal questions

| Question | Finding |
| --- | --- |
| Why were position and heading errors not converging? | A point-intercept objective did not impose the required runway-facing terminal course. The requested lateral acceleration subsequently exceeded delivered acceleration by orders of magnitude. Longitudinal AoA selection and lateral steering were not budgeting the same maneuver. |
| Was MM304 delivering an attainable terminal approach? | Attainability was not established. In the cited records, its delivery had substantial crossrange and nearly reciprocal course. The code could continue pursuing the point even when the downstream family was inadmissible. |
| Was capture already impossible at MM305 transfer? | The selected and examined paths were rejected by the available model. That establishes an unsupported transfer, not a mathematical proof that every possible physically admissible trajectory was impossible. Earlier archived “feasible” predictions also have model qualifications. |
| Did planning understand actual turning capability? | Partly. Existing force-based work was present, but a true required radius was clipped to the search-radius ceiling; other diagnostics still used the supported-level-turn assumption; predicted moment authority could replace a smaller measurement with a larger provisional prior. |
| Were HAC paths physically feasible? | Geometric closure was insufficient. Examples required unavailable lift, excessive bank/load, a negative upward normal force, or exhausted energy. An advisory was nevertheless allowed to become the control reference. |
| Did the controller execute its commands? | At the important early MM305 sample, bank and AoA were close to their limited targets. This supports a planning/allocation failure there. Later very large bank errors remain a separate unresolved control/plant issue. |
| What changes address the causes? | Earlier advisory steering with entry ownership retained; turn-aware AoA/energy selection; truthful turn-radius and energy pricing; force-feasible tracking; explicit admission; actual final capture gates; and consistent predicted controller state across phases. |

## 2. Evidence, provenance, and preservation

### Evidence used

- **E1 — Overnight summary:** `~/Desktop/KSL-Overnight-Results.md`.
- **E2 — Main engineering chronology:** [Overnight-Orbital-Baseline-20261031.md](Overnight-Orbital-Baseline-20261031.md), especially “Entry-checkpoint flight and derived 1,200 m/s replay,” “HAC candidate pricing experiment and checkpoint flight,” and “Earlier immediate terminal ownership.”
- **E3 — Detailed v21 reconstruction:** [MM304-MM305-v21-Investigation.md](MM304-MM305-v21-Investigation.md). This was existing, uncommitted work and was not edited in this investigation.
- **E4 — Current-comparison flight:** [live.jsonl](../../runs/overnight-entry72-current-comparison-20261031/live.jsonl), its configuration/source patch, and [forensic/geometry.csv](../../runs/overnight-entry72-current-comparison-20261031/forensic/geometry.csv).
- **E5 — v21 flight and archived analysis:** [live.jsonl](../../runs/live-swift-v21-rwy27-crossrange-checkpoint72-20261031/live.jsonl), `capture-analysis.json`, and `capture-geometry.csv` in the same directory.
- **E6 — Physical-model provenance:** [Reference-Audit.md](../../swift/KSL-Landing/Documentation/Reference-Audit.md), the bundled `STSAerodynamics.csv`, the archived STS polar metadata, and existing C-compatible/Rust physical-model implementations.
- **E7 — HAC design background:** [STS-TAEM-Heading-Alignment-Cone-20261021.md](STS-TAEM-Heading-Alignment-Cone-20261021.md), plus the NASA sources in section 9.

The current-comparison raw live log was read directly at the relevant rows, and its SHA-256 was recomputed from bytes:

`a48a6aa248db8f5c2454fdb6613add1e46312efc293f98ca6fb8a83f56835414`

Its archived flown HEAD was `b9d272d8c156e8702542f2561c74c2ce7f71023f`; configuration SHA-256 was `7d562b97f847432c84b3c911a22cab7b0315cd314d445579c62edb357f913a45`, and source-patch SHA-256 was `fc975a9d079e1c40583e8191f4db8cbe48f71914cc23bad543e700b4c53b5fb9`. The current checkout includes later changes and must not be treated as the binary that produced that flight. E2/E4 retain the binary and run provenance.

The v21 and current-comparison runs are different flights. Elapsed times are relative to their own first telemetry records. They are not interchangeable matched-state experiments.

### Evidence categories

1. **Flight measurements:** positions, velocities, mass, aerodynamic force telemetry, actual attitude, and contact observations in recorded live logs.
2. **Recorded controller outputs:** limited targets, actuator commands, mode/ownership changes, and planning proposals. A command is not an achieved acceleration.
3. **Derived recorded-data quantities:** runway coordinates, heading errors, acceleration projections, and drag-work balances computed from those records. These are read-only analysis, not newly flown trajectories.
4. **Numerical predictions:** archived point-mass pricing and closed-loop model results. These remain predictions even when a field is named “feasible” or “certified.”
5. **Engineering deductions:** force-cone constraints, geometric differentiation, and the causal interpretation below. Their assumptions are stated explicitly.

No new flight-performance measurements were invented. Static polar interpolation in section 5 is labeled separately from measured lift.

### Existing work preserved

The repository had 16 modified tracked files and an untracked transition probe before this work. The initial working diff was retained at `/tmp/ksl-runway-capture-before-20261031.patch`. Working-tree originals of files edited here were retained under `/tmp/ksl-runway-capture-originals-20261031/`.

A read-only comparison of the initial and final diffs confirmed unchanged pre-existing modifications in E3, `FlightControlSnapshot.swift`, `EntryDragReference.swift`, `TerminalTrajectoryEvaluator.swift`, `EntryEnergyProbe.swift`, `Main.swift`, `Tests/GuidanceTests/GuidanceTests.swift`, and `Tools/investigate_transition.py`. The untracked probe was not edited. Changes in shared guidance/physics files were layered onto their working-tree contents rather than restored from HEAD.

The previously committed fair candidate-pricing work, dynamics-revision protection, and rejection of immediate high-altitude terminal ownership were preserved. Existing tests were neither modified by this investigation nor executed.

## 3. Reconstructed flight pipeline

All paths in this section are under `swift/KSL-Landing/Sources/` unless stated otherwise.

| Stage | Actual implementation and data flow |
| --- | --- |
| Navigation and frames | `KSLKRPC/Telemetry/KRPCFlightIO.swift` samples the live vessel. `KRPCFrameConverter.swift` converts the kRPC surface/body conventions into KSL planet-fixed vectors, FRD body rates, and attitude. |
| Vehicle/model adaptation | `KSLRuntime/FlightComputer.swift:prepare` observes configuration/dynamics changes and updates the planning revision before snapshots are accepted. Measured mass, force, and moment/inertia telemetry remain distinct inputs. |
| Fast MM304 guidance | `KSLGuidance/Entry/EntryGuidanceEngine.swift:update` passes control feedback into the stateful `EntryCommandLaw`. The law combines the entry objective, drag/energy budget, AoA choice, and signed lateral demand. |
| Prospective planning | `KSLGuidance/Landing/MissionPlanning.swift` owns a separate planning actor. It receives immutable state, controller, target, environment, and HAC-context snapshots. A terminal request takes priority over an entry forecast in that worker. |
| Terminal geometry and pricing | `HACPlanningEngine` creates acquisition/spiral/unload/final geometry; `EnergyManagementEngine` prices the force and energy requirements. Preview planning is separate from `TerminalTrajectoryEvaluator` closed-loop prediction. |
| Ownership and path tracking | `LandingMissionController.acceptTerminal` decides whether a candidate can replace the active owner/reference. `PathTrackingEngine` projects onto the path and converts tracking errors into lateral and vertical force demands. |
| Final approach | `ApproachGuidanceEngine.canCapture` checks measured alignment, corridor, speed, height, and final energy/capture geometry before normal final-approach ownership. |
| Command continuity | `KSLCore/Models/GuidanceTargetLimiter.swift` limits commands across phases using the existing slew/response limits. Its history is retained through normal handover. |
| Surface control | `KSLControl/Attitude/FlightController.swift:command` uses the shared air-relative quaternion target, identified trim/authority, and body-rate control to produce pitch/roll/yaw inputs. `KRPCFlightIO.apply` sends the direct control inputs. KSP autopilot is not the owner of these MM304/MM305 commands. |

The live asynchronous path rejects stale dynamics revisions. The older same-UT rejection problem had already been corrected with a revision counter and was not reintroduced.

A planning lifecycle defect remained: the mission could consume an MM304 request epoch and then send the worker a terminal request that caused that MM304 work to be discarded. Terminal priority could also be asserted between actual survey deadlines. The revised request path determines which work will be sent before consuming the entry planning epoch and prioritizes terminal work when its cadence is due.

### Runway geometry and signs

`RunwayFrame` uses a spherical azimuthal-equidistant chart centered on the selected threshold:

- Along-track is positive in the selected runway's landing direction.
- Cross-track is positive to the right of that direction.
- Height is radial altitude above runway elevation.
- The entry objective's altitude is ASL; conversion subtracts runway elevation when constructing a chart position.

For runway 27, positive cross-track is approximately north. An eastbound crossing of the centerline can reduce point-intercept error while remaining almost 180 degrees from the westbound landing course. Bearing to the runway, bearing to the entry aim, and transported runway-forward heading are therefore different quantities.

E3 verified the reciprocal threshold transformation: runway 27 is at approximately latitude -0.048610689129 degrees, longitude -74.489600832741 degrees, heading 269.999797454241 degrees. Along +2500 m returns the opposite threshold. The 68.44 m ASL/runway-height distinction was not a 25 km lateral-error mechanism. The inspected direct-control and runway bank signs did not justify a sign reversal.

There was, however, a real differential-frame inconsistency. Projecting velocity onto fixed threshold axes, or treating a transported orthonormal rightward speed as the derivative of chart cross-track, is not generally the derivative of the spherical chart. The correction adds the analytic chart differential and its inverse:

- `RunwayFrame.direction(at:tangent:)` maps a chart tangent into the local tangent plane.
- `RunwayFrame.coordinateVelocity(at:velocity:)` maps physical velocity into chart-coordinate rates, including the altitude scale.
- HAC inlet direction and entry cross-track rate now use that mapping.
- Path heading comparison is performed at the vehicle's local tangent plane.

For chart radius rho, central angle theta = rho/R, and radial/transverse chart components, the forward differential scales the transverse component by sin(theta)/theta. The inverse scales it by theta/sin(theta), with an additional R/r factor for physical velocity at radius r. The origin uses the analytic limit; the antipode is explicitly singular. This is a mathematical correction, not evidence that a coordinate sign was the dominant late-flight cause.

## 4. Recorded sequence and causal separation

### Current-comparison flight

The following values are from E4's existing geometry CSV, cross-checked against selected raw JSONL rows. Aerodynamic lift below is force divided by recorded mass. Turning acceleration is the existing measured-force/rotating-frame projection, not commanded bank interpreted as acceleration.

| UT | Elapsed s | ASL m | Mach | Airspeed m/s | Along m | Cross m | Runway-heading error deg | Requested lateral m/s² | Measured turn m/s² |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 196404.2488 | 399.98 | 33831 | 4.746 | 1535.5 | 85056 | 38759 | 176.43 | 60.20 | 2.39 |
| 196444.2888 | 440.02 | 27949 | 4.018 | 1239.5 | 32587 | 33227 | 171.06 | 182.93 | 2.99 |
| 196464.2488 | 459.98 | 25421 | 3.455 | 1046.4 | 11188 | 29117 | 166.60 | 610.61 | 5.33 |
| 196480.6488 | 476.38 | 23816.7 | 2.844 | 857.53 | -3170 | 25013 | 161.17 | MM305 ownership begins | 5.70 |
| 196488.4688 | 484.20 | 23166.5 | 2.656 | 799.66 | -8945 | 22891 | 158.68 | MM305 tracking | 2.87 |

The handover is raw log line 5324. Its measured mass is 25501.56 kg, lift 294412.44 N, q 6976.55 Pa, actual bank 29.73 degrees, and actual AoA 18.68 degrees.

The closest-threshold sample is raw line 5409, only 7.82 seconds later. Its horizontal speed is 793.59 m/s, actual bank 60.44 degrees against target 59.10 degrees, and actual AoA 5.51 degrees against target 6.73 degrees. Lift falls to 84352.63 N at mass 25491.84 kg: 3.309 m/s² of lift acceleration. Thus:

```text
(L/m) sin(bank) = 3.3090 sin(60.4378 deg) = 2.8782 m/s²
recorded measured turning acceleration                  = 2.8671 m/s²
```

The small difference is compatible with the additional force/frame terms in the recorded reconstruction. A large bank with this small lift cannot produce a tight turn. The near agreement between target and measured attitude at this instant argues against a bank-sign or gross roll-tracking error as the explanation for its poor heading convergence.

The flight subsequently impacted water. E2 records the final water/debris observation around UT 197045.8288, with no verified runway touchdown or rollout.

### v21 flight: independently useful mechanism evidence

E3/E5 provide a separate, more detailed chain:

- Around +400 s, MM304 requests approximately 72.70 m/s² laterally while delivering approximately 3.25 m/s².
- At +463.92 s, requested lateral acceleration reaches approximately 3343.42 m/s² versus measured 4.36 m/s². The direction is corrective; the magnitude is unattainable.
- The first terminal survey occurs at +463.92 s, and the first MM305 control record follows at +464.02 s. The approximately 0.1 s proposal delivery does not support blaming worker latency for the long preceding delay.
- At that survey, zero of 54 examined candidates complete the relevant admissibility checks, yet the chosen advisory is accepted.
- Its archived pricing reports 31.222 g, 89.981 degrees bank, and a minimum lift margin near -292.111 m/s². These are **model requirements**, not measured aircraft loads.
- At +464.02 s, the selected reference's upward normal demand is approximately -24.95 m/s², with vertical curvature approximately -4.4502e-5 per metre. The raw command combines approximately -0.229 degrees AoA with 58.58 degrees bank.
- At +483.94 s, the measured AoA is about 0.48 degrees, bank about 64.96 degrees, lift acceleration about 0.424 m/s², and turning acceleration about 0.364 m/s².
- After +494.18 s, a roughly 175.95 km rejoin within a roughly 203.88 km route is labeled as spiral continuation; the diagnostic join distance of about 6.24 km represents a response distance, not the actual rejoin.
- At +519.42 s, actual bank reaches about 147.50 degrees while the limited target is about 48.46 degrees. This later discrepancy is real evidence of a separate control/plant or coupled-attitude problem; the early near-target sample does not explain it away.
- The last v21 sample remains airborne at approximately 1974 m ASL with a large sink rate before disconnection. It is not evidence of landing success.

E3's existing force/energy audit over the transition interval reports median measured/model lift and drag ratios of about 0.997 and 0.951. Mechanical-energy loss was approximately 771557 J/kg; measured drag work was approximately 772221 J/kg, a difference of about 0.086%. This supports energy loss by aerodynamic work in that interval, rather than an invented unexplained energy source. It does not validate the full polar or future torque response.

### What the counterfactuals establish

The archived earlier-ownership experiment transferred MM305 authority at UT 196396.4488, about 35044 m ASL, Mach 4.85, and 1582 m/s, with about 39.46 km crossrange. It produced a substantially worse water-impact result, about 75.64 km from the threshold in the final log. The experiment coupled earlier searching to immediate advisory ownership. It does **not** test the architecture implemented here, in which MM304 can use terminal advice while retaining its own force/energy law and control history. [E2]

The fair-pricing change improved modeled trajectory loads in archived comparisons. A later live flight still failed, and the initial conditions differed. This is evidence that fair pricing matters; it is not evidence that ranking alone fixes capture. [E2]

An archived v21 survey around +402.94 s found a modeled candidate approximately 124971 m long with predicted touchdown speed about 87.21 m/s. Those results predate the corrections to energy continuation and seeded attitude authority. They motivate earlier exploration but do not establish a verified last recoverable state. [E3]

## 5. Physical capture envelope

### 5.1 Available force, not bank angle alone

Let A = L/m be available lift acceleration, sigma be bank, V airspeed, Vh horizontal speed, gamma flight-path angle, and r planet radius at the vehicle. In the still-air, coordinated point-mass approximation used by terminal pricing:

```text
lateral acceleration       a_lat = A sin(sigma)
upward wind-normal force   a_up  = A cos(sigma)
heading rate               psi_dot approximately a_lat / Vh
horizontal turn radius     R_turn approximately Vh² / abs(a_lat)

reference upward normal demand
a_up,req = (g - V²/r) cos(gamma) + V² k_vertical
```

For a required positive upward component B, an available total lift bound Amax, and a bank limit sigma_max:

```text
If Amax < B: the vertical demand is already infeasible.
Otherwise:
a_lat,max = min(Amax sin(sigma_max), sqrt(Amax² - B²)).
```

The load limit must constrain Amax before lateral authority is computed. It cannot be applied only after choosing a radius. The code uses the established polar, actual mass, q, Mach, trim ceiling, applicable gear state, effectiveness estimate, configured 70-degree bank limit, and 3.5 g load limit. It does not substitute a new aircraft performance constant.

The supported-level-turn identity a_lat = g tan(sigma) requires the appropriate vertical support condition. It is not a measured capability at low lift or during a steep descent.

At the current-comparison handover, the local observed radius is approximately **127.84 km** and heading rate **0.3826 degrees/s**. At the 60.44-degree-bank sample, the local observed radius grows to **219.66 km** and heading rate falls to **0.2070 degrees/s**. A 180-degree turn at those frozen states would take approximately 470 s and 870 s respectively. Actual speed, lift, altitude, and turn rate change continuously: these are instantaneous scale estimates, not simulated trajectories or global reachability bounds.

### 5.2 More AoA has a substantial energy cost

A static interpolation of the unchanged bundled polar at the handover's Mach 2.84363, q 6976.55 Pa, mass 25501.56 kg, and 250 m² normalization gives:

| AoA deg | Modeled CL | Modeled CD | Modeled L/m m/s² | Modeled D/m m/s² | Modeled L/D |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 10 | 0.08730 | 0.12929 | 5.97 | 8.84 | 0.675 |
| 20 | 0.15501 | 0.20400 | 10.60 | 13.95 | 0.760 |
| 30 | 0.20774 | 0.30234 | 14.21 | 20.68 | 0.687 |
| 40 | 0.23623 | 0.39335 | 16.16 | 26.90 | 0.601 |

This table is a **bundled-table calculation**, without historical local correction, trim qualification, uncertainty scaling, actuator dynamics, or a trajectory prediction. It is not measured maximum capability. It shows why neither “bank harder” nor “always maximize AoA” is a defensible general fix: extra lateral force consumes substantial drag work, and the available force depends strongly on AoA.

For a local turn, the energy spent per heading radian scales as:

```text
abs(dE/dpsi) approximately (D/m) V Vh / abs(a_lat).
```

This motivates a maneuver-efficiency recovery criterion based on drag divided by the useful lateral acceleration, capped at the requested acceleration. It does not justify assuming that locally efficient controls define a globally reachable runway path.

### 5.3 Heading, crossrange, and energy must close together

A useful terminal solution must satisfy all of the following, with the same state and force history:

- A forward-traversable ground track ending in the correct landing direction.
- Sufficient integrated heading change: the required change must fit the time integral of available heading rate.
- Sufficient crossrange correction, including current crossrange velocity and the cost of reversing/rolling.
- Positive, attainable normal force throughout the path; bank, load, q, AoA, and commanded slew limits.
- Energy remaining for the complete final profile, touchdown-speed envelope, and braking distance.
- A terminal state inside the actual final-capture corridor.

For a local fixed-axis approximation, crossrange evolves as e(T) = e(0) + e_dot(0) T + the time-weighted integral of lateral acceleration. Position error alone is not enough to establish capture; neither is a small bearing error. An S-turn that improves one coordinate can spend the very energy or heading-change budget needed for the final return.

For entry's rotating-frame mechanical energy, the existing source uses:

```text
E = |v_fixed|²/2 - mu/r - |omega cross r|²/2.
```

In the still-air case, drag removes that energy at approximately (D/m) V. Terminal pricing remains a reduced point-mass calculation with spherical gravitational potential and sampled chart geometry; its qualifications are listed in section 10. The source correction removes invented positive kinetic energy after exhaustion rather than presenting this reduced model as a full reachability proof.

### 5.4 What “feasible” now means

| Result category | Meaning and permitted use |
| --- | --- |
| Geometric candidate | Its sampled acquisition/HAC/final geometry exists. It may still fail force, rate, energy, or operating constraints. |
| Fully priced but inadmissible candidate | The model can calculate a full path, but one or more requirements exceed limits. It remains advisory. |
| Energy-exhausted/incomplete candidate | The prediction ends before the next station can be reached with positive kinetic energy, or pricing is cancelled/invalid. Remaining geometry has no predicted speed or arrival time. |
| Nominal model-feasible candidate | Complete point-mass pricing passes the implemented force, energy, rate, terrain/profile, touchdown, and braking constraints. This is necessary evidence for admission, not a live-flight certificate. |
| Admitted control reference | The nominal candidate also satisfies the current-state/epoch/inlet/authority/operating contract. A known failed nominal closed-loop case blocks admission; missing closed-loop qualification remains visible. |
| Qualified flight | Requires future actual execution and observations. Nothing in this investigation establishes it. |

The configured aerodynamic uncertainty remains a sensitivity allowance, not a measured confidence interval. Nominal and stressed assessments remain distinct; stress failures are reported and are not silently described as robust feasibility.

## 6. Source corrections and justification

### 6.1 MM304: a shared turning and energy decision

Files: `EntryCommandLaw.swift`, `EntryLateralController.swift`, `EntryGuidanceEngine.swift`, `EntryCaptureForecaster.swift`, and `EntryDiagnostics.swift`.

The pre-change working-tree entry law selected an AoA primarily to support its vertical demand and match drag. Lateral steering then used the lift left by that choice. The escalating demand/delivered-acceleration separation in E3/E4 is consistent with this structural mismatch.

The revised law evaluates its existing AoA choices against one conservative lift budget and drag-work budget:

1. Apply the existing trim, AoA, load, and uncertainty limits.
2. Identify choices that can provide the requested upward component.
3. For energy-affordable supporting choices, prefer smaller unmet lateral demand, then drag-reference agreement.
4. If the energy budget has no supporting choice, use a locally maneuver-efficient recovery choice when turning is needed; otherwise use minimum-drag support. If no choice supports the vertical demand, retain bounded lift recovery.
5. Command bank from the **measured current lift**, using the inverse of L sin(bank), while reserving vertical support. The law does not spend lift that the requested AoA has not yet produced.

This preserves the distinction between entry's fixed-current-lift bank inversion and MM305's joint AoA/bank allocation. It avoids incorrectly using a bank from a freely adjustable lift vector while leaving entry's actual lift unchanged.

Fresh terminal geometry supplies prospective lateral steering and the remaining return/HAC/final distance. When that advice is used, the energy budget is evaluated against the configured touchdown state and the complete remaining route, rather than a 25 km ASL point that may already be abeam. The entry altitude/flight-path objective and its bounded command shaping remain separate.

Entry forecast ranking now prefers a feasible branch before comparing scalar costs. A large finite/infinite heading penalty cannot accidentally make an infeasible branch beat a feasible one merely through an arbitrary additive penalty.

New diagnostics expose requested/available lateral acceleration, the local heading-turn time estimate, terminal advice age and use, energy-budget endpoint, allocation mode, and terminal deficit. A force-limited command is labeled as a force deficit rather than simply “tracking.” The legacy `bestRangeRecovery` flag is retained for compatibility; `energyAllocationMode` distinguishes the new recovery objective.

### 6.2 Exploration, feasibility, and ownership

Files: new `HAC/TerminalExploration.swift`, `LandingMissionController.swift`, `MM305Admission.swift`, and `TAEMCaptureEvaluator.swift`.

Exploration no longer requires satisfying MM305's altitude/Mach ownership conditions. The existing terminal survey region remains a baseline, and a local force-derived runway-heading turn time can advance exploration within the configured maximum path horizon. Live and forecast paths use the same scheduling helper. The estimate only schedules investigation; it does not establish reachability.

The configured five-kilometre centerline objective remains an entry delivery objective. It is not used as permission to investigate a return HAC or as a declaration that an abeam vehicle is beyond all terminal options.

The mission now holds an advisory plan separately from an admitted active plan. Rejected candidates feed MM304 their geometry and classified deficit without acquiring control. Admission requires:

- Complete, nominally feasible pricing, rather than the existence of a selected candidate.
- A current, non-future planning epoch, bounded by the existing replan-age policy.
- Measured inlet position and course compatible with the age-adjusted reference and existing handoff tolerances.
- Current inlet force within available lift and the configured bank/load envelope.
- Existing altitude/Mach/q/safe-speed/sideslip/bank limits and usable roll/pitch authority evidence.
- No recorded failed nominal closed-loop assessment for that candidate.

An initial opening range or being inside the entry aim radius is no longer an automatic veto on a feasible return HAC. These are geometrical circumstances, not physical impossibility proofs.

The direct backend MM305-engage path also computes a fresh inlet lift bound before admission. Normal transfer retains the target limiter and flight controller history. Rejection leaves MM304 in control; rejection of a replacement while already in MM305 retains the previous route with bounded tracking/recovery. The existing near-ground best-effort approach fallback remains. No inactive/no-command state was introduced.

### 6.3 HAC geometry and honest radius

Files: `HACPlanningEngine.swift`, `DubinsAcquisition.swift`, `HACFitContext.swift`, and `TerminalPlan.swift`.

The required instantaneous radius is no longer clipped to `maximumHACRadius`. The latter limits the searched final HAC radius; it is not an aircraft-capability limit. Diagnostics now separate the required radius, search-ceiling exceedance, actual acquisition distance, and roll-response distance. When positive lateral authority is unavailable, the geometry search hint is explicitly not represented as a measured finite required radius.

The acquisition inlet uses the corrected chart velocity. Rejoin segments remain acquisition segments even when continuing an existing spiral. Spiral establishment and continued winding constraints require measured proximity and heading agreement; a segment label or an old establishment latch alone is insufficient. A genuine tracking loss allows a fresh acquisition, while an on-route unload preserves the remaining winding.

The normal vertical join is now a quintic connection matching height, slope, and curvature at the measured inlet and final-profile exit. This removes the unconstrained inlet second derivative of the former position/slope-only cubic. The q-scheduled alternative remains a separately priced candidate. Neither geometry construction grants feasibility.

Dubins roll smoothing formerly shifted positions to close the endpoint without updating the corresponding tangent and curvature. The closure correction is now a C2 polynomial whose derivatives update those fields. Corrected sampled length is checked against the path horizon. Candidates with non-finite curvature or degenerate directions are rejected before pricing; excessively long Dubins paths are rejected before large sample allocations.

### 6.4 Force, energy, and rate pricing

Files: `EnergyManagementEngine.swift`, new `Energy/NormalForceEnvelope.swift`, and `TerminalPlan.swift`.

`NormalForceEnvelope` is shared force-allocation mathematics. It is not a second aerodynamic model: all lift bounds still come from the existing aerodynamic engine.

Pricing now uses the consistent normal equation (including V² rather than Vh² in the vertical spherical-curvature term), preserves the sign of upward normal demand, and rejects a negative upward component outside the configured positive-lift/bank envelope. The q-scheduled candidate's push/pull authority spends the same lift vector as its lateral demand. Load limits constrain available force before turn sizing.

Nominal/advisory geometry is fully priced on a consistent basis while positive kinetic energy remains. A failed force constraint can remain visible in a fully priced advisory, but it cannot make that advisory feasible.

**The speed-floor continuation is removed.** Non-positive/non-finite kinetic energy stops pricing, including advisory pricing. The result records `predictionComplete`, `predictedThroughDistance`, `energyExhaustionDistance`, and `unpricedDistance`. Exhaustion distance identifies the last priced station before the failing interval; it is not a numerically located stop point. A numeric zero touchdown field on an incomplete legacy-compatible assessment is a sentinel, not a predicted landing speed.

Unpriced tail geometry is retained with speed zero so bounded recovery can still use its shape. It is excluded from predicted touchdown extrema, energy exchange, and arrival-time visualization. Fully priced candidates are distinguished from incomplete ones during ranking; feasible candidates retain priority.

Bank-rate pricing uses the traversed interval, and the first interval starts from measured bank/AoA rather than assuming an instantaneous first-reference substitution. AoA change is also checked against the existing guidance slew envelope. Gear state follows the existing deployment altitude/latching policy consistently for lift and drag instead of using contradictory assumptions at the same station.

### 6.5 Tracking, final capture, and lifecycle

Files: `PathTrackingEngine.swift`, `LandingMissionController.swift`, `TerminalDiagnostics.swift`.

Tracking walks contiguous path segments and interpolates the current segment. It no longer chooses a later nearby station merely because the path winds close to itself. Tangent comparison uses the vehicle's tangent plane. Segment interpolation retains the segment that actually contains the interpolated distance.

MM305 allocates AoA and bank together under the shared positive-lift force cone and configured load/bank limits. A physically feasible priced reference may unload within that cone. Infeasible/unpriced geometry retains support-first recovery; a negative upward demand cannot be implemented as nearly zero lift plus a large bank and then credited with the desired turn.

Final ownership now requires `ApproachGuidanceEngine.canCapture(state)`, matching the existing closed-loop evaluator's gate. Reaching a station labeled final does not bypass measured runway alignment or energy/capture checks.

Dynamics invalidation retains geometric progress and limiter history while marking the active speed/energy prediction stale. Persistent tracking error requests a fresh acquisition rather than resetting progress to the beginning of an old winding. Gear remains deployed once already down.

The immutable reference visualization is cached once per plan rather than rebuilt at every control tick. Its time integration follows the same preceding-station interval convention as pricing. Diagnostics distinguish an active reference, a new advisory, prediction coverage, current dynamics, requested lateral acceleration, and allocated modeled acceleration. None of those allocation fields is labeled as a measured aircraft response.

### 6.6 Predicted attitude authority and handover state

Files: `ShuttlePhysicsEngine.swift`, `RotationalDynamicsEngine.swift`, `TrajectoryPropagationEngine.swift`, and the forecaster/evaluator.

The existing seeded prediction could take the maximum of measured total moment authority and a larger provisional q-only prior, then label the result identified. E3's representative pitch values are about 0.079 versus 0.899 rad/s² around +400 s and 0.150 versus 0.991 rad/s² near transfer. A model that grants that excess authority cannot establish live tracking capability.

The correction preserves the valid measured/identified seed torque and inertia as a **local held estimate**, explicitly labeling its q evolution unvalidated. It does not add an RCS prior on top of total measured actuator authority. An unanchored simulation still uses its explicit provisional plant. A measured refresh can update the anchor; a propagated identified seed survives the next predicted phase instead of reverting to a larger q-only prior.

Rigid-body coupling uses valid telemetry inertia rather than silently reverting to nominal vehicle inertia. An identified trim ceiling takes precedence over the configured RCS-availability flag. Existing RCS configuration and all actuator limits remain intact.

The MM304-to-MM305 forecast previously supplied the original entry controller snapshot at the later predicted terminal state. Propagation now exposes its terminal controller/limited-target snapshot through an optional handoff callback, and the forecaster passes that state into requested terminal qualification. Forecast terminal surveys also receive the same roll/pitch-response and identified-trim context as the live planning path.

A requested closed-loop certification that returns a failed nominal case now remains a failed certification/feasibility result. The added paths were inspected as source only; no such prediction or certification was run here.

### 6.7 Default backend and interfaces

Files: `ksl/config.py`, `ksl/programs.py`, and `KSLLanding/BackendFlightActions.swift`.

Static inspection found a legacy `landing.engine = "c"` default in the Python configuration. It now defaults to the existing `swift-backend` NDJSON adapter so normal live landing uses Swift without an opt-in flag. Descriptive output and missing-key fallback use the same selection. Explicit legacy/direct-Swift selections and the existing `KSL_BACKEND` override remain usable. ShuttleSim scenario/protocol paths were not changed.

New Swift arguments are optional with defaults; existing call signatures remain usable. New recorded-data fields are optional for older archives. No polar resources, external protocol version, vehicle limit, controller gain, or unrelated ascent/deorbit algorithm was changed.

## 7. Why this addresses the earliest divergence

The original sequence let MM304 carry a growing crossrange/course problem toward a point, discover the cost of the terminal turn late, and then transfer responsibility despite an inadmissible result. Fixing only MM305's local bank controller would leave that upstream delivery mechanism intact.

The corrected sequence supplies the entry law with a prospective runway-facing path before terminal ownership is allowed. Entry uses its remaining energy for the whole maneuver, chooses AoA with the required lateral force in view, and exposes unmet demand. The planner cannot shrink the aircraft's true turn radius to fit its search box or replenish exhausted speed. The mission cannot promote a rejected advisory into a feasible control reference. The tracker cannot claim a turn from bank angle while removing the required lift.

These are mechanistic reasons to expect improvement. They are not evidence that the new local policy finds a viable path from the cited checkpoints. In particular, the available source and records do not establish that the original arrival state has any successful runway-capture solution, or that the revised entry policy preserves one from 72 km.

## 8. Static review performed

The review included source/call-site tracing, read-only recorded-data extraction and arithmetic, inspection of original working-tree snapshots, mathematical differentiation of the chart and quintic closures, review of Codable/default-argument compatibility, and `git diff --check`.

The whitespace/diff check reported no errors. This is only a static formatting check. No Swift or Python project entry point, compiler, package manager, test runner, model executor, replay command, or flight endpoint was invoked. In particular, the CLI itself was not run even for help because its wrappers can initiate build work.

The complete production stack was not compiler-checked. The new source therefore still requires future compilation and executable validation under separate authorization. Existing simulation failures and non-monotonic step-refinement results in E2 were read as historical evidence and were not rerun.

## 9. Aerospace methods used and their limits

1. **NASA TM-104744, “Space Shuttle Entry Terminal Area Energy Management,” Thomas E. Moore, 1991.** [NTRS record](https://ntrs.nasa.gov/citations/19920010688) · [PDF](https://ntrs.nasa.gov/api/citations/19920010688/downloads/19920010688.pdf). Sections 1, 2.1, 2.2, 2.4, and 3 discuss the distinction between entry and TAEM, reference altitude with energy/q constraints, large-turn HAC energy preparation, and coordinated normal-acceleration/bank limits. The concrete lesson applied here is to budget vertical and lateral force together and prepare the energy state for a required terminal turn. Earth-Shuttle speeds, bank schedules, gains, and dimensional constants were not transplanted into KSL.

2. **NASA-CR-151480, “Optimization of Space Shuttle Entry Guidance Lateral Deadband, Minimum Bank Angle Logic,” 1977.** [NTRS record](https://ntrs.nasa.gov/citations/19770022264) · [PDF](https://ntrs.nasa.gov/api/citations/19770022264/downloads/19770022264.pdf). This supports treating reversal/deadband policy as coupled to lift/drag and crossrange authority, rather than treating the number of bank reversals as an independent success objective. Zero reversals alone does not prove a sign bug; the relevant evidence is residual position/course error and the integrated attainable lateral acceleration.

3. **“Space Shuttle GN&C Development History and Evolution,” 2011.** [NTRS record](https://ntrs.nasa.gov/citations/20110014833). Its discussion of optional TAEM targeting and heading-alignment cones supports retaining spiral/cone and return-approach options rather than forcing a fixed-radius circle or a straight-in arrival from every state.

A bounded receding-horizon family of existing HAC/acquisition geometries with explicit force/energy admission is the simplest defensible improvement in this architecture. A new global optimal-control solver would still depend on the unqualified moment/trim model and would require substantial execution to establish correctness. It would not resolve the demonstrated ownership, energy-floor, or force-allocation defects by itself.

## 10. Remaining limitations and competing explanations

- **No global reachability proof.** Exhausting this candidate family is not proof that the physical reachable set is empty. Radius/growth/angle sampling and vertical-profile choices remain limited.
- **No demonstrated successful corrected trajectory.** All new behavior is unexecuted, including compile-time correctness, control timing, and planning latency.
- **Reduced-model geometry and energy.** Pricing uses sampled chart distances/curvatures and a still-air spherical point-mass approximation. It does not exactly reproduce chart-to-surface arc-length/curvature metrics, all rotating-frame force/energy terms, wind gradients, or six-DOF tracking between stations. Continuous extrema and mesh sensitivity remain unqualified. The full propagation model retains the existing rotating-frame physics.
- **Local moment hold is not identification over the trajectory.** Measured total torque includes the enabled actuator set. Holding it through changing q, Mach, fuel, RCS state, or payload configuration is an explicitly unvalidated extrapolation. The correction removes demonstrated upward inflation; it does not invent a q-dependent decomposition of surfaces, wheels, and RCS.
- **Aerodynamic uncertainty remains provisional.** The bundled table has 294 cells and only 53 observed cells in its supplied metadata. The recorded local force agreement does not validate all AoA/Mach/q combinations used by a future route. History-based estimates and trim ceilings can change the local envelope.
- **Finite-rate feasibility is incomplete.** Slew and roll-transition checks are necessary constraints. They do not establish full pitch/roll/yaw tracking, actuator saturation recovery, or a continuous force-feasible transient at every point.
- **Late roll divergence remains unresolved.** The approximately 147.5-degree bank excursion at a much smaller target needs separate coupled-attitude/authority/model investigation. No gain increase or unsupported axis-sign change was introduced.
- **Fallback is best effort.** If no affordable supporting/turning choice exists, the entry recovery policy prioritizes bounded force and locally efficient maneuvering. It is not a solved optimal emergency trajectory. A rejected terminal replacement preserves control but does not preserve physical reachability indefinitely.
- **The entry objective remains an objective.** The nominal capture height and five-kilometre centerline point are not themselves a terminal admissibility certificate. Advice changes steering and the energy budget; a difficult late state may still be unable to meet the retained vertical objective.
- **Classification is diagnostic.** Dominant violation counts and deficit labels describe the searched family. They do not uniquely establish a global root cause for each state, particularly when no candidate can be examined or several constraints fail together.
- **Historical numerical instability is unresolved.** E2 reports 14 failed assertions in four simulation tests and non-monotonic timestep effects. Reading those results cannot qualify a new model.
- **Upstream orbital issues remain separate.** Recorded RCS exhaustion, OMS/reconnect behavior, and orbital attitude problems can prevent reaching entry. They were not used to explain away the independent 72 km entry-checkpoint failures and were not modified here.

## 11. Future validation specification — not executed

The following are concrete acceptance tasks for a later authorized validation effort. They are specifications, not a request to execute them now.

### A. Source and interface checks

1. Compile the Swift package and check the Python configuration/program changes. Confirm the optional new fields decode older archives without reinterpreting zero/unpriced speed as a valid arrival.
2. Confirm default KSL live landing selects the Swift NDJSON backend with no opt-in flags; explicit C/direct-Swift selection and `KSL_BACKEND` still work. Preserve ShuttleSim stream and scenario interfaces.
3. Confirm existing dynamics-revision rejection, payload adaptation, telemetry-gap handling, limiter history, RCS settings, and direct-control lifecycle behavior remain intact.

### B. Geometry and physical invariants

4. Check the analytic chart differential/inverse against position perturbations at both thresholds, reciprocal runways, high altitude, large along/cross offsets, and the origin. Verify signed cross-rate and tangent transport.
5. Check quintic acquisition/vertical endpoints for position, tangent/slope, and curvature continuity. Verify reported corrected length and actual acquisition distance, including repeated spiral windings.
6. Exercise the force cone below/at/above vertical support, near the bank/load limits, and with negative requested upward force. Entry's fixed-lift bank must produce the bounded requested lateral component; terminal joint allocation must satisfy its own lift vector.
7. Construct energy-exhausting path inputs. Pricing must stop at the failing interval, report incomplete coverage, exclude the tail from touchdown extrema/times, and leave only advisory geometry. Reducing spacing must not create energy by a floor.
8. Check current-state inlet admission for stale time, displacement, heading mismatch, missing authority, insufficient lift, excessive load/bank/q, known failed nominal tracking, and an opening-range but genuinely admissible return HAC.
9. Check gear-state, trim-ceiling, mass, inertia, and measured/identified authority consistency through the same predicted handover. An entry seed's provisional maximum must never overwrite a smaller measured total authority.

### C. Model qualification before flight interpretation

10. Re-evaluate the archived states at identical configuration, source revision, and controller snapshot. Label all outputs numerical predictions. Compare point-mass force/energy requirements with the full plant without accepting a nominal “completed” path as landing evidence.
11. Perform mesh/control-step refinement with a stated convergence criterion and identical initial state. Investigate any non-monotonic change before using a predicted feasible margin for operational conclusions.
12. Compare measured and predicted moment/trim response across the actual q/Mach/RCS range. Separate total actuator authority from components; validate or replace the local-hold assumption with evidence.
13. Check planning cadence/cancellation and memory allocation under the real work budget. Earlier exploration must not starve the urgent control loop or continually discard entry planning.

### D. Observations that would confirm or falsify this diagnosis

A later controlled comparison should keep the same initial checkpoint, runway, payload, aero history exclusions, controller limits, and recorded configuration.

**Confirmation would require more than a smaller miss distance:**

- Terminal advice becomes available while MM304 still owns control and before the large heading/crossrange deficit consumes the maneuver opportunity.
- Demand and delivered lateral acceleration remain consistent with logged lift and bank; increasing bank while AoA/lift collapses is no longer credited as successful turning.
- The full runway-facing state improves: crossrange, course, energy-to-go, and remaining turn must be assessed together.
- No ownership change occurs for an incomplete or nominally rejected path. A direct/automatic MM305 start uses the same current-state contract.
- Final ownership occurs only inside measured alignment and energy/capture conditions.
- Prediction residuals, actual actuator saturation, and late roll excursions are recorded explicitly.
- A landing claim requires verified runway wheel contact, acceptable touchdown state, rollout, and stop.

**The diagnosis would be weakened or falsified in specific ways if:**

- Correctly signed, force-feasible commands are delivered with adequate modeled energy, but measured forces consistently have the opposite lateral sign: revisit frame/control mapping.
- Measured lift and bank deliver the expected acceleration, yet runway-relative error evolves inconsistently with the coordinate differential: revisit navigation/geometry.
- Nominally feasible references repeatedly demand more force or work than the measured vehicle can provide: revise the aerodynamic/trim/authority envelope rather than increase gains.
- The new advice arrives early and is tracked within force/energy limits, but every admissible terminal route still disappears: the upstream entry/deorbit target or the candidate family is inadequate; earlier advice alone is insufficient.
- Large attitude departures persist while following a complete feasible reference with modest commands: the remaining root cause is in coupled control/plant execution and must be investigated independently.

## 12. Engineering conclusion

The strongest supported diagnosis is **insufficient upstream terminal preparation followed by acceptance and execution of a physically rejected reference**. The observed low turning acceleration at large bank is quantitatively explained by low delivered lift, and the archived references show how guidance could command that combination.

The repository now expresses the necessary distinctions: a geometrically closed path, a fully priced path, an admissible nominal reference, and a vehicle that actually tracks and lands are different results. The changes remove demonstrated sources of false feasibility and give MM304 earlier, force- and energy-aware information about the terminal maneuver. Whether those corrections produce runway capture remains an explicit future validation question.


## 13. Follow-up: offset ground track after the first source correction

### 13.1 New evidence and its limits

The user supplied a ground-track image and asked for the remaining cause to be corrected without time-consuming tests. The original stronger prohibition on executing tests, builds, simulations, replays, flights, KSP, or kRPC remains in effect. None was executed in this follow-up. The image has no telemetry timestamp or verifiable coordinate transform; its drawn distances were not treated as measurements.

A subsequently recorded flight exists in [runs/live-runway-capture-20261031-1241](../../runs/live-runway-capture-20261031-1241/). Its archived source patch exactly matched the working-tree diff at the beginning of this follow-up. The real Swift source members of its untracked-source archive also matched; macOS AppleDouble metadata members were not mistaken for source files. This is evidence for analyzing the first correction's remaining defects, rather than assuming that another old binary was flown.

Provenance:

- Archived HEAD: 678f653e6f19dd44b4893e5da29ee54f7cd1b903.
- Archived source patch and initial working diff SHA-256: fd911223eceaa6fb7a2e2bf3e3487e13f248797a1cd204f92771f8424019dd97.
- Archived executable SHA-256: ce64cb697b6f139b4fe134c869c5707e702b4cf0fef25834c4f532d4ee271c2f.
- Archived configuration SHA-256: 7d562b97f847432c84b3c911a22cab7b0315cd314d445579c62edb357f913a45.
- Read raw-log SHA-256: 050566efe2c0bc4b4ab114b715922f9e3f28a2bd965e8864313afe7f3e277a1f.
- Examined record: 3,903 rows, UT 195998.30879777222 through 196296.54879761598, lasting 298.24 simulation seconds.

Every row in this record is MM304/entry. It stops at approximately 51.94 km ASL and 260.7 km ground range from the selected threshold, before an observed MM305 transfer or runway passage. It does **not** document a new terminal impact or prove that its depicted eventual path was actually flown. The older flights in sections 4–5 remain separate evidence for terminal failure.

Selected measured/derived quantities:

| Raw line / elapsed | ASL m | Airspeed m/s | Along m | Cross m | Chart cross-rate m/s | Absolute runway-course error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 / 0.00 s | 71013.20 | 2097.84 | 827784.21 | -13279.38 | +267.98 | 174.16 deg |
| 774 / 60.02 s | 66938.89 | 2112.65 | 715101.14 | +1600.60 | +228.84 | 174.61 deg |
| 1517 / 120.00 s | 62992.77 | 2121.16 | 600963.13 | +14246.32 | +193.05 | 175.15 deg |
| 2325 / 179.98 s | 59239.71 | 2122.98 | 485687.19 | +24760.95 | +157.02 | 175.83 deg |
| 3099 / 239.98 s | 55625.82 | 2109.88 | 369888.20 | +33004.89 | +117.03 | 176.74 deg |
| 3903 / 298.24 s | 51941.72 | 2080.46 | 257812.16 | +38573.09 | +72.31 | 177.90 deg |

Coordinates and coordinate rates were derived from recorded position/velocity by the analytic RunwayFrame chart and its differential. No state was propagated. The cross-rate decreases, but remains positive: the vehicle is still moving farther north of runway 27. Its course approaches the reciprocal runway direction. Reduction of cross-rate alone is therefore not terminal capture.

At the last row, the limited bank target is 70.00 degrees and actual bank is 70.146 degrees. Measured lift/mass is 0.544654 m/s². Its rightward component is:

    0.544654 * sin(70.146 deg) = 0.512279 m/s²

Direct projection of the recorded aerodynamic force gives 0.512300 m/s². The logged lateral authority is 0.511807 m/s², while requested lateral acceleration is 16.75197 m/s². This is a roughly 32.7-fold demand/authority mismatch. At horizontal speed 2079.37 m/s, the measured aerodynamic component corresponds to a local curvature radius of about 8.44 million metres and a heading rate of about 0.0141 degrees/s. These frozen local quantities are not a future trajectory or a maximum-performance measurement. Atmospheric density, speed, and trim change during descent.

The bank controller is following its target in this interval. Increasing bank-loop gains would not create the missing aerodynamic force.

### 13.2 Confirmed upstream disconnection: range correction did not reach AoA selection

EntryGuidanceEngine.plan explores range multipliers by changing the drag reference consumed by EntryCommandLaw. However, the first correction's affordable-AoA comparator minimized unmet lateral acceleration **lexicographically before** considering drag-reference error. Whenever the requested turn exceeded the capacity of every affordable AoA, this selected the largest available lateral force regardless of the drag request. The drag reference only decided exact ties.

Consequently, several receding-horizon branches could fly the same commands even though their range multipliers differed. A stronger demand for deceleration could be ignored in favor of a slightly larger instantaneous turn force. This is particularly inappropriate when reducing speed is necessary to make the later HAC turn affordable. It was a remaining defect in the previous source correction, not a newly discovered sign reversal.

The last row records reference drag acceleration 2.49240 m/s², measured drag acceleration 1.10794 m/s², and zero requested upward normal lift. The discrepancy is not by itself proof that the full drag reference was achievable. The unchanged polar illustrates the competing choices:

| AoA | Table CL | Table CD | Static table L/m, m/s² | Static table D/m, m/s² |
| ---: | ---: | ---: | ---: | ---: |
| 20 deg | 0.127026 | 0.234596 | 0.431239 | 0.796426 |
| 27.5 deg | 0.149601 | 0.315422 | 0.507879 | 1.070823 |
| 30 deg | 0.157435 | 0.363821 | 0.534474 | 1.235132 |
| 35 deg | 0.144079 | 0.498802 | 0.489131 | 1.693375 |
| 40 deg | 0.126436 | 0.690137 | 0.429236 | 2.342936 |

These are scalar interpolations of the existing STSAerodynamics.csv between Mach 6.60386666 and 7.11385250, at recorded Mach 6.72078026, q 347.80283 Pa, mass 25612.26758 kg, and the table's 250 m² reference area. They are **not** measured performance at those AoAs, a new model, a trim certificate, or an executed guidance decision. Measured AoA at the sample is 27.00 degrees; the limited target is 27.79 degrees. Local measured/model adaptation, uncertainty, and trim remain in the actual guidance. The table shows why maximizing instantaneous lift and demanding useful drag are different objectives.

**Correction:** EntryCommandLaw.update retains the existing polar samples, identified trim ceiling, load bound, upward-force reservation, uncertainty treatment, and remaining-energy budget. Within the affordable set it bounds the requested lateral component by the capacity of that same set, then minimizes the Euclidean specific-force residual:

    A_target = min(abs(raw lateral demand), maximum affordable lateral authority)
    a_i      = min(A_target, lateral authority of candidate i)
    b_i      = sqrt(max(0, L_i² - a_i²))
    J_i²     = (D_i - D_reference)² + (b_i - B_requested)² + (a_i - A_target)²

Here L, D, a, and b are accelerations, not forces in newtons. This is a constrained projection onto the existing polar's achievable force vectors. No arbitrary axis gain, aerodynamic coefficient, or relaxed limit was introduced. An unachievable raw lateral demand no longer overwhelms every range correction. The existing turn-energy recovery remains when no supporting choice is affordable, and the existing maximum-support fallback remains when upward support itself is unavailable. The actual bank command still uses measured current lift; future selected AoA is not credited as force already delivered.

The new optional diagnostics lateralAllocationTarget, selectedDragAcceleration, and dragTrackingResidual expose the modeled allocation separately from the raw request and measured aerodynamic forces.

### 13.3 Confirmed numerical failure: candidate scores could become infinite

All 52 recorded entry proposals selected range multiplier 0.8; none reported a feasible terminal route. Of those 52 selected forecasts, 26 contain a zero availableTurnRate for which the old heading-cost expression overflows:

    turningDistance = V * requiredHeading / max(Double.leastNormalMagnitude, turnRate)
    headingCost    = turningDistance / runwayRange

Substituting the smallest normal floating-point number is not a physical turning rate. At ordinary flight speeds and nonzero heading change it produces infinity. When otherwise different infeasible branches receive infinity, score(lhs) < score(rhs) cannot order them; enumeration order can decide the winner. The archive retains only the selected branch, so it cannot establish that this was the sole reason for every one of the 52 selections. The source defect and the 26 nonfinite selected scores are established independently.

**Correction:** the ranking now evaluates the uncompleted heading change over the local approach time:

    T_local      = runwayRange / max(predictedSpeed, configuredMinimumSafeSpeed)
    headingCost  = max(0, requiredHeading - max(0, turnRate) * T_local) / headingHalfWidth

If a separate minimum-safe speed is absent, the existing configured landing speed is used. Configuration validation already makes that speed and the heading half-width positive. For finite valid inputs the heading term lies between zero and pi/headingHalfWidth; zero turn authority produces a finite deficit instead of an overflow. Range, altitude, energy, and violation terms can still discriminate between unsuccessful branches. Feasible-first ranking is retained. This is explicitly a local ranking term, not a global reachability certificate.

No newly selected forecast or flight was executed to assess the correction.

### 13.4 Exploration timing and coverage were still conflated

The first correction computed a maneuver-based exploration lead but clipped it to maximumPathLength, 220 km in the archived configuration. At the new record's end, the vehicle was about 260.7 km from the threshold, so the live terminal search had not yet published a result despite a very large local heading-turn deficit. The entry predictor had already explored future terminal states; live current-state exploration and forecast future-state exploration are distinct.

**Corrections:**

- TerminalExploration.searchRange now limits the extrapolated **time** by the existing maximumPredictionDuration, and does not use the complete-candidate path-length ceiling to prohibit earlier advice.
- LandingMissionController separates a survey being due from permission to cancel an active entry forecast. Distant advice uses an available worker; within the terminal geometric horizon, an urgent survey can still preempt entry work. Early exploration does not transfer control.
- HACPlanningEngine checks a rigorous lower bound in its own chart metric: every sampled path from the current point to touchdown is at least the straight-line distance between them. If this already exceeds maximumPathLength, it returns search-coverage information without constructing a full impossible-within-that-budget HAC family. The path-length limit itself is unchanged.
- TAEMCaptureEvaluator no longer calls an entirely unpriced/empty candidate family “insufficient energy” or “adverse heading” merely from heading. No evaluated candidate is not proof of global physical unreachability.
- Optional search-horizon and authority-assessed diagnostics distinguish “not assessed within this search” from an actual assessed zero turn rate.
- EntryCaptureForecaster uses the same early exploration condition without its former extra descending-only gate. Prospective search on a rising portion of entry does not imply MM305 admission.

A horizon-only response supplies a useful limitation, not a terminal trajectory. MM304's forecast still has to find and evaluate future terminal states, and the actual complete-path/force/energy admission contract still has to pass. This change does not claim that a physically feasible HAC was found at 260 km or at the atmospheric interface.

### 13.5 Static convergence defects beyond the recorded interval

The new record contains no MM305 execution. The following defects were demonstrated in the source and corrected, but their contribution to that record cannot be measured.

**Perpendicular-heading attenuation and reciprocal false equilibrium.** The fallback entry law blended cubic crossrange interception into bearing pursuit as runway alignment approached zero. Both terms nevertheless used the along-track closure time. Near perpendicular flight, that time grew toward alongDistance / 1 m/s, suppressing bearing pursuit exactly where it was supposed to take over. It now uses total horizontal speed and full chart distance for the pursuit term, while preserving the existing signed cubic projection when applicable.

When the point target was behind the vehicle, cubic interception could request zero on a reciprocal centerline departure, and a sine-only bearing term also vanished at 180 degrees. Rear-hemisphere recovery now uses a bounded heading-capture sine with the bearing angle limited to ±90 degrees before evaluating the sine. The exact reciprocal numerical ambiguity retains the current turn direction. Force and bank limits still bound the response. This prevents a zero-command equilibrium from being mistaken for convergence; it does not make a return trajectory feasible by itself.

**Regenerated HAC inlets needed anticipatory tracking.** The corrected C2 acquisition geometry matches measured inlet position, tangent, and curvature. The former tracker used only the projected current station: at an exactly matching inlet its feedback errors were zero and its feedforward reproduced current curvature. Replanning from measured state could repeatedly erase the small accumulated lead toward the intended later bend.

PathTrackingEngine now uses the forward-point nonlinear guidance relation described by Park, Deyst, and How, also presented in [MIT 16.333 Lecture 12, pp. 10-17 onward](https://ocw.mit.edu/courses/16-333-aircraft-stability-and-control-fall-2004/03fb92311b62c652e222e14efb49573b_lecture_12.pdf):

    a_lateral = 2 V_horizontal² sin(eta) / L1

The existing response/lookahead times determine how far along the current winding to look. Forward traversal is contiguous and stops before accumulating more than a quarter-turn of tangent change, so a long response distance cannot point through a complete loop of an advisory. The reference point is transformed into the vehicle's actual tangent plane; chart distance is not mistaken for physical velocity. A one-control-interval distance is used only as a numerical endpoint denominator guard. The nearest reference still owns progress, height, phase checks, and final-admission checks.

The preview already supplies curvature, so the old separate V² * curvature term is not added again. In the local planar circle case, L1 = 2 R sin(phi/2) and eta = phi/2, which recovers a = V²/R. For a straight route it supplies bearing/cross-track correction. These are mathematical identities, not executed tests. The rear-hemisphere capture handling also prevents loss of turn demand at reciprocal heading.

The same tracker serves MM304 terminal advice, MM305 control, and the terminal prediction evaluator. New lookahead-distance and lookahead-heading diagnostics make it possible to distinguish nearest-station agreement from a required upcoming turn. Existing aerodynamic allocation, trim, vertical recovery, target limiting, and ownership gates remain downstream.

The use of an achievable force set is consistent with established constrained control-allocation practice; [NASA's X-33 allocation work](https://ntrs.nasa.gov/citations/20000097370) is methodological context, not certification of this implementation. [NASA TM-104744](https://ntrs.nasa.gov/api/citations/19920010688/downloads/19920010688.pdf), particularly its entry/TAEM distinction and large-turn energy-management discussion, supports treating deceleration and turn preparation together. No Earth Shuttle numerical gains or operating thresholds were copied into KSL.

### 13.6 Scope, preservation, and remaining verification

This follow-up changes eleven source files: EntryCommandLaw, EntryGuidanceEngine, EntryLateralController, EntryCaptureForecaster, PathTrackingEngine, TerminalExploration, HACPlanningEngine, TAEMCaptureEvaluator, LandingMissionController, and the two entry/terminal diagnostic models. It updates this report. No aerodynamic tables, atmosphere, force integrators, attitude-control gains, vehicle limits, CLI protocol, backend default, or archived run inputs were changed.

The complete initial working diff, status, and file-hash manifest were preserved under /var/folders/94/4dkhkjbj2c1515zym0htkt9w0000gp/T/ksl-runway-capture-followup-o0isfpqg/; first working copies of edited files were retained in its originals subdirectory. Changes were made against the existing dirty working tree. No commit was made.

Only source/call-site inspection, difference inspection, whitespace inspection, existing-record decoding, recorded-state coordinate/force arithmetic, and scalar interpolation of the existing polar were performed. No updated guidance code or physical propagation was executed. Compiler acceptance, runtime costs, closed-loop stability, and actual runway capture remain unverified.

The important remaining uncertainties are whether the initial deorbit/entry state leaves enough integrated turn and drag opportunity, whether high-AoA pitch trim and the provisional polar remain accurate in this regime, whether candidate geometry covers a useful complete route, and whether the already observed later attitude excursions require a separate FCS/plant correction. The frozen force deficit cannot prove that every future descent/turn is impossible, and the source corrections cannot prove a landing.

Future validation, under a separate instruction that permits execution, should specifically establish:

1. The selected AoA and modeled drag now respond to differing range/energy requests while the lateral request is saturated; any residual drag/turn deficit remains visible rather than being hidden.
2. Zero-turn-authority and empty-search cases have finite, discriminating ranking and truthful coverage diagnostics; a different range multiplier is selected only when its projected state warrants it.
3. Early surveys do not starve entry forecasts or grant premature ownership, and a coverage-only response is not presented as a failed aerodynamic maneuver.
4. A regenerated on-track C2 inlet produces the appropriate preview command for a downstream bend, without adding curvature twice or skipping a later winding; reciprocal and perpendicular capture retain corrective authority.
5. Measured AoA, lift, drag, heading change, and energy dissipation follow the newly selected force vector within identified uncertainty; the complete runway-facing terminal state improves, not merely cross-rate.
6. The existing MM305 admission and final-capture gates still reject incomplete, stale, or force/energy-infeasible references. Only an actual runway touchdown and stopped rollout can establish landing success.
