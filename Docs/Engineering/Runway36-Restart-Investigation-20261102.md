# Runway 36 restart investigation — 2026-11-02

The restart failed through a chain of unresolved guidance problems: MM304 delivered the shuttle beyond the threshold with insufficient turning support, then TAEM committed an immediately unachievable acquisition path and never joined its spiral. The planner predicted failure before flight. Neither burn execution nor the repaired missing-NEP issue explains this result.

This is an investigation of `runs/20261102T122809Z-dessert-normal-oms-restart`, flown with source `3d7aab71e6ed6e45b068640ff408177503e0743b`. No production guidance was changed or additional live attempt initiated. The controller remains stopped. Original save files were not written.

## Evidence and reproduction

The frozen binary, resource bundle, exact selected Runway 36 configuration, source patch and flight log remain in the run directory. `Validation/Runway36-Restart-Investigation-20261102/provenance.json` identifies the log, selected plan and configuration by SHA-256. The earlier live results report records burn closure, contact, damage and independent postflight inspection.

An exact replay of all 4,239 MM304 records plus the first TAEM record, using recorded accepted planning proposals, produced **zero command or guidance mismatches** across 4,240 records. It includes measured attitude, force and actuator history and the three atmospheric configuration-invalidation events. This validates reconstruction of the flown decisions; it is not a new trajectory or landing prediction.

Two temporary diagnostic checks answered narrower questions: where the actual TAEM candidate checks first fail, and how the MM304 range solver ranks two endpoints from one recorded state. Their source and output are retained outside the build target. They use unchanged vehicle, propulsion and atmosphere capability. The temporary test source was removed after execution; no permanent compile-time tests were added.

## MM304: unresolved delivery, already forecast beyond the threshold

At entry, UT 215984.991, the shuttle was at 69,497.78 m ASL and 2,102.95 m/s, 745.05 km from the threshold. All **88 accepted entry forecasts** reported infeasible capture. The first forecast already predicted terminal along **+35,958.58 m**, cross +22,276.40 m, rather than the objective along **−35,023 m**, cross zero. The last predicted along +33,367.12 m/cross +8,399.83 m. This is not a forecast that promised successful delivery and suddenly failed at handoff.

All **4,239 MM304 records** had `rangeReferenceBracketed=false` and `jointForceAttainable=false`. The range reference selected its maximum-incidence branch for 3,601 samples and best-glide branch for 638. Incidence was therefore often already near the configured 40° ceiling; simply requesting more drag or more bank would not solve the trajectory within its envelope.

The same previously identified selection defect remains active: `EntryRangeReference.solve` prices only predicted along-coordinate error at interface energy. It omits crossrange and heading from that residual. `EntryCommandLaw` then selects incidence from this reference before assigning supported lateral force. Its separate joint-force assessment does not choose the actual operating point.

A narrow reconstruction from UT 216350.051 demonstrates the consequence in this restart:

| Existing range trial | Along at interface energy | Cross | ASL altitude | Speed |
| --- | ---: | ---: | ---: | ---: |
| Best-glide branch, selected | +32,649.49 m | +37,902.85 m | 25,361.16 m | 751.00 m/s |
| Maximum-incidence branch | +34,111.42 m | +96.89 m | 27,114.00 m | 729.83 m/s |
| Objective | −35,023 m | 0 m | 25,000 m | 754.77 m/s |

Both along residuals are positive: neither brackets the objective. The solver selects best glide for only 1.46 km improvement along, overlooking a 37.8 km increase in cross miss. These point-mass endpoints are diagnostic alternatives, not validated flyable arrivals or evidence the maximum-incidence branch would land.

At that recorded state, actual incidence was 20.19° and bank 0.316°; observed lift was 3.029 m/s² while the support request was 3.592 m/s². The selected force allocation therefore left no lateral remainder. At UT 216310, incidence was already 39.34°, observed lift 1.231 m/s² versus support 2.694 m/s². This distinguishes a physically limited earlier state from the later scalar-reference choice; no single higher bank gain repairs both.

The closest threshold approach was still airborne in MM304: UT 216410.051, range 33,004.61 m, height 27,554.52 m ASL, speed 917.81 m/s. TAEM inherited along **+33,356.97 m**, cross **+8,144.47 m**, height **26,680.61 m ASL**, speed **735.43 m/s**, at UT 216422.151. The along error to the requested delivery station was 68.38 km. A return HAC is permitted, but it must be priced from this actual state.

## TAEM: geometry exceeds its own force envelope, fallback remains trapped

The exact handoff diagnostic reconstructed authority observation with the recorded resets and history: lift effectiveness 0.8418595, drag effectiveness 1.4936436, no identified incidence ceiling below the configured 40°. The NEP was available, including its separate insufficient-runway-braking advisory. The earlier missing-NEP defect was absent.

The acquisition geometry uses `EnergyManagementEngine.acquisitionBank`, derived from configured maximum bank/load and uncertainty, then `speed²/(g tan(bank))`. It does **not** derive turn radius from attainable lift at the measured dynamic pressure and trim. At handoff this produced a 25,846.69 m acquisition radius and an 81,199.78 m initial turn arc for every candidate.

All three candidates failed the same-profile force check at the **very first station**, before any integrated future energy loss:

| Candidate | Total priced path | Remaining HAC turn | Initial normal | Initial lateral | Total lift demand |
| --- | ---: | ---: | ---: | ---: | ---: |
| Selected right overhead | 145,202.24 m | 536.60° | 9.045 m/s² | 20.709 m/s² | 22.598 m/s² |
| Left | 128,999.97 m | 201.98° | 8.630 m/s² | 20.709 m/s² | 22.435 m/s² |
| Short right | 126,332.62 m | 176.58° | 8.538 m/s² | 20.709 m/s² | 22.400 m/s² |

The conservative available trim lift was only **5.076 m/s²**, at modeled q 2,956.98 Pa. Even the profile's initial normal demand alone exceeded it. The failure is not merely a bank cap: the path and altitude profile demand a lift vector unavailable at that state. Reference height 25,859.66 m is runway-relative; adding the runway's 820.95 m elevation gives the measured ASL height above.

When no candidate passes, `TAEMGuidanceEngine.update` sorts paths by descending length and selects the first: the **longest** path. It latches that winding. A later shorter downmode requires another candidate to pass the full force/energy check; the common acquisition geometry makes those candidates fail too. Spiral acquisition requires reaching the committed tangent within one roll-response distance.

Throughout all **4,298 TAEM records**, the phase remained acquisition, `referenceTracking=false`, `descentCheck=trimLift`, and `downmoded=false`. There was no spiral/heading-alignment segment. The actual control law bounds the lift vector and gives lateral force only what remains after vertical/pressure recovery. It therefore cannot fly the aggressive curvature used to price its committed path. The point-mass price and actual bounded commands disagree; retaining the rejected tangent does not regenerate a realizable recovery.

The huge legacy EN/EMIN energy-line values are advisory telemetry. They are not independent evidence that every return route is impossible. The decisive evidence here is the first-station force failure and the actual absence of spiral capture. A gentler descending return might differ, but no attainable, fully priced alternative has been established by this investigation.

## Deorbit and outcome

This restart selected the existing 11.25° fixed-angle maneuver, target retrograde impulse 34.489541 m/s; `normalDeltaV` was absent. The new independent normal-first family was available but did not win selection. Actual retrograde delivery was 34.664547 m/s, periapsis 50,244.60 m ASL, with no thrust-loss or unobserved-impulse fault. This flight does not demonstrate a failure of the independent-normal burn controller.

The saved nominal full-route forecast was already failed, with contact around along +11.9 km/cross +82.9 km. The known cross-runway failed-candidate ranking limitation remains: tied failed branches fall back to entry heading travel and orbital miss rather than carrying the complete downstream delivery residual into the final comparison. The selected plan does not retain enough competing screen detail to establish which rejected maneuver would have delivered the best actual recovery.

The actual first ground contact was 55.75 km off the threshold at 31.62 m/s. A bounce was followed by loss of one part. Independent inspection confirmed a stopped 25-part vehicle 55.65 km off-field. This is a failed, damaged off-runway arrival, not a landing success.

## Required repair direction

1. Replace the unbracketed along-only MM304 ranking with comparable spatial, heading, energy and altitude delivery costs, using attainable coupled lift/drag/bank and actual response history. Keep explicit failure evidence when no arrival solves the objective.
2. Generate acquisition curvature and descent jointly from measured q, trim lift, energy and response. A configured bank/load ceiling is an upper bound, not evidence that the corresponding turn is available.
3. When all paths fail, rank partially attainable recovery on consistent physical costs and regenerate it from measured progress. Do not require a wholly passing plan before improving an already rejected fallback; preserve winding and command continuity.
4. Carry downstream delivery residuals and compact candidate-selection evidence across runway comparisons. Keep forecast qualification separate from permission to continue recovery.

A narrow matched-state check should first show attainable initial force and useful acquisition progress under these repairs. No broad campaign, vehicle-authority perturbation or additional live flight is warranted by this investigation alone. Successful full-chain live landing remains unvalidated.
