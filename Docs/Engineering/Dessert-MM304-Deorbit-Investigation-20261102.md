# Dessert MM304 crossrange and deorbit prediction — 2026-11-02

## Conclusions

MM304 did not turn enough to deliver a useful terminal inlet. The actual course changed from 89.972° at atmospheric entry to 93.648° at MM304 exit; the selected runway heading was approximately 180.943°. This was predominantly a guidance/force-allocation limitation, not a failure to track commanded bank.

The deorbit planner **did foresee a failed landing**. Its saved nominal continuation predicted off-runway contact at along -55,755 m, cross -56,104 m and sink -18.68 m/s. It reported the plan as unqualified before execution. The issue is why its best available candidate still had a large delivery deficit, how little of that diagnosis was retained/presented, and why MM304 continued with a range reference that could not achieve its target.

This investigation does not establish a physically landable alternative burn or entry trajectory. It identifies a demonstrated control allocation problem and limitations of the selection/prediction contract. No further production guidance was changed.

## Evidence and scope

Same flight as the preceding TAEM investigation: operation `runs/20261102T080307Z-deorbit`; native session `B090715F-7F7C-4F5E-AE19-05E4FC473E85`. Full preflight plan is `runs/native-deorbit-FF2AA0DD-685E-4198-9A1E-BEB6A0EFD1A0.json`, also embedded in the first native record. Exact configuration and original flight-log identity are documented in `Validation/Dessert-TAEM-20261102/provenance.json`.

The MM304/entry sources are unchanged between the flown-source investigation checkpoint `68ea70c` and the subsequent TAEM reference repair `8ca5470`. The source-reference caveat in the preceding investigation still applies. Evidence extracted here is in `Validation/Dessert-MM304-20261102/`.

There are two different crossrange quantities:

- The deorbit window's 67,234 m `crossrange` is a great-circle miss of the projected incoming ground track relative to the site. It is **not** `RunwayFrame.cross`.
- At MM304 exit, runway along was -57,153 m and runway cross was +32,048 m (positive right), for threshold range 65,525 m. The incoming trajectory was still nearly perpendicular to the runway. Reducing runway cross during eastward travel does not mean the needed southward turn or terminal alignment was accomplished.

## Why the commanded turns accomplished little

All 5,876 MM304 records have `rangeReferenceBracketed=false` and `jointForceAttainable=false`. These flags alone do not prove no partial correction was available, but show that the requested combined capture state was never solved by the reference/instantaneous force assessment.

Above 40 km ASL, the vehicle often tracked approximately 49° bank, but lift was tiny. Integrated measured lateral lift, L/m × sin(bank), supplied only 14.66 m/s of lateral impulse over 410.9 s. Most useful steering occurred briefly between 40 and 30 km: 76.34 m/s of lateral impulse over 86.22 s. Below 30 km, the remaining 90.54 s added only 5.40 m/s. The approximate aerodynamic heading changes from integrating lateral acceleration / horizontal speed were 0.40°, 2.90° and 0.38°, respectively. These are explanatory force integrals, not an exact rotating-planet course decomposition; the directly measured course change is 3.676°.

Bank tracking RMS error was approximately 1.09°, 0.88° and 0.52° in those altitude bands. The heading shortfall was not caused by a hidden request for a large turn that the attitude loop refused to execute. At the late handoff the shuttle still needed about 87° of runway-heading change.

## Demonstrated allocation defect: ranging chooses incidence first, steering receives the remainder

`EntryCommandLaw.swift:180–191` computes joint force feasibility, but that result does not select the actual command. Lines 198–213 instead:

1. Solve a scalar incidence range reference.
2. Pick the polar point nearest that incidence.
3. Call `EntryRangeReference.bank` with only that point's lift.
4. Reserve vertical support and give lateral guidance whatever remains.

The diagnostic of an unsupported joint request is therefore followed by a separate incidence choice that can remove **all** turning authority, even when another allowed polar point can provide a partial correction. `EntryForceAllocator.decide` contains other allocation logic but is not called by this command-selection path; changing that unused method would not fix the flown behavior.

A narrow offline diagnostic reconstructed the actual demand and the same 65-point polar sampling from two genuine recorded states, with their accepted objectives and measured feedback. It retained the configured uncertainty, trim/load/bank envelope and vehicle properties. It did not alter the plant or fly a continuation.

| Recorded UT | Raw lateral demand | Required vertical support | Selected incidence / reserved lift | Selected bank | Maximum lateral authority across allowed polar points |
| --- | ---: | ---: | --- | ---: | ---: |
| 218940.074 | +5.247 m/s² | 4.752 m/s² | 20° / 4.651 m/s² | 0° | 3.458 m/s² at 30° AoA |
| 219000.074 | +7.359 m/s² | 7.027 m/s² | 21.875° / 4.892 m/s² | 0° | 0 m/s² with vertical support reserved |

At the first state, full demand would need 7.079 m/s² lift, while the allowed maximum was 5.877 m/s²: the complete request was unattainable, but **some steering was available**. Selecting the 20° range point made its lift smaller than the vertical reservation, so the bank law necessarily returned zero. Actual incidence/bank were still approximately 27.17°/25.02° because incidence response and target limiting were shedding earlier demands; the newly selected operating point explains why the request then unloaded.

By the second state, even the maximum reserved lift, 6.915 m/s², was below the 7.027 m/s² support request. The support-first allocator had no lateral remainder anywhere in its current conservative envelope. That later condition must be handled physically; it is not permission to increase bank limits or disregard lift support.

The probe source, recorded samples and output are `MM304-probe.swift`, `force-samples.json` and `force-probe.log`. One temporary diagnostic XCTest executed the probe and was removed from the test target. Its passing status means only that it ran successfully, not that the trajectory is validated.

## Why the range reference does not solve the lateral delivery problem

`EntryRangeReference.swift:98–102` ranks its endpoint trials exclusively by:

`predictedAlongAtInterfaceEnergy - objective.along`

It does not include endpoint crossrange, heading, altitude error or the downstream turn work in that scalar residual. If both endpoint residuals have the same sign, it immediately returns the closer endpoint as an unbracketed reference. There is no solved arrival to track.

From UT 218940.074, the unchanged shooting algorithm produced:

| Existing command endpoint | Along at interface energy | Cross at interface energy | ASL altitude | Speed |
| --- | ---: | ---: | ---: | ---: |
| Best-glide incidence branch | -57,776 m | +40,432 m | 25,590 m | 747.27 m/s |
| Maximum allowed incidence branch | -58,333 m | +76,078 m | 28,143 m | 716.08 m/s |
| Requested objective | -35,023 m | 0 m | 25,000 m | 754.29 m/s |

Neither branch brackets the requested along coordinate. The best-glide endpoint wins the existing residual, but is still about 22.75 km short along and 40.43 km off centerline at the interface energy. The returned reference is an advisory fallback, not a useful solved capture. At the same measured state its selected incidence also yields zero bank, as shown above.

This supports replacing the scalar along-only fallback decision with consistently priced spatial/energy/heading recovery candidates and a jointly attainable force allocation. It does **not** establish that simply choosing 30° or maximum incidence throughout entry would land: extra lift also incurs drag, finite attitude response, and future energy/altitude costs that need to be priced.

The native configuration requested a 35,023 m terminal range. Although `captureTargetRange=5,000 m` and `handoffAlong=-8,000 m` also exist in configuration, the accepted objective was normally along -35,023/cross 0. For 11 accepted proposals, UT 218609.074 through before 218664.734, the fallback planner instead chose cross aim +32,737 m, with along -12,445.857 m on the same range circle. It later returned to centerline. This was a deliberate alternative-inlet search, not random sensor drift. All these proposals remained infeasible; the shift cannot be counted as successful crossrange cancellation, and its causal contribution cannot be isolated without a matched-history comparison.

## What the deorbit planner knew

The saved plan recorded:

- 619 orbital-window evaluations; these are orbital probes, **not** 619 complete landing forecasts.
- Selected normal burn angle 23.8403°, delta-v 31.4614 m/s, predicted entry range 1,022.744 km and incoming great-circle miss 67.234 km.
- A nominal native full-route continuation with `passed=false`, off-runway contact coordinates above, and `hardUnsafe=false`.
- `nominalPassed=false`, `fullRouteQualified=false`, `executionQualified=false`, and `targetCaptureAchieved=false` in the operation's review data.

After atmospheric entry, **all 117 logged accepted MM304 forecasts also reported infeasible capture**, with zero completed candidate routes in their terminal assessments. The first forecast already predicted handoff at along -59,334 m, cross -6,941 m, about 59.7 km from the threshold, with `inadequateControlAuthority`. It did not predict a good inlet and then discover the problem only at TAEM.

Its initial endpoint differs from the flown handoff (+32.05 km cross rather than -6.94 km), but the live trajectory accepted different objectives and refreshed control/response information. That comparison is not a matched-command aerodynamic-model validation. Later forecasts converged toward the measured bad inlet; the final accepted forecast predicted along -57,146/cross +31,886 m near handoff. There is no evidence here of a model confidently predicting successful crossrange cancellation.

## Why it still selected and permitted this window

Within each runway branch, `DeorbitPlanner.plan` runs the actual MM304 forecaster, tries nominal continuations, and ranks failed candidates by modeled inlet proximity when none passes. Across runway branches, however, the saved downstream inlet residual is **not carried into the final comparison**: when qualification/pass/safety fields tie, `planSelectingRunway` falls back to necessary entry-interface heading travel and then orbital-window miss (`DeorbitPlanner.swift:87–104`). That can choose a geometrically easier incoming direction without establishing a realizable terminal turn.

The final `DeorbitPlan` retains the selected nominal case and orbital window but discards the selected screen's full terminal geometry and the competing candidates' diagnostics. Therefore the original run artifacts cannot establish whether another tested window/runway had a better delivery residual or whether an unsearched burn would have worked. The older `LandingAwareWindowSelector` is not the active selector on this `DeorbitPlanner.plan` path; reading its ranking policy alone would misdescribe this attempt.

Authorization intentionally separates a physically screened OMS-to-entry window from a qualified landing forecast. `DeorbitPlan.hasExecutableEntryWindow` requires nominal evaluation, a completed modeled burn and no hard-unsafe case; it does not require a successful landing. The runtime and operation review logged degraded/unqualified continuation before the explicitly executed burn. Changing this into an unconditional no-burn veto would conflict with the project's advisory/recovery policy.

There are also prediction-fidelity limits: nominal full-route qualification disables recursive entry replanning, whereas the live controller accepts asynchronous proposals and changes its objective. The saved plan explicitly lists that limitation. The prior missing-NEP/rollout defect also affected the terminal forecast, so failed full-route cases cannot be interpreted as proof every physically possible route is infeasible.

## Next engineering work

1. Use a common spatial/energy/heading endpoint and genuinely attainable lift/bank/drag candidates for the MM304 range reference and lateral allocation. Keep a priced recovery candidate when no target is solved; do not present an unbracketed scalar reference as successful capture.
2. Reconcile the 5 km delivery requirement with the actual 35.023 km objective and the separate legacy handoff fields. Changing one nominal setting does not by itself fix the coupled entry trajectory.
3. Carry the selected screen's terminal position, heading, speed, energy, deficits and provenance into `DeorbitPlan`. Compare failed runway branches on those same downstream physical residuals before orbital geometric tie-breaks, and retain a compact explanation for rejected alternatives.
4. Present the predicted inlet miss and the failed nominal endpoint in plan review. Keep search permission, best-available selection, landing qualification and explicit actuation authorization separate.

No additional flight, KSP action, full-route campaign or altered-vehicle test was run. The only new numerical check was the two-state force/reference diagnostic. Flight readiness remains unknown. The already committed TAEM reference repair restores a control path; it does not repair this MM304 delivery problem.
