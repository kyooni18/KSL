# Polar MM305 Off-Axis Capture and Final Gate

**Date:** 2026-10-13; **scope:** native predictor/guidance and recorded fourth polar-orbit STS replay. Initial verification offline; new live evidence must be recorded separately.

## Failure mechanism

The MM305 energy law stores guidance in runway coordinates (x along runway heading, y left, psi relative heading). The fourth STS return reached physical MM305 ownership at x ~+1 km, |y| ~68 km, course ~185 deg vs Runway 09's 90 deg. The old `tel_plan_open()` allowed geometry planning only while x was farther back than (-finalStation-8 km). The predictor therefore never compared overhead/tangent HAC candidates for an already-abeam but very wide polar inlet. The fallback straight-in law immediately switched to `TEL_LEG_FINAL` on the predicate `-D-x < 6000` alone. Once in that leg, the local capture law clipped cross-track to +/-2 km, discarding the actual ~68 km separation. No full runway capture could follow.

## Change

- Permit geometry planning from an off-axis inlet while there is still meaningful lateral turning distance, even when x is abreast of Final. Existing flyable HAC scoring and speed/bank limits decide whether to commit.
- Keep the full straight-in centreline capture leg until the aircraft is longitudinally near Final **and** <=min(2 km, 0.25 * final-station distance) off centreline **and** within 15 deg of runway heading. The narrow Final-capture bank is no longer applied to a perpendicular polar overflight.
- `Validation/TaemEnergyGeometryTests.c` checks polar 68/39 km off-axis cases, rejection of lateral or heading-only near passes, and valid aligned arrivals. Added to fast `make test`.
- Added optional `KSP_LANDER_TRACE_TERMINAL_WITNESS=1` trace to the MM304/native MM305 continuation predictor; disabled by default.

## Controlled recorded replay

The pre-existing `Validation/.DeorbitNativeCandidateProbe.c` supplied frozen orbital state UT 146949660.947894, vehicle aerodynamic envelope and named fourth-flight physical model. Diagnostic C continuation was copied to an independent temporary fixture; KSP was not touched during this comparison.

| Retimed burn | Before changes: modeled final range | After changes: modeled final range | Result |
| --- | ---: | ---: | --- |
| Nominal +45 seconds; MM305 ~68 km from KSC | ~31 km | ~20 km | No certified Final; direct capture leg retained |
| Nominal +60 seconds; MM305 ~39 km from KSC | ~31 km | ~11 km | Overhead/HAC geometry selected, still no Final capture |

The two variants remain low-energy and unqualified; these are offline-model results, not demonstrated KSP landings. The nearest point to KSC can occur at 17-20 km altitude and **must not** be mistaken for touchdown or runway capture.

## Verification

`make -C CLanding taem-energy-geometry-test` passed. `make test` passed the native fast C suite and 144 Python tests with 56 subtests. Full slow MM305 qualification and live guidance remain independent gates. The operational `sts-polar` policy continues attempting physically commandable, uncertified guidance with warnings rather than terminating on failed forecasts.
