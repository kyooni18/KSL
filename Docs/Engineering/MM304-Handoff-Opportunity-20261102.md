# Opportunity-based MM304 handoff — 2026-11-02

The authorized restart using `637cff1` failed off-field. During it, the user correctly identified late MM304 handoff and requested a more generous condition that judges whether waiting is still useful. This follow-up implements that timing decision. It has passed focused recorded-state checks and a release build, but has not been flown live.

## Actual restarted flight

Run: `runs/20261102T140105Z-dessert-recovery-fix-live`, frozen revision `637cff1d0696054f6ca7bf30995c2ad38abf949a`, executable SHA-256 `9b7694d9668e066a34a3975c3a71aa3ed947150a28e37e2b6ce75525f7e14a5e`. The existing orbital quicksave was loaded only after the previous controller was confirmed stopped. No save was written. Frozen executable/resources, empty flown-source patch, exact input and selected configuration, plan, logs and postflight inspection remain in that run.

Fresh planning selected Dessert Runway 18, a legacy 19.995° fixed-angle burn with 34.79834 m/s retrograde target. The maneuver delivered 34.97917 m/s, periapsis 50,235.11 m ASL, without thrust loss or unobserved impulse. Throttle was zero afterward. The retained entry screen predicted a 23.12 km delivery miss; the complete nominal route was failed before flight. No landing qualification was claimed.

MM304 started at UT 215832.632, altitude 69,496.03 m ASL, speed 2,102.67 m/s. The new incidence/reference ranking operated and the incoming lateral arrival was much closer than the preceding first attempt, but these were different starting epochs and maneuvers and are not a matched-state A/B validation.

Closest actual threshold approach occurred in MM304 at UT 216421.092: along −28,014.93 m, cross +2,394.06 m, range 28,117.04 m, height 26,776.39 m ASL, speed 785.08 m/s. Actual TAEM transfer waited until UT 216440.552: along −29,144.43 m, cross −10,912.29 m, height 24,921.81 m ASL, speed 649.71 m/s.

The transfer gate still combined interface energy with proximity to the delivery aim. The preceding patch restored the 5 km aim without removing that existing ownership-radius condition, making it more restrictive. That should have been removed in the earlier repair: delivery position is an objective, not a reason to keep spending terminal energy after the energy handoff is due.

At UT 216427.172 the measured rotating-frame specific energy first reached the accepted objective's energy. The shuttle was at 26,246.26 m ASL, 740.80 m/s, cross −1,999.66 m, but the radius condition blocked transfer. The additional **13.38 seconds** lost **1,324.46 m altitude**, **91.09 m/s speed**, and approximately **8.91 km crossrange** before the Mach-floor fallback finally transferred ownership.

An earlier scratch analysis looked at the reference's reserve-adjusted `captureEnergyMargin`; that is not the actual specific-energy crossing. The corrected timing extraction uses `V²/2−mu/r−|omega×r|²/2` against the recorded `captureSpecificEnergy`. `actual-handoff-timing.json` is the authoritative timing result.

TAEM selected a short unqualified fallback rather than the old long overhead. It still spent all 3,431 terminal records in acquisition, never joining its spiral. Initial turn support was unavailable and later profile checks failed trim lift. Thus the previous repair did not establish a usable recovery or landing.

First ground contact was 65,429 m off the threshold at 31.53 m/s with 26 parts. Subsequent bounce/contact damage left 24 parts. After 21.96 seconds of continuous stopped telemetry, the controller was cancelled and exited through cleanup. Independent inspection at UT 216962.012 confirmed stopped contact, about 0.014 m/s, throttle/thrust zero and brakes on. Last continuous flight record was 65,534.63 m from the threshold. No guidance owner remains active.

## New timing behavior

`EntryHandoffTiming` is shared by live mission guidance and the entry forecaster. It separates a control-transfer decision from a successful capture or qualified plan:

1. The existing altitude and lower-Mach deadlines remain unconditional recovery transfers.
2. Reaching actual interface energy transfers immediately, regardless of distance to the delivery aim.
3. Earlier transfers are allowed inside the existing configured upper altitude/Mach terminal window. This preserves the guard against a thin-air or high-energy transfer based on Mach alone.
4. A response-energy reserve leads the transfer by `measured drag acceleration × airspeed × response time`. Response uses the existing configured fallback/lookahead and identified pitch/roll response, not a scenario timing constant.
5. Closest approach to the current delivery objective is computed from measured runway-chart position and velocity. A local heading-turn estimate uses the model's maximum trim/load/uncertainty-bounded lift, the configured bank limit and measured horizontal speed. If heading work plus handover response exceeds the approach time left, transfer now rather than waiting for proximity or exact energy.
6. Otherwise compare the current inlet with the same propagated incidence reference used by MM304, on the same position/altitude/speed/heading residual and configured tolerances. Waiting is allowed while that fresh forecast improves the inlet; a current inlet already better than the predicted later one triggers transfer. Stale comparisons are ignored.

The heading-time estimate is optimistic about normal support and local to the current force/q state. Future q and speed change, so it is a latest-start warning, not a global minimum-turn-time proof. Likewise current-versus-forecast comparison is a bounded timing choice, not a claim that a globally optimal handoff or a landable trajectory has been found. No failed feasibility advisory vetoes recovery.

The command law and timing comparison now share one arrival-residual function. Objective changes invalidate their old range reference, preventing a comparison against another objective. Predictor and live boundary use the same timing function, response convention and trim-bounded lift calculation. The reason and assessed timing quantities are logged; unassessed values are not fabricated as zero.

## Validation and limits

`Fixtures/mm304-handoff-opportunity-20261102.json` contains five genuine unchanged-flight snapshots and their actual objective/reference data. The check does not fly alternative trajectories or alter vehicle authority.

- The excess-energy state outside the upper terminal Mach window remains in MM304.
- At UT 216414.292, 834.15 m/s and 27,348.31 m ASL, the heading estimate is about 221 s versus 12.88 s to closest approach. The new decision transfers for heading/response lead—**26.26 seconds before the flown transfer**, before the centerline crossing. This is a held-history decision check, not a prediction of the resulting later path.
- Without assessed heading authority, the same logic correctly continues while the fresh reference improves the inlet. At UT 216422.012, available energy margin 32.01 kJ/kg has fallen below the 40.93 kJ/kg handover reserve and transfer becomes due, still before the exact energy crossing.
- At actual interface energy, transfer is unconditional even outside the 5 km aim.
- The updated existing ownership regression preserves high-energy protection, Mach/altitude deadlines and no repeated reset once TAEM owns control. The two recorded arrival/fallback regressions still pass.

Four focused test methods pass; release build passes. No extra live attempt, synthetic plant scaling or broad flight campaign was run for this timing change. An earlier test expectation placed the reserve crossing at UT 216419.952; measured energy was still 45.57 kJ/kg versus 38.87 kJ/kg reserve then. The expectation was corrected using the next genuine recorded state, without changing the production threshold. Development output is retained.

Evidence and reproducible offline extraction are under `Validation/MM304-Handoff-Timing-20261102/`. The existing orbital quicksave remains available. Earlier transfer and successful full-chain live landing remain unvalidated until a further explicitly authorized attempt.
