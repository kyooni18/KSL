# READY FOR TERRA

Workspace handoff is ready. **Flight readiness is NOT established; the latest offline continuation fails terminal delivery.** No other agent from this task is editing or building now.

Continue in `/Users/kyooni18/Code/KSL`, Swift package `swift/KSL-Landing`, branch `mm305-nep-energy-recovery`, checkpoint `da7cfffa057cff05c3adced0b9b88e2b77324f7e`. Read AGENTS.md and this directory's `findings.md` first. Search Foundation for KSL TAEM capture. Preserve all unrelated dirty files. Commit only your own coherent changes.

## Objective
Make TAEM deliver the actual STS to a qualified Final capture using a physically attainable planned descent. Preserve the repaired high-altitude Final guard and winding continuity. Do not reopen MM304 without contradictory evidence: the last measured handoff was 738.28 m/s about 35.6 km before the runway, not the older 85–90 km failure.

## First bounded task
Establish why the same recorded handoff loses speed from 741.5 to 137.0 m/s between 40.1 and 80.1 seconds in `continuation.json`. Trace actual altitude, incidence, bank, drag/energy loss, q, commanded lift and controller response. The current EN/EMIN lines price a different q-scheduled path from the committed Hermite descent. Geometric `descentPlanFits` is not an energy certificate. Also inspect whether the attainable-maximum-CL support floor permits sustained high-incidence energy starvation. These are hypotheses to verify, not instructions to blindly replace a coefficient.

Read `TAEMGuidanceEngine.swift`, `TAEMDescentReference.swift`, `LandingMissionController.swift`, and `ApproachGuidanceEngine.swift`. Relate planned altitude/curvature/turn demand to attainable lift and energy loss on the SAME path. Prefer the smallest cause-based change. Do not add arbitrary altitude/speed gates, gains, new caps, runway-specific conditions, or weaken existing physical protections. Preserve an advisory recovery owner when no candidate is feasible.

## Evidence and reproduction
- Exact measured handoff: `Validation/TAEM-Planned-20261102/handoff.json`.
- Recorded premature Final: `premature-final.json` in the same directory.
- Live source/config/log: `runs/20261102T040424-mm304-shooting-live-72km`.
- Latest offline result: `continuation-final.txt`, `continuation.json`.
- Reproduction harness: `ZZScratchPlannedTAEMTests.swift.archived`; temporarily restore under GuidanceTests if needed, then remove/archive it again. It warms controller history and uses recorded applied controls. Do not replace this with a position/velocity-only synthetic seed.
- The scratch test PASS only means its initial/gate assertions passed. Its printed continuation FAILS: unqualified Final at 386.9 s, along -5494.7 m, cross -3248.1 m, height 961.2 m, speed 41.1 m/s. No touchdown or stop was validated.
- Three focused `TAEMTransitionTests` pass; release build passes. Logs and executable hash are recorded here.

## Working procedure
1. Diagnose the first material trajectory/model mismatch before editing. If more logging is needed, collect only the missing quantities from this same recorded continuation.
2. Fix that cause. Run the smallest decisive regression, then one continuation if the change merits it. Diagnose any failure before another attempt; no overnight loops, sweeps, or synthetic vehicle perturbations.
3. Keep angle, frame, gravity, density, altitude datum, and units explicit. Use Kerbin's configured atmosphere/gravity, not Earth constants for trajectory physics. Preserve configured limits, timeout/reconnect recovery and RCS availability.
4. Preserve user saves and unrelated work. Archive experiment source/configuration/log identity. Commit useful checkpoints; update findings and Foundation with what actually passed and failed.
5. Stop with an honest report if a required model is unidentified; do not invent measurements or claim success from phase labels, build results or a modeled stop.

No new live flight is authorized by this handoff. Do not load, unpause, command, monitor, or restart KSP. An older controller may still use an archived executable; do not interfere or replace any running executable. Ask the user for a new live run only after the offline result and remaining uncertainties are concrete and reviewable.
