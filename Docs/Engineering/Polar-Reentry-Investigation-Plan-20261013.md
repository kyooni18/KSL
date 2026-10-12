# KSL Polar Reentry: Post-Flight Investigation Plan

**Prepared:** 2026-10-13 (KST)  
**Branch / live code:** `mm305-campaign-unpaused` / `c5ae066`  
**Status:** Live flight deliberately stopped; investigation only. **No further KSP flight or guidance changes are authorized in this phase.**

## 1. Frozen evidence and exact outcome

The authorized polar STS return started from an approximately 100 km orbit. At the user's request, the KSL mission controller received SIGINT and completed its own safe-control abort at **01:29 KST**. The mission's saved state is `aborted`, phase `mm305`; the native backend and Python mission process terminated. A separate KSP pause request did not confirm physics pause, so do not claim the game itself is paused.

- Run: `runs/20261012T155948Z-polar-offaxis-native-20261013/` (plan, telemetry, events, backend trace, status).
- Paired native telemetry: `FlightLogs/2026-10-12T15-59-49Z-STS-vehicle.jsonl` and `...-planner.jsonl`.
- Save backup made **before** the live attempt: `runs/20261013T0059Z-polar-guidance-preflight/persistent-before-live.sfs` and `polar100-checkpoint-preserved.sfs`, with `checksums.txt`.
- Plan: 42.01 m/s finite deorbit, target postburn periapsis 50 km; actual calculated postburn periapsis approximately **51.5 km**. First significant thrust observed about 0.8 seconds after planned start, far better than the previous ~34 second delay.
- Prediction vs flight at a common altitude: at 70 km, predicted/actual time difference ~30 s; at 65 km ~68 s; at 50 km ~91 s. At 50 km, predicted speed ~2192 m/s vs measured ~2172 m/s; predicted KSC range ~500 km vs actual ~398 km. Verify these with precise time interpolation and rotating-frame coordinate checks; they are preliminary comparisons.
- MM305 ownership transition was reached, but the shuttle was roughly **27.5 km altitude, 833 m/s and 5.5 km from KSC**, much too high/fast/close for a normal bounded HAC and Final approach. Immediately after handoff it overflew/retreated from KSC.
- At the deliberate mission stop it was approximately **14.35 km altitude, 286 m/s, 42.37 km from KSC**, still in MM305, without Final capture. This is a **stopped test**, not a scored landing or automatic crash.
- Prior nominal full-terminal prediction had `terminalInletEnergyMargin` around **-293 kJ/kg** and failed strict qualification. The flight continued by authorized best-effort policy; do not turn this negative certificate into a software abort.

## 2. Investigation questions, in priority order

### A. Find the **first** model/flight divergence (highest priority)

Build one timestamped, inspectable trajectory comparison using the recorded finite-burn state and the same configured planet/site/model. Compare **both common UT and common altitude**; report latitude/longitude in a validated planet-fixed frame, runway along/cross distance, range and bearing, vertical speed, flight-path angle, Mach, dynamic pressure, actual/model lift/drag, bank and AoA (commanded **and** achieved), and source model snapshot. Quantify uncertainty and first divergence at ~70–65 km; avoid interpreting an altitude-indexed 90 s UT offset as one independent cause.

Discriminating checks:

1. Re-propagate from the **measured cutoff state**, not the planned state. Determine how much of the error is attributable to the ~1.5 km periapsis offset and small burn/attitude mismatch.
2. Compare planet epoch, prime meridian, inertial/surface rotation transforms, interpolation of KSC runway coordinates, and the predictor's postburn UT. Verify identical coordinate semantics before attributing ground-track drift to aerodynamics.
3. Replay the **recorded MM304 control** against the native plant and examine predicted vs measured vertical speed, lift and drag. Isolate atmospheric density, lift/drag coefficients, vehicle mass, AoA actuator response, and bank authority one mechanism at a time. If the same-speed/shifted-altitude-time issue is from vertical lift and flight-path angle rather than drag, demonstrate it numerically.

**Gate A:** Produce a high-confidence earliest divergence with evidence, an estimated contribution to KSC cross/range error, and a reproducible, frozen-state test. Do not tune MM305 before separating these upstream effects.

### B. Determine why MM304 delivered an unusable MM305 state

Reconstruct admission conditions 40→20 km, including actual heading, speed, sink rate, range, cross-track, predicted turn radius/bank/response, and the sign/meaning of the certified energy margin. Inspect the difference between native MM305 ownership, physically qualified inlet, committed HAC route, actual Final capture, and touchdown. A handoff at KSC geographic closest approach cannot be called a landing corridor simply because it meets phase/altitude/Mach gates.

Compare target inlet distributions from deorbit opportunity candidates with the **measured live inlet**. Identify whether MM304's lateral guidance, its bank/drag allocation, or deorbit phasing should have established cross-track and runway standoff earlier. Check if the trajectory passed close to KSC while still above 27 km because entry arrival was too far north/south or carried excess nonrecoverable energy. Do not weaken limits or reverse the negative energy-margin semantics to make admission pass.

**Gate B:** Demonstrate a physically flyable MM305 inlet with adequate distance, turn authority, and energy under the actual shuttle model, or establish quantitatively why none exists in the searched opportunity window.

### C. Audit MM305/HAC/Final response separately

On the recorded posthandoff state, replay the exact native controller (`taem/energy_law.inc`, route acquisition, terminal reference, attitude/actuator limits) and compare the planned turn with observed bank/heading/ground track over time. Validate the `c5ae066` off-axis and premature Final-mode gates, including the risk of an overhead-loop route being committed too late. Examine *what causes* a committed HAC path to miss Final rather than counting commitment as success. Use actual heading/course/roll slew rates to detect unreachable turn geometry.

Run a small controlled set of earlier and later orbital opportunities and MM304 policy variants, reusing native C prediction. Reject misleading metrics such as closest KSC approach while high above the runway. For Final admission, evaluate runway-relative geometry, altitude, energy, velocity vector, lateral error and flare/rollout constraints.

**Gate C:** Full native replay either enters a correctly aligned, physically admissible Final approach, or reports a falsifiable failure condition linked to the commanded vs achieved trajectory.

## 3. Proposed investigation workflow

1. **No code change / no KSP flight:** extract a reproducible baseline from the captured flight and compare model/live records with validated time and coordinate frames. Publish a single error-vs-altitude and error-vs-UT timeline.
2. **Root-cause isolation:** perform at most a few model/geometry A/B replays, each changing exactly one credible mechanism. Freeze model snapshots and log all predictions and achieved states; avoid broadly sweeping arbitrary gains and permissive thresholds.
3. **Minimal repair:** once a cause is established, modify only its actual owning layer (burn planner, MM304 state/force estimator, lateral energy manager, MM305 admission, or native route). Keep actual flight control envelopes intact. Preserve best-effort attempted flight under failed **forecast** qualifications; only truly missing commands/transport, explicit operator abort, and terminal impact are hard stops.
4. **Offline qualification:** run targeted C tests, `make test`, original recorded fourth polar fixture and this new live-flight replay; verify native MM305-to-Final continuation, non-vacuous robust stress cases, and distinct failure flags. Report whether 15-scenario robustness passes improve without falsely certifying unsafe trajectories.
5. **Only after offline evidence:** propose **one** additional KSP checkpointed live test from a preserved copy of the 100 km polar save. Require explicit renewed authorization to actually start it. Evaluate cutoff periapsis, MM304 arrival, qualified MM305 inlet, HAC geometry, Final admission, touchdown, and rollout; document any remaining mismatch.

## 4. Acceptance criteria

- One canonical, rerunnable live-vs-model time series proves where and why the >100 km runway-distance discrepancy begins (or identifies the exact missing evidence).
- A proposed correction improves the **same** failed recorded return, not just an unrelated synthetic fixture; compare before/after with fixed initial conditions.
- The simulator distinguishes (a) merely entering MM305, (b) physically qualified runway-standoff inlet, (c) HAC route commitment, (d) actual Final capture, (e) touchdown and rollout. Never merge them into a single optimistic capture boolean.
- Guidance/mission continues best effort when a candidate exists but predictions or certified limits fail; preserves warning evidence and control authority; does not crash or silently disengage for model uncertainty.
- No fabricated landing result, no relaxation of bank/AoA/G/stall/runway limits to make tests green, and no modification of the preserved checkpoint.

## 5. Explicit boundary

The user requested **stop and an investigation plan**, not a continuation of the live test. No automated retries, simulator reloads, or further flight experiments should be launched until requested again. The existing untracked `Validation/.DeorbitNativeCandidateProbe.c` is not part of this plan and should remain untouched.
