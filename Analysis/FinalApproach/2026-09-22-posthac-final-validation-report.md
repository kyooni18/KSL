# Post-HAC Final-Approach Validation Report

**Project:** KSPShuttleLander  
**Date:** 2026-09-22  
**Scope:** ShuttleSim only; post-HAC final alignment through TAEM final-intercept and final approach  
**Result:** **Rejected at initial final-approach admission; no landing success claimed**

## Executive Summary

A dedicated ShuttleSim validation path was built to start directly from an already-recorded post-HAC final-alignment state without invoking or regenerating HAC geometry. The validation deliberately preserved the production final-approach, authority, energy, dynamic-pressure, load, preflare, and TAEM delivery gates.

The selected state came from:

`ShuttleSim/runs/mm305-flylearn-posthac-600-20260922/simulator-telemetry.jsonl`

at UT `66768.169814`. It is a genuine recorded post-HAC state near the current final-alignment station, approximately 3.5 km before the runway threshold, essentially on centerline, and aligned with RW09.

The state passes the runway-capture, lateral-authority, local vertical-recovery, dynamic-pressure, load, AoA, and instantaneous minimum-speed checks. It does **not** pass the production preflare energy check. The first exhausted predicate is:

`preflare-speed-aoa-or-stall-gate`

The failure is specifically a kinetic-energy deficit during the certified preflare pull-up projection. The predicted speed-squared becomes negative before the vehicle can complete the required pull-up while retaining the 85 m/s minimum safe speed. Because the initial physical-reachability check fails, the state is not promotable to the requested 120-second controlled final-approach horizon.

An archive-wide check found no alternate recorded state near the same final-alignment station that satisfies the unchanged production gate. Of 412 telemetry files containing a 3 km-class runway station crossing, 23 states met the strict airborne/alignment/geometric filter; after excluding validator-generated duplicates, 19 original recorded states were replayed through the production admission logic. All 19 were rejected at the same preflare gate.

The evidence therefore points to a **post-HAC energy-state / TAEM handoff problem**, not a lateral HAC-exit alignment problem and not a lack of instantaneous aerodynamic control authority.

## Scope and Constraints

This validation intentionally excluded HAC planning and HAC regeneration. The purpose was to isolate the final approach system downstream of HAC and determine whether a recorded HAC-exit/final-alignment state can be delivered through TAEM final-intercept into the existing final approach logic.

The test preserved the existing production constraints:

- ShuttleSim only.
- No live KSP.
- No orbital or full-entry mission.
- No HAC replanning or regenerated HAC geometry.
- No relaxed final-approach authority or energy gate.
- No synthetic "success" state.
- No airbrake-based energy correction.
- No weakening of preflare or TAEM delivery predicates.
- No forced capture.
- No claim of landing unless touchdown and rollout are actually reached.

The direct-final validation path records the fact that HAC is already complete solely to establish downstream ownership. It does not construct a circle, lead, spline, shortcut, alternate circuit, or any other HAC path.

## Recorded Input State

The selected source state is the current-day recorded post-HAC state closest to the approximately 3.5 km final-alignment station while remaining near the runway centerline and aligned with RW09.

| Quantity | Recorded / replayed value |
| --- | ---: |
| UT | 66768.169814 |
| Runway along-track | -3496.797 m |
| Runway cross-track | +7.525 m |
| Mean altitude | 1480.088 m |
| Radar altitude | 1480.088 m |
| True airspeed | 87.343 m/s |
| Horizontal speed | 79.370 m/s |
| Vertical speed | -36.457 m/s |
| Flight-path angle | -24.670745 deg |
| Air-relative course | 90.0338 deg |
| Ground track | 90.0338 deg |
| Bank | 13.4751 deg |
| AoA | 2.9973 deg |
| Dynamic pressure | 3391.625 Pa |
| Mass | 40252.918 kg |
| Lift | 362688.9 N |
| Drag | 166062.0 N |
| Vessel state | flying |

The exact inertial state replayed by ShuttleSim was:

- Position: `(383783.813096, 463128.505803, -517.844708) m`
- Inertial velocity: `(-219.410775, 134.472349, -0.015457) m/s`
- Air-relative inertial velocity: approximately `(-84.375978, 22.572156, -0.015457) m/s`
- Air-relative local velocity: approximately `(E +79.369986, N -0.046822, Up -36.457000) m/s`

This matters because the validation did not reconstruct the state from heading/FPA alone; ShuttleSim restored the recorded Cartesian state.

## Final-Alignment Reference

The production final geometry uses RW09, runway heading 90 deg, with the final-alignment station approximately 3500 m before the runway threshold.

At the replayed range:

- Target runway-local position: approximately `along=-3500 m, cross=0 m`
- Along-track error: `+3.203 m`
- Cross-track error: `+7.525 m`
- Heading/course error: `+0.0338 deg`
- Production final glide slope: `-20 deg`
- Final-alignment target speed: `160 m/s`
- Final-approach reference speed: `115 m/s`
- Minimum safe speed: `85 m/s`
- Flare altitude: `55 m`
- Touchdown sink target: approximately `-1.5 m/s`

The 20 deg final profile is approximately 1342.73 m MSL at the replayed range. The state is therefore about 137.36 m above the geometric glide profile, but substantially below the required energy state.

## Admission Predicate Results

The production runway-capture envelope is healthy.

| Predicate | Available / actual | Required / limit | Margin | Result |
| --- | ---: | ---: | ---: | --- |
| Runway/capture time | 75.555 s | 5.657 s | +69.898 s | PASS |
| Lateral capture time | — | 5.657 s | within available time | PASS |
| Heading capture time | — | 4.677 s | within available time | PASS |
| Vertical recovery height | 1480.088 m | 238.190 m | +1241.898 m | PASS |
| Instantaneous speed | 87.343 m/s | >= 85.000 m/s | +2.343 m/s | PASS |
| Dynamic pressure | 3391.625 Pa | <= 45000 Pa | +41608.375 Pa | PASS |
| Load | 1.0105 g | <= 3.5 g | +2.4895 g | PASS |
| Runway remaining | 5996.797 m | > 0 | +5996.797 m | PASS |
| Available lateral acceleration | 32.264 m/s^2 | capture proof satisfied | positive | PASS |
| Maximum normal acceleration | 34.335 m/s^2 | positive recovery authority | positive | PASS |
| Recoverable FPA envelope | 64.136 deg magnitude | current 24.671 deg magnitude | positive | PASS |

This rules out a basic inability to turn onto or remain on the runway line. The vehicle has ample computed lateral authority and sufficient local vertical-recovery height.

The failure appears in the production preflare plan.

| Preflare quantity | Value |
| --- | ---: |
| Trigger altitude | 470.220 m |
| Required recovery height | 415.220 m |
| Pitch/response time | 8.056 s |
| Pull-up recovery time | 10.386 s |
| Target AoA | 28.0 deg |
| Target sink | -1.5 m/s |
| Modeled target drag acceleration | 20.243 m/s^2 |
| Air path during pull-up | 907.160 m |
| Predicted speed-squared after pull-up | -20991.386 m^2/s^2 |
| Minimum permitted speed-squared | 7225.000 m^2/s^2 |
| Half-v^2 kinetic margin | -14108.193 J/kg |
| Equivalent initial speed required with other terms fixed | about 189.33 m/s |

The preflare predictor therefore returns `preflare-speed-aoa-or-stall-gate`. The current speed itself is barely above the 85 m/s floor, but the vehicle cannot perform the required pull-up while preserving that floor under the measured/modelled drag and finite response time.

Because the preflare plan is infeasible, the terminal delivery contract is invalid and TAEM final-intercept ownership is not transferred.

## Specific-Energy Analysis

Using energy height

`H_E = h + V^2 / (2g)`

the replayed state has approximately:

- Actual `H_E = 1870.84 m`
- 20 deg final-profile altitude at this range: `1342.73 m`
- Target `H_E` at 115 m/s: `2020.12 m`
- Error versus 115 m/s final reference: `-149.28 m`
- Target `H_E` at 160 m/s: `2653.97 m`
- Error versus 160 m/s final-alignment target: `-783.13 m`

The state is therefore already energy-low even relative to the 115 m/s downstream final reference, not merely relative to the 160 m/s final-alignment schedule.

Expressed as an energy-height correction per metre of remaining approach distance:

- Versus 115 m/s target: approximately `-0.04269 m/m`
- Versus 160 m/s target: approximately `-0.22396 m/m`

The negative sign is important: there is no excess energy to dissipate. The vehicle needs additional retained energy. This is incompatible with an unpowered final approach once the state has already reached this point.

## Transition Trace

The validation terminates before a commanded final approach begins.

| UT | Event | Result |
| ---: | --- | --- |
| 66768.169814 | Recorded post-HAC state restored | Airborne, valid telemetry |
| 66768.169814 | Runway-capture envelope evaluated | Valid and reachable |
| 66768.169814 | Preflare plan evaluated | REJECT: `preflare-speed-aoa-or-stall-gate` |
| 66768.169814 | Terminal delivery contract evaluated | Invalid because preflare plan is infeasible |
| 66768.169814 | TAEM/final-intercept handoff | No transition |
| 66768.169814 | Recovery/replan | Not entered; `recoveryActive=false` |

No guidance command was issued after admission because the state was rejected first. No preflare, flare, touchdown, or rollout was executed.

## Archive Search and Falsification

To test whether the selected state was simply a poor sample, the recorded ShuttleSim archive was searched for alternate states around the same runway station.

The search found:

- 412 telemetry files containing a 3 km-class runway-station crossing.
- 23 states satisfying the strict geometric filter: airborne, approximately -3.5 km along-track, near centerline, approximately RW09-aligned, and in the relevant altitude range.
- 19 original recorded states after excluding validation-generated duplicates.
- 0 accepted by the unchanged production final-admission logic.
- 19 rejected.

All 19 original states failed at `preflare-speed-aoa-or-stall-gate`.

The selected current-day state is also the least-steep-FPA state among the strict candidates at approximately `-24.67 deg`. Faster alternatives around `114-117 m/s` were already descending at approximately `-36.8 deg` to `-40.0 deg`, which makes the vertical/energy situation worse rather than providing a clean final-alignment substitute.

This substantially weakens the hypothesis that the validation failed merely because the wrong recorded final-alignment snapshot was chosen.

## Diagnosis

### Symptom

No recorded post-HAC state near the final-alignment station can be promoted through production final-approach admission into the requested 120-second controlled landing validation.

### Immediate failure

`terminal_preflare_plan()` rejects the state through:

`preflare-speed-aoa-or-stall-gate`

The numerical cause is negative predicted kinetic margin during the finite-response preflare pull-up.

### What is not failing

The evidence does not support lateral capture as the primary blocker. Runway alignment, cross-track capture time, heading capture time, dynamic pressure, load, instantaneous speed, vertical-recovery height, and available control acceleration all have positive margins in the selected case.

### Root-cause classification

The observed failure is best classified as:

1. **Post-HAC energy-state failure.** The vehicle reaches final alignment with too little retained kinetic/specific energy and too much sink for the downstream preflare maneuver.

2. **TAEM handoff predicate failure.** Because the executable final/preflare delivery contract cannot be certified, TAEM correctly refuses to hand the vehicle to Final.

The evidence does not justify weakening the final energy or authority gates. Those gates are detecting an actually unlandable downstream state under the present unpowered constraints.

## Why the 120-Second Run Was Not Executed

The requested long controlled run was intentionally gated on initial physical reachability.

The selected state fails that gate before control begins. Running it for 120 seconds would therefore test the evolution of a state already certified as non-deliverable, not validate nominal final-alignment-to-touchdown behavior.

Stopping at initial admission is consequently a test result, not an incomplete execution.

No landing success is claimed.

## Required Upstream Correction

The smallest evidence-supported correction is upstream of Final.

A new genuine post-HAC state must reach the final-alignment region with materially more retained kinetic energy and a shallower descent. It should be substantially closer to the existing 160 m/s final-alignment speed schedule and the -20 deg final glide condition before TAEM attempts final delivery.

This does **not** mean changing the final-alignment speed target to approximately 189 m/s. The calculated 189.33 m/s value is only the speed that the present failed state's preflare equation would require if all other terms—especially its large sink, response time, target AoA, drag, and path—were artificially held fixed. It is diagnostic evidence of the size of the deficit, not a new guidance target.

The upstream post-HAC/HAC-exit guidance should instead be corrected so that the vehicle naturally arrives at the existing final-alignment contract with a viable combination of altitude, speed, FPA, sink, and attitude.

## Rerun Criteria

The 120-second final-only validation should be rerun only after a newly recorded post-HAC state satisfies all of the following before engagement:

1. The state is genuinely recorded from ShuttleSim, not synthetically constructed as a success case.
2. It is airborne, near the final-alignment station, near runway centerline, and aligned with RW09.
3. Replaying the exact Cartesian state under the current aero model reproduces its force and kinematic state closely enough to be trustworthy.
4. The runway-capture envelope is valid and reachable.
5. The production preflare plan is feasible.
6. Predicted kinetic margin remains non-negative against the 85 m/s minimum safe speed.
7. The terminal delivery contract is valid and feasible.
8. No recovery, replan, stale-plan, or lost-capture condition is active at admission.
9. Only after those checks pass should the continuous 120-second final-alignment -> TAEM final-intercept -> Final -> preflare -> touchdown -> rollout validation begin.

## Validation Artifacts

The detailed machine-readable result is stored at:

`Analysis/FinalApproach/2026-09-22-posthac-final-validation.json`

The exact replay fixture is stored at:

`ShuttleSim/scenarios/recorded-posthac-final-align-20260922.ini`

The selected diagnostic run is stored under:

`ShuttleSim/runs/final-only-selected-diagnostics-20260922/`

The archive admission summary is stored at:

`ShuttleSim/runs/final-only-archive-admission-20260922.json`

## Conclusion

The post-HAC final validation does not fail because the shuttle cannot capture the runway line. The recorded state has adequate instantaneous lateral authority, heading capture capability, runway time, local vertical recovery, q-bar margin, and load margin.

It fails because it arrives at final alignment with insufficient energy for the finite-response preflare maneuver. The production gate correctly blocks TAEM-to-Final handoff before control begins.

The next engineering target should therefore be the upstream HAC-exit/post-HAC energy delivery state. Once that state arrives at the final-alignment station with sufficient retained energy and a shallower descent, the same isolated final-only harness can be used to perform the requested 120-second validation through touchdown and stable rollout.
