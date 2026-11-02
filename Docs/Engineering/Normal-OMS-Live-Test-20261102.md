# Normal OMS implementation: live attempts — 2026-11-02

Both authorized live attempts failed to land on the selected Dessert runway. The independent normal-impulse controller completed its requested maneuver in the first attempt. The later user-requested restart selected a legacy fixed-angle burn and also ended off-field. Neither is landing qualification.

## Provenance and ownership

Both flights used committed source `3d7aab71e6ed6e45b068640ff408177503e0743b`, a frozen copy of the release executable plus its physics resource bundle, and an explicit configuration. Each run retains `provenance.json`, executable/resource/configuration hashes, `source.patch`, `command.json`, full-rate `live.jsonl`, the selected native plan, actual selected-runway configuration, stdout/stderr, and `exit.json`. No executable was rebuilt or replaced while in use. No guidance or vehicle settings were changed during either trajectory.

First run: `runs/20261102T114658Z-dessert-normal-oms-live`.
Restart: `runs/20261102T122809Z-dessert-normal-oms-restart`.
Compact results and log/configuration hashes are in `Validation/Normal-OMS-20261102/live-results.json`.

The first controller was stopped before the user's explicitly requested `SpaceCenter.quickload()`. No save was written by the agent. The reloaded orbital checkpoint differed in epoch, mass and orbit from the first live initial state; these are not matched-state A/B flights. The restart used the same frozen binary/configuration and planned again from its fresh measured state. No automatic third attempt was started.

## First attempt: normal-first maneuver, Runway 18

The planner selected +206.012936 m/s normal followed by 32.034003 m/s retrograde. Both orbital and full-route evidence remained visible: the nominal landing continuation failed, while the orbital hard-unsafe flag was false. The saved nominal continuation predicted off-runway contact around along -18.4 km/cross -183.4 km.

Measured burn closure at UT 213468.337:

- Normal delivery 206.012953 m/s; retrograde delivery 31.595605 m/s.
- Periapsis 50,305.67 m ASL; throttle zero; 26 parts.
- No telemetry-gap impulse uncertainty or thrust-loss event during the burn.

MM304 began at UT 213966.157, altitude 69,492.79 m ASL, speed 2,106.20 m/s. Incoming great-circle ground-track miss was 12,159.61 m, compared with 11,523.12 m in the ideal window. Threshold range was 1,015,789.47 m. The ideal window and controlled execution have different timing; the raw ideal-window range alone is not a matched-command error metric.

The vehicle crossed the extended centerline at UT 214518.877, along -22,800.23 m/cross -99.17 m, altitude 32,357.38 m ASL and speed 1,464.02 m/s. Thus the overshoot was already developing while MM304 still owned control. TAEM began at UT 214624.937, altitude 24,328.75 m ASL, speed 649.25 m/s, along -28,674.58 m/cross -105,603.02 m, threshold range 109,426.83 m.

TAEM ran its normal law and retained the runway-braking advisory; the earlier missing-NEP control blocker was absent. It reported energy and lift/trim deficits and did not regain the runway. Contact occurred about 157.1 km from the threshold. The last record before the user's restart request showed a stopped vehicle with 26 parts at approximately sea level. The controller was cancelled before the checkpoint reload. This was an off-runway failed attempt, not a landing success.

## Restart: fixed-angle maneuver, Runway 36

After the existing quicksave loaded, native preflight found intact orbital STS at UT 215274.892, altitude approximately 88.94 km, inclination approximately 5.20°, mass 26,065.08 kg and throttle zero. The fresh planner selected Runway 36 with a legacy 11.25° fixed-angle burn: 34.489541 m/s retrograde-projected target, no explicit normal-first leg. The new candidate family was available but did not win selection. The nominal modeled landing was again failed, with predicted contact around along +11.9 km/cross +82.9 km.

A transient zero-inertia optional-telemetry report was rejected and recovered before ignition. The measured burn ended with 34.664547 m/s retrograde impulse, periapsis 50,244.60 m ASL and throttle zero. There was no thrust-loss fault or unobserved impulse bound during execution.

The closest logged threshold approach occurred in MM304 at UT 216410.051: range 33,004.61 m, altitude 27,554.52 m ASL and speed 917.81 m/s. TAEM took over at UT 216422.151: altitude 26,680.61 m ASL, speed 735.43 m/s, along **+33,356.97 m**/cross +8,144.47 m, range 34,336.86 m. This was on the opposite along-runway side from the requested before-threshold delivery objective. A return HAC is permitted, but the actual recovery reference failed its energy/trim assessment.

The recovery did not reach the runway. First ground contact at UT 216952.891 was about 55,754.10 m from the threshold, at 31.62 m/s, with 26 parts. At UT 216955.331, after contact and a bounce back into the airborne/flare path, the part count dropped to 25. The final stop was about 55.65 km off the selected runway.

The controller was stopped only after 38.64 seconds of fresh, continuous stopped telemetry, not because a phase label implied success. Independent native postflight inspection at UT 217190.131 confirmed:

- Stopped contact, 25 parts, all three wheels reported; main and nose gear contact measured.
- Throttle/thrust zero, brakes on, gear down, RCS/SAS off.
- Runway along -7,025.64 m/cross +55,202.37 m; threshold range **55,647.65 m**.

The restart ended failed, with damage. Its native outcome is `cancelled` because the stopped off-runway controller could not legitimately report `complete`. No guidance process remains active.

## Overshoot evidence collected before the restart request

The user's initial investigation request produced a narrow two-state diagnostic from the first attempt, retained under `Validation/Normal-OMS-Overshoot-20261102/`. No vehicle capability was altered. The temporary diagnostic XCTest was removed from the build target afterward; no permanent compile-time test was added for this investigation.

All 6,394 MM304 samples in that first attempt had an unbracketed range reference. At UT 214300.117, the actual production range-shooting algorithm evaluated these endpoints at interface energy:

| Branch | Predicted along | Predicted cross | Along error to -35,023 m |
| --- | ---: | ---: | ---: |
| Best-glide incidence | -30,532.09 m | -127,093.71 m | +4,490.91 m |
| Maximum allowed incidence | -19,238.72 m | +61,426.09 m | +15,784.28 m |

The along-only residual in `EntryRangeReference.solve` chooses the first branch despite its much larger spatial miss. Neither endpoint brackets the along objective, so the code returns the closer endpoint without solving the delivery geometry. At UT 214480.077, the same choice preferred cross -101,693.26 m over -44,126.00 m because the former was closer along.

The selected incidence then determines the lift left for lateral steering. At the later probe, the command-selection point was 20° AoA, reserved lift 2.583 m/s² versus support 3.201 m/s², so its bank allocation was zero. Raw requested lateral acceleration was -20.923 m/s² and the full joint request was unattainable; another permitted polar point offered only 0.577 m/s² of supported lateral acceleration. The issue must be addressed through physically priced coupled range/energy/heading references, not a higher bank limit or a tuned turn gain.

At nearby logged UT 214470.117, the legacy reference drag was 11.683 m/s², while selected drag was 4.426 m/s² and measured drag 4.373 m/s². The actuator was following the selected low-incidence branch; the legacy drag reference was not binding that selection. These values do not establish that all of the reference drag was attainable. Nor do the counterfactual endpoints establish that either alternative would land. They identify why this guidance chose the overshooting branch.

No further production fix was made during the investigation or restart. The normal-impulse capability is implemented and physically exercised; MM304 delivery and TAEM recovery remain unqualified.
