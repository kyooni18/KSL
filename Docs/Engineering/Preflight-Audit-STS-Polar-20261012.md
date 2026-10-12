# STS polar full-chain preflight audit — 2026-10-12

Scope: KSL `sts-polar` launch, OMS trim, deorbit, MM304, forced MM305, Final and rollout. **No live flight was performed during this audit.** At the final preflight probe the STS was intact, on the pad (`pre_launch`), and no KSL flight controller was running.

## Evidence from the three preceding live launches

| Run | Last reached | Observed issue |
| --- | --- | --- |
| `20261012T113529Z-live-full-fix-20261012` | Orbit / OMS trim | Master throttle fell to zero while ET remained attached; MECO after intervention, then OMS root was infeasible |
| `20261012T114958Z-full-retry-20261012` | Orbit / OMS trim | The zero-master-throttle defect recurred; external throttle recovery let the craft reach orbit handoff, but the OMS planner rejected a second-burn root before the first burn |
| `20261012T120721Z-full-third-fixed-20261012` | Ascent, 19 km | All three SSMEs changed to `active=False` at ~6.1 km, with LF/Ox unchanged and the SRBs still burning; after SRB separation the vehicle lost thrust and exceeded the 45-degree nose-attitude abort threshold |

Independent KSP evidence: `KSP.log` reports `EngineIgnited=False` for three separate SSME part IDs at **21:10:12.404–.405 KST**, corresponding to the third flight's simultaneous core shutdown. The event is also observed in Luna Multiplayer's (`[LMP]`) field log, but **that log does not establish whether LMP, another plugin, a KSP action group, or an external control input originated the change**. No normal KSL ascent code commands a three-engine shutdown at 6.1 km.

## Repairs and constraints

- The previously committed master-throttle fix (`55a6f42`) prevents a zero throttle from latching during post-SRB powered ascent. The native engine `thrust_limit` allocator still controls target acceleration; global throttle is a G/Q hard governor.
- Detect an inactive SSME during ET-attached powered ascent. Restart only restartable, fueled engines, reject relight at high dynamic pressure with large attitude error, log an explicit restart event, limit attempts to two, and check delivered thrust after three seconds. This mitigates, but **does not diagnose the origin** of the simultaneous shutdown. Failure terminates powered flight rather than silently continuing on SRBs.
- Restore the structural failure contract: release control, create a KSP checkpoint, and pause KSP for inspection. This repaired a failing unit test that expected a pause.
- Do **not** require periapsis to be above Kerbin's atmosphere at MECO. The normal `sts-polar` target remains approximately 100 km apoapsis / 70 km periapsis. For ET-depletion handling, only a below-surface periapsis is rejected; atmospheric periapsis alone is explicitly permitted because OMS owns the subsequent correction.
- OMS planning (`55a6f42`) now flies the first resolved burn before planning the next; older code could reject a valid first rescue because a predicted second-burn root was infeasible.

## Reproducible offline checks

- `make test`: **all native C tests passed; Python 139 passed + 51 subtests**.
- Clean-rebuilt `ShuttleSim/build/shuttlesim` and reproduced the L5 MM305 standalone closed-loop scenario; complete, on runway, gear down, no crash, touchdown 66.129 m/s, sink 1.844 m/s, stop 1101.616 m. The prior test's exact golden 66.735/2.055/1068.915 did not match the current native+simulator executable pair. Updated the golden to this independently repeated current-source outcome **without relaxing numeric tolerances or acceptance gates**. L7 low-energy flare replay also passed.
- `ksl validate --connect -p sts-polar`: all checks pass; KSP shows active STS `pre_launch`.

## Remaining live-only gates

1. On the next flight, watch the three SSME `active`, `has_fuel`, `thrust`, `thrust_limit`, and ET LF/Ox telemetry continuously through at least the historical ~6 km event. Capture any `ssme-auto-restart` event and actual thrust verification.
2. Confirm stable ascent and SRB separation, MECO/ET separation and the first *executed* OMS burn. An offline root test is not flight proof of a viable, finite-duration OMS rescue.
3. Measure the actual deorbit planning wall time; check forced MM304-to-MM305 handoff and Final/runway completion in the **live** KSP plant.
4. If unexplained SSME shutdown persists, correlate the 6 km event with the KSP/LMP log and action-group/module state before changing aerodynamic or flight-controller gains.

No launch, throttle write, staging, save load, or flight resume was initiated in this preflight audit.
