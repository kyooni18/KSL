# Waiting-aware deorbit planning and 150 m/s mission budget

The user requested that the shuttle wait for better orbital geometry and that the deorbit maneuver remain inside 150 m/s. This repair treats 150 m/s as the total propulsive budget, including both normal correction and retrograde braking, rather than as a limit on either projection separately. No further flight, quickload, engine activation or burn was initiated.

## Evidence

The stopped flight `runs/20261102T144457Z-dessert-handoff-live` selected 863.56207 m/s normal plus 34.82828 m/s retrograde, 898.39035 m/s total. Its ideal orbital miss was still 434.84 km and its downstream capture miss 318.92 km. That was an unresolved targeting iterate, not evidence that such a large plane change was necessary.

The old planner calculated 901.96 m/s from vessel mass, structural dry mass and Terrier Isp. Structural dry mass excludes resources the OMS cannot burn, including the RCS monopropellant. It was the wrong burnout mass for this maneuver.

After stopping the owner and confirming throttle zero, a read-only standard kRPC inventory queried each Terrier's propellants, ratios, reachable amounts and resource densities. Both engines reported the same shared LF/Ox pool, with tiny differences while RPC samples were collected. Taking the smaller reachable amount per resource avoids double counting shared tanks and conservatively handles differing feed pools. Mixture-limited usable mass is:

`min(available_i / ratio_i) * sum(ratio_i * density_i)`

The recorded stopped inventory has 4,075.43 kg OMS-usable propellant at 25,631.60 kg vessel mass. Burnout mass is therefore 21,556.17 kg, not structural dry mass 19,965.50 kg. Remaining ideal OMS delta-v is **585.86 m/s**, versus the incorrect **845.21 m/s** dry-mass calculation at the same recorded mass. This is remaining fuel after the interrupted burn, not a fabricated initial-state budget.

The resource API's available amounts follow engine resource-flow rules; density is kg per resource unit. References: [official Propellant source](https://github.com/krpc/krpc/blob/main/service/SpaceCenter/src/Services/Parts/Propellant.cs), [official Resources source](https://github.com/krpc/krpc/blob/main/service/SpaceCenter/src/Services/Resources.cs).

## Changes

- OMS telemetry now derives a burnout mass from the Terrier feed pools and mixture. It rejects inconsistent mixtures/resource sets and fails closed if the inventory cannot be measured. Shared availability is never summed once per engine. Fuel inventory is refreshed at arming, after planning, and checked against the actual proposed maneuver.
- The existing `PropulsionTelemetry.dryMass` field retains its encoded name for compatibility, but new live telemetry supplies OMS burnout mass, including non-OMS resources and unusable mixture remainder. Historical records using structural dry mass remain suspect; they are not silently reclassified as correct measurements.
- `DeorbitConfiguration.maximumBurnDeltaV` defaults to 150 m/s, including when old JSON omits it. The operator key is `guidance.deorbitMaximumDeltaV`. Search bounds, candidate admission, plan authorization and final arming enforce the mission limit in addition to the smaller physical fuel budget.
- `DeorbitWindow.propulsiveDeltaV` counts separate legs as `abs(normal)+abs(retrograde)`. A legacy fixed-angle burn is priced as its total impulse, `abs(retrograde)/cos(angle)`.
- The live controller integrates **total measured thrust acceleration**, independent of its normal/retrograde projections. Remaining mission budget bounds throttle over the control/snapshot horizon; telemetry-gap impulse uncertainty spends the same budget. Reaching the budget stops with an incomplete-burn error, rather than falsely reporting completion. Total measured impulse and its limit are logged. Existing timeout/reconnect behavior and vehicle/RCS limits remain unchanged.
- The orbital search already spans later passes and, for this inclined orbit, a planetary rotation. That physical horizon is retained. Distinct coarse seeds must now occupy different orbital revolutions, with the existing passage-spacing condition also retained.
- Every selected passage first contributes a time-refined **retrograde-only** window. Plane-correction options cannot consume that portion of the existing candidate budget. Corrections use only remaining slots; unresolved independent-normal solves outside the configured orbital capture radius are recorded as rejections rather than admitted as costly fallback maneuvers.
- Complete qualified routes prefer lower propulsive delta-v, then earlier burn time. Failed routes retain safety/feasibility/altitude/Mach priorities; position is compared at the configured capture resolution, then fuel and wait time break comparable results. Resolution buckets keep sorting transitive. The same downstream comparison survives reciprocal-runway selection. This is an advisory ranking, not a relaxed capture tolerance or a landing certificate.
- A first successful but expensive nominal continuation no longer stops examination of the remaining configured candidate budget. No extra candidate count, flight campaign, or synthetic vehicle capability was introduced.

## Validation

Five focused methods pass: three new checks cover the genuine OMS inventory, rejection of the recorded 898 m/s plan under 150 m/s, and fuel/wait ranking; the existing authorization and independent-normal execution checks also pass. The independent-normal regression now retains the mission limit rather than assuming its former 360 m/s correction remains permitted. Its bounded example commands approximately 115.80 m/s normal plus 31.45 m/s retrograde; orbital miss remains about 66 km, explicitly unresolved. Neither that model result nor the policy test proves a landing.

The ranking tests change candidate metadata only; no vehicle, atmosphere, propulsion or actuator capability is scaled. The actual inventory is retained in `Fixtures/oms-propellant-inventory-20261102.json`. The over-budget plan is retained as a genuine rejection fixture. Logs and the raw read-only inventory are in `Validation/OMS-Waiting-20261102/`.

No new live test has validated this repair. The next plan must be generated from the then-current measured orbit or an explicitly reloaded checkpoint; the interrupted normal burn already changed the orbit. A future live attempt still requires explicit authorization. No new successful landing or suitable within-budget landing window is claimed.
