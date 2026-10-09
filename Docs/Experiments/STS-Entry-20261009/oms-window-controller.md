# OMS and prompt deorbit planning changes

The human requested that the ongoing 86 km flight continue while the next controller is corrected. The live executable remains pinned to its original campaign artifact. Changes below are compiled into `/tmp/sts-oms-window-backend`; they have not executed a live burn.

## Findings from the actual implementation and live inventory

The old orbital search unconditionally expanded its horizon to a planetary rotation, even when a circular equatorial orbit repeats useful ground-relative geometry after approximately one pass. Its comparator preferred any smaller predicted miss before waiting time. This explains searching later revolutions for marginal geometric improvements.

The old engine inventory included every fueled, throttleable engine, and the actuator activated them all when throttle was requested. A read-only inventory of original STS shows three SSMEs at about 1 MN each and two explicitly titled Orbital Manoeuvring Engines at 30 kN each. Their thruster positions and directions are retained in `oms-engine-inventory.json`. The OMS pair has a measured canted resultant axis and an off-center thrust moment. No flight commands were sent by the inventory probe.

## Implemented changes

* Circular equatorial search uses its orbital/ground-relative repeat period. Inclination-induced lateral excursion or eccentric apsis span greater than the configured capture radius retains the longer rotating-site search. No STS orbit altitude, inclination, or waiting time is baked into this decision.
* Within the existing configured geometric capture radius, the comparator prefers the earlier burn. It still reports downstream qualification independently and does not enlarge any gate or tolerance.
* After selecting the nearby pass, the planner refines a normal component of the finite burn when it reduces predicted entry miss. Every probe re-solves delta-v for the configured periapsis. The angular search retains at least as much retrograde as normal thrust (45-degree geometric bound). The chosen normal angle is carried through nominal and stress simulations, plan JSON, commanded acceleration direction, and projected delta-v accumulation.
* Engine roles come from explicit OMS/SSME tags or part metadata. Recognized SSMEs are excluded. STS/STS-N profiles require recognized OMS engines; missing identity cannot silently fall back to an unidentified launch engine. Generic vessels preserve generic engine selection when no OMS/main role is identified.
* Global throttle stays zero. Selected engines receive independent throttle; excluded engines have inherited independent throttle explicitly suppressed and are never activated by this path. Reported planning thrust uses the selected engines' resultant rather than whole-vessel available thrust.
* Runtime inventory uses selected thruster directions and positions relative to the vessel reference frame to measure the resultant axis and moment. A tested rigid-rotation transform maps the canted engine axis onto the requested acceleration vector, with an explicit roof/up orientation. This compensation is applied to preburn/burn attitude commands, not entry attitude capture. Counter-canted components reduce the reported net acceleration authority.

Off-center thrust torque is recorded, but this change does not prove that OMS gimbal/RCS authority can cancel it through an actual burn. Rigid attitude rotation cannot eliminate an off-center moment. A future live OMS-only burn must verify alignment, torque authority, achieved periapsis, and cutoff dynamics; the previous all-engine burn does not validate this implementation.

## Focused validation

The production backend builds with warnings treated as errors. `EngineRoleTests.c` verifies metadata identity and launch-engine exclusion. `ThrustAlignmentTests.c` verifies that a canted, laterally asymmetric body axis maps onto the requested inertial force vector, preserves orthogonal nose/roof axes, and reduces to the original attitude for an aligned engine.

`DeorbitWindowTests.c` exercises the actual internal orbital probe and comparator. A circular 86 km equatorial test orbit has period 1899.7 s and search horizon 2083.3 s. The cheap search selects an OMS burn at 584.9 s with predicted geometric miss 6927.3 m. Inclined and eccentric cases retain the longer horizon. A 5-degree normal burn changes entry cross-track by 302.2 m while re-solving achieved periapsis to 50000.0 m. These are offline orbital calculations, not atmospheric recovery or landing validation.

The ongoing live flight entered MM304 at UT 14084.8, altitude 68.9 km, range 1133.7 km. At UT 14226.0 it remained active at 58.5 km, range 864.5 km, AoA 25.6 degrees, with no MM305 admission yet. Do not restart or replace it to apply the new orbital code mid-flight.
