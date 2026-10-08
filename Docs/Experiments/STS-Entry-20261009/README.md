# STS entry diagnostics — 2026-10-09

These artifacts investigate live yaw-to-roll response and whether kRPC can produce an original-STS aerodynamic wrench grid without advancing the loaded vessel. They are diagnostic only. Neither artifact supplies a shadow-gate pass, qualified aerodynamic cells, or landing evidence.

## Yaw pulse and inertia

[`live-yaw-inertia-analysis.json`](live-yaw-inertia-analysis.json) recomputes body-rate slopes from the repeated vacuum yaw pulse in the 2026-10-08 raw capture and compares them with a pure body-yaw torque counterfactual using the measured full-vessel and root-part inertia tensors. Pitch and roll commands remained zero, and the bench recorded SAS/RCS off, zero dynamic pressure, and zero thrust.

The measured roll-to-yaw angular-acceleration ratio is about 0.385 for both pulse polarities. Inverting the full-vessel inertia tensor for a unit body-z torque predicts about 0.036, roughly one tenth as large. Off-diagonal inertia alone therefore does not explain the measured vacuum cross-axis response. This result does not isolate the actuator or control-frame cause, and it says nothing about atmospheric yaw/roll coordination.

## kRPC aerodynamic wrench grid

[`live-aerodynamic-wrench-grid.json`](live-aerodynamic-wrench-grid.json) evaluates 240 hypothetical states using `Flight.simulate_aerodynamic_wrench_at` on the loaded original STS. The grid varies Mach, angle of attack, sideslip, and bank while holding the directly measured live position and UT fixed. Each simulated velocity includes the body's rotational surface velocity so the requested Mach and air-relative angles match the intended values. The existing vessel, control surfaces, and action-group state supply the aerodynamic model; the probe issues no control or vessel-state setters.

At the captured state, the force and torque parity checks against live telemetry differed by 0.41% and 0.42%. The grid's achieved angle errors were below 0.00002 degrees. The live state remained paused and unchanged, and the named checkpoint hash remained `bb27c762add826731df3ad57332efd77d3d3a5d0928caa2ca361907ae0b1703f`.

This validates the API as a way to query counterfactual wrench values for the original vessel while preserving explicit input and checkpoint provenance. The retained grid is still only an upper-atmosphere slice at one position/UT (62.8 km, about 3.7–47.1 Pa). It is not a measured flight envelope or a lower-atmosphere certification set. A useful envelope needs additional altitude/UT slices and uncertainty checks against later live measurements; every simulated sample must remain labeled as unqualified diagnostic data.

## Live campaign

The 2026-10-09 wrench-grid probe connected to a paused original STS at UT 10086.871826 and mean altitude 62,836 m, not the campaign checkpoint. The vessel remained unchanged. Independent landing attempts must still reload `STS-campaign-72km-20261008`, retain the normal shadow gate, and verify touchdown and rollout before reporting success.

One normal live admission attempt after the no-speedbrake guard was built is recorded in [`live-no-speedbrake-admission.json`](live-no-speedbrake-admission.json). It restored and settled the named 72 km checkpoint, loaded 60 prior aerodynamic cells with the current attempt held out, and the unchanged shadow gate rejected reentry (4.5 km closest miss, +64.0 km range error, 5.3 kPa peak q, 1.63 g). Guidance never engaged, no descent began, and no live controls were issued. This matches the known rejection, so it was not repeated. Cleanup reloaded and paused the named save; the checkpoint file hash remained unchanged.
