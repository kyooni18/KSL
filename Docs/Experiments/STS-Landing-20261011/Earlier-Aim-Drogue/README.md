# Earlier runway aim and drogue deployment

The user identified the shuttle's drogue and requested an earlier runway aim.
A read-only kRPC inspection after the third flight found one `radialDrogue`,
Mk12-R Radial-Mount Drogue Chute, still stowed. Its deployment altitude was
2500 m and minimum pressure 0.02 atm. Prior rollout evidence therefore did
not include drogue braking. The inspection did not deploy or arm it.

The STS live profile now sets `guidance.touchdownAimAlong = 150` m from the
selected runway threshold, replacing the default 15% runway aim (375 m on
this runway). Missing/zero retains the historical aim; no acceptance gates
change. `guidance.rolloutParachutes = true` enables native rollout deployment
through stock kRPC `Control.Parachutes`. This controls the vessel's parachutes;
the inspected STS has one drogue. Other profiles default off.

Deployment is requested once per session after confirmed main/nose wheel
contact (with telemetry availability) or a landed flag, gear confirmed down,
rollout control ownership and finite surface speed 5–100 m/s. An airborne
Rollout phase alone cannot deploy it. The native backend logs the request
with UT/speed. This does not stage the craft, use the speedbrake or arm the
chute during approach. A successful RPC request is not proof of inflation:
record chute state during the next approved flight. Reconnect resets the
request latch; KSP retains the actual deployed chute state.

## Offline result and limits

`bin/ksl simulate l7-measured-mm305 --label earlier-aim-l7 --no-color` uses the
unqualified measured-state L4-fit body plant. Compared to its prior aim:

| Aim | Contact speed | Sink | Along | Result |
|---|---:|---:|---:|---|
| 375 m | 65.616 m/s | 3.024 m/s | 1951.406 m | Failed contact/rollout |
| 150 m | 66.762 m/s | 2.989 m/s | 1920.117 m | Failed rollout; runway departure |

The commanded aim moves 225 m earlier but actual contact moves only 31 m.
Energy delivery and flare response remain open. The simulator does not model
parachute drag, inflation or deployment, and does not call the live deployment
RPC. Its failed rollout cannot establish the drogue's braking benefit.
[result.json](result.json) retains input/binary provenance and the contact
summary. Existing focused tests cover airborne rejection, contact availability,
valid deployment, invalid speed/profile and configuration round-trip. Build
and full tests pass; historical L5 regression remains unchanged. No new live
flight has been performed with these changes. STS remains paused landed.
