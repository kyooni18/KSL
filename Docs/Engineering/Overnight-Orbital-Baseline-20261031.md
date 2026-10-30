# Orbital baseline campaign, 2026-10-31

## Result: no full flight or landing verified

The first actual orbital-start attempt exposed an upstream startup failure, not
an MM304/MM305 trajectory result. The documented `fly` command initializes
`FlightRuntime` in `.entry`; it does **not** plan or execute deorbit. We
interrupted it after observing the wrong phase and no OMS burn. This is not a
completed full mission and provides no landing, HAC, or checkpoint evidence.

## Save and provenance

The active game was `shuttletester`. Its user `quicksave.sfs`, copied without
modification to `runs/overnight-orbit-baseline-20261031/original-quicksave.sfs`,
has SHA-256 `350d430cef02ad41c8f374223a0b6c0691520f0ae01cdd50546d9b80a800d588`.
Its STS orbit metadata has SMA 686050.06 m, eccentricity 0.00039878, inclination
23.06287 degrees; after normal kRPC load, live apoapsis/periapsis were
86323.09/85776.67 m. The user save and old 72 km checkpoint were not replaced.
The game had two vessels named STS; the selected active vessel was the orbiting
one. At UT 208572.6065, live ASL was 86309.44 m, latitude 21.47745 degrees,
longitude -162.99570 degrees, mass 24847.13 kg; resources were 210 electric,
370.5224 monopropellant, 305.9581 liquid fuel and 373.9488 oxidizer. An
immediate post-reload speed sample was zero while the scene settled; the
controller's first recorded airspeed below is the meaningful moving sample.

Run `overnight-orbit-baseline-20261031`: HEAD `e6f1f37` plus the substantial
pre-existing dirty source and test changes; exact `source.patch` SHA-256
`212ca6813f02d06c8c0de0a088eb5231b58ecfdf9d72f8259d511afec7e83383` and
untracked probe copy are in the run directory. Runway-27 configuration SHA-256
`7d562b97f847432c84b3c911a22cab7b0315cd314d445579c62edb357f913a45`;
release executable SHA-256
`8fdfb683d168e632940cff4c3794ce2ea7f75433725578e864c15c0043319889`.
Release build succeeded. Logs, exact configuration, source status, build output,
and hashes are preserved locally in that run directory.

| UT | Observed phase | ASL m | airspeed m/s | throttle | thrust N |
|---:|---|---:|---:|---:|---:|
| 208569.2265 | entry (initialPhase entry) | 86310.60 | 2084.06 | 0 | 0 |
| 208597.3865 | entry | 86305.12 | 2084.11 | 0 | 0 |

298 samples span 28.16 s. The vehicle remained airborne; RCS was enabled by
the entry controller. No deorbit, 72 km descent, 1200 m/s checkpoint, runway
approach, touchdown, or rollout was observed. A diagnostic `fly --help` was
mistakenly used before the run; this CLI treats it as `fly`, not help. Its
process was stopped promptly before reloading the preserved user quicksave;
this was an unintended short control operation, not flight evidence. The
preflight telemetry helper also errored on an unsupported Python `vessel.id`;
the independently captured kRPC sample and native flight record above are
the actual evidence.

The CLI now dispatches `--help` or `-h` to help before any subcommand can
connect or arm. A debug build succeeded and `ksl-landing fly --help --host
127.0.0.1 --port 1` exited 0 with help output, without needing a server. This
is a startup safety fix only, not a deorbit or landing validation.

## Startup experiment and outstanding risks

An experimental explicit `fly --deorbit --reduced-assurance` connects the
existing `DeorbitPlanner` and `DeorbitRuntime` to the CLI, records the computed
plan, and retains existing plan authorization and fresh-orbit checks. It does
not force an old burn time or change vehicle physics. Its flight binary and
source patch are preserved in `overnight-orbit-deorbit-attempt-20261031`:
binary SHA-256 `32631d42f6d4985c3fafa16c0c99b6a64e32012ccf78550b2da640e84f17e78c`,
source patch SHA-256 `978a6a10c01fc81ce622f1239d686de2c621f6bcbcad1a5cd0a89e6892a73e38`.
This is an experimental startup route, not a verified guidance fix. The
full-route planner performs synchronous expensive trajectory qualification;
sampling its process showed `DeorbitPlanner.evaluate` inside the HAC planner.
The planner's modeled results are not evidence of a real landing. Its existing
authorization requires a nominal pass and no modeled hard-unsafe cases even
for reduced assurance; an infeasible report cannot be claimed as a flown
recovery. Check the run's `stdout.txt`, plan JSON, log, and process state for
the actual final result. After more than 19 minutes of CPU-active planning,
there was still no plan JSON, telemetry, or burn; two process samples showed
the nominal full-route evaluation inside HAC planning. It was terminated while
unarmed rather than left to initiate an unmonitored, potentially stale flight.
At UT 209610.0265, a fresh independent kRPC sample showed the active STS still
orbiting, ASL 85807.93 m, surface speed 2086.19 m/s, throttle zero and RCS off.
No derived checkpoint was created. This is an interrupted planning attempt,
not proof that no solution exists.

The existing `DeorbitRuntime` does not use `FlightRuntime`'s RPC reconnect
loop during orbital execution. The subsequent entry runtime does have it, and
TCP RPC timeout remains 10 s. This is an unresolved full-chain recovery gap,
not evidence of successful reconnect. An additional source change passes the
configured `--max-outage` through the orbital-to-entry handoff; it was not in
the binary identified above. Do not conflate it with the flown binary.

## Offline tests

Both release builds succeeded; a subsequent debug build of the handoff source
also succeeded. The full `swift test -c debug` run did **not** pass: the
SimulationTests suite reported 14 assertions failing across four tests,
including the entry-to-rollout simulation reporting `crashed`, and the step
refinement discrepancy. The entry rollout simulation took about 984 seconds.
These are offline results on the preserved dirty source, not causal attribution
to the small CLI experiment and not live-flight evidence. Replay/failure tests
including injected disconnect and timeout recovery passed. Full output is in
`overnight-orbit-deorbit-attempt-20261031/tests.txt`.

## Orbital screen timing follow-up

`overnight-deorbit-screen-20261031` extracted the first actual baseline
`VehicleState` at UT 208569.2265 and ran the unchanged native finite-burn
window search offline, without arming anything. On this host, release search
took 1.17 s wall time, evaluated 978 probes over a 21549.43 s horizon and
returned 12 candidates with no reported rejected probes. This isolates the
observed >19-minute delay to the downstream full-route evaluations, not the
orbital window search. It does not establish their eventual outcome. The
new `deorbit-screen` diagnostic emits the native candidates and explicitly
labels them *screening only*, not flight authorization.

| Screened burn UT | lead s | modeled delta-v m/s | modeled entry FPA | modeled entry crossrange m | modeled miss m |
|---:|---:|---:|---:|---:|---:|
| 228885.20 | 20315.97 | 30.89 | -1.67° | 32328 | 32427 |
| 218472.84 | 9903.61 | 30.91 | -1.67° | 59093 | 59252 |
| 216408.44 | 7839.21 | 30.79 | -1.68° | 91380 | 91666 |
| 210114.68 | 1545.46 | 30.72 | -1.68° | 208779 | 208939 |

These are selected examples, in search order except that the last row shows
the earliest of the returned windows. Search order and modeled geometry are
not proof of closed-loop execution, capture, safety, or an optimal route.
The nominally best screened geometry would require more than five hours of
orbital coast from this snapshot; the earliest returned window has materially
worse modeled crossrange. The full-route planner could choose differently.
No burn time or delta-v from this table should be copied into flight guidance.
The screening run preserves its input/config/source/binary hashes and JSON.

The prior v21 investigation and its uncommitted native probe remain separate;
its MM304/MM305 findings have not been verified on this orbital attempt. The
next experiment is to obtain a computed, current, executable orbit plan, with
observable planning progress and safe handling of stale plan epochs and RPC
reconnection, then re-fly the original quicksave through the entire chain with
derived checkpoint saves and continuous evidence. Preserve plan qualification
and physically based limits; do not silently bypass them to manufacture a flight.
