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

To distinguish orbital pointing/burn propagation from atmospheric evaluation,
`Tools/probe_orbital_screen.swift` also replayed the **nominal model only** for
all 12 returned windows using the baseline initial attitude, measured vacuum
authority and propulsion. All 12 modeled burns completed and reached a
descending interface; modeled FPA was approximately -1.65° to -1.67°. All
12 took 22.21 s wall time in the debug native probe, versus the full-route
evaluation still running after 19 minutes. This is not a measured burn, a
stress matrix, a predictor validation, or permission to bypass safety. It
corroborates that the expensive atmospheric closed-loop evaluation—not the
orbital search or the nominal orbital propagation—is the bottleneck for this
state. Output and timing are in `orbital-probe.txt` and
`orbital-probe-time.txt`; probe binary SHA-256
`4df59c9abf0e86692d2041a11cbd6bb905ef9bcbfb1f40ec5ee61d25d52065c8`.
The experimental CLI now reports the existing planner's stage, candidate and
stress-case progress to stderr; this improves observability but does not speed
the evaluation, change authorization, or prove that a candidate will execute.
The revised source built in debug; no new live flight was performed with it.
Rebuild the debug modules and probe, then run against the preserved screen:

```sh
swift build --package-path swift/KSL-Landing -c debug --product ksl-landing
B=swift/KSL-Landing/.build/out/Products/Debug
swiftc -parse-as-library -I "$B" -I swift/KSL-Landing/Sources/CSQLite \
  swift/KSL-Landing/Tools/probe_orbital_screen.swift \
  "$B"/{KSLCore,KSLControl,KSLPhysics,KSLGuidance,KSLDiagnostics,KSLRuntime}.o \
  -lsqlite3 -o runs/overnight-deorbit-screen-20261031/orbital_probe
runs/overnight-deorbit-screen-20261031/orbital_probe runs/overnight-deorbit-screen-20261031
```

## Completed qualification attempt, still no flight

Run `overnight-orbit-qualified-attempt-20261031` reloaded the original save
and used commit `b8f1921` plus the recorded pre-existing working-tree patch.
At UT 208576.2265 the active STS was orbiting at 86308.45 m ASL, 2084.09 m/s
surface speed, with apoapsis/periapsis 86323.13/85776.66 m. The configuration
hash is `7d562b97f847432c84b3c911a22cab7b0315cd314d445579c62edb357f913a45`,
source-patch hash `8496a2a268555801be45e52c9ad6bbf729253c391dc531963abaede09df48474`,
and release-binary hash `52d3c96889c01562b414518cd1f9ef4aa35a5eea39115ce451171c114126f093`.
The run directory also contains the separately copied untracked transition probe.

After approximately 100 minutes, the existing full-route planner returned a
complete report for plan `E4524116-D9DA-4A59-AC70-BB2AD6D7EA03` (JSON SHA-256
`0e759311ded77d1499216859c5e64eddb88eb6f88a4893d734f8e9ef7772d103`). Its
selected screen predicts burn at UT 228885.1291; this is **not** a commanded or
observed burn. All 15 modeled cases completed the orbital burn, but **zero**
modeled routes passed; each reported failed ground contact or runway
containment. Case 14 reported 3.618 g peak load, above the configured 3.5 g
maximum (modeled q 32047 Pa versus configured 45000 Pa). The other 14 were
not marked hard-unsafe. These are provisional offline-model results, not
measured impacts or validated proof of real vehicle limits. The pre-existing
authorization contract rejected the plan with nominal=false, 0/15 passes,
one modeled hard-unsafe case; the CLI exited before arming, and the flight log
remained empty. This is a **planning/authorization outcome**, not an observed
deorbit, trajectory failure, or landing. At UT 214258.2665 independent kRPC
showed STS still orbiting, ASL 86215.75 m, speed 2084.19 m/s, throttle zero,
RCS off. The checkpoint observer made no saves and was stopped.
The focused existing `testExplicitAuthorizationIsBoundToPlanAndCompleteMatrix`
passed on the later debug build; it tests the current authorization contract,
not whether that contract satisfies the requested advisory policy.

This establishes why the explicit path still cannot provide a full-chain
baseline: full-route model verdicts are a pre-burn execution veto, despite
their stated limitations. A safe redesign must reconcile the user's advisory
policy with real structural/actuator limits, measured orbital authority,
current-window freshness and orbital RPC recovery; treating 0/15 provisional
passes as proven live failure or silently deleting real limits would be wrong.

### Nominal offline trace of that rejected plan

The `deorbit-trace` diagnostic reproduces the nominal orbital plant and
synchronous native flight computer from the plan, logging at one-second
intervals. It is an offline analysis, not a flight. It took 209.14 s in the
release build. One-second sampled peak q was 2706.52 Pa and peak load 0.87049 g,
consistent with the plan's nominal 2706.52 Pa and 0.87052 g (full-rate peak).
The trace's binary SHA-256 is
`7ac1d331bf1e700e5f7b2c45736481a4e7c1962f01a95001db894c513801e402`, and
`nominal-model-release.jsonl` SHA-256 is
`4bdf843f7290ab33818f4ea6161a80f96fa772be03427d0d0df2f707e8a7f013`.

| Modeled UT | modeled phase | ASL m | airspeed m/s | q Pa |
|---:|---|---:|---:|---:|
| 229356.07 | entry | 69497 | 2114.6 | 0.4 |
| 229800.17 | entry | 37646 | 1660.1 | 1987.2 |
| 229892.17 | acquisition | 28937 | 835.0 | — |
| 230000.17 | acquisition | 17655 | 184.8 | 1088.7 |
| 230200.17 | acquisition | 7540 | 50.9 | 553.8 |
| 230400.17 | acquisition | 1860 | 35.3 | 564.5 |

The modeled final contact at UT 230482.556 was off runway, along +36026 m,
cross -42022 m, horizontal speed 23.37 m/s and sink -21.32 m/s; the plant
labeled it crashed. The acquisition persisted and energy collapsed long before
contact. This describes **only the provisional offline model**, with its
unvalidated high-q attitude/turn model and synchronous planning behavior; it
does not establish what would happen in KSP or identify the physical first
point of no return. The one-second trace and final contact summary remain in
the run directory.

The same spherical runway frame reconstructed from that sampled model shows
the early modeled delivery problem more directly: at UT 229800.17, MM304 was
along +201105 m and cross +18631 m; at the first acquisition sample, UT
229892.17, it was along +98042 m and cross -22708 m, descending 92.6 m/s,
with approximately 40° actual and target AoA. At UT 230200.17 it was still
along +44134 m and cross -41068 m, descending 34.6 m/s at approximately
40° AoA and 50.9 m/s airspeed. Those are **modeled states**, not measurements;
they suggest an early energy/delivery problem in this predictor but do not
resolve whether the live shuttle would behave that way. The along/cross numbers
are relative to the configured runway-27 threshold, not the site marker.

Reproduce with the preserved plan and a unique output path:

```sh
swift/KSL-Landing/.build/release/ksl-landing deorbit-trace \
  --input runs/overnight-orbit-qualified-attempt-20261031/live.jsonl.deorbit-plan.json \
  --log runs/overnight-orbit-qualified-attempt-20261031/nominal-model-replay.jsonl
```

The prior v21 investigation and its uncommitted native probe remain separate;
its MM304/MM305 findings have not been verified on this orbital attempt. The
next experiment is to obtain a computed, current, executable orbit plan, with
observable planning progress and safe handling of stale plan epochs and RPC
reconnection, then re-fly the original quicksave through the entire chain with
derived checkpoint saves and continuous evidence. Preserve plan qualification
and physically based limits; do not silently bypass them to manufacture a flight.

### Final state and focused test

At 07:31 local on 2026-10-31 all six `DeorbitExecutionTests` passed on the
current debug working tree, including the existing model-verdict authorization
tests. This does not change the earlier full-suite failure or establish live
reconnect or landing behavior. The authorization veto remains in place and
conflicts with the requested advisory-only policy; it is not a demonstrated fix.
An unqualified attempt to change it and reuse a saved plan was not retained:
freshness, configuration binding and orbital reconnect require a coherent design
before flight. No `ksl-landing fly` process was running at the final check.

The existing dirty predictor and investigation work remains uncommitted. The
dirty `EntryEnergyProbe.swift` and its CLI dispatch in `Main.swift` also contain
an unfinished offline comparison experiment; neither is a flight-tested fix.
Foundation Memory tools were not discoverable in this session's available tool
catalog, so this report is the durable record rather than a claimed memory write.
