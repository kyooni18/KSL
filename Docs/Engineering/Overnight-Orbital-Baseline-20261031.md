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

### Matched-state ranking experiment (offline, not retained)

At v21's measured UT 196417.088797553 (+402.94 s), we replayed the same
warm controller/authority history and compared the existing HAC family ranking
with a local experiment that evaluated every candidate to the end using the
existing `advisoryOnly` continuation before sorting. The experimental patch,
one-state probe, builds, timing and outputs are under
`runs/overnight-ranking-comparison-20261031/`; patch SHA-256 is
`cea214569b3b0fa0666761febe45dc3a53b3ffb224b67fab8c9a3e73e8aefc0b`.
The experiment was reverted after measurement; it is not a flight binary.

| Matched-state diagnostic | Existing | Full advisory pricing |
|---|---:|---:|
| Candidates / modeled feasible | 47 / 1 | 47 / 1 |
| Selected modeled feasible | no | no |
| Selected modeled maximum load | 31.571 g | 2.680 g |
| Selected modeled minimum lift margin | -302.38 m/s² | -17.77 m/s² |
| Debug planning wall time, one sample | 0.275 s | 0.887 s |

These are assessments of model references, not measured loads or closed-loop
captures. Warm-controller reproduction error was below 1e-13 in both runs.
The result supports the incomplete-pricing diagnosis but disproves the notion
that full pricing alone will select the known modeled-feasible family member:
the current fit cost still prefers an infeasible one. It also illustrates
substantial search cost and thousands of repeated station violations. The
existing advisory continuation imposes a landing-speed-derived energy floor,
so its downstream reference after energy exhaustion is not physical evidence.
Do not ship this experiment as a validated fix. A next controlled experiment
should compare consistently priced candidates using physically meaningful
violation severity and energy-exhaustion treatment, preserving a non-veto
fallback; measure planner latency and closed-loop tracking before live use.

### Planning proposal dynamics revision

The v21 epoch-equality issue was addressed independently of the ranking
experiment: a planning request now captures the dynamics revision *after*
configuration observation, and the worker carries it into the recorded
proposal. `FlightComputer` accepts only a matching revision; a proposal made
at exactly the dynamics-change UT can therefore be accepted if it actually
used the new configuration, while an older same-UT snapshot is rejected.
Historical proposals with no revision retain the prior conservative timestamp
rule. No bank, RCS, feasibility or target-limiter limit was changed.

`ShuttleSimAdapterTests` passed all five tests, including a new same-timestamp
old/fresh revision test, JSON round-trip and legacy rejection; the new test
took 66.65 s because it runs two real entry planner requests. A current-source
replay of the older v21 log reported 3,211 command/guidance mismatches out of
10,587 records both before and after this change, so this comparison only
shows no change to that aggregate legacy result; it is not command-identical
replay or live validation. V21 used different flown source and working-tree
patch. Revision behavior across a telemetry gap with unrecorded configuration
changes still needs explicit replay testing. The original orbital flight has
not exercised this fix.

### Reduced-assurance advisory policy change (not flight-qualified)

After the earlier final-state note, the explicit reduced-assurance path was
changed so a complete 15-case assessment is not vetoed merely because the
provisional model predicts failed routes or a load violation. The default
qualified path is unchanged; explicit reduced assurance remains required, the
report and all predicted violations remain available, and the orbital runtime
prints a warning before arming. Plan identity, current orbital window and
measured-orbit checks, propulsion validation, configuration matching, command
throttle bounds and existing configured flight limits were not relaxed.
This is policy alignment, **not** a claim that predicted structural violations
are harmless or that the selected route is safe. Assessment completeness still
gates reduced-assurance execution; the design does not yet handle an unfinished
or unavailable assessment as a fallback plan.

The six `DeorbitExecutionTests` passed on the current debug working tree,
including zero-pass and modeled-unsafe advisory cases, wrong plan identity and
incomplete-matrix rejection. This does not establish live recovery: orbital
RPC reconnect during coast or burn is still not implemented by `DeorbitRuntime`,
and the pre-existing KRPC recovery deliberately requires orbital rearm. Do not
confuse the changed model policy with a flight-qualified full-chain controller.
No new live burn, derived checkpoint, touchdown or stop was observed.

### Recorded-plan startup and repeated test suite

The experimental CLI now also accepts `fly --deorbit --deorbit-plan <json>`
to reuse a recorded computed plan rather than repeat the expensive planner.
The ordinary fresh-orbit/epoch, plan/configuration and propulsion checks still
run before arming, as does plan authorization. This option does not bypass
orbital reconnect requirements and has not flown. A debug build and help
dispatch succeeded; all seven focused `DeorbitExecutionTests` passed, including
a JSON-loaded plan's fresh state and rejected altered-mass, expired-window and
pre-plan-epoch checks. Actual kRPC rearm and mid-burn recovery are untested.

The full debug suite was rerun before this CLI/test addition and still exited
1: four `SimulationTests.EndToEndTests` failed with 14 assertions. Approach
step refinement differed by 147.55 m along, 1.56 m/s, and 0.66 m/s sink;
the entry-to-rollout model crashed after 1050 s of test computation; low-energy
returned timedOut instead of the legacy aborted expectation; and the 10% lift
loss model crashed. The low-energy expectation must be interpreted against the
current non-abandonment policy, not mechanically restored. Entry replay and
injected landing-runtime timeout/disconnect tests passed. Full output is
`runs/overnight-ranking-comparison-20261031/final-full-tests.txt`; this suite
did not include the subsequent loaded-plan addition, and its modeled results
are not evidence of KSP touchdown.

A later independent, read-only kRPC observation at UT 215514.3065 showed STS
orbiting at 85848.84 m ASL, surface speed 2086.71 m/s, throttle zero and 26
parts; UT advanced on a second sample. This was not a new flight or checkpoint.
The plan-loading route and advisory policy have not resolved the material
orbital reconnect gap. The user-created original save remains unchanged.

### Orbital reconnect prototype rejected before flight

A local prototype added a retry loop, neutralization on reconnect, and a
forward-gap reset; a fake-IO coast test injected a deadline failure and a
three-second gap and passed, as did the other seven deorbit tests. The debug
prototype compiled. This is **not a real kRPC reconnect test**. It was reverted
and is not included in the guidance commits. Its diagnostic patch is preserved
locally at `runs/overnight-ranking-comparison-20261031/orbit-recovery-prototype.patch`
(SHA-256 `4e2cf6e09240c54c98809618b0c534ca3af29d227194cb562abf0ab1c73db257`).

Review found two blocking problems. First, a timeout during OMS shutdown can
mean the engines actually stopped even though the client did not receive the
reply; the prototype then tries to rediscover *active* engines and can fail
to restore control. Reconnection must handle that ambiguous, idempotent
shutdown case while validating vehicle identity and keeping thrust neutral.
Second, the prototype adds a maximum possible unobserved impulse directly to
`deliveredDeltaV`; that is an upper bound, not a measurement. It can trigger
premature cutoff and mislabel an uncertain quantity as measured. A sound design
needs separate measured/uncertain impulse bounds and a recovery decision based
on fresh orbital energy/periapsis and actuator state, not invented history.
There is also no real transport-injection test or live mid-burn observation.
No safety or timeout limit was changed to make the fake test pass, and this
prototype was not flown. The orbital recovery gap therefore remains open.

### Follow-up candidate and original-save flight in progress

A second experimental recovery candidate separates measured impulse from an
upper bound on unobserved impulse, neutralizes controls on reconnect, rebinds
the original OMS references even when a shutdown response is ambiguous, and
records the gap for replay. It also retains configured RCS in orbital commands.
Nine focused deorbit tests passed, including a synthetic coast timeout and
command-identical replay and a separate mid-burn impulse-bound unit test; six
landing-runtime replay/failure tests passed. Neither test proves real KRPC
mid-burn recovery. Reconnection, engine identity and the conservative impulse
bound remain experimental; a large gap could cause an early deorbit cutoff.

The first launch of this candidate (`overnight-orbit-recovery-v2-20261031`)
was inadvertently interrupted when its launching shell exited: 67 logged
coast records, no burn or checkpoint. The original save was reloaded. The
new `overnight-orbit-full-v2-20261031` flight was launched using a persistent
background job, the original quicksave and the recorded computed plan; its
source patch, untracked probe, configuration and binary hashes are in its run
directory. It began in orbital coast with throttle zero and has a read-only
kRPC checkpoint monitor. Its final outcome must be determined from continuous
telemetry and process status; an initial coast is not a complete baseline.
The selected plan predicts zero full-route passes, with one modeled load
violation; this was explicitly reported under reduced assurance and is not
proof of an actual load. No landing is claimed here.

### Observed upstream flight failure: orbital RCS exhaustion

`overnight-orbit-full-v2-20261031` ran from the original quicksave with the
recorded plan and release binary SHA-256
`adfdf18e7034a58933551add65ee233c552cd7e7224e250548d2f56b7836d9af`.
Its flown source is the pre-flight HEAD `7dc1827` plus run `source.patch`
SHA-256 `38f3268311c578c41f557e40e3579957ab37abd30c7beb4f852c55e596c9b920`;
the later commit `1f3ac7e` captures the isolated candidate, not the other
pre-existing dirty work. Config and plan hashes are in the run directory.

The first record was UT 208579.3665. The commanded OMS throttle stayed zero
and the shuttle remained in orbital coast, but the attitude controller
chattered its RCS/reaction-wheel axes despite near-zero pointing error. At
UT 209205.2065 independent kRPC reported MonoPropellant **0**, versus 370.5224
in the original save; apoapsis had risen from 86323.23 m to 94066.16 m, with
periapsis 85436.11 m. ElectricCharge was 210, LiquidFuel 305.901 and Oxidizer
373.879. The configured RCS capability had been consumed before entry, and
the precomputed burn plan was no longer within its own fresh-orbit reviewed
dispersions. We cancelled at UT 209217.8065 to investigate and preserve the
remaining test opportunity. This is an **observed upstream resource/plan
failure and interrupted flight**, not an observed crash, deorbit, full-chain
baseline, or landing. No derived save was made.

An independent two-body propagation of the recorded plan source, using its
planet constants and one-second RK4 steps, compared with measured body-fixed
states. The deorbit settings were omitted from the old JSON and decode to the
current defaults (position 50 m, velocity 0.5 m/s, mass fraction 0.03); the
existing validator adds 1 m, 0.01 m/s, and 0.01 kg numerical allowances.

| First reviewed-dispersion crossing | UT | elapsed s | observed difference |
|---|---:|---:|---:|
| velocity | 208593.6465 | 14.28 | 0.512 m/s |
| position | 208634.0665 | 54.70 | 51.19 m |
| mass | 208721.5065 | 142.14 | 3.001% |

At +20–60 s the median alignment error was only 0.046°, yet pitch and yaw
commands were at least 95% magnitude in 76.0% and 58.7% of recorded samples;
mass fell at about 5.26 kg/s. In +60–120 and +120–180 s similar saturation
and loss persisted. After propellant depletion, commands mostly became small.
This strongly associates the actual waste with closed-loop actuator chatter,
not an MM304 bank sign. The controller's q-free torque model and discrete
actuator/transport response require matched-state identification before a
physically justified fix; turning off configured RCS or merely tuning a
deadband would mask the cause. `coast-analysis.py`, `coast-drift.csv`, summary,
and full telemetry are local to the run. Monitor startup initially failed due
to a relative working directory and was corrected before any descent; the
final event file contains no checkpoint saves. Fresh reload verification and
an original-orbit full-chain comparison are still required after a fix.

As a preliminary **correlated**, not isolated-actuator, response check over
+20–200 s, regressing successive measured body-rate differences divided by
sample interval against the preceding command gave slopes approximately
0.126, 0.052 and 0.049 rad/s² per unit roll, pitch and yaw command, with
uncentered correlations 0.746, 0.737 and 0.708. A two-sample lag gave slopes
0.132, 0.058 and 0.055, with correlations 0.781, 0.819 and 0.806. At +60 s
the reported minimum pitch authority was about 20523/896151 = 0.0229 rad/s²;
this is substantially below the correlated observed pitch response. These
numbers may include cross-axis dynamics, wheel/RCS interaction and timing
alignment; they are not an identification of isolated RCS or a calibrated
gain. They motivate a matched-state, delayed-actuator comparison and a
physically derived response controller, not a hardcoded compensation factor.

The actual vacuum controller (`OrbitalAttitudeController.boundedDemand`) uses
full opposite command whenever its computed stopping distance exceeds the
remaining error, even when both are very small; its one-hold predictive
fractional command only applies on the other branch. Combined with uncertain
minimum rather than identified total authority, sampled telemetry and actuator
latency, this is a plausible mechanism for a limit cycle, not proof of a unique
cause. The next discriminating experiment should replay the logged states and
commands with measured timing and independently characterize the effect of
wheel and RCS actuation, then compare a discrete-time stopping controller with
explicit actuator response and uncertainty. A small-error deadband or disabling
RCS would not establish or solve the underlying plant mismatch. Check remaining
propellant and plan-dispersion evolution during a controlled original-save
comparison before attempting the long coast; independently validate any
mid-burn reconnect with actual transport interruptions and ambiguous shutdown.

### Isolated original-save orbital actuator pulses

A subsequent **diagnostic, not an autonomous flight** independently reloaded
`quicksave` for each of six short positive single-axis pulses, with no other
controller running. The `runs/overnight-orbital-actuator-identification-20261031/`
`pulse-v2.py` and JSONL contain the commands, UTs, resources and telemetry. The
script explicitly disabled SAS, held throttle zero, and tested wheel-only and
wheel-plus-RCS cases separately; it neutralized all axes and disabled RCS at
the end of each test. RCS was disabled only for this isolated identification,
not as a proposed mission-control change. These were normal control inputs,
not altered physics or landing evidence. The untouched original save was
reloaded for each pulse. The first `pulse.py` iteration measured angular
velocity relative to the rotating vessel frame and therefore returned zero;
those angular-rate readings are invalid. Version 2 measures angular velocity
in the body's nonrotating frame and transforms the vector to vessel axes.

Each nominal pulse was 0.4 s; actual logged command intervals ranged from
0.38 to 0.42 s (commands and rates are separate RPC calls, not atomic). Table
entries are the change from mean pre-pulse to mean post-pulse angular rate in
the corresponding raw vessel axis; sign follows kRPC axes rather than FRD.

| Positive axis pulse | wheel only Δrate rad/s | wheel + RCS Δrate rad/s | RCS case mono consumed |
|---|---:|---:|---:|
| pitch | -0.00935 | -0.03376 | 0.35638 |
| yaw | -0.00863 | -0.03630 | 0.41440 |
| roll | -0.04960 | -0.10941 | 0.42422 |

The wheel-only responses are broadly consistent in scale with the reported
20,000/915,375, 20,000/949,934 and 10,000/83,574 rad/s² respectively.
Enabling RCS substantially increased the observed response and consumed
propellant on all three axes. At the RCS-enabled pulse start, the kRPC reported
RCS torque components were approximately positive (5307, 0, 3825) and
negative (-520, 0, -3825) in raw vessel axes; the control-authority logic
uses the minimum positive/negative torque. In particular, the reported
near-zero roll RCS torque does not describe the substantial additional roll
response actually observed in this experiment. The estimates are pulse
integrals, not a calibrated linear actuator model: command timing differs by
up to one telemetry interval, torque may vary with direction/attitude,
translational RCS effects and cross-axis dynamics are not separately modeled,
and these initial pulses tested only positive commands. Still, they independently confirm
that wheel-only minimum torque is not the actual combined plant authority,
and strongly support plant mismatch as a contributor to the observed chatter.
They do not validate a production controller or establish a safe resource
budget for the long coast.

At UT 208590.0465 after the final pulse, fresh kRPC showed STS orbiting,
throttle and all three command axes zero, RCS off and 370.09818 mono remaining
in the **loaded test state**. This is not a saved checkpoint; the original
quicksave remains unchanged. Before the next flight reload the original and
restore configured RCS through the controller. The next experiment should
measure response latency and smaller commands, fit an uncertainty-aware
combined-actuator model, and validate a sampled-data controller against the
actual resource consumption and plan drift. Do not treat these six pulse
samples as grounds for a hardcoded gain or for disabling RCS.

A further six negative-command pulses (`pulse-negative.py` and JSONL) also
started from the original quicksave. Pre-to-post angular-rate changes for
wheel-only / wheel+RCS were +0.00751 / +0.03557 rad/s pitch,
+0.00864 / +0.03213 yaw, and +0.04226 / +0.11474 roll. Command durations
were 0.34–0.40 s, varying with sequential-RPC sampling; RCS cases consumed
0.41574, 0.36047 and 0.44162 mono respectively. Thus increased response with
RCS is observed in both directions, but these short and variably timed pulses
are insufficient to determine precise asymmetry or closed-loop stability.

Finally, `pulse-small.py` repeated wheel+RCS pulses at ±0.2 command, again
reloading the original for each. For negative / positive commands, angular
rate changes were +0.00752 / -0.00700 rad/s pitch, +0.00639 / -0.00674 yaw,
and +0.02297 / -0.02176 roll. Command intervals were 0.38–0.42 s and mono
consumption was 0.0701–0.0871 per pulse. These are approximately proportional
to the full-command response and fuel usage at this state, subject to the
sampling caveats above; they are useful evidence that fractional input can
produce fractional authority here, not proof of global linearity. Fresh kRPC
after the last small pulse at UT 208581.9265 reported STS orbiting, throttle
and command axes zero, RCS off, and 370.44287 mono in the loaded test state.
No flight or derived checkpoint was created by these pulse tests. The original
save was not rewritten. Foundation Memory was searched again but no Foundation
capability was available in the tool catalog.

### Offline pointing-law hypothesis and rejection

A local, **not flown and not merged**, exploratory one-axis simulation is in
`runs/overnight-orbital-pointing-prototype-20261031/`. Its fixed combined pitch
plant uses the measured positive-pulse rate change divided by command interval
(0.08441 rad/s² per unit), while the original controller receives the
wheel-only minimum 0.02185 rad/s². The synthetic vehicle starts 0.3 rad off
its target at zero rate, has a 0.1 s sampling interval and a 600 s simulation;
there is no moving target, rigid-body coupling, trim, resource model or
independent measured trajectory. The original stopping law's full reversal
can persist at effectively full command in this toy plant (one-sample delay:
599.1 s integral of absolute command), qualitatively resembling the observed
coast oscillation. This synthetic reproduction does not prove causality.

A smooth candidate used a horizon derived from acceleration, remaining angle
and assumed sample/actuator latency, plus least-squares acceleration learning.
With exact knowledge of applied commands and acceleration, its one-sample-delay
absolute-command integral fell to 2.43 s, but that ideal observer was given
knowledge unavailable in the flown logs. A follow-up (`stress.py`) instead
paired the prior issued command with the next measured rate increment and
assumed one-sample latency; independent Gaussian rate perturbations were scaled
to the observed 6.43e-6 rad/s standard deviation of a short wheel-only
post-pulse interval. This is only a noise proxy, not a sensor specification.
With actual two-sample delay the integral rose to about 88.8 s and estimated
pitch authority fell to about 0.0759; with three-sample delay it rose to
275 s and authority fell to about 0.0601, versus the synthetic true 0.08441.
These results expose latency/identifiability sensitivity, not realistic fuel
predictions or a validated new control law. The prototype was rejected for
production and no release binary was changed. A credible next step needs
aligned applied-command/acceleration evidence, uncertainty over delay and
control authority, moving-frame tracking, cross-axis effects and matched-state
closed-loop validation before a controlled live comparison.

A separate least-squares fit (`fit-delay.py`, `fit-delay.txt`) integrates the
logged sample-and-held pulse command and fits a constant angular acceleration
and time offset to the measured body-axis rate. Across full-command pulses the
wheel-only fitted acceleration magnitudes were 0.0221–0.0223, 0.0215–0.0216,
and 0.1175–0.1183 rad/s² for pitch, yaw and roll; wheel+RCS magnitudes were
0.0842–0.0891, 0.0844–0.0864 and 0.2863–0.2874. The ±0.2 pulses gave
0.0869–0.0894, 0.0805–0.0841 and 0.2732–0.2867 respectively. Best-fit time
offsets were 0.02–0.05 s and individual fit R² values exceeded 0.9996, but
this is a single pulse with fitted intercept and offset, **not** independent
validation of the control response. The recorded UT is read before setting
the command and the rate is read in later RPCs, so the fitted offset conflates
RPC ordering, game tick sampling, and actuator delay. It must not be treated as
a physical latency measurement or inserted as a production constant. A
coherently timestamped command/telemetry experiment with varied pulse widths
and transitions is needed to separate those effects.

### Simulation step-refinement investigation

A debug native probe (`runs/overnight-simulation-refinement-investigation-20261031/`)
ran the unchanged nominal `.approach()` simulation with 0.020, 0.010 and
0.005 s maximum physics steps; guidance remained at its configured 0.1 s
interval. All three model runs completed with model-acceptable runway contact,
but touchdown along / horizontal speed / vertical speed were respectively
369.73 m / 57.60 m/s / -1.55 m/s, 517.28 m / 56.04 m/s / -2.22 m/s,
and 375.18 m / 57.56 m/s / -1.56 m/s. Thus the 0.010 s result is not between
the other two and the failing refinement assertion reflects a genuinely
non-monotonic closed-loop result, not simply a changed contact acceptance
flag. These are simulations, not live landing evidence.

The full control logs show repeated switches between identified and provisional
pitch trim evidence (19, 13 and 19 switches respectively). At UT 1034.0,
for example, the 0.020 and 0.005 s cases used identified feedforward about
0.201/0.200 with integral trim about -0.101/-0.102, while the 0.010 s case
used provisional feedforward 0.117 and integral trim -0.006. Transfer attempts
to preserve the combined command; nevertheless different switching times and
state histories can create divergent later commands. The first logged pitch
command difference over 0.01 was at UT 1034.0, and the first target AoA
difference over 0.01 rad at UT 1042.9. At UT 1074.4 the 0.020 s case had
entered preflare while 0.010 s was still in approach. These are observations
and a lead, **not** proof that trim switching uniquely causes the final
147.55 m discrepancy. A controlled matched-state replay freezing or separating
trim-source changes is needed before altering the model or the test.

The probe executable SHA-256 is
`fb88f634a24d5f3e328e01a3aa1705e413b72ac209e5594577f84b34cc57f64c`;
its Swift source, logs, results, source diff and hashes are in the run folder.
It initially linked existing debug objects. An incremental debug rebuild of
the preserved current source followed by relinking and repeating all three
runs reproduced each touchdown event exactly; the second executable hash and
build output are in the run folder. The first attempt to repeat was rejected
by the JSONL writer because the original logs existed; they were retained under
`original-*.jsonl` before the successful repeat. The substantial pre-existing
dirty work was not reset or included in this report commit. The other failing
simulations remain as previously reported.

#### Trim-source counterfactual

As an offline discriminating experiment only, a debug build temporarily forced
`identifiedTrim` to nil, leaving the provisional trim prior and the rest of
the controller unchanged. The experiment patch, executable SHA-256, build and
three logs/results are under the refinement run's `no-identified-trim/` folder.
The source was restored byte-for-byte and the debug product rebuilt afterward;
the release binary and live controller were not changed. No flight validation
is implied and disabling identified trim is **not** a proposed fix.

| Maximum physics step | Baseline touchdown along m | Counterfactual along m | Counterfactual horizontal / vertical m/s |
|---|---:|---:|---:|
| 0.020 s | 369.73 | 319.19 | 52.66 / -1.55 |
| 0.010 s | 517.28 | 322.35 | 52.61 / -1.60 |
| 0.005 s | 375.18 | 344.09 | 52.92 / -1.68 |

The 0.020 vs 0.010 s modeled discrepancy shrank from 147.55 m to 3.15 m in
this counterfactual, supporting a causal role for the identified-trim path
or its interaction with the rest of the controller. It also changes the
trajectory and the 0.005 s result remains 21.74 m from the 0.010 s result;
this is not a matched-state isolation of the exact defect, proof of full
convergence, or evidence that removing measured trim is correct. The next
experiment should replay identical measured state and trim history across
source-selection transitions, examining continuity and model support rather
than weakening the test or suppressing useful identified data.

A further log-only check reconstructed the existing approach/flare trim-settled
conditions (absolute AoA error <0.5° and AoA rate <0.3°/s) and compared
successive body pitch-rate differences to the reported pitch authority.
Of 236, 198 and 234 matching samples in the 0.020, 0.010 and 0.005 s runs,
respectively, 8 in each exceeded 3% of reported authority; maxima were about
18%. Most qualifying samples did not exceed 3% (median ratios 0.25–0.42%).
These finite differences are sampled dynamics, not a direct measurement of
net trim moment, and the reported authority is provisional in this model.
The existing trim learning criterion may therefore admit some non-equilibrium
samples, but this check neither shows they caused the divergence nor justifies
adding a new threshold. It is a useful target for matched-state examination.
