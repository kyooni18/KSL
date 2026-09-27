# MM305 live handoff investigation — 2026-09-28

## Scope and setup

Live KSP testing used `Configuration/live-mm305.json`, the named
`STS-N-MM305-handoff` save, and the guarded headless `--hac-only` path. The
checkpoint starts near 21.6 km and 764 m/s, so these runs exercise MM305 route
acquisition through Final but do not qualify a full MM304 reentry.

The independent crash guard remains enabled. `KSP_LANDER_UNPOWERED_ONLY=1` was
required by the live launcher. No save or control action was attempted by the
telemetry-only connection probe.

## Live evidence

Three repeated runs from the same checkpoint (one normal trace and two
diagnostic traces) reached MM305 ownership, committed the 12 km HAC route, and
handed a runway-aligned state to Final. They did not establish a flyable
touchdown: the guard restored the checkpoint during steep Final descent each
time. The third run is recorded in
`FlightLogs/2026-09-27T18-57-28Z-STS-N-{vehicle,planner}.jsonl`; its runner
trace is `Runtime/Headless/mm305-finaltrace-repeat.log`.

The first recorded run is in
`FlightLogs/2026-09-27T18-39-29Z-STS-N-{vehicle,planner}.jsonl`; the next two
are in the 18-49-16Z and 18-57-28Z vehicle/planner logs. The latter two share
the pinned executable and live profile in campaign identity
`b3c802204458831c-718c8ac914d1c07e-23edce430a42f1af` under
`Runtime/Headless/acceptance-artifacts/`.

Representative measurements from the diagnostic run:

- MM305 entered from the checkpoint at 21.6 km and 763.6 m/s. The vehicle
  tracked the selected route, replanned as the measured drag residual changed,
  and reached Final with a runway-line error below 12 m.
- The Final gate was accepted around 4.2 km altitude and 9.2 km from the
  runway. The live approach then steepened. Near 1.3 km wheel height and 3.9 km
  from the runway, speed was 157.5 m/s, sink was 78.9 m/s, commanded AoA was
  3 degrees, and the aim point was about 1.35 km short.
- The new opt-in Final trace recorded a valid refreshed preflare plan with a
  591 m planned trigger. The local pull calculation was about 659 m plus 4.7 s
  response time, putting the live latch near 1.03 km at the measured sink.
- At 1,197 m radar altitude and about -79 m/s vertical speed, the independent
  guard initiated recovery. It quickloaded while the native backend was
  polling, which reset the kRPC scene and produced the subsequent
  `fast telemetry batch procedure 641 failed` fault. The runner restored and
  froze the named checkpoint; a read-only probe confirmed it was paused.
- A separate shadow Final trace reached its modeled ground at about 1.8 km
  short of the runway. This is a prediction from a shadow trajectory, not a
  live touchdown observation, but it shows that the downstream model itself
  does not support treating contract readiness as a landing pass.

## Interpretation and next work

The backend RPC error is a consequence of the guarded scene reload, not evidence
of a spontaneous transport failure. The live approach had not reached its
planned pull-up point when recovery began. These runs therefore prove MM305 can
commit its route and meet the current Final admission contract, but they do not
prove that the admitted state survives through Final or touchdown.

The measured Final entry is already steep (about -25 degrees) at 4.1 km and
9.2 km from the runway. The controller later follows a moving aim point that
drives the commanded path toward -31 degrees, while the preflare latch remains
below the guard boundary. This points to route/energy shaping and Final path
geometry as the first issues to resolve; simply moving the latch earlier is
not yet justified by the shadow projection, which still misses short. Keep the
independent recovery guard enabled; evaluate changes against runway range,
pull response, and touchdown energy, then repeat live testing. Do not treat
the contract-ready log or the guard's recovery as a landing success.

Foundation Memory and Jev tools were not exposed in the working session. This
file preserves the confirmed measurements until those tools are available.

## Guarded repeat with same-UT AoA command preservation

After changing the AoA slew limiter to retain its previous command on a
same-UT evaluation, a fourth guarded live run used the same checkpoint,
profile, and HAC-only settings. The run is represented by
`FlightLogs/2026-09-27T19-59-12Z-STS-N-{vehicle,planner}.jsonl` and campaign
identity `e7e9ba28486911ff-97b2d3fd98c52ab3-23edce430a42f1af`.

- It passed below the former 1,200 m guard recovery point and continued to
  radar altitude 2 m near runway along-track 1,243 m and cross-track 11 m.
  At that point the flight log had true airspeed 65.9 m/s, sink 5.8 m/s,
  pitch 2.2 deg, and AoA 7.2 deg. It did not report `landed`.
- The Final trace showed requested AoA reaching the 15 deg ceiling while the
  measured AoA remained about 7–9 deg through the low flare. The flight log
  showed a sharp pitch-rate event near the end; the terminal monitor initiated
  attitude recovery with only a few metres of radar clearance, so the existing
  insufficient-height guard ended the attempt in Abort. The named checkpoint
  was restored and a read-only probe confirmed it was paused and safe.
- This is progress in descent arrest and runway alignment, not a landing pass.
  The control departure that triggered recovery is not yet identified. Repeat
  with `KSP_LANDER_HAC_DIAGNOSTICS=1` before changing recovery thresholds or
  terminal control gains; retain the independent guard.

The new AoA limiter behavior is compiled, but this run does not isolate its
effect from the existing low-altitude pitch/flare tuning. Use the recovery
diagnostic to identify the trigger and compare requested versus measured AoA
and pitch response before selecting a control change.

## Recovery-trigger diagnostic repeat

A fifth guarded run repeated the same profile with
`KSP_LANDER_HAC_DIAGNOSTICS=1`. Its detailed runner trace is in
`Runtime/Headless/mm305-aoa-fix-recoverydiag-live.log`; its flight records are
`FlightLogs/2026-09-27T20-05-26Z-STS-N-{vehicle,planner}.jsonl`.

- This run reached runway altitude near along-track 1,273 m and cross-track
  11.5 m, but it was a physical crash, not a touchdown: at 1.48 m radar
  altitude the logged mass fell from 43.5 t to 12.3 t and then 4.6 t. It never
  reported `landed`. Immediately before the break-up, speed was 63.4 m/s,
  vertical speed -4.15 m/s, pitch 3.81 deg, and AoA 7.57 deg despite a
  14.79 deg AoA command. The final trajectory trace reported about 3 m/s sink
  at zero modeled wheel height, but that value alone did not predict structural
  survival.
- The control diagnostic identified the recovery trigger: pitch rate spiked to
  53.09 deg/s at the same 1.48 m radar-altitude sample; `extreme=1`,
  `pitch=1`, and `emergency=1`. The subsequent Abort for insufficient recovery
  height occurred after the mass-loss event. Thus recovery was reacting to the
  impact/break-up transient, not causing the crash.
- The independent guard did not fire before the collision because sink had
  fallen below its low-altitude sink threshold. The named save was restored
  afterward; a read-only probe again reported a paused, flying STS-N at the
  saved 22 km checkpoint.

The same-UT AoA command change still needs an isolated before/after comparison.
The present landing failure is at least partly a touchdown-geometry/energy
problem: requested incidence is not reached, actual pitch is below the
8-degree touchdown target, and the shuttle breaks up at contact at only about
63 m/s. Do not suppress the pitch-rate recovery or infer success from zero
modeled height. Before changing contact gates, determine the gear contact
attitude and structural limits in KSP, then change the upstream energy and
flare control that produces a survivable actual state.
