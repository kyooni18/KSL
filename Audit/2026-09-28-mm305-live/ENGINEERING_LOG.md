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

## Rejected profile and flare experiments

Two isolated configuration trials lowered the glide slopes to the built-in
defaults (Final 20 deg / TAEM 12 deg) and to Final 20 deg with TAEM held at 24
deg. Both repeated live checkpoints failed to qualify any native HAC join and
aborted near 1.1 km before committing the route; the recorded reasons were
energy-infeasible vertical profiles or insufficient live curvature authority.
The configurations are kept only as ignored Runtime experiment files; the
checked-in live profile remains at 28/24 deg.

A separate live trial changed `FINAL_FLARE_ACCEL` from 0.1g to 0.05g. It still
reached main-wheel contact near 2.77 m radar altitude at about 66 m/s and
-2.58 m/s, then sink worsened to -3.62 m/s. KSP reduced vehicle mass from
43.5 t to 43.4 t and then 12.3 t, followed by an impact recovery trigger.
This did not produce a survivable touchdown and was reverted. The exact trial
is `Runtime/Headless/mm305-finalflare-05g-live.log` with vehicle records
`FlightLogs/2026-09-27T20-27-55Z-STS-N-vehicle.jsonl`.

Both successful-route landing attempts show the full 8 deg touchdown attitude
command arriving too late for the observed pitch response: measured pitch was
about 1–2 deg at first main-wheel contact even though the AoA command was near
15 deg. The next focused test starts the existing smooth 8 deg attitude floor
at 400 m wheel height rather than 120 m, with the 0.1g final sink profile
restored. No contact or crash checks are being bypassed.

The 400 m attitude-ramp run was stopped by the independent guard at 66.3 m
radar altitude and -13.75 m/s vertical speed. The vehicle log at recovery had
pitch -2.91 deg, AoA 8.05 deg, and a 15 deg AoA command; the early attitude
floor did not produce the commanded attitude or reduce sink before the guard.
That ramp change was reverted. The run is represented by
`Runtime/Headless/mm305-touchdown400-live.log` and
`FlightLogs/2026-09-27T20-33-47Z-STS-N-vehicle.jsonl`.

A moderate slope profile (Final 24 deg / TAEM 20 deg) did qualify a route and
reach Final, but the live Final trace still followed about a -26 deg outer
glide. Near the runway it reported 70 m/s, -8.4 m/s sink at 15 m wheel height;
the final diagnostic then saw -73 deg/s roll rate at 3 m radar altitude and
aborted for insufficient recovery height. It did not report `landed` and the
guarded runner restored the saved checkpoint. The isolated profile is
`Runtime/Headless/mm305-final24-taem20-profile.json`; its trace is
`Runtime/Headless/mm305-final24-taem20-live.log`.

This shows the config slope changes do not yet change the live Final capture
geometry enough to improve delivery. The 400 m attitude floor likewise did
not make actual pitch follow the command; its guard sample was pitch -2.91 deg
at AoA 8.05 deg versus a 15 deg command. Both experiments are rejected. The
next test keeps the checked-in profile and varies only the already-supported
live pitch-loop bandwidth override to measure whether faster AoA tracking
raises actual pitch and lowers sink before main-wheel contact.

The supported live pitch-loop override at `KSP_LANDER_PITCH_WN=1.30` was not
safe: at first main-wheel contact (2.72 m radar altitude) actual sink was
-5.82 m/s, pitch 2.11 deg, and AoA 7.32 deg versus a 14.99 deg request.
Flight control released pitch on wheel contact, mass fell from 43.5 t to
43.4 t and then 12.0 t, and the recovery diagnostic later saw a 61 deg/s
pitch-rate spike. The attempt aborted; the guard restored the checkpoint.
Trace and vehicle log:
`Runtime/Headless/mm305-pitchwn130-live.log`,
`FlightLogs/2026-09-27T20-46-54Z-STS-N-vehicle.jsonl`.

The bandwidth override is rejected. The baseline Final trajectory also
repeatedly hit the `terminal_geometry_fpa` -25 deg clamp. The next guarded run
tests capping that nominal Final glide at -20 deg while preserving the same
route admission, touchdown gates, and recovery guard.

The first -20 deg geometry-only cap did not constrain the live path: Final's
underspeed correction adds up to 12 deg of dive after the geometry calculation,
and the actual trace still showed about -25 deg. It reached the runway region
but aborted after an impact-rate event at 2 m radar altitude, with 64 m/s
speed and 3.2 m/s modeled sink. It did not report `landed`; the checkpoint was
restored. The next experiment applies the same -20 deg lower bound after speed
correction as well, so the speed loop cannot recreate the steep trajectory.

Applying the -20 deg bound after speed correction held the live outer reference
at -20 deg, but the descent still did not arrest: the independent guard
quickloaded at 251.6 m radar altitude with -31.69 m/s vertical speed and
102.8 m/s TAS. The live vessel never contacted the runway. This profile change
is reverted; a shallower Final target by itself is not enough when the MM305
handoff is too close/low-energy. The attempt is logged in
`Runtime/Headless/mm305-final20-combined-live.log` and
`FlightLogs/2026-09-27T21-00-34Z-STS-N-vehicle.jsonl`.

A final-approach-distance trial increased `guidance.finalApproachDistance` to
11,000 m while leaving the 28/24 deg glide slopes and all gates unchanged.
The live vessel remained in MM305 acquisition and never qualified either
reciprocal HAC candidate; recurring reasons were `fixed-HAC candidate is
infeasible` and `finite lead exceeds live curvature authority`. It reached
1.1 km altitude at 65.5 m/s and aborted because no qualified route remained
before the height needed to fly one. This profile did not commit a route or
reach Final, so the distance increase is rejected. The runner restored the
named save, and the follow-up guard probe confirmed STS-N flying at the 22 km
checkpoint with KSP paused. Run: `Runtime/Headless/mm305-final11k-live.log`;
profile: `Runtime/Headless/mm305-final11k-profile.json`.

A live 8,500 m final-approach-distance trial qualified and committed a native
route, then reached Final. Its route was later replanned to a 9,000 m final
station. At first main-wheel contact, KSP telemetry showed 2.77 m radar
altitude, 64.6 m/s TAS, -3.2 m/s vertical speed, 0.7 deg pitch, and 3.6 deg
AoA. Vessel mass fell from 43.5 t to 43.4 t at contact and to 12.3 t on the
following sample; the next sample was 4.6 t with a 46 deg pitch / 45 deg AoA
breakup transient. The controller aborted at 2 m radar altitude for a control
departure. It did not complete a sustained landing. Guard recovery restored
STS-N to the paused 22 km named checkpoint. Run and evidence:
`Runtime/Headless/mm305-final8500-live.log`,
`FlightLogs/2026-09-27T21-13-23Z-STS-N-vehicle.jsonl`.

This trial confirms the touchdown-attitude request still does not produce the
required contact pitch, while the vehicle arrives below the configured 75 m/s
touchdown speed. The next isolated live experiment raises only the final
alignment speed target, with the 8,500 m route and all touchdown, contact, and
recovery gates held fixed, to test whether retaining more terminal energy
improves contact speed and actual pitch.

Raising `finalAlignmentSpeed` from 160 to 180 m/s at the same 8,500 m route
profile made both HAC candidates infeasible throughout acquisition. The vessel
fell from 763.5 m/s at the 21.6 km handoff to 63.4 m/s at 1.1 km while route
search alternated between energy-infeasible profiles and insufficient live
curvature authority. MM305 aborted before route commitment. All gates remained
active; the guard restored the named save, and the subsequent probe confirmed
STS-N paused and flying at 22 km. Run: `Runtime/Headless/mm305-final8500-speed180-live.log`.
The requested target is too aggressive for this route; the next check uses an
intermediate 170 m/s target to determine whether the planner can still qualify
a route while preserving more approach energy.

The intermediate `finalAlignmentSpeed=170` m/s test also failed to qualify a
route at the same 8,500 m final-distance setting. At 1.1 km the vehicle was
moving 67.6 m/s, and both candidates failed on energy-feasible vertical
profiles and/or live curvature authority. It aborted before route commitment;
the guard restored the checkpoint, and the probe again confirmed a paused
flying STS-N at 22 km. Run: `Runtime/Headless/mm305-final8500-speed170-live.log`.
The 170 and 180 m/s station targets are rejected. The next experiment keeps
the 160 m/s alignment target and raises the touchdown-speed target from 75 to
85 m/s, testing whether earlier terminal speed scheduling can preserve the
energy needed for a survivable contact.
