# KSL polar STS return: causal verification and checkpointed live test

Date: 2026-10-13 KST; branch: mm305-campaign-unpaused.
Status: the 06:39 live polar return was still underway when this report was opened. No Final capture, landing or rollout is claimed without recorded evidence.

## 1. Original frozen flight and first divergence

The original run, runs/20261012T155948Z-polar-offaxis-native-20261013/, was **operator-aborted** in MM305, not an automatic crash or a completed landing. Its associated native logs are FlightLogs/2026-10-12T15-59-49Z-STS-vehicle.jsonl and FlightLogs/2026-10-12T15-59-49Z-STS-planner.jsonl. Checksums and source identifiers are frozen in runs/20261013-offline-polar-investigation/audit-summary.json.

The earliest significant divergence is already measurable at 99 km, before appreciable aerodynamic load. The following comparison is at the SAME altitude (actual minus original predicted UT). The clock-excess column is a diagnostic based on observed displacement versus recorded UT; it is NOT substituted for UT or asserted to be a sole physical cause.

| Altitude km | Delta UT s | Clock excess s | Residual s | Actual KSC km | Original predicted KSC km |
|---:|---:|---:|---:|---:|---:|
| 99 | +14.117 | 0.000 | +14.117 | 1684.242 | 1662.527 |
| 95 | +19.216 | 0.000 | +19.216 | 1688.709 | 1712.338 |
| 90 | +21.175 | 0.000 | +21.175 | 1552.910 | 1594.623 |
| 80 | +24.398 | -0.001 | +24.399 | 1298.523 | 1351.012 |
| 70 | +30.491 | +2.107 | +28.384 | 1051.811 | 1113.570 |
| 65 | +66.952 | +35.422 | +31.530 | 917.740 | 986.412 |
| 50 | +89.583 | +42.458 | +47.125 | 397.651 | 497.764 |
| 28 | +191.517 | +154.962 | +36.555 | 10.756 | 86.417 |
| 27.5 | +195.837 | +158.380 | +37.457 | 5.804 | 80.843 |
| 15 | +262.019 | +233.759 | +28.260 | 41.391 | 21.317 |

Same-UT states and altitude-aligned position, vertical speed, FPA, ground track, AoA/bank, available commanded and achieved controls are in runs/20261013-offline-polar-investigation/same-ut.csv and same-altitude.csv. The round-trip planet-fixed coordinate closure was below 0.00021 m, ruling out a large transformation bookkeeping error in these reconstructed states. The original prediction did not record every force/control field; absent fields must remain unavailable, not fabricated.

At the observed MM304-to-MM305 handoff the shuttle was ~27.5 km high, ~833 m/s, only ~5.5 km from KSC. At the deliberate operator stop it was 14.35 km high, 285.7 m/s and 42.37 km from KSC, still without Final. A phase transition and proximity to KSC are not a feasible runway/HAC capture.

## 2. Causal replay, burn residual and genuine reachability

Original planned ignition: UT 146991197.250966. Recorded thrust onset relative to it: 1 N at +0.241 s; 100 N at +0.781 s; 1000 N at +2.321 s. The nominal burn duration was 117.75 s, but observed zero thrust followed its scheduled end by +20.871 s. The planned osculating postburn periapsis was 50.002 km and actual measured cutoff periapsis 51.477 km. A later episode at ~42.1 km logged approximately 59.2 kN of thrust despite a zero-throttle command, with 0.84 s reconstructed physical duration and ~1.98 m/s impulse per unit mass. This is why measured actuator thrust was brought back into the control loop.

Starting a frozen native replay from the MEASURED cutoff state, without substituting the idealized planned cutoff, gives KSC range 400.787 km at the 50 km gate, compared with actual 397.651 km. The original trajectory projection gives 497.764 km. The measured-cutoff replay therefore reduced the same-altitude range error from 100.113 km to 3.136 km (~96.9%). The first discrepancy is upstream of MM305; changing only the final HAC mode cannot eliminate it.

A controlled retrograde burn-closure replay adding approximately 1.143 m/s, and obtaining 50.301 km osculating periapsis, changed the 50 km gate KSC range from 400.787 to 443.754 km (+42.967 km standoff). Nevertheless the native replay still reported no physically qualified inlet, committed feasible HAC route or actual Final capture. The MM304 energy/lateral allocation and deorbit opportunity must deliver a genuinely reachable runway geometry, not just a positive admission flag.

## 3. Implementation and reproducible offline verification

Existing verified branch commits:

- ce349f9: frozen polar evidence, clock-aware audit and native causal replays.
- a3d4388: use measured throttle feedback and reassert zero throttle in unpowered flight.
- ab77688: measured periapsis closure overrides premature advisory cutoff.
- 9b79d1f: 72 km and 45 km descending KSP checkpoint support and regression tests.
- 122e024: repair the standalone KSP guard's project import resolution.

The off-axis premature Final-mode fix c5ae066 and best-effort safety policy d51617d must be preserved. Do not soften AoA/bank/G/stall/structural limits, disable real constraint checking, add speedbrakes, or treat a negative qualification as landing success.

Latest prelive make test: runs/20261013-offline-polar-investigation/prelive-make-test-9b79d1f.log; exit 0, **155 pytest tests and 56 subtests passed**. This is code-level/offline qualification, not proof of successful Final capture. The frozen replay outputs remain inspectable at runs/20261013-offline-polar-investigation/ and are NOT labeled physically qualified when Final=0.

Recreate the frozen diagnostic comparison (the output directory must not overwrite prior evidence):

```sh
.venv/bin/python Tools/polar_reentry_audit.py \
 --vehicle FlightLogs/2026-10-12T15-59-49Z-STS-vehicle.jsonl \
 --planner FlightLogs/2026-10-12T15-59-49Z-STS-planner.jsonl \
 --plan runs/20261012T155948Z-polar-offaxis-native-20261013/deorbit-plan.json \
 --output runs/reproduction-polar-20261013
make test
```

A local worktree modification to Tools/polar_reentry_replay.c and a pre-existing untracked Validation/.DeorbitNativeCandidateProbe.c are separately owned and have not been overwritten or included in these commits.

## 4. Live experiment initiated 06:39 KST, 2026-10-13

The experiment uses the same real 22-part STS airframe, a ~100 km orbit with 89.999 deg inclination, in the live KSP instance. The native run is runs/20261012T213949Z-polar100-live-20261013-0640/ and its paired vehicle/planner logs begin FlightLogs/2026-10-12T21-39-50Z-STS-*.jsonl.

**Save loader issue independently proven.** The archived polar checkpoint was in saves/LunaMultiplayer, while KSP's active campaign attempted to open from saves/Main. KSP.log explicitly said "No save file found" in Main for the prior attempt. The preserved checkpoint was copied under a unique new file name in Main with SHA256 equality verified. The independent save loader then returned load-save-settled at UT 146960927.357, altitude 100029.8 m, 22 parts and stable mass 25610.84 kg (instead of the just-loaded placeholder 22000 kg). The orbit was separately checkpointed as saves/Main/ksl-20261013T0639-live-polar100-orbit.sfs, SHA256 f024a17c4574112ea9d3bd09898e1889883d203128f9abd8d7a19f7fd5e83fb5. No prior save was replaced. For the standalone guard, use the local .venv/bin/python with krpc installed.

The live native planner selected a tier-1 ~42.2 m/s finite OMS deorbit targeting a 50 km periapsis, forecast MM305 inlet ~26.0 km altitude, 866 m/s and 67.5 km range, with a 3.5 degree course debt. Forecast qualification remained truthful: nominal=0, robust=0/15, recovery=0/15, executable best-effort=1. Predicted Final capture remained 0. The test was flown under those warnings rather than silently accepted or blocked.

At 06:51 KST the burn ended with **measured periapsis 50.283 km**, within +283 m of target. The earlier, nonidentical live return had +1477 m error. This comparison is not falsely reported as a same-initial-state controlled A/B trial. An independent read-only postburn kRPC probe showed throttle=0, actual thrust=0 N, mass ~25125 kg. At 06:57 the vehicle remained in MM304 pre-atmosphere prograde/entry-attitude capture, ~87.7 km high, descending ~72 m/s with zero dynamic pressure.

## 5. Completion checks and next physical decision

Pending actual flight record: 72 km and 45 km descending saves, position and speed versus the selected projection, achieved heading/bank/AoA under lift/drag loading, physical MM305 inlet and actual Final qualification, touchdown and rollout or precise failure classification. Complete only after checking the actual live data. The native planner's predicted HAC commitment or MM305 ownership is **never** sufficient for a landing claim.

If this trajectory fails to capture Final, the next justified live experiment is a single controlled return **from the new 72 km saved state**. Reproduce its measured attitude, energy, range and bank envelope offline first, then change only the demonstrated MM304 delivery/approach geometry mechanism, not arbitrary admission/Final flags. Preserve the achieved postburn trajectory and use the 45 km checkpoint for finer replay if available. No concurrent KSP controller and no new live flight started at or after 08:00 KST.

## 6. 72 km checkpoint, external controller interruption and deterministic restart

**07:02 KST:** The original live KSL run saved `ksl-20261012T213949Z-polar100-live-20261013-0640-descent-72km.sfs` at measured altitude **71,998.245 m**, UT **146970175.9372**. Both `descent-checkpoints.json` and the physical file in the KSP `saves/Main` folder independently exist. SHA256: `0a1b249392cc752d4e09a332775ae28a78a751c6ab2e71c68f4df51b862d0a92`.

At ~07:10:03 KST, the external Yeet detached shell execution abruptly disappeared after approximately 30 minutes, taking down the parent `bin/ksl land` and the native backend without running their normal mission completion/abort sequence. This is an **execution infrastructure interruption**, not a predicted Final failure or evidence that the MM304 controller itself elected to abort. KSP continued unpaused and uncommanded. The final original-controller observation was UT **146970362.3573**, altitude **57,774.12 m**, speed **2,278.74 m/s**, AoA **+26.96 deg**. The game was independently paused at UT **146970381.5973**, altitude **56,435.90 m**, mass **25,121.37 kg**, with a new physical checkpoint `ksl-20261013T0711-polar56km-controller-interrupted.sfs` (SHA256 `c0f512aa2a31a21acefd6a2a36e25e7aad35527b7ee419fc670d1d516a7ea1ac`).

A first continuation from that live state used a persistent `launchctl submit` service (instead of Yeet's expiring detached command) and `KSP_LANDER_CONTINUE_UNQUALIFIED=1`, preserving warning flags. Its first telemetry **before engagement**, UT **146970381.5973**, already had AoA **-65.26 deg** and roll **+6.98 deg**. This proves the large attitude departure preceded the new MM304 guidance; blaming reentry handoff for the initial -65 deg attitude is not supported. The continuation entered attitude recovery, but after 15 s it remained far outside the requested +18 deg AoA and warned of recovery-duration failure. It was explicitly paused for investigation and a preservation save `ksl-20261013T0713-polar-attitude-recovery-departure.sfs` (UT 146970419.2373, 53,119.64 m, AoA -42.70 deg, 22 parts, SHA256 `256ef8c2bcc00401be17c06429e18d9e5fa05488c07709cba4411880cd421feb`) was written. That controlled recovery attempt is not an observed Final/landing.

The resumed continuation process was terminated while KSP was paused; a unique already-captured **72 km** save was then explicitly reloaded, with verified scene and rewind. The generic KSP save loader initially accepted a placeholder mass **22,000 kg** for the atmospheric save, so no new guidance was allowed until an independent read-only kRPC probe found consistent physical values (~**25,123.02 kg** wet mass, **20,160.00 kg** dry mass, **22 parts**, **0 N** thrust). The instance was paused at UT **146970184.4772**, altitude **71,276.65 m**, AoA **+3.03 deg**, roll near **0.09 deg**, and the independent persistent service **`com.ksl.polar0715`** began native MM304 guidance in run `runs/20261012T221523Z-polar72-durable-0715/`.

The 72 km continuation's shadow recovery is still unqualified (predicted closest 181.8 km, range-error +85.3 km, peak q 4.8 kPa, peak g 1.35). The failure is **not overridden** by successful engagement. KSP was unpaused after controlled engagement at ~07:15:49 KST, from ~70.6 km. This service is independent of the ephemeral remote command-worker lease and is the **only live controller**. The earlier KSL execution directory remains preserved as interrupted evidence, not reclassified as a clean completion.

A future engineering fix should add an explicit controller-liveness watchdog/pausing fail-safe for command-worker termination and gate atmospheric checkpoint loads on whole-vessel mass/parts physics convergence, not only a scene/UT transition. These are distinct from aerodynamic and path-feasibility corrections, so neither should be hidden by relaxing Final admission.

## 7. Authorized 08:04 KST 72 km retake: independently verified checkpoints, repeated loss of roll control, and destruction

**Authorization and classification.** At 08:04 KST on 2026-10-13 the operator explicitly authorized continuing from the preserved 72 km checkpoint. This superseded the earlier instruction not to begin a new live flight at or after 08:00. The one-shot live continuation is `runs/20261012T230733Z-polar72-retake-0807/` (paired native logs `FlightLogs/2026-10-12T23-07-40Z-STS-{vehicle,planner}.jsonl`). It **did not** enter MM305, HAC, or Final, and it **did not** land on the runway. KSP recorded multiple destructive collisions roughly 393 km from KSC. The run's final software status `aborted` is a **post-destruction cleanup SIGINT**, not the cause of its physical failure or a user-requested midflight stop. Likewise, brief backend `Rollout` transitions reflect off-runway terrain contact, not successful touchdown and runway rollout.

### 7.1 Preflight, single-owner controls and preserved checkpoints

The prior `com.ksl.polar0715` launchd service had been repeatedly respawning on the already-grounded, paused, damaged STS. It and `com.ksl.polar45watch` were explicitly booted out before any new load/control operation. No concurrent live controller remained. The true checkpoint `saves/Main/ksl-20261012T213949Z-polar100-live-20261013-0640-descent-72km.sfs` was copied, never modified in place, to `ksl-20261013T0806-polar72-retake.sfs`; both SHA256 hashes were `0a1b249392cc752d4e09a332775ae28a78a751c6ab2e71c68f4df51b862d0a92`.

The independent loader confirmed scene/UT rewind but initially read the known placeholder **22,000 kg**. The measured-physics settle guard withheld guidance until UT **146970177.19720045**, altitude **71,891.96 m**, mass **25,123.0293 kg**, dry mass **20,160 kg**, **22 parts**, **0 N thrust**, paused at 1x warp. `runs/20261013T0806Z-polar72-retake/load-and-settle.log` and `physics-profile.json` retain the complete validation. The actual start was from MM304 without recomputing a new deorbit burn.

A unique one-shot `com.ksl.polar0807retake` controller was used (`KeepAlive=false`, avoiding indefinite relaunch). An independent `com.ksl.polar0807ownerwatch` parent-loss guard would freeze KSP if its owner disappeared. The alternative independent 45 km watcher blocked in kRPC and was taken down without changing flight controls; the native KSL checkpoint trigger remained armed. Later, after the physical crash, KSL was safely stopped with SIGINT, its owner watchdog independently verified `paused=true` at UT **146970933.277606**, and both launchd jobs were removed. No second flight was started, and the game was left paused.

Two additional physical saves were created and hash-verified in `saves/Main`:

- **50 km intermediate:** `ksl-20261013T0819-polar50-intermediate.sfs`, measured altitude **50,329.8884 m**, UT **146970462.97735375**, SHA256 `cee49ec884821bce177d7038b11f068f443340624d930a0e0759d3de5dd267c1`. Independent metadata at `runs/20261013T0806Z-polar72-retake/intermediate-50km-manual.json`.
- **45 km descending:** `ksl-20261012T230733Z-polar72-retake-0807-descent-45km.sfs`, crossing altitude **44,996.9527 m**, UT **146970516.537382**, SHA256 `a2328c026b3f8194cee54259d85b562732cf7934a8458fd19038461ec3d2da71`. Verified KSL event plus `runs/20261012T230733Z-polar72-retake-0807/descent-checkpoints.json` and physical `.sfs`. The 72 km checkpoint was correctly marked already passed on restart, **not** forged/backfilled, so 45 km could trigger.

### 7.2 Matched-altitude causal evidence: the physical delivery miss and unstable attitude are separate

Reconstructed raw vehicle keyframe/delta streams from the previous 07:15 retake and this 08:07 run at identical altitude. The inspectable comparison is `runs/20261013T0806Z-polar72-retake/matched-altitude-controller-comparison.csv`, with independently checked subsets in `retake-verification.json`. Flight position/timing were closely matched upstream of the upset: KSC range difference at 50/45/40/38/37/36 km was only **+0.162/+0.233/+0.190/+0.283/+0.249/+0.368 km**, respectively; arrival UT difference at 36 km was **-0.22 s**. Yet the actual roll with **target roll = 0 deg** diverged:

| Altitude | Previous roll | Retake roll | Previous applied roll axis | Retake applied roll axis |
|---|---:|---:|---:|---:|
| 50 km | +0.90 deg | +1.56 deg | -0.004 | -0.004 |
| 45 km | +1.37 deg | +2.86 deg | -0.006 | -0.006 |
| 40 km | +2.61 deg | +5.72 deg | -0.012 | -0.011 |
| 38 km | +3.96 deg | +9.24 deg | -0.021 | -0.019 |
| 37 km | +5.93 deg | +15.13 deg | -0.044 | -0.037 |
| 36 km | +11.14 deg | +20.67 deg | -0.080 | -0.048 |

This is **not** a controlled gain A/B: actuator inputs differed and live run initialization/time steps are not bit-identical. It does establish that the larger roll departure developed while the commanded bank remained zero, even though the gross position and energy trajectories remained almost identical. At around **36.06 km**, actual roll was ~20 deg, measured sideslip only about -0.41 deg, measured dynamic pressure ~2.43 kPa, reported roll input approximately -0.047, and yaw input around -0.93. Thus large sideslip is not a prerequisite for the earliest roll buildup. At **33.86 km** (08:25) roll was ~+30.6 deg and AoA ~+39.7 deg; the native controller entered `Attitude Recovery`. It warned after **15 s without stabilization** and continued best-effort commands without fictitiously certifying the approach. At **32.14 km**, actual roll reached ~-78.6 deg and AoA ~-61.5 deg; at **21.46 km**, speed ~307 m/s with sink ~-190 m/s. This is uncontrolled, unsafe descent, not a viable MM305 inlet.

Independently of the control departure, MM304's lateral/runway delivery was not certified. At **50 km** the shuttle was still approximately **669 km** from KSC, at **45 km** about **574 km**, and at **35 km** about **451 km**. The guidance's own distance/energy margin was already negative near 55 km (around -79 km), and the recorded flight never approached a flyable HAC/Final state. This cannot be repaired by loosening phase/admission/Final flags. Native preflight shadow qualification had already rejected meaningful Final capture; the actual trajectory confirms the warning.

### 7.3 Physical crash versus native software state

The KSP log in the active instance explicitly recorded `Active Vessel is about to crash` at **08:38:29** KST; body flap and SSME explosions began around **08:38:43**; an explicit Collision Enhancer **nose-gear explosion at velocity 65 m/s** occurred at **08:39:28**; and further fuselage, elevon, rudder, and wing explosions followed. A separate read-only kRPC observation after the destruction showed `GameScene.flight`, `paused=true`, active vessel `STS`, situation `landed`, **0 parts**, **0 m/s** observed actual flight speed, UT **146970927.057603**, and altitude about **1,768.01 m ASL**. The subsequent KSL summary retained pre-destruction telemetry (65.345 m/s and -39.424 m/s sink) while KSP exposed an empty vessel, so that stale post-destruction velocity is **not** treated as true rollout. The last recorded KSC range was **393,256 m**, runway cross-track **-392,743 m**. No certified Final capture, flare, touchdown, or rollout occurred.

At **08:40:09** KST, only *after* confirming KSP explosions, zero surviving parts, and paused physics, the still-running native controller was asked to stop via its existing SIGINT safe-release path. Hence `status.json` says `aborted` / `rollout`; the forensic flight outcome is **off-runway destructive impact following MM304 attitude departure, with no MM305 or Final**. All KSP saves/logs and the unrelated staged and unstaged source edits were preserved.

### 7.4 Verification and next focused experiment

`runs/20261013T0806Z-polar72-retake/retake-verification.json` independently checks all four save SHA256 identities, the exact 45 km crossing record, six matched-altitude pairs, absence of an MM305/Final phase event, and truthful native termination metadata. `checkpoint-regression.log`: **5 passed**, exit 0. Earlier full `make test` at commit `85c279e` already reported **157 tests and 56 subtests passed**; no native source/control limits were changed in this retake. Do not reinterpret either software pass as a landing qualification. Reproduce the paired flight comparison from the two recorded vehicle JSONL streams (keyframes **plus** cumulative deltas), not just instantaneous snapshots.

**Next experiment worth performing is offline, before another live return:** replay the new **45 km** physically saved state against the preceding near-identical 07:15 run, including time-aligned actual roll acceleration, body yaw/roll rates, sideslip, rudder/elevon physical deflections, controller-predicted axis authority, and measured applied controls. Explicitly test roll/yaw control effectiveness and reference-frame/authority assumptions around **40-33 km** while holding the recorded vehicle and atmosphere constant, and separately solve a physically reachable MM304 lateral/energy delivery. Only then repeat one live continuation from a preserved save, with bounded-control changes verified in the frozen replay. Keep best-effort attempted guidance, honest qualification flags, and the existing AoA/bank/stall/g/load limits; do not introduce speedbrakes or positive-margin fiction.


## 8. 2026-10-13 live RCS-q retake and operator-operated bay (investigation in progress)

The RCS-on-by-dynamic-pressure implementation was committed as `2495309` (request ON at q >= 700 Pa, OFF at q <= 500 Pa, with hysteresis). The physical test launched native MM304 from a hash-verified copy of the previous **50.33 km** checkpoint, rather than silently replacing the source file. The independent whole-vessel physics/parts guard accepted only settled **25,120.87 kg**, dry mass **20,160 kg**, **22 parts**, and **0 N actual thrust**, paused before flight. The live controller is `runs/20261013T003649Z-polar50-rcs-q-0940/`, vehicle and planner logs `FlightLogs/2026-10-13T00-37-01Z-STS-{vehicle,planner}.jsonl`; the one-shot owner service is monitored separately. Checkpoint/observer provenance is in `runs/20261013T0940Z-polar50-rcs-gate/`.

**Actual RCS activation confirmed, not inferred from the algorithm.** A read-only kRPC observer recorded `control.rcs = true` at **UT 146970490.1974**, altitude **47,795.22 m**, q **701.82 Pa**, roll **1.64 deg**, with monopropellant already decreasing. The physical 45 km checkpoint `ksl-20261013T003649Z-polar50-rcs-q-0940-descent-45km.sfs` persists `RCS = True, 146970489.997368`; SHA256 `17b2ce0f4476eb85ab9701b0193094b27c3fd2af5e2c3b42ac27a3fbe473561d`. RCS remained on throughout the observed 40-33 km interval and monopropellant consumption was independently measured. Representative roll: at 40 km roughly **3.6 deg** vs **5.7 deg** in the prior, at 38 km roughly **3.1 deg** vs **9.2 deg**, and at 36 km roughly **2.7 deg** vs **20.7 deg**. This demonstrates improved observed attitude in a *related, not identical*, return; the native MM304 controller started at a different save and no controlled isolation proves RCS alone caused all improvement. The midflight intact 32.57 km, 22-part, 0 N checkpoint `ksl-20261013T0943-polar33-rcs-live.sfs` has SHA256 `8a38997b3dccce9f0e5c91dea7aec9851bc6468f4f467b2c4f64e353af8b8081`.

**Operator correction (do not misattribute door movement).** The user explicitly confirmed **they personally closed the payload-bay doors**. KSL did not automatically close them during this flight. In the original 50 km and new 45 km saves, the Benjee10 `benjee10.shuttle.midFuselage` `ModuleAnimateGeneric` had `animTime = 0`, whereas `ModuleCargoBay.closedPosition = 1` in the installed OV_midFuselage.cfg. The manually operated 32.57 km checkpoint shows `aniState = MOVING`, `animTime = 0.820049465`, `animSwitch = False`, consistent with a closure in progress but **not** a precisely measured click time or proof that the door was fully shut at that checkpoint. Cargo-bay drag may contribute to range loss; it cannot be singled out as the entire cause of the shortfall, and the later manual action must not be labeled as automated behavior.

**Corrective implementation for subsequent flights.** Commit `b882aeb` adds a Benjee-specific, idempotent payload-bay preflight to both initial deorbit and resumed atmospheric guidance. It examines the actual active `PartEvent` GUI label for `ModuleAnimateGeneric` on the mid-fuselage and invokes `PartEvent.trigger()` only when an active **Close** action exists. It never blindly toggles a bay currently showing **Open**, reports missing/unknown/moving states without stopping best-effort flight, and does not wait on an atmospheric control handoff while the bay moves. Focused unit tests cover open/closed/absent/disabled/unknown/paused/incomplete cases; full `make test` passed **168 Python tests and 56 C subtests**, result recorded at `runs/20261013T0940Z-polar50-rcs-gate/payload-bay-full-regression.log`. A read-only-equivalent actual KSP invocation while the operator-closed bay was observed returned `status=already-closed`, `triggered=0`; a live **open-to-closed** trial of the new helper is still pending.

**Range and MM305 remain separate failures.** At around 50 km the measured guidance `rangeToSite` was **~673 km**, with `aero.energyExcessRange` around **-153 km** and target-site bearing near the shuttle's heading. By ~26 km KSC remained ~362 km away. At UT **146970670.02**, altitude **25,348 m**, Mach **2.85**, MM304 performed an explicitly *forced physical-boundary* handoff into MM305 with **`routeQualified=0`, `routeCommitted=0`**, as confirmed by `backend.stderr.log`; it is not evidence of a feasible Final/HAC trajectory. At ~9 km KSC still remained ~294 km away. The underlying deorbit opportunity/drag/lift/path-length deficit has not been repaired by enabling RCS or an MM305 mode transition. A successful physical landing must be verified separately; this subsection is intentionally marked as an in-progress record until impact/stop classification is available.
