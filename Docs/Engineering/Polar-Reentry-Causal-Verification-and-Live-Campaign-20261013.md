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
