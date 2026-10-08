# STS entry experiment record — 2026-10-08

All closed-loop runs below are **unqualified downstream diagnostics** from the exact archived post-load state (UT 9932.411826, 71,851.542 m, 25,816.105 kg). They use `vehicle.modelId=STS`, the provisional STS polar, the identified Kerbin atmosphere, and borrowed STS-N attitude dynamics. The simulator fast path skips shadow admission and now says so explicitly. No live landing or rollout is established.

## Controlled controller comparisons

| Candidate | Abort altitude (m) | Speed (m/s) | Along (km) | Cross (km) | Along at 25 km altitude (km) |
|---|---:|---:|---:|---:|---:|
| current baseline | 7316.4 | 84.988 | +180.6 | -36.6 | +166.9 |
| L/D 0.75 | 7331.7 | 84.973 | +180.7 | -36.6 | +167.0 |
| drag-altitude feedback | 7308.1 | 84.938 | +154.2 | -38.0 | +142.3 |
| boost lift margin | 7320.7 | 84.987 | +178.9 | -34.1 | +161.1 |
| coupled feedback | 7176.5 | 84.965 | +158.1 | -34.1 | +140.1 |

Every run aborts at the unchanged 85 m/s recovery floor before MM305/TAEM. Positive along-track is downstream of the runway. Commands remained at or below the existing 28° AoA and 40° MM304 planning ceilings; throttle and speedbrakes stayed off. Exact provenance, raw-log digests, and delivery states are in [controller-comparison.json](controller-comparison.json).

The drag-altitude prototype converts log drag error into an altitude-equivalent error using the atmosphere scale height, adds radial-speed damping, and derives lift demand from the measured inertial tangential speed and air-relative flight-path angle. It uses the existing energy module’s 0.07 rad/s bandwidth and 0.8 damping without solving a second energy profile. The independent boost prototype only permits extra AoA when the demanded vertical lift can accommodate the minimum lift at the current bank ceiling. The coupled run combines these two independently tested changes. All three compiled cleanly in an isolated worktree. Their incomplete trajectories do not justify adopting them in production.

Compressed source patches are retained for each prototype. Apply the corresponding patch to its `provenance.gitHead`, then build with `make -C CLanding BUILD=/tmp/experiment-build -j2`. The patch also records the identical baseline 50 Hz terminal propagation change. Replays used `run_guidance.py`, the scenario and configuration paths above, `--max-sim-time 1200 --no-mirror --quiet-progress --compact-guidance-log --skip-build`, and the isolated backend build directory. Production flight-law sources and `Configuration/sts.json` were not changed by these prototypes.

## Fixed-control descent sensitivity

Standalone plant probes held AoA at 0/8/18/28° and bank at 0/40°. These are finite open-loop probes, not a reachability proof, gate qualification, or landing attempt. Constant-bank probes can exceed the low-q guidance bank ramp. The commands, input hashes, full-duration peak loads, and raw-log locations are recorded in [fixed-control-summary.json](fixed-control-summary.json).

| AoA / bank (deg) | Along at 25 km (km) | Speed at 25 km (m/s) | Altitude at first 860 m/s (km) | Along at first 860 m/s (km) |
|---|---:|---:|---:|---:|
| 0 / 0 | -73.7 | 1302.7 | 19.61 | -53.5 |
| 0 / 40 | -73.7 | 1302.6 | 19.61 | -53.5 |
| 8 / 0 | +56.5 | 1050.5 | 22.76 | +70.5 |
| 8 / 40 | +18.3 | 1110.1 | 22.06 | +34.3 |
| 18 / 0 | +196.0 | 777.8 | 25.91 | +187.6 |
| 18 / 40 | +101.6 | 879.3 | 24.78 | +103.1 |
| 28 / 0 | +223.0 | 615.2 | 27.71 | +197.9 |
| 28 / 40 | +105.6 | 722.9 | 26.46 | +95.2 |

The zero-lift probe decelerates upstream; the nominal high-AoA cases decelerate downstream. This demonstrates sensitivity to descent timing in the provisional model. The 0°/8° cases are outside measured polar support: the identified hypersonic samples span roughly 22.5–28°, with no certified cells or independent alpha sweep. They also fall below the current thermal AoA floor. They are unsuitable as live-flight candidates.

## Polar uncertainty and unused bank authority

`ShuttleSim/tools/probe_sts_low_alpha_uncertainty.py` repeats fixed-control probes with CL or CD perturbed below 14° AoA. The quadratic perturbation vanishes at 14°; every table row at or above that magnitude was checked to remain exactly unchanged. Multipliers 0.5 and 2 are deliberately broad scenario brackets, not measured confidence intervals. All drag coefficients remain positive. Input digests, commands, full-duration loads, and retained compressed traces appear in [low-alpha-uncertainty-summary.json](low-alpha-uncertainty-summary.json). All ten probes completed. The baseline repeats the previous crossing locations within a metre.

At 0° AoA / 40° bank, the 25 km crossing moves from along -73.7 km to +110.4 km when low-alpha drag is halved, or -216.9 km when doubled. At 8° AoA it moves from +18.3 km to +54.1 km or -38.6 km. Low-alpha lift changes also move the 8° crossing (-10.6/+22.1 km relative to baseline). Therefore the earlier upstream descent is not robust evidence for a live low-AoA policy; neither the polar nor the protective floor was changed.

The same script's `--bank-authority` mode holds AoA at 18/28° and bank at 40/55/70°, retaining the original polar. These six probes are upper-bound plant experiments, not controller candidates: constant bank ignores the low-q ramp, lateral steering, and nominal 40° MM304 planning ceiling. The 70° cases stay within the configured physical maximum, but do not establish that the planner can legitimately use that bank. [bank-authority-summary.json](bank-authority-summary.json) records complete provenance and peak loads.

| AoA / bank (deg) | Along at 25 km (km) | Altitude at first 860 m/s (km) | Along at first 860 m/s (km) | Cross at first 860 m/s (km) |
|---|---:|---:|---:|---:|
| 18 / 40 | +101.6 | 24.78 | +103.1 | +32.4 |
| 18 / 55 | +35.1 | 23.76 | +42.1 | +36.5 |
| 18 / 70 | -32.6 | 22.49 | -21.5 | +36.0 |
| 28 / 40 | +105.6 | 26.46 | +95.2 | +39.9 |
| 28 / 55 | +23.4 | 25.33 | +21.5 | +44.2 |
| 28 / 70 | -58.4 | 23.91 | -53.9 | +42.6 |

This is evidence of additional descent authority in the provisional plant, not whole-trajectory recovery. The 18°/40° uncontrolled full-duration probe reaches 5.49 g, exceeding the unchanged 3.5 g limit; it must not be described as safe. The unresolved engineering question is whether a legitimate bounded execution/recovery mechanism can use the existing physical bank authority while preserving nominal planning, coordination and qualification contracts. Production flight laws and configuration remain unchanged; no live attempt was made.

## Paused-load physics correction

Read-only live audit [live-state-audit.json](live-state-audit.json) found original STS paused at UT 9934.591826 and 71,723.560 m, with 22 parts each reporting exactly 1,000 kg. Vessel mass was 22,000 kg, and resource subtraction produced an aft-fuselage dry mass of zero. These values are consistent with uninitialized part rigidbodies and do not match the preserved save or archived physical flight state. SAS, throttle, and speedbrakes were off. The named checkpoint’s SHA256 remains `bb27c762add826731df3ad57332efd77d3d3a5d0928caa2ca361907ae0b1703f`. No scene or control writes were issued.

Commit `00f6f76` makes both generic settling paths reject a paused restored state without unpacked-physics evidence. The existing explicit checkpoint-profile path still independently checks the expected mass, dry mass, part count, vessel, and save hash. The regression reproduces both previous false acceptances and passes after the change; all 15 focused headless tests pass. A 22,000 kg profile must not be created from this placeholder observation.

[Checkpoint mass accounting](checkpoint-mass-accounting.json) reconstructs 20,348.000425 kg dry mass plus the preserved resources, giving **25,816.104933 kg**. The archived first post-load keyframe reports 25,816.105469 kg, agreeing within 0.001 kg. The expected physical state is retained in `Configuration/checkpoints/sts-campaign-72km-20261008.physics.json` for the existing `--checkpoint-physics-profile` option. This profile provides a strict expected-state check; it does not certify the currently paused placeholders or the aero model.

## Bounded execution experiments

Four isolated controller prototypes preserve the nominal MM304 planning ceiling at 40° and add a separate execution allocator inside configured physical bank authority. Extra magnitude is calculated from measured local lift and required vertical lift only when measured lift is valid, structural margins are positive, and both drag and predicted-range errors demand more dissipation. The original low-q ramp, predictive coordination reduction, 4.5°/s roll-rate and 1.8°/s² acceleration settings remain. Nominal `entry_control_bank` stays with the planner; the issued command and feedback carry the execution bank. The additional allocator state is reset with Entry control. This is experimental source in an isolated worktree, not an adopted flight policy.

| Candidate | Along at 25 km (km) | Along at first 860 m/s (km) | Abort along (km) | Abort altitude (km) |
|---|---:|---:|---:|---:|
| measured bank execution | +128.6 | +132.2 | +142.5 | 7.30 |
| drag-altitude + bank execution | +69.4 | +72.2 | +78.7 | 7.33 |
| same, 18° AoA cap | +67.3 | +76.2 | +83.7 | 8.06 |
| mission drag target + bank execution | +58.6 | +63.6 | +69.8 | 6.87 |

Every run aborts at the unchanged 85 m/s floor before MM305/TAEM. Commands remained inside physical 70° bank and the configured AoA cap; peak g was 2.03–2.27. This does not validate coordination because the STS attitude model is still borrowed. The 18° configuration tightens the cap without lowering the thermal floor. The last prototype splits the existing drag profile's mission request from its local feasibility clip: altitude feedback uses the structurally bounded mission request and its velocity derivative, while range predictions and reported reference drag retain local aerodynamic caps. It avoids tracking a nearly constant `Dref/D=1.85` altitude error. This improves descent somewhat but still does not repair the mission.

[Comparison](bank-execution-comparison.json) records actual backend hashes, states and limits. The runner's Git fields refer to the root workspace; the explicit source base `2d33b96` and compressed source patch identify each isolated binary's source. Patches include the identical 50 Hz terminal baseline. Apply to the source base and build as described above. All prototypes compiled cleanly. No failed prototype was adopted, and no live flight was attempted.

## Effect of the preserved low-q bank ramp

`probe_sts_ramped_bank.py` uses simulator lockstep directly, with fixed positive bank capped at `70*clamp(q/360,0,1)^2`, the existing structural limits and the lateral allocator's roll-reference motion/capture rule. These are plant experiments, not the nominal 40° plan, lateral recovery, coordination validation or admission. Five successful traces have monotonic times, zero throttle, closed speedbrakes and subsequent commands inside 18–28° AoA / 70° bank. The initial t=0 frame retains checkpoint AoA 1.841°. Source and input digests and full traces are retained in [ramped bank](ramped-bank-summary.json) and [incidence transitions](ramped-bank-incidence-summary.json).

At 28° fixed AoA the ramped probe reaches 860 m/s at altitude 23.82 km, along **+51.1 km**, cross +35.8 km. The earlier unrestricted 70° probe reached the same speed at along **-53.9 km**. The original 18° ramped case is also downstream (+56.1 km). Holding 18° then switching to 28° at 50/45/40 km produces +46.2/+49.0/+50.1 km respectively. These finite probes indicate that early authority matters more than the tested bounded incidence schedules; they do not prove global infeasibility or justify removing a safety ramp.

[Archived thin-air bank evidence](thin-air-bank-evidence.json) independently decodes original STS run 9 at Mach >=5. Commands stay essentially zero throughout q=1–1000 Pa, with actual bank below 0.90°. Thus that archive cannot identify coordinated roll authority in the critical thin-air region. The current ramp's source comment cites STS-N experience; borrowed attitude dynamics and unexcited STS telemetry do not validate an alternative STS envelope. Establishing usable original-STS authority remains an unresolved evidence requirement. Production configuration and guidance remain unchanged.

Final read-only live audit still finds original STS paused at UT 9934.591826, altitude 71,723.560 m, placeholder mass 22,000 kg, SAS/throttle/speedbrakes off. Checkpoint SHA256 is unchanged. Only one live KSP process remains; all offline runners completed.
