# Repeat runway-capture run: source corrections

Date: 2026-10-31 (Korea). Execution was explicitly prohibited: no KSP/kRPC connection, controller launch, replay, simulation, build, benchmark, or test was performed for this work.

## Evidence and provenance

The inspected run is `runs/live-runway-capture-repeat-20261031-1349/`. Its 781 JSONL records span UT 195984.84879777927 through 196044.1687977482, approximately 59.32 seconds. Every recorded guidance phase is entry. The last position is approximately 67.900 km ASL. This record ends before MM305 or runway capture; it cannot establish a flown terminal-guidance outcome.

The run identifies HEAD `678f653e6f19dd44b4893e5da29ee54f7cd1b903`. Before editing, the working tracked diff exactly matched the archived `source.patch`. The actual source files in the untracked snapshot also matched the initial working copies. The initial status, diff, and source hashes were preserved separately, and every edited existing file has its original working copy in the task backup. No original run inputs, executable, log, or save were altered.

`stderr.txt` reports an invalid torque/inertia observation, recovery after four total faults, then Cocoa error 640 with underlying POSIX error 28 (ENOSPC). The rejected raw torque observation was not recorded, so its exact invalid component cannot be reconstructed from this archive. All numeric fields in the 781 retained records are finite, and their sequence numbers are contiguous. At inspection time the filesystem had approximately 16 GiB available; no disk cleanup or deletion was needed or performed.

## Recording must not terminate flight control

The original `FlightRuntime` awaited a throwing recorder inside its control loop. A Cocoa write failure bypassed recoverable-link handling, reached runtime cleanup, and shut down the active flight I/O. Recording failure therefore removed the control owner despite valid flight telemetry.

Both flight and deorbit runtimes now use the same `ResilientTelemetryRecorder` observer. Disk/encoding failures retain only a bounded gap summary and cannot enter link recovery or end flight control. Cancellation still propagates. Successful recording resumes with the missing record count, sequence endpoints, UT interval, and last error explicitly attached to the first recovered record. Close/flush failures are reported without replacing the flight outcome. The wrapper is shared across deorbit-to-entry transfer.

`JSONLFileWriter` tracks the last complete line offset. After a failed write it truncates and seeks to that boundary before another append, preventing a partial JSON object from being joined to a later record. Operator telemetry is published independently of the disk sink. A derived observation-journal failure no longer marks an already written primary flight record as lost. Failure warnings use the throwing FileHandle API with bounded error handling, including when stderr is redirected to the same full filesystem.

## Optional actuator telemetry is isolated from valid kinematics

Invalid torque/inertia previously discarded the entire coherent snapshot. The adapter now omits an invalid optional control-authority report and records its source and raw values as explicit diagnostic text. Invalid optional CoM/tensor observations are also marked unavailable. No replacement measurement, stale payload inertia, or measured zero is fabricated. The existing controller can use its explicitly provisional/identified fallback while fresh valid pose and velocity remain available. Mandatory kinematic, clock, identity, topology, and propulsion checks remain in force; orbital pointing retains its existing authority requirements.

Detailed faults are retained in each structured record; stderr warnings are emitted on changes in the affected sensor categories. The old archive does not establish the underlying KSP sensor defect, so this correction addresses invalid-observation handling and captures the previously missing evidence.

## Entry attitude preparation and the target limiter

In vacuum every force-allocation candidate has zero aerodynamic force. The old tie-break chose an AoA near the initial approximately 1.9-degree attitude rather than preparing the configured 18-degree entry attitude. The law now slews toward the configured nominal entry AoA while dynamic pressure is zero, retaining the existing trim and slew limits.

At positive but very small q, the target limiter used only the provisional q-scaled surface model. It ignored measured active reaction-wheel/RCS authority, producing a discontinuity at atmosphere entry. At record 374, q was 0.0224684 Pa: the old pitch-acceleration bound before response-time limiting was 4.20693e-6 rad/s², while the conservative measured torque/inertia bound, with the same reserve, was 0.0186128 rad/s². At record 781 those bounds were 0.000972309 and 0.0189772 rad/s² respectively. These are arithmetic comparisons of archived observations, not a new flight or model run.

The limiter now accepts the current active torque/inertia envelope. Its use is consistent in live mission guidance, entry prediction, terminal prediction, and the existing probe tool. The configured bank, rate, entry roll-acceleration, and response-time ceilings remain. A measured zero acceleration now holds the reference rather than falling through to unrestricted first-order slew. The no-overshoot/never-move-away behavior is retained.

## Failed-path diagnosis and ranking

The terminal envelope counted every violating path station. For example, the forecast carried by record 78 examined 39 candidates, all with insufficient energy, but counted 2,460 insufficient-lift occurrences. Record 631 had 35 energy-deficient candidates alongside 593 terrain occurrences and 273 lift occurrences. Station repetition and summed overlapping categories overrode the energy deficit, producing control-authority or altitude classifications.

Envelope/advisory counts now count each failure kind once per candidate. Candidate scoring likewise does not multiply a penalty merely because the same violation repeats at many stations. When every examined candidate has an energy deficit and none reaches a completed admissible envelope, that deficit takes precedence over associated low-speed lift/slew failures. This enables the existing MM304 lower-drag search direction. Unknown energy margins and an empty set of physical violation evidence no longer masquerade as excessive energy. These changes do not certify any candidate or relax admission.

## Planning lifetime

The proposal recorded at sequence 245 arrived approximately 13.94 simulated seconds after its source state. The existing entry acceptance window was 10 seconds. The runtime now cancels work once it exceeds that same established acceptance window, allowing a fresh request without changing flight-control ownership.

Review also found an adjacent source defect: a proposal-serialization error remained latched and was repeatedly thrown as a recoverable link I/O fault. It is now reported and cleared; the current bounded guidance continues and planning can retry. No claim is made that this serialization defect occurred in the inspected run.

## Validation boundary

Seven focused regression cases were added as source: atmosphere-boundary actuator continuity, measured zero authority, target no-overshoot, per-candidate violation counting, energy-deficit attribution, vacuum attitude preparation, and nonfatal recording failure with explicit recovery gaps. None was executed.

Validation consisted of archived JSON decoding and arithmetic, source and call-site inspection, comparisons against the initial working copies, and `git diff --check`. Unrelated tracked and untracked source hashes remained unchanged. Compiler acceptance, new closed-loop behavior, performance under load, and actual runway capture remain unverified. No executable was rebuilt or replaced.
