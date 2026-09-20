# Terminal trajectory and startup ownership repair — offline validation

Primary evidence: `FlightLogs/2026-09-08T13-14-14Z-STS-N.jsonl`, native build `Sep 8 2026 22:06:08`. No live flight, save load/reload, or interactive simulator testing was performed. No older flight was used to judge the current Frenet follower.

## Causal findings

- This log is a late-terminal restart, not a complete Entry/S-turn flight. It directly establishes the startup/Recovery/TAEM failure, but does not directly measure the final S-turn termination of a full entry.
- The supplied startup-holder observations place release at UT 23877.26, before the first applied landing command at UT 23888.0198. The code stopped the holder on connection, then optionally launched another preparation client which neutralized controls before engagement. The first landing sample has roll -45.03 degrees and body roll rate +58.39 degrees/s.
- The first Recovery command reports roll authority 529.26 degrees/s² and remains near that estimate. `_AxisAuthority` accepted availableTorque/inertia as a full-strength baseline. Cross-axis rejection and weak excitation could then prevent that baseline being corrected. The observed 2.6-second rate decay does not, by itself, identify full-input actuator authority; the initial prior nevertheless produced the wrong braking decision in the reproduced initial state.
- TAEM resumes at UT 23890.5998, 20.464 km MSL (approximately 19.5 km terrain clearance), 693.79 m/s. The old TAEM branch repeatedly recomputed a tangent to a disposable circle, independently of the constrained selector. The selector was not called until the narrow altitude window near the 12 km handoff.
- Abort begins at UT 23936.9798, 12.070 km MSL (roughly 11.75 km terrain clearance), 267.77 m/s, along-track -13.429 km, cross-track -4.130 km. No HAC was committed. Calling the selector earlier against unchanged recorded states still rejected them: early trajectory shaping, not a relaxed final gate, is needed. This log cannot validate or invalidate committed-path tracking.
- Independently, the deterministic committed-join test exposed a near-cusp whose tangent reversed about 180 degrees between the old 28 curvature samples. A second allocation issue increased bank using anticipated reduced lift before the pitch response had actually unloaded the aircraft. Neither finding is attributed to a committed HAC in the latest flight.

## Changed behavior and implementation

`CLanding/guidance.c`, `CLanding/landing.h`:

- `terminal_predict` begins upstream within the terminal prediction envelope, independent of the phase enum. It propagates position/course with finite roll response, integrates altitude/FPA with finite pitch response and lift, and estimates speed/energy with measured drag/speed loss. The nominal future capture altitude is 14 km; the actual qualified forecast state accounts for response and need not equal that number.
- Before a qualified join exists, the inbound reference converges toward the fixed upstream alignment station. The final Entry controls progressively blend toward the shared bank, heading and FPA-derived incidence reference. Blend timing accounts for remaining forecast time and actuator response, leaving margin before the join.
- `guidance_terminal_control_plan` supplies the same blended policy to Entry and to the native controller's Entry forecast (`CLanding/controller.c`). Final-reversal classification and `terminal_forced_acquisition_ready` require the retained trajectory and convergence rather than an altitude-only transition.
- The existing constrained selector qualifies a complete future join. `TerminalCandidate` retains its curve, circle, side, exit, altitude and speed. Re-solves are paced; accepted refinements keep the side/exit and bound radius/control-point movement. Incompatible proposals do not replace the retained candidate. Control points are not interpolated outside the selector's feasibility checks.
- `terminal_candidate_commit_ready` checks projected arrival altitude/speed, course, lateral corridor, candidate age, and one to three response intervals of remaining lead. `terminal_publish_candidate` freezes the same selected geometry. No command limiter reset occurs at commit; the committed vertical reference slews continuously. The committed geometry is excluded from subsequent prediction/refinement.
- `hac_bezier_regular` rejects near-cusps/tangent reversals before candidate scoring. `taem_guidance` accounts for current measured lift when allocating bank during an anticipated AoA unload. The Frenet tracking law and bank/yaw ownership are preserved.
- `CLanding/models.c` and `controller.c` add flight-log fields: `terminalPredictionValid`, `terminalCandidateValid`, `terminalPathCommitted`, candidate radius/altitude/speed, `terminalReferenceFPA`, and `terminalBlend`.

`Tools/headless_flight.py`, `Tools/ksp_test_guard.py`, `PythonBridge/krpc_bridge.py`:

- A unique per-run handoff file is inherited by holder and bridge. `_write_owned_axes` serializes their writes with a file lock. Ownership transfers only after all three finite, bounded axis writes succeed. A failed/partial write leaves the holder eligible to continue; a waiting holder cannot overwrite the first landing command.
- Backend connection/idle neutralization cannot clear the holder's commands. The holder remains active throughout backend initialization and engagement. Its setup disables SAS/warp, so a separate neutralizing preparation client is unnecessary while it is active. Explicit teardown releases ownership and permits safe neutralization.
- `_AxisAuthority` bounds the unidentified reported prior at 20 degrees/s², clears learned/prior state on reset/reload, and grows from measured response. Repeated clean response can establish substantially greater authority; the prior bound is not a flight-regime gain table or a permanent authority cap. Single positive innovations are bounded, cross-axis contamination rejection remains, and invalid sample gaps cannot bridge stale rate differences. The existing asymmetric control-use filter remains downstream.
- Bridge diagnostic revision: `terminal-prediction-20260908-r15`.

`Configuration/default.json`: synchronized its stale 25 km TAEM configuration value with the existing native 20 km default. This fixes a protocol-test mismatch; it is not the new acquisition/commit rule.

## Deterministic regressions and results

`Validation/HACPlanningStateTests.c` now covers future prediction before phase acquisition; a nominal zero-blend continuation reaching commit above 15 km with the join ahead; candidate retention through telemetry perturbations and re-solves; rejection of large radius/side/control-point changes; final-reversal revalidation; shared Entry/TAEM and forecast commands; finite-response vertical convergence; projected energy/altitude commit rejection; immutable publication; and near-cusp rejection.

`Validation/CLandingSmokeTests.c` updates obsolete altitude-forced phase expectations to require upstream prediction and gradual descent shaping. Existing power, abort, recovery, and terminal tests remain.

`Validation/PythonBridgeTests.py` adds bounded cold priors at reported 529/1395 values; measured-response convergence to weaker and stronger synthetic plants; contamination rejection/reset; braking at the latest startup attitude/rate; failed/invalid first commands; suppression of idle neutralization; and competing startup/backend writers.

Final offline runs:

- `make -C CLanding test`: PASS — C smoke, TAEM forecast, HAC join tracking, HAC planning state tests.
- Existing committed-join plant: completion in 105.8 s; maximum path error 126.2 m against its unchanged 350 m limit.
- `python3 Validation/PythonBridgeTests.py`: PASS.
- `python3 Validation/BackendProtocolTests.py`: PASS.
- Python compilation of the bridge and both startup tools: PASS.

Concurrent vessel-physics/force-vector edits appeared in the shared workspace during validation, including a transient JSON API compile error which was corrected by that work. Those edits were preserved. The final C results above include the shared tree and its vessel-physics object; they are not a claim of authorship of that separate work.

## Next-flight uncertainties

These are reduced-order offline regressions, not proof of a successful landing. The forecast uses a local drag/lift and response approximation; density/drag evolution, ground-track projection, roll/pitch coupling, and the timing of actual aerodynamic authority learning need live validation. Inspect candidate prediction versus measured arrival state, commit lead, and whether provisional retention becomes too restrictive under larger model errors. The original logged trajectory remains infeasible in the constrained selector: this repair depends on the changed upstream control history and continuous startup ownership, and does not claim to rescue that unchanged history.

Confirm the new native build/revision in the next log, holder release after the first valid landing-axis publication, sensible roll-authority growth, Entry/TAEM command continuity, a candidate well before capture, and exactly unchanged committed geometry through the join and circle. A run without commit still supplies no evidence about committed Frenet tracking.
