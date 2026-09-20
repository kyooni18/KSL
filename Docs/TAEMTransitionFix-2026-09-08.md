# S-turn / TAEM handoffs and high-pass HAC circuits

Implemented against the September 8 checkout following the user's requirement: passing the runway final-alignment station high should lead into a feasible returning HAC circuit, not silently complete HAC or necessarily abort at the first exit. The configured station remains fixed (normally 8 km before the threshold).

## Behavior

- **S-turn → TAEM:** retain the aerodynamic/energy-qualified normal handoff, with an additional lower-speed high-pass gate near the alignment station. Both circle sides are evaluated. Ordinary HAC selection now also honors the 1.5°/s sustained course-rate limit; acquisition radial tolerance is capped at 5 km, rather than expanding to 28.8 km on a 96 km circle.
- **High pass:** before an invalid final exit, with runway-local lateral alignment and inbound course, explicitly budget one complete additional circuit on the frozen circle. Command preparation allows finite roll-response lead; this is not an instantaneous turn exactly at the station. Each circuit requires finite measured/modelled turn authority, speed margin, a trackable radius, a descent slope between the configured nominal TAEM slope (at least 8°) and 35°, 300 m of height reserve above the final-entry profile, and specific energy covering 115% of estimated drag work along the additional arc. A 96 km circle at 4 km altitude is rejected. Merely having positive altitude does not qualify a return.
- **HAC-only rehearsal:** tries the same fixed-station high-pass circuit first when aligned. Its partial-arc fallback also returns to the configured station; it no longer obtains a better fit by moving the final exit. Committed test final slope, HAC slope and displayed reference now agree, with an additional circuit's descent slope kept separate from its final approach slope.
- **TAEM → S-turn:** sustained loss of turn authority or capture geometry for 5 seconds can explicitly relinquish TAEM. It requires at least max(12 km, half the configured TAEM altitude), 8 km radar clearance, and twice minimum safe speed. A missed exit can also take this branch. The old HAC, circuit, reversal events, final-heading lock and control-plan validity are cleared; fresh entry planning takes ownership with 15 seconds of HAC acquisition inhibition. Command limiters remain continuous. This authorizes replanning; it does not assert that a runway return is guaranteed.
- **Low/infeasible missed exit:** retains an explicit abort when no final capture/circuit or high-altitude entry fallback qualifies. Final capture still requires the full position/course, altitude, speed, FPA and sink envelope.
- **Progress:** before first capture, geometric acquisition is not labeled established arc progress. After capture (or explicit circuit commit), signed angular progress is retained through branch cuts and temporary capture loss; reverse motion restores distance instead of earning another increment when it moves forward again. Exhausted arc is never recycled through a modulo wrap. Only an explicit qualified high pass adds a circuit.
- **Energy commands:** the dissipatable-energy ceiling is no longer a throttle target. Nominal TAEM is unpowered; a separate bounded speed schedule drives low-energy AoA unloading and steeper descent. Configured thrust is recovery-only below 120% of minimum safe speed, after measured incidence is unloaded and descent is steep. Status text labels the energy ceiling separately from the speed schedule.
- **Entry tangent:** removed a later runway-aimpoint override that contradicted the HAC tangent heading already computed for the final S-turn exit.

The predictor uses the shared circuit feasibility function, capped capture tolerance, course-rate radius constraint, explicit circuit slope, and TAEM-loss/re-entry cooldown. Its first TAEM-arrival measurements remain historical when active guidance returns to entry. The reduced predictor still approximates aerodynamic control response and uses site-relative altitude where live guidance has terrain clearance; it is not a bit-for-bit live flight simulator.

New flight-log fields: `hacCircuitCount`, `hacCircuitGlideSlope`, and `terminalReentryAfterUT`. Status messages distinguish a high-pass circuit and a deliberate return to S-turn planning.

## Validation

Passed:

- `make -C CLanding test`, including the new `TAEMForecastTests.c` suite.
- `python3 Validation/PythonBridgeTests.py`.
- `python3 Validation/BackendProtocolTests.py`.

Regressions cover normal entry into a high-pass circuit, no repeated circuit addition at a stationary gate, full prescribed circle replay in both directions through final capture and angle branch cuts, signed backtracking without fake progress, infeasible circuit budgets, high-altitude fallback/cooldown/reacquisition, low off-axis missed exits, valid final capture without an extra circuit, fixed rehearsal station, large-radius capture misses, and energy-ceiling/throttle separation with low-speed AoA unloading. Predictor state tests independently cover the high pass, entry fallback and low missed-exit rejection.

The new high-pass regression fails with the saved original guidance (`hac_circuit_count == 1`), while it passes with the repaired implementation. The full-circle test prescribes flight states: it verifies state transitions and geometry bookkeeping, not closed-loop aerodynamic capability or successful landing.

Original edited files are preserved under `Runtime/Repairs/2026-09-08-taem-transitions/`. The backend has been rebuilt. No running flight was restarted, no save reloaded, and no live flight or landing was claimed. These changes supersede the previous single-pass missed-HAC-exit policy for explicitly feasible high passes; they do not relax final-capture or low-altitude failure gates.
