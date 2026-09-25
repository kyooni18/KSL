# Remediation status (2026-09-25)

Follow-up to `REPORT.md`. It lists what changed for each finding, the evidence
behind each change, and what is still open. All code is on branch
`claude/kspshuttlelander-audit-medqee`. Every commit passed `make -C CLanding test`
(7 suites).

## Summary

| Finding | Status | Where |
|---|---|---|
| R1 FCS never exercised offline | **Addressed (tooling).** `run_guidance.py --direct-control` runs the real FCS against a new surface-moment plant. The plant coefficients are generic, not KSP-identified. | `ShuttleSim/src/attitude.c`, `CLanding/transport/krpc.c` |
| R2 inverse crime | **Partly.** Servo parameters are no longer published to guidance. `--terminal-*` options give guidance a different aero/attitude model from the plant. The default run is still perfect aerodynamic knowledge. | `sim_telemetry.c`, `run_guidance.py` |
| R3 plant not in repo | **Fixed.** A tracked reference plant exists, `make test` passes on a clean checkout, and fitted data is preferred when present. | `ShuttleSim/reference-model/` |
| R4 confidence scales lift | **Fixed in MM304.** Bank comes from `cos φ = a_v / L_measured`; confidence plays no part. | `guidance/entry_energy_control.c` |
| R5 equilibrium-relative demand | **Fixed.** Drag tracking uses an altitude-equivalent error, ḣ damping and a bounded integral. Saturation is explicit and flagged. | same |
| R6 drag reference ≈ 1.85·D | **Fixed.** D_ref(V) is a constant-then-linear profile whose level is re-solved each cycle so predicted range equals range-to-go. It is flagged when it saturates at max/min. | same |
| High: no ḣ damping / bias | **Fixed** | same |
| R7 eastbound lateral assumption | **Fixed.** Delta-azimuth reversal, velocity-dependent deadband, leg capture, and a turn-bank floor when far off the nose. Covered by a mirror-symmetry property test. | `guidance/entry_lateral.c`, `Validation/EntryLateralFeedbackTests.c` |
| Medium: deadband = predictor residual | **Fixed** (deadband is now 4° + 6°·velocity fraction) | `guidance/entry/contract.inc` |
| R8 admission is a lower bound | **Partly.** A closure veto was added (flying away from the site). MM305 no longer aborts on its first tick; it acquires and replans. There is still no upper energy bound or planner-feasibility test at admission. | `prediction/decision/taem_capture.inc` |
| R9 MM305 plans once, sync, aborts | **Fixed.** Planning is re-entrant on the prediction worker. An acquisition law flies while no route is held. Divergence releases the route and replans. Replanning is event-driven: it triggers on tracking degradation, not on a timer. Planning runs on its own thread, and ShuttleSim has a lockstep barrier. | `guidance/guidance_taem.c`, `app/controller/*` |
| MM305 planning cost (found during remediation) | **Improved.** A failing search took tens of seconds; in lockstep it ran the simulator at 0.08–0.14× real time. Planning now has its own thread, retries back off after failures, the join sweep is coarser, and the radius lever adds only the larger HAC. On the fixture a failing single-end search went from ~19 s to ~8 s. That is still slow for live use at 1× (the vehicle covers several km per plan). | `guidance/guidance_taem.c`, `taem_candidate_search.c`, `app/controller/prediction_workers.inc` |
| R10 no energy feedback in MM305 | **Partly.** Replans start from the measured state, and the planning model is scaled by the measured/model lift and drag ratio. HAC radius is the route energy lever. There is still no continuous energy loop inside a route (no S-turn/speedbrake lever). | `guidance/guidance_taem.c` |
| R11 Final 10-s flare latch, no energy closure | **Addressed; landing quality still not met** (see below). Changes: energy-placed aim point, outer-glide speed closure, a preflare latch sized from sink/pull/incidence response, a re-solved arc, a Newton step on measured lift, long-landing handling, and an authority-aware lateral law. | `guidance/final/sequence.inc` |
| Medium: misleading abort reasons | **Fixed.** Long-landing abort added; the departure abort now carries position and phase. | `guidance/guidance_terminal.c` |
| Medium: HAC exit not conditioned for Final | **Changed.** An aligned vehicle inside Final's steep-glide envelope is handed to Final instead of being aborted. | same |
| g_force definitions | **Fixed** (lift/(m·g₀) everywhere) | F1 |
| FCS env-var gain overrides | **Fixed.** Overrides are read once and honoured only with `KSP_LANDER_FCS_TUNING_OVERRIDES=1`. | `control/flight_control.c` |
| FCS gain scheduling / trim across regime | **Open.** Authority normalisation already scales with q. Retuning needs a KSP-identified rigid-body plant. | — |
| R12 tests can't detect defects | **Partly.** Added lateral azimuth and mirror tests, MM305 planning acceptance tests, and a clean-checkout native stack. Closed-loop campaign gates are not in `make test`. | `Validation/` |
| Docs/build targets | **Fixed.** Updated ARCHITECTURE/VALIDATION/README/Validation README, marked a stale doc historical, and verified the ASan/UBSan claim. | — |
| Dead entry slices | **Removed** (~1,800 lines) | — |
| Built-in vs JSON terminal geometry | **Open.** Documented at source; the native-stack fixture depends on the built-ins. | `vehicle/models.c` |
| Predictor uses legacy interface capture | **Open.** The advisory forecast still ends entry on `entry_taem_interface_capture`, not the live admission set. | `prediction/entry_simulation.inc` |
| Ground model (derotation, crash) | **Open** | ShuttleSim |

## Evidence

All runs used ShuttleSim with the tracked reference plant (B) and the ideal servo attitude plant, unless noted. The runner's success gate is on-runway touchdown at ≤3 m/s sink. **No run passed it, before or after these changes.**

**Final checkpoints** (`final-mm305-ideal-3p5km-185mps*`, `engageFinalTest`):

| Checkpoint | Audit (before) | Now |
|---|---|---|
| aoa10 (audit "ideal"), plant B | 1,450 m past threshold, 25.6 m/s sink, preflare latched at t=0 | 774 m past, 13.8 m/s sink |
| aoa3, plant B | — | 342 m past, 12.8 m/s sink, on centreline |
| aoa3, plant B, `--direct-control` (real FCS) | not possible | 497 m past, 13.3 m/s sink (the FCS closes the loop; same energy limit) |
| aoa3, plant A (seed) | 191 m short, 37 m/s | still short (~−550 m, ~35 m/s); plant cannot glide |
| aoa3, plant C | 5.1 km past (aoa10) | about 2.2–3.1 km past depending on flare variant; energy excess |

**MM305 sweep** (`Audit/.../scenarios/mm305-*.ini`, `engageHACTest`, plant B):

- **Audit baseline (plant C, perfect knowledge):**
  - 5 of 12 aborted before landing: 4 in MM305 at t=0 ("route rejected"), and W60 later on a tracking divergence.
  - 7 touched down off the runway at 13–21 m/s sink.
- **After re-entrant planning and Final admission** (sweep `s2`, 9 of 12 completed; the 90° cases timed out on planning cost):
  - All 9 got through MM305 and touched down.
  - None landed on the runway surface: 18–31 m/s sink, with cross-track up to ~440 m and some far along or past the runway.
- **90° cases:** they start 90 km off-axis at 25 km. After the planning-cost fix, N90/E90/S90 end with a clean "no qualified route before the height needed" abort at 1 km. Plant B cannot glide that far (it flies −20° at α_max all the way).
- **Route switching found by the sweep:**
  - Periodic 20-s replans adopted markedly different routes. Each switch caused bank spikes of 20–35° and AoA jumps between 10° and 25°.
  - A stale roll-limiter state at MM305→Final threw vehicles up to 2.2 km sideways.
  - Both are fixed: replanning is now event-driven, and the limiters are reseeded on leaving the bypass.
- **Latest sweep (`s9`, 30°/60° cases, 7 of 8 finished; E60 still running at commit):**
  - N30, S30 and W60 found no route before 1 km.
  - N60, S60, W30 and E30 hit the ground off the runway: 11–27 m/s sink, 176–416 m cross-track.
  - Route releases dropped from 6–19 per flight to 0–5.

In short, MM305 no longer aborts on its first tick and hands vehicles to Final far more often. The closed-loop MM305→touchdown chain still does not produce landings on this plant. The causes are listed below.


## Remaining limits

- **Final on the reference plant (B)** has subsonic L/D ≈ 2.9 at 12–15° and a 1-g
  speed at the 15° incidence limit (`terminalMaximumLiftAngleOfAttack`) of
  74.7 m/s, which equals the configured touchdown speed.
  - Every flare therefore ends at α_max with speed decaying to the 1-g speed.
  - Touchdown sink is 10–14 m/s from the ideal Final checkpoints (it was 26 m/s
    before). The runner's ≤3 m/s gate is not met.
  - Two variants were tried and removed:
    - A forward-simulated predictive latch: too sensitive to the unknown lift
      level at high α.
    - A predictor-corrector arc: same problem.
  - Meeting the gate on this plant needs one of: more incidence at touchdown, a
    drag device for approach energy, or a KSP-identified table showing better
    L/D than this synthetic reference.
- **Plant A (seed aero)** glides at −32° at α_max from the Final checkpoints.
  No law can land it; it is a plant-validity problem, not guidance.
- **Plant C (high L/D)** from the steep Final checkpoints has more energy than
  a −35° glide can dissipate in 3.5 km without a speedbrake. It lands long.
- **MM305**:
  - It still often finds no energy-feasible route while acquiring. Its only
    energy levers are HAC radius (now tightened by the search, plus one larger
    radius) and lead length. There is no S-turn or speedbrake dump.
  - The acquisition law itself does not manage energy.
  - A failing search still costs ~8 s wall on the fixture, too slow for live
    1× flight.
  - Route tracking still gets released after commit in some cases.
  - Next steps:
    - An energy-managing acquisition law (S-turns toward the HAC region when
      high, direct when low).
    - Cheaper route construction.
    - Replay qualification under model error (`--terminal-*`).
- **Handoff admission** still has no upper-energy bound and no planner-feasibility
  test. The Final-admission fallback accepts states steeper than the configured
  28° final glide, which on plant B cannot be flared.
- **Plant C (audit plant) sweep** has not been re-run on the final build.
