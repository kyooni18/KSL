# MM305 construction plan

Source: the supplied `MM305_Reconstruction_Engineering_Plan.md` and `analytic_checks.json` (25 September 2026). The JSON validates algebraic identities only; it is not a flight or plant-parity result. This plan is for the active Git checkout at `/Users/kyooni18/Code/KSPShuttleLander`. Preserve its existing MM305 purge edits and generated runs.

## Architecture and sequence

1. **Baseline and model snapshot.** Record source/configuration hashes, dirty state, independent build/test results, active scenario inputs, and the *effective loaded* world, aero, response, vehicle, runway, and Final model. Keep seeded defaults distinct from overrides. Establish immutable, versioned model and complete dynamic-state types.
2. **Shared native plant.** Extract ShuttleSim's world, force, attitude-response, and airborne-step kernels into a dependency-free shared target used by ShuttleSim and CLanding prediction. Preserve pre-step force evaluation, velocity/position advance, then attitude advance. Require tick-for-tick parity before planner acceptance work.
3. **Frames and physical accounting.** Implement inertial/planet-fixed/runway transformations, both runway ends, gravity and energy/work diagnostics, force allocation, and signed constraint margins. Treat the supplied numerical checks as independent oracles; verify them against the shared kernel.
4. **Actuator-aware tracker and Final acceptance.** Build finite-response turning witnesses and a bounded tracker. Separately characterize the unchanged Final controller's joint delivery set, including learned state and complete touchdown/rollout tails.
5. **Offline trajectory planner.** Add direct, lead/turn, and justified energy-extension families. Derive analytic seeds, then forward-propagate full-horizon commands with hard limits. Use bounded shooting/refinement only after deterministic native propagation and Final-tail qualification work. Keep infeasible, unqualified, and search-exhausted statuses distinct.
6. **Shadow integration and ownership.** Run the planner without command ownership first. Add qualified incumbent, versioned worker results, commitment, validated splicing, and a single command writer. Replace MM304 admission and MM305/Final handoff contracts only after shadow evidence and native tail qualification.
7. **Flight qualification.** Replay MM305 to runway touchdown and stable rollout, then broaden to MM304 handoffs and full flights under declared model dispersions. Require three consecutive valid runway landings for designated nominal cases. Do not count a selected HAC, forced capture, touchdown flag, abort, or zero-simulation-time run as success.

## Module boundaries

Use stable `.h`/`.c` interfaces by responsibility: `terminal_model` and a shared `flight_physics` leaf target; `terminal_propagator` and `terminal_solver`; `taem_geometry`, `taem_reachability`, `taem_planner`, and `taem_tracker`; `final_interface`; `taem_exec` and the narrow `guidance_taem` owner. Keep model snapshot lifecycle and worker cancellation in the existing controller layer. Version diagnostic/replay records. Do not restore deleted historical `.inc` planning paths or mix Final controller changes into an MM305 feasibility claim.

## First parallel assignments

- **A — baseline/model contract:** produce an effective-model manifest/export and a reproducible baseline report without changing flight behavior or shared build files.
- **B — shared plant:** extract the native airborne transition and its physical dependencies into a small reusable target, with parity verification; own relevant ShuttleSim and build surfaces.
- **C — frames/energy:** implement isolated runway-frame and energy/work diagnostic primitives plus algebraic tests, using the supplied JSON as reference; avoid B's files.

Integrate A/B/C only after interface review. The gate to subsequent planner work is an immutable effective snapshot, native tick parity, and frame/energy tests. The gate to live MM305 command ownership additionally requires complete candidate and unchanged Final-tail qualification.

## Integration checkpoint, 25 September 2026

Stage 0 exported `Analysis/MM305Stage0/effective_model_manifest.json`; its captured runtime telemetry came from an older binary and remains labeled stale against the source snapshot. Stage 1 extracted the ShuttleSim airborne kernel into `flight_physics`, now built by both ShuttleSim and CLanding. Stage 2 added the independent `taem_frames_energy` module to the CLanding build and maintained test gate. The two math libraries had three global-name collisions, resolved by namespacing ShuttleSim's `v3`, `clampd`, and `runway_coordinates` functions; no arithmetic was changed.

Fresh builds in separate `/tmp/mm305-unified-*` directories passed for ShuttleSim, its offline and expert libraries, and the CLanding backend. The 512-tick native parity fixture, the 256-case frame/energy algebraic suite, and the maintained `make -C CLanding -j1 BUILD=/tmp/mm305-unified-clanding test` gate passed; `git diff --check` passed. These are build and focused numerical checks. CLanding links the shared physics code but the future terminal propagator does not yet call it, the model snapshot is not yet an immutable runtime object, and neither Final-tail qualification nor MM305 flight execution has been attempted at this checkpoint.
