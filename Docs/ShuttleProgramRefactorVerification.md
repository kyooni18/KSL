# Shuttle Program Refactor Independent Qualification

Verification snapshot: 2026-09-09 22:46 KST

Verifier: `@VERIFY`

Scope: independent offline qualification of the integrated `KSPShuttleLander` refactor against `AgentWork/ShuttleProgramRefactor/COMMON_CONTEXT.md`, `DECISIONS.md`, worker contracts 01-10, and the current production path. This snapshot supersedes the earlier 21:54 pre-integration report.

## Overall verdict

**FAIL - regression health is green, but the integrated architecture is not yet qualified for live KSP testing.**

The integration work is materially present. Production MM304 now calls the dedicated Entry executive, drag-v/range reference, scheduled alpha, Entry lateral allocator, and bounded predictor supervisor. `GuidanceMachine` persists `EntryExecutive` and `TaemExecutive`; MM305 ownership is latched and exported in snapshot telemetry; the explicit A/L state machine is active. The prior integration failures V-001 through V-005 are therefore superseded.

Qualification nevertheless found three current production integration defects. First, normal MM304 initialization skips the load-gated Pre-entry phase because the drag-reference profile supplies a phase floor that `entry_exec_initialize()` treats as restart classification even when `checkpoint_restart=false`. Second, the production MM305 profile hard-disables the architecturally required optional excess-energy TAEM S-turn. Third, `controller.c` still runs the legacy broad Entry planner as a live state producer and writes the same control-plan timestamp/state used by the new supervisor, leaving contradictory ownership and allowing a legacy forecast solve to suppress a scheduled supervisor refresh.

No live KSP flight was started because the offline qualification gate is red.

## Requirements traceability

| Requirement | Verdict | Current source / test evidence | Qualification note |
| --- | --- | --- | --- |
| MM304 five-phase executive | **FAIL** | `CLanding/entry_exec.c`; live `entry_program_guidance()` in `CLanding/guidance.c`; `Validation/EntryExecTests.c`; `Validation/GuidanceIntegrationContractTests.c` | The executive and nominal production dispatch exist, but a normal first production frame below the Pre-entry load threshold initializes at least at Temperature Control. The verifier production-path test reproduces the skipped Pre-entry gate. |
| Drag-v / range reference guidance | **PASS** | `entry_drag_reference_compute()` and `entry_drag_reference_exec_profile()` are called by `entry_program_guidance()`; `Validation/EntryDragReferenceTests.c` | The nominal command path now derives continuous drag/range demand and vertical-lift demand from DRAGREF. The legacy altitude law is no longer the ordinary nominal dispatcher. |
| Velocity-scheduled / bounded alpha | **PASS** | `entry_alpha_command()` in live MM304; `Validation/EntryAlphaTests.c` | Scheduled alpha is a caller-owned nominal input and remains bounded by q/g/stall and vehicle limits. |
| Entry bank / crossrange reversals | **PASS** | `entry_lateral_update()` in live MM304; `Validation/EntryLateralTests.c` | The live nominal bank command comes from the dedicated longitudinal/lateral allocator with bank-capture, dwell, hysteresis and authority limits. |
| Predictor as supervisory layer | **FAIL** | `predictor_supervise_entry_control()` is called by `entry_program_guidance()`; `Validation/EntryPredictorSupervisionTests.c`; legacy call remains in `CLanding/controller.c` | The final MM304 command is correctly formed from DRAGREF/ALPHA/LATERAL and then supervised, but `dynamic_prediction()` still calls `predictor_plan_entry_control()` and writes legacy plan/reversal fields plus `entry_control_plan_ut`. The supervisor refresh test in `guidance.c` uses that same timestamp, so the old producer still has live ownership side effects. |
| Nominal one-way Entry -> TAEM handoff | **PASS** | `terminal_force_acquisition()`, `taem_exec_enter()`, `terminal_entry_recovery_available()`, `Validation/ShuttleArchitectureContractTests.c` | MM305 ownership is latched. Once `terminal_region_entered` is set, the ordinary terminal path cannot chatter back to Entry; stale MM304-complete inputs cannot clear `TaemExecutive` ownership. |
| TAEM phases 0/1/2/3 with optional high-energy S-turn | **FAIL** | `CLanding/taem_exec.c`; `Validation/TAEMExecTests.c`; `Validation/TAEMProductionIntegrationContractTests.c` | The executive implements S-turn -> Acquisition -> Heading Alignment -> Prefinal and unit tests pass, but `taem_exec_profile_production()` sets `s_turn_enabled=false`. Production can therefore never select phase 0 under the architecture's excessive-energy condition. |
| HAC Acquisition / Heading Alignment / Prefinal semantics | **PASS** | persistent `TaemExecutive`; `taem_exec_sync()`; phase projection in terminal guidance; current HAC guidance path | Acquisition/Heading Alignment/Prefinal are now driven from persistent MM305 ownership rather than reconstructed solely from coarse public booleans. The optional phase-0 production defect above is separate. |
| A/L Trajectory Capture / Steep / Shallow / Final Flare | **PASS** | `terminal_set_stage()` and `terminal_approach_sequence()` in `CLanding/guidance.c`; `Validation/ApproachSequenceContractTests.c` | A/L progression is monotonic and active: Trajectory Capture -> outer/steep glide -> Preflare/shallow transition -> Inner Final -> Touchdown Flare -> Ground. Backward stage requests are rejected. |
| q/g/stall / attitude-recovery protections | **PASS** | Entry alpha/lateral inputs, predictor supervision, `control_recovery_needed()`; focused tests and vessel-physics suite | Existing load, stall, bank/authority, roll-oscillation, pitch/yaw departure and recovery protections remain present. No verifier change weakened these gates. |
| Nominal progression vs off-nominal recovery | **PASS** | `TaemExecutive` recovery state/reasons; `PHASE_ATTITUDE_RECOVERY`; terminal path invalidation/replan logic | Recovery remains explicitly separate from nominal phase progression and does not clear latched MM305 ownership. Pre-TAEM legacy fallback is explicitly labeled recovery/degraded behavior. |
| External phase / status terminology | **PASS** | `phase_string(PHASE_ENTRY_ENERGY)` now emits `MM304 Entry`; nominal Entry status emits `MM304 <subphase>` | The misleading public `Entry / S-Turn` label is gone. Old `Entry MPC` / `S-turn altitude capture` strings remain only inside the explicitly labeled legacy recovery fallback and are not the nominal MM304 status surface. |
| Executive telemetry | **PASS** | `controller.c` snapshots `entry_exec_telemetry()` and `taem_exec_telemetry()`; `models.c` serializes `entryExecutive` and `taemExecutive` | The active subphase and MM305 ownership/recovery information is exported for logs/UI qualification. |

## Regression and qualification evidence

The ordinary offline regression suite is green:

```sh
make -C CLanding test
python3 Validation/PythonBridgeTests.py
python3 Validation/BackendProtocolTests.py
```

Results on this snapshot: vessel physics PASS, A/L contract PASS, Entry Alpha PASS, Entry Lateral PASS, Entry Executive PASS, TAEM Executive PASS, DRAGREF PASS, predictor supervision PASS, MM304->MM305 architecture seam PASS, Python bridge PASS, backend protocol PASS. Native code is compiled with C17 `-Wall -Wextra -Werror -pthread`.

`@VERIFY` added a separate canonical qualification target so module/regression health and integrated architecture qualification cannot be confused:

```sh
make -C CLanding qualification
```

Current result: **FAIL** after all ordinary native tests pass. It stops at `Validation/GuidanceIntegrationContractTests.c:75` because the first normal MM304 frame does not remain `ENTRY_PHASE_PREENTRY` below the load gate.

The second production contract was also compiled and executed independently so it was not hidden behind the first failure:

```sh
cc -ICLanding -O2 -std=c17 -Wall -Wextra -Werror -pthread \
  Validation/TAEMProductionIntegrationContractTests.c \
  $(find CLanding/build -maxdepth 1 -name '*.o' ! -name 'main.o' ! -name 'guidance.o' -print | sort) \
  -lm -pthread -o CLanding/build/taem_production_integration_contract_tests
CLanding/build/taem_production_integration_contract_tests
```

Current result: **FAIL** at line 11 because `taem_exec_profile_production().s_turn_enabled` is false.

## Blocking qualification defects

### Q-001 - Normal MM304 startup bypasses Pre-entry

Owner request: `@INTEGRATE` with `@EGEXEC` / `@DRAGREF`.

`entry_drag_reference_exec_profile()` always supplies `has_phase_floor=true` and a phase floor of at least Temperature Control. `entry_exec_initialize()` always calls `restart_phase()`, which consumes that floor even when `EntryExecObservation.checkpoint_restart` is false. This turns a normal initial frame into restart classification and bypasses the declared Pre-entry load gate.

Required correction: keep phase-floor classification for true checkpoint/re-entry restoration, but make a normal initialization start at Pre-entry and advance only after its load gate. `Validation/GuidanceIntegrationContractTests.c` must pass afterward.

### Q-002 - Production MM305 cannot select its optional high-energy S-turn

Owner request: `@INTEGRATE` with `@TAEMEXEC`.

`TaemExecutive` correctly implements the optional excessive-energy S-turn, but `taem_exec_profile_production()` explicitly sets `s_turn_enabled=false`. That contradicts the current architecture contract in `COMMON_CONTEXT.md` and `09_VERIFY.md`.

Required correction: either enable the production phase with bounded, physically justified entry/termination energy thresholds and terminal-feasibility gating, or explicitly revise the architecture contract if phase 0 is intentionally being removed. Under the current contract, `Validation/TAEMProductionIntegrationContractTests.c` must pass.

### Q-003 - Legacy Entry optimizer still mutates live control ownership state

Owner request: `@INTEGRATE` with `@EGPRED`.

`CLanding/controller.c::dynamic_prediction()` still calls the compatibility `predictor_plan_entry_control()` broad optimizer and writes `entry_control_plan_ut`, bank/AoA/heading, reversal scheduling and associated control-plan state. The new nominal path then calls `predictor_supervise_entry_control()`, but its refresh predicate also uses `entry_control_plan_ut`. A legacy prediction solve can therefore advance that timestamp before `entry_program_guidance()` runs and leave the new supervisor reusing stale correction state.

Required correction: the controller-side predictor may remain a forecast/telemetry producer, but it must stop writing nominal MM304 control ownership or the supervisor's timestamp/reversal state. Give forecast caching separate state if needed. The only nominal MM304 producer should remain DRAGREF + ALPHA + ENTRYLATERAL, followed by bounded predictor supervision.

## Verifier-owned additions

- `Validation/GuidanceIntegrationContractTests.c` - production MM304 startup/load-gate contract; currently failing and intentionally preserved as a qualification gate.
- `Validation/TAEMProductionIntegrationContractTests.c` - production optional TAEM S-turn availability contract; currently failing and intentionally preserved as a qualification gate.
- `CLanding/Makefile` - new `qualification` target that runs the ordinary green regression suite first, then the production integration contracts.

## Qualification boundary

This qualification is complete for the current offline tree and the verdict is **FAIL**. Live KSP qualification was not started because known deterministic architecture violations already make the candidate non-qualifying, and the `@VERIFY` contract also requires explicit `@INTEGRATE` authorization for live flight work. After Q-001 through Q-003 are fixed, `make -C CLanding qualification` must pass before any Entry -> TAEM -> HAC -> A/L live qualification run is considered meaningful.
