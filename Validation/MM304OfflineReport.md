# MM304 → TAEM offline changes — 2026-09-14

No KSP flight was launched, continued, restarted, or supervised. The working tree,
recorded v28/v29/v32 logs, deterministic guidance/predictor tests, and fake transport
were used. The source geometry remains approximately (-32.9 km, ±12 km, 21.8 km,
775 m/s, runway course, -9.8°) with the recorded 20 km interface configuration.

## Causes and changes

- The vertical ceiling used a quarter-ETA pull-up demand, capped at 45 seconds.
  When that demand exceeded measured lift, both longitudinal and geometry bank
  were clipped to zero. It now allocates lift against remaining altitude and
  terminal vertical-speed budgets, including response lead. This is a local
  constant-acceleration bound; executable trajectory replay checks the changing
  aerodynamic/energy response. It does not force bank through a lift deficit.
- DRAGREF integrated a feasible drag floor but published a lower, unexecutable
  command. The v29 regression was already failing on entry. Reference drag and
  its derivative now use the same bounded profile as the range integration;
  the clipping flag preserves evidence of the impossible unconstrained request.
- A retained reversal was not included in the reshaped forecast. It now is.
  Routine updates cannot postpone or downgrade a committed final reversal.
  Only a committed-segment boundary miss from executable replay allows replacement.
  A missing reversal is exposed even when same-side geometry demand returns zero.
- MM304 performed broad candidate search, supervision, and diagnostic prediction
  synchronously under the controller mutex. Entry planning now uses the existing
  prediction worker and an immutable copy of guidance, telemetry, configuration,
  aerodynamic calibration, and the physics data book. Live guidance recomputes
  the coupled demand and safety overrides each tick. No PID, limiter, actuator,
  executive, or reversal-execution state is copied back.
- Admission checks generation (including operator/configuration changes), phase,
  lineage, side, reversal, target, finite plan data, a three-second age bound,
  segment expiry, and event deadlines. A pending initial fallback requests a real
  topology immediately. An accepted result creates new lineage; duplicate or old
  results cannot overwrite it.
- Production forecasts replay actual guidance at its configured Entry tick rate,
  with the same target, event, bank/alpha law, vertical shaping, safety overrides,
  and modeled attitude response. They do not recursively run future candidate
  searches. Legacy topology candidates remain advisory and cannot publish an
  executable feasibility certificate. Forecast readiness also requires clean
  terminal geometry and regular executable terminal policy, plus safety limits.
- Same-side final-event replay is supported; the legacy tangent predictor cannot
  invent an opposite bank after its final event. Measured stall evidence is retained
  conservatively until the reduced response model lowers incidence.

The shared capture function, veto bits, HAC feasibility logic, and attitude-recovery
thresholds were not relaxed. `math.c` and `taem_exec.c` match the pre-change copies.

## Validation

Commands:

```sh
make -C CLanding mm304-seam-test mm304-recorded-replay
make -C CLanding native-component-tests cnano-transport-test
make -C CLanding BUILD=build-mm304-sanitized \
  CFLAGS='-O1 -g -std=c17 -Wall -Wextra -Werror -pthread -fsanitize=address,undefined' \
  LDFLAGS='-pthread -fsanitize=address,undefined' \
  mm304-offline-test mm304-worker-test
```

- Dynamic-interface, reversal/admission, lift-budget, predictor-parity, blocked-worker,
  native component, C-Nano transport/client/batch, guidance integration, TAEM production,
  and offline landing gate tests passed. ASan/UBSan reported no errors in the focused
  control/admission/worker tests.
- The production worker was deliberately held at the replay call while 50 control
  ticks ran. Worst normal-build tick was 1.598 ms in the final normal-build run; the five-second-old
  result was rejected. No transport was opened.
- A separate 1,000-tick test held planning unavailable for 100 simulated seconds,
  checked the dynamic bank limits, then executed exactly one final reversal and
  prohibited opposite post-reversal bank.
- Replayed 2,245 complete MM304 measured states from the v32 vehicle stream with
  planning deferred. Worst local guidance call was 2.772 ms in the final recorded
  evidence run. At UT 67559.443642, historical guidance took 1,863.379 ms and the
  recorded total loop took 1,984.837 ms; the offline local replay took 0.122 ms.
  These are local timing tests, not end-to-end RPC latency or trajectory acceptance.
- With worker outputs deliberately withheld in that measured-state replay, 2,213
  frames explicitly reported unproven delivery due to lift or missing reversal.
  The replay is open loop through recorded states; it does not claim a new flight
  would follow the old trajectory or reach TAEM.
- A reconstructed v32 worker snapshot ran a real search (~29 ms) and actual-guidance
  replay (~854 ms). Two identical replays produced identical final state and event
  results without modifying the input guidance or launching another search. Its
  reduced model reached the capture interface; this is not a regular-HAC flight proof.
- `make qualification` is not green: its existing full-75-km success assertion
  (`EntryPredictorSupervisionTests.c`, `saw_taem`) fails. A separately compiled copy
  of the original source also fails the same assertion, at UT 67561.6 with speed
  563 m/s and no reversal. The changed-source replay also misses that scenario.
  The assertion was preserved. That fixture uses the default 16.5 km interface
  configuration, not the recorded 20 km configuration used for this task's seam.

Historical corroboration: v29 first logs bank 0/Bgeo 70 at UT 67426.7 near 41.0 km;
v32 first logs it at UT 67423.5 near 41.1 km. No matching Bgeo format was found in
v28. Original/current 75-km failure logs and focused outputs are in
[MM304OfflineEvidence](MM304OfflineEvidence/).

## Remaining limits

The arrival-budget bound assumes locally constant acceleration, and the shadow
uses a reduced bank/AoA response model. A future explicitly authorized flight is
needed to verify measured lift/drag evolution, actuator delay, the continuous final
arc under disturbances, and delivery into a regular HAC. This work does not certify
mission completion, a regular-HAC live outcome, or the failing full-75-km scenario.

## Changed files

Production: `CLanding/guidance.c`, `CLanding/controller.c`, `CLanding/predictor.c`,
`CLanding/entry_drag_reference.c`, `CLanding/landing.h`, `CLanding/Makefile`.

Updated tests: `Validation/DynamicTaemInterfaceTests.c`,
`Validation/EntryDragReferenceTests.c`.

Added tests: `Validation/MM304OfflineTests.c`, `Validation/MM304WorkerTests.c`,
`Validation/MM304PredictorParityTests.c`, `Validation/MM304RecordedReplay.py`.

Report and captured evidence: `Validation/MM304OfflineReport.md`,
`Validation/MM304OfflineEvidence/`.
