# Post-S-Turn failure-chain repair — 9 September 2026

Scope: fixes derived from `FlightLogs/2026-09-09T03-52-54Z-STS-N.jsonl` and the two preceding substantive TAEM failures. No live flight is claimed by this document.

## Repaired failure chain

- Degraded HAC candidates are still retained and exposed as the best available plan, but they are preview-only. A degraded join can no longer bypass prediction convergence and become a frozen path. Commit now requires a nominal candidate plus the normal prediction, response-time, arrival-energy, altitude, cross-track, FPA and course gates.
- Production TAEM can genuinely relinquish to Entry / S-Turn again. The former recovery predicate was impossible after normal TAEM acquisition because acquisition enabled `terminal_glide_mode` while the fallback required that mode to be false. Production and explicit HAC-rehearsal ownership are now distinguished, and return-to-entry clears terminal ownership before running fresh entry guidance.
- A persistent missing/degraded HAC during acquisition now accumulates a planning-loss timer. While meaningful maneuvering margin still exists, TAEM returns to Entry for a fresh S-Turn after the candidate has failed to become nominal for its response-dependent grace period. Reaching the actual alignment deadline without valid geometry still aborts rather than pretending a late S-Turn is safe.
- TAEM vertical-energy closure now has incidence priority whenever the aircraft is more than 4 deg shallower than the requested FPA or measured/estimated loss exceeds the remaining-path drag budget. Lateral guidance may still use bank, but it may not raise AoA and cancel the terminal unload during that state. Once vertical/energy convergence returns, normal lift sharing resumes.
- Roll-departure detection now uses one signed roll-rate observation consistently for both magnitude and braking direction. It no longer combines a large Euler-rate magnitude with an opposite-sign body-rate braking test, the pattern that falsely entered recovery in the 03:47 and 03:49 runs.
- Entering Attitude Recovery invalidates any frozen HAC join immediately. A successful recovery with adequate height/energy returns to Entry/S-Turn; a lower recovery remains terminal but must build a new HAC candidate. The pre-upset path is never resumed from a changed position/energy state.
- Recovery timeout and insufficient-height aborts now report distinct reasons.
- The post-flight terminal roll-controller correction that scales overspeed guard command against identified high-q authority is retained and now identified as `terminal-prediction-20260909-r19`. A regression prevents the old +/-0.14 high-authority bang-bang guard from returning.

## Validation

`make -C CLanding test` passes:

- Vessel physics tests
- C landing smoke tests, including degraded-HAC non-commit, real production TAEM->S-Turn return, consistent signed roll-rate recovery detection, and stale-HAC invalidation after recovery
- TAEM forecast transition tests
- HAC closed-loop join regression: 86.2 s, maximum path error 108.0 m
- HAC planning-state invariants

`python3 Validation/PythonBridgeTests.py` passes, including the inflated-authority terminal roll-braking regression.

`python3 Validation/BackendProtocolTests.py` passes.

Re-evaluating the recorded high-q states at log sequences 536-546 with the current `r19` bridge reduces the largest absolute roll command from the flown +/-0.140 pattern to 0.0462 while preserving the required braking sign. This is a controller-state replay, not a full KSP flight validation.

Backups of the pre-repair source used for this change are in `Runtime/Repairs/2026-09-09-post-s-turn/`.
