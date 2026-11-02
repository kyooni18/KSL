# Replacement completion audit and evidence dependency

Audit baseline: 8173e7e2c62fc57846d851f49ef86637a815201e. The requested improvement is not complete. No landing-readiness claim is justified. This audit retains the complete scope of the replacement research rather than declaring the implemented prerequisites sufficient.

## Implementation checkpoints from research section 8

| Requirement | Current authoritative evidence | Verdict / missing work |
| --- | --- | --- |
| 1. Complete immutable seed and separate proposal/evaluation/qualification/selection/tracking identities, including pending commands and progress, routed through telemetry/UI | NativeCommandSeed, full limiter snapshots, controller/plant continuation, terminal comparison contexts, trial IDs/ledgers exist | Partial. Actual old raw limiter/pending RPC history is not completely recovered. Unified runtime/UI identity and progress handoff are not integrated. |
| 2. Shared deterministic command/force/response rollout, accounting, explicit completeness | NativeCommandRollout is used by entry/terminal; schedule propagation shares it. Additive prefix/terminal work and envelope accounting, explicit unknown/invalid ends, requested/limited/issued histories exist | Software path implemented; physical closure incomplete. A shared interface is not empirical validation. |
| 3. Dynamics-generated acquisition/spiral families, reached join, bounded shooting, incumbent, no support-preserving radius search gate | Explicit command seed factory (direct, both senses, extra course turns, reversals), coupled sampled force seeds, bounded coordinate shooting, motion-derived reference exist | Partial and disconnected from production. Physical seed coverage/return/prolonged choices, dynamically reached regenerated spiral, timing and supported trajectory evidence remain missing. Legacy production geometry/radius search is unchanged. |
| 4. Complete all-bad recovery ranking, actual coupled alpha/bank/descent ownership, local no-forecast protection | Completion/context comparator and narrow q/load dominance for otherwise accepted stops exist | Partial. Crash/miss tradeoffs lack justified policy/model, general terminal deficits are not fully priced, and no new recovery schedule owns flight. TAEMGuidanceEngine's vertical-first/cannotHoldReference zero-bank law remains. |
| 5. Measured-state rebase/winding/progress, priced transitions, stale-plan rejection, four-layer display | Independent sampled course history, explicit spiral phase/reprojection helper, serializable propagated references exist | Partial. Helpers are not connected to runtime measured-state replanning or the display. Actual winding/join/control continuity and full visualization contract remain unverified. |

Source searches for TAEMCommandSearch, TAEMNormalForceFitter, TAEMTopologySeeds and SpiralPhaseHistory in Sources find their definitions/helper extensions but no production mission-controller caller. The latest recorded failed flight remained acquisition throughout, so it supplies no actual on-spiral rebase sequence. No new helper should be described as a deployed guidance replacement.

## Minimum decisive checks from research section 9

| Check | Evidence and scope | Completion |
| --- | --- | --- |
| Recorded-command reconstruction | Warm FCS reconstruction matches archived commands (maximum prebranch error 2.96e-14, held-out command error zero). Original raw limiter/reference rates and pending command history remain unknown. | Partial, not an exact complete warm planning seed. |
| Force/response closure | First supported balance at UT216412.69235289673; original support expires at +1.94 s. Last sampled supported prefix 1.82 s has alpha error 0.428°, bank -0.227°, speed +0.502 m/s, height -0.094 m; no acceptance uncertainty is identified. Longer 6.06 s alpha error is 4.61°. Local residual/regression and directional-torque substitutions have not established a valid model. | Not achieved. The rotational engine has not changed since a4b511bd, so rerunning its unchanged replay solely for reassurance would not resolve this. |
| One paired recovery comparison | No counterfactual turn has met the common seed/model-history and validated response prerequisite. Narrow command-path/event software tests are not that comparison. | Not achieved; do not launch a family-wide campaign to replace it. |
| Complete-price invariant | Focused metadata/accounting tests reject missing tails and context mismatch, retain incomparable outcomes, and allow a narrow better failed recovery. | Software invariant checked within its documented partial-order scope; full mission recovery ranking incomplete. |
| Join/winding/replan continuity | Pure geometry/progress bookkeeping and short native checkpoint continuation checks pass. No matched measured spiral-transition/replan sequence validates runtime integration. | Not achieved at requirement scope. |
| Terminal/display closure | Model contact/stop accounting exists; complete selected-candidate supported end-to-end continuation and correlated tracked display are absent. | Not achieved. No live landing/full stop is proven. |

All physical protections remain required: selected runway geometry/datums/right-positive crossrange, configured bank/AoA/RCS, limiter no-overshoot, ten-second RPC timeout/reconnect, one owner, exact experiment provenance, preserved saves and sustained intact runway stop. Inspection confirms transport/mapper defaults remain ten seconds; no new experimental flight limit is introduced by these helpers. No broad test suite or flight campaign was run for this audit.

## Evidence dependency and next action

The current blocker is empirical actuator/rotational response support, not a demonstrated optimizer runtime limit. The report explicitly requires the first two recorded-state checks before giving interpretive weight to alternative geometry. The repository likewise requires predictor validation against measured forces/response and identical command/actuator history before guidance changes. Enabling the new path on the strength of unit checks would violate that evidence requirement.

A fresh standard-kRPC read-only query attempt, run 3, replaced no process and failed in connection bootstrap with ConnectionRefusedError errno 61. It terminated with code 1 after 23.571 s before the query body; no game state/control changed and no new measurement was obtained. The safe record is `Validation/TAEM-Completion-Audit-20261103/krpc-read-only.json`. This is not proof that KSP is closed, only that the configured endpoint refused the connection. The previous availability question remains unanswered.

Next external prerequisite: make KSP/kRPC available and leave the game paused for read-only actuator/configuration inspection. Then determine the smallest matched-command measurement needed to close the model, with exact current vehicle/configuration and original/derived-save provenance. Loading/unpausing or any live control experiment still requires separate explicit authorization and the existing flight gates; endpoint availability alone grants none.

This is the first explicit impasse audit after completing independently useful prerequisites. Goal remains active; completion is unproven. Further generic helpers do not establish the missing physical model. Resume substantive guidance integration after new evidence supports it, then finish every missing checkpoint above and validate the full chain. Never declare the task complete around the software subset already implemented.
