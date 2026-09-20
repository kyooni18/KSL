# TAEM/HAC and glide control correction — 2026-09-07

The current C guidance, shared HAC geometry, predictor, and Python direct
controller were inspected together. These corrections address reproducible
control and state-machine defects; they are not a claim of flight qualification.

| Defect | Correction |
| --- | --- |
| HAC look-ahead chord adds nominal curvature on top of bank feed-forward | Shared course error removes the half-look-ahead angular bias. Both HAC sides and the predictor consume the corrected feedback. |
| Entry AoA limiter is interpreted as Euler pitch after TAEM transition | Preserve the AoA coordinate through TAEM, heading alignment, approach, and flare. |
| Banked terminal pitch loop uses Euler pitch and loses elevator authority through cosine attenuation | Bridge closes terminal pitch on measured AoA and its rate, retains aerodynamic trim, and uses the corresponding authority estimate. |
| Missed HAC exit resets remaining arc to a positive exit window | Preserve signed progress and latch abort beyond the exit window. Predictor stops the failed terminal propagation. |
| HAC completion can latch before a valid final approach, then continue commanding the circle | Complete HAC only inside the exit window with the final approach envelope satisfied. |
| Invalid flare alignment can leave its failure timer at zero because the wider final envelope passes | Use the flare envelope once flare is due, including during final approach. |
| Rollout/complete is revalidated against airborne speed and course limits | Ground rollout owns wheel steering and braking after touchdown. |
| Final approach can retract previously deployed gear | Preserve deployed gear. |
| Predictor effective-bank sine can decrease beyond 90 degrees | Bound the effective angle to 89 degrees before taking sine. |

Validation: `make -C CLanding test`, `python3 Validation/PythonBridgeTests.py`,
and `python3 Validation/BackendProtocolTests.py` pass. Added regressions cover
both HAC sides, nominal curvature, AoA handoff continuity, signed missed-exit
abort, normal HAC/final/flare progression, flare-invalid timing, terminal pitch
authority under bank, and rollout completion.

Recent September 7 logs examined during this audit predominantly stop in entry
or attitude recovery before TAEM. They do not validate the changed terminal
controller. No live KSP flight, running-process restart, or save reload was
performed. The rebuilt backend and updated bridge apply when those processes
are next started. Predictor post-TAEM vertical propagation remains an approximate
scheduled-AoA model, not a full replay of the live PID and powered approach.
