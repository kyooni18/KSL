# Lift/drag table continuity (scratch variants, rejected)

**Finding.** `terminal_aero_q` (`CLanding/guidance/final/sequence.inc`) interpolates the
learned per-degree lift/drag nodes only when both neighbouring integer-degree nodes were
observed at a matching Mach (within 0.06). Otherwise it uses the analytic table scaled by
a single area EMA. The two levels can differ by tens of percent, so modelled lift is
discontinuous at the edge of the learned incidence range. In the L7 seed state the modelled
lift jumps from 186 kN (5.5 deg) to about 266 kN (6.0 deg). This breaks the monotonic
"smallest incidence for the lift" search (`terminal_aoa_for_lift`), the preflare planner and
the forecast; native flare partly hides it with a local Newton step on measured lift.

**Experiment.** `run.py` builds scratch backends (production untouched) that anchor the
analytic fallback to the nearest learned node's lift/drag ratio, fading with distance
(e-folding 1, 2 or 4 deg, applied within 3, 6 or 20 deg). Speed m/s / sink m/s / along m:

| Variant | L5 (golden 66.129 / 1.844 / 297.6, accepted) | L7 (baseline 61.929 / 3.353 / −376.2) |
|---|---|---|
| fade 1 deg | 57.15 / 1.57 / 618 (fails speed) | 56.57 / 1.16 / −295 |
| fade 2 deg | 75.28 / 5.85 / −44, off runway | 63.69 / 1.22 / −470 |
| fade 4 deg | 75.86 / 7.33 / −54, off runway | 52.55 / 0.80 / −227 |

**Decision: rejected.** Making the model continuous destabilises the accepted L5 golden
(hard, short landings for fades >= 2 deg), so the flare gains and triggers are tuned
around the current discontinuous model. A real fix would need joint retuning against the
full matrix and a qualified plant. Nothing was adopted or rebaselined; no gate changed. The
discontinuity is recorded as a known model defect to address with the plant-gap work.
