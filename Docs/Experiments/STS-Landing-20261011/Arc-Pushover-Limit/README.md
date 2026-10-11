# Shallow-glide push-over limit (scratch variants, not adopted)

Follows `Final-Native-Stage-Forecast`: the native L7 tail arrests sink to 1.5 m/s at
~11 m, the shallow-glide sink law (`arc_accel=ff+(sink-reference_sink)/FLARE_TAU`,
height <= 12 m) then commands a push-over of −1.4 to −2.0 m/s², incidence falls
6.55 to 4.84 deg, and sink regrows to 4.1 m/s before contact. `run.py` builds scratch
backends (production files untouched) that floor that push term and flies L5 and L7.

An earlier first attempt that floored the *arc-branch* push (`(sink-SGS_SINK)/PREFLARE_TAU`,
height > 12 m) changed nothing in either flight (all outcomes identical), confirming
the push-over is in the below-12 m branch.

| Floor on push term (m/s²) | L5 speed / sink / along | L7 speed / sink / along |
|---|---|---|
| none (baseline, −0.3 g clamp) | 66.129 / 1.844 / 297.6 (golden, accepted) | 61.929 / 3.353 / −376.2 (fails) |
| −1.5, −0.75 | identical to baseline | identical to baseline |
| −0.3 | identical to baseline | 62.185 / 3.698 / −381.2 |
| 0 (no push) | 64.916 / 1.372 / 327.5 (success) | 57.311 / 0.895 / −278.4 (fails) |

Removing the push-over cuts L7 touchdown sink from 3.35 to 0.90 m/s and lands 98 m
less short, but L7 still fails (speed 57.3 m/s is below the 63.75 m/s gate and it
touches down 278 m short), and L5's accepted golden moves (+30 m along, −1.2 m/s).
Floor −0.3 makes L7 slightly worse, so the effect is not monotonic or understood.

## Decision

Not adopted. The change would alter the accepted L5 golden and trade sink for speed;
no golden was rebaselined and no gate changed. It is a replay in the unqualified
attitude plant, so it is not evidence for live behaviour. Candidate follow-up: a
push-over limit scaled by the incidence response (lag) rather than a fixed floor,
evaluated across the L5/L6/L7/high-energy matrix; live approval remains pending.
