# MM305 roll response and entry sweep — 2026-09-26

All flight evidence here is from ShuttleSim with `engageHACTest`, the identified
STS-N plant, ideal attitude servo, and the unchanged Final admission code. No
live KSP connection or control was used. The entry states are physically
consistent synthetic surrogates, not measured MM304 handoffs. Their source and
generator are `ShuttleSim/scenarios/mm305-entry-envelope.json` and
`Tools/generate_mm305_entry_envelope.py`. Run manifests and compressed flight
records are in `ShuttleSim/runs/` under the IDs below.

## First divergence and control change

At 26 km, Mach 2.60, along −60 km, heading 90°, and FPA −8°, the old lateral
PD horizon was 3.36 s (`4/(roll_wn*roll_zeta)`). The identified maximum roll
rate is 12.45°/s, so reversing a permitted +70° bank to −70° takes at least
11.2 s. In run `mm305-baseline-60km-20260926T101729Z-mh73rs7v`, the target
reversed between near-full banks five times before 100 s (22.9, 37.8, 47.1,
56.8, and 66.3 s). This is the earliest source of the observed S-turn wobble;
route replanning had not yet caused it. The controller now uses the greater of
settling time and full-bank reversal time as its lateral response horizon.
Native route replay and live tracking share the law. The regression checks
that slowing the roll actuator softens the same cross-track correction.

With that one change, the same entry had no near-full reversal in the first
100 s (`mm305-roll-response-horizon-20260926T102506Z-7o3bebot`). The path
still made a smooth ~2.6 km lateral excursion to acquire its HAC. With the
runtime 3.2 km/28° Final geometry, it contacted at along +1728 m/cross −125 m
and failed. Smoother MM305 commands alone are not landing qualification.

## Isolated exit geometry experiments

Temporary configs changed `finalApproachDistance` and `finalGlideSlope`
together; none was committed as production tuning. They affect both MM305's
fixed exit and the existing Final path, so the result is a whole-interface
experiment rather than an isolated MM305 parameter estimate.

| Distance / slope | Exact entry result | Nearby-state result |
| --- | --- | --- |
| 3.2 km / 28° (runtime) | failed: +1728 m, −125 m contact | not swept |
| 4.5 km / 20° | passed once: +609 m, −26.7 m, 63.8 m/s, 1.61 m/s sink (`mm305-4500m-roll-horizon-20260926T102534Z-7uuu0vi7`) | 1 of 13 perturbed states passed |
| 4.5 km / 18° | ~61 m/s contact, 2.4–3.0 m/s sink | four nearby states all speed-short |
| 4.2 km / 18° | ~55 m/s contact | runway misses among four states |
| 4.8 km / 18° | ~47 m/s, 8.4 m/s sink | four states failed |
| 4.5 km / 19° | ~62 m/s, 2.5 m/s sink | four states speed-short; one cross −36 m |
| 4.5 km / 19.5° | 53.8 m/s, 9.3 m/s sink | four states failed, including route-side instability |

The 4.5 km/20° accepted run exited HAC at along −4689 m, cross −3.7 m,
height 1605 m, 144.1 m/s, FPA −17.6°, and bank +7.3°. Its large lateral
excursion was smooth rather than a rapid series of target reversals. It does
not establish a robust handoff setting.

## 13-state perturbation sweep at 4.5 km / 20°

The generated states varied altitude 24.5–27.5 km, airspeed 750–830 m/s,
heading 87–93°, lateral offset ±1.2 km, downrange offset ±4 km, and FPA −6 to
−10°. All 13 reached live HAC exit. Near-full bank reversals before 100 s were
zero or one per case, compared with five before the controller change.

| Case | Contact speed (m/s) | Sink (m/s) | Along (m) | Cross (m) | Full result |
| --- | ---: | ---: | ---: | ---: | --- |
| nominal-quantized | 64.0 | 4.7 | 610 | −29 | fail |
| alt-low | 64.1 | 1.6 | 614 | +11 | pass |
| alt-high | — | — | — | — | runway miss |
| speed-slow | 63.9 | 4.7 | 633 | −25 | fail |
| speed-fast | 63.7 | 1.6 | 533 | +21 | fail |
| heading-left | 62.7 | 5.8 | 638 | −2 | fail |
| heading-right | 64.2 | 4.2 | 617 | +15 | fail |
| cross-north | 50.3 | 9.3 | 861 | −13 | fail |
| cross-south | 64.3 | 3.5 | 596 | −13 | fail |
| range-far | 62.9 | 3.0 | 621 | −10 | fail |
| range-near | 61.7 | 8.0 | 675 | −38 | fail |
| fpa-shallow | 63.9 | 4.0 | 613 | +11 | fail |
| fpa-steep | 55.7 | 2.1 | 802 | −21 | fail |

The exact checkpoint and the rounded nominal reconstruction differ by less
than 0.001 m/s initially, yet the 4.5 km/20° runs handed Final roughly 8 m
apart in height and contacted with 1.61 versus 4.75 m/s sink. At 19.5° the
same tiny perturbation selected opposite HAC sides and one route required a
later release/replan. Route selection and downstream response remain fragile.
The next MM305 change should be driven by exit-state margin and topology
stability over nearby states, with Final's existing gates left intact.

## Direct-control check

`mm305-direct-control-roll-horizon-20260926T104138Z-66aml3yu` used the same
exact entry and 4.5 km/20° temporary config with ShuttleSim's surface-moment
attitude plant. The first 100 s again had no near-full bank-target reversal.
Vehicle tracking was weaker: one live exit event occurred at cross −518 m,
232 m/s, and +57.5° bank, followed by another near cross +6 m and 148 m/s.
Contact failed at 61.6 m/s and 8.57 m/s sink. The direct-control moment
coefficients are generic, not a qualified STS-N identification, so this run
exposes a control-loop sensitivity without predicting live-KSP response.

## Rejected transition change

An experiment required the existing priced Final contract to pass before
latching HAC exit. It failed: the controller stayed on the exhausted HAC
endpoint while post-HAC runway-line conditioning was needed, and the exact
entry crashed in TAEM at along +1819 m/cross +164 m with 8 m radar height
(`mm305-final-contract-latch-servo-20260926T104437Z-_96_4gyu`). The change
was removed. HAC geometry exit and Final admission are distinct boundaries;
future MM305 work must explicitly account for the conditioning segment.
