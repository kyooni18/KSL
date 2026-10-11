# Native-stage Final forecast (passive, rejected)

Experiment after `efae6ea`. `forecast.inc` is a temporary function spliced into a
scratch copy of `final/sequence.inc` by `run.py`; production files and binary are
untouched. At every Final tick above 12 m it evaluates a point-mass forecast that
reproduces the **native** logic instead of the existing predictor's idealised
profile: latch height with the incidence-pull response time and the admission
trigger, hold-until-`on_arc` onset with the `response+1` reserve, the
`design_sink` arc, the shallow-glide and float laws, and the 5 deg/s incidence
slew, with the learned force model frozen at the call. Two incidence-plant lags
are tried (0 and 0.5 s; neither is fitted to live data).

## Parity

`analyze.py` asserts, against `Final-Warmed-Tail`, that all 3,654 L5 / 4,928 L7
published packets (excluding wall-clock `sim_rate`) and all 725 / 930
`FINAL_TRACE` records are identical. Summaries unchanged (L5 66.129 m/s accepted,
L7 61.929 m/s unaccepted). The forecast is therefore passive.

## Forecast error against the flown touchdown speed (m/s, forecast minus actual)

| Tail | Samples | Existing predictor | Native-stage lag 0 | Native-stage lag 0.5 s |
|---|---|---|---|---|
| L5 pre-latch | first / last / mean abs | +21.4 / +10.5 / 17.8 | +8.4 / −3.0 / 5.9 | +9.0 / −0.8 / 6.8 |
| L7 pre-latch | first / last / mean abs | +8.7 / −0.13 / 1.5 | −3.1 / −18.6 / 13.1 | −2.0 / −17.5 / 12.1 |
| L5 post-latch (>12 m) | first / last / mean abs | n/a (NaN) | −2.5 / −12.7 / 6.3 | −1.3 / −4.5 / 2.4 |
| L7 post-latch (>12 m) | first / last / mean abs | n/a (NaN) | −18.3 / −16.5 / 15.2 | −17.6 / −14.3 / 12.3 |

Per-tick values: `l5-forecast.csv`, `l7-forecast.csv`; summary `analysis.json`.

## Result

Modelling native onset removes most of the L5 last-pre-pull error (+10.5 to
−3.0/−0.8) but makes L7, which the existing predictor got right (−0.13), 17–19 m/s
low. The existing L7 agreement was therefore partly offsetting errors. Mean
absolute error is worse on L7 than the existing predictor and better on L5, so
the change is **not adopted** and it does not meet a both-flights criterion.

The forecast's large L7 deficit is consistent with it flying the modelled float
(1 m/s sink, long high-incidence bleed), while the native L7 tail reached contact
at 3.35 m/s sink after 3.5 s below 12 m with achieved AoA maximum 7.6 deg. That
suggests an unmodelled low-speed lift/sink shortfall near the ground (the below-12 m
control and the lift actually achieved), not an onset error. This is a hypothesis;
it is not tested here. The forecast also omits the touchdown-attitude incidence
floor, contact physics and body-rate dynamics.

## Next

Do not add a scalar correction. Test the hypothesis by comparing the forecast's
below-12 m lift, sink and incidence history with the native tail on the same
timestamps. No production code, gate, golden or live setting changed; the
aim/drogue live verification remains unapproved and pending.
