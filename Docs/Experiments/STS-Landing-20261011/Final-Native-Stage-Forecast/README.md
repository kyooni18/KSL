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

The first run (ideal "smallest incidence for the required lift") ballooned: the
frozen learned lift table has a ~80 kN step between 5.5 and 6.0 deg at the L7 seed,
so the target overshot and the forecast floated at ~18 m. Native Final avoids this
with a local Newton step on measured lift that trusts only the table slope; the
forecast now does the same (`forecast.inc`). The final scores below use that step.
(First run, for the record: L5 pre-latch last −3.0/−0.8, L7 −18.6/−17.5.)

| Tail | Samples | Existing predictor | Native-stage lag 0 | Native-stage lag 0.5 s |
|---|---|---|---|---|
| L5 pre-latch | first / last / mean abs | +21.4 / +10.5 / 17.8 | +9.9 / +3.7 / 7.6 | +10.8 / +4.3 / 8.4 |
| L7 pre-latch | first / last / mean abs | +8.7 / −0.1 / 1.5 | −0.5 / −7.3 / 7.2 | −0.2 / −6.3 / 6.4 |
| L5 post-latch (>12 m) | first / last / mean abs | n/a (NaN) | +3.2 / −2.9 / 1.6 | +4.3 / −0.9 / 1.5 |
| L7 post-latch (>12 m) | first / last / mean abs | n/a (NaN) | −7.2 / −9.1 / 8.0 | −6.2 / −8.2 / 6.7 |

Per-tick values: `l5-forecast.csv`, `l7-forecast.csv`; summary `analysis.json`.

## Result

Modelling native onset and the Newton lift step cuts the L5 pre-pull error from
+10.5 to about +4 m/s and keeps post-latch L5 within about 4 m/s. L7, which the
existing predictor happened to get right (−0.1), is now 6–7 m/s low. It is
better than the existing predictor on L5 and worse on L7; **not adopted**.

Trajectory comparison at the seed ticks (`KSL_NSF_DUMP_UT`, lag 0.5 s) against the
native tails:

- Speed history agrees closely until contact (L7 at 66.6 m/s at +9 s in both).
- Native L7 arrests sink to 1.5 m/s at 11 m (t+8.5 s), below the 3.5 m/s shallow-glide
  sink, so the sink law commands a push-over (−1.4 to −2.0 m/s²). Achieved incidence
  falls 6.55 to 4.84 deg, lift falls from 339 to 199 kN, sink regrows to 4.1 m/s, and
  the incidence lag/slew cannot recover it before contact at 3.35 m/s sink. The
  forecast reaches 12 m with ~3 m/s sink and never enters this overshoot/re-descent
  limit cycle, so it floats 3.5 s longer and bleeds ~6 m/s more speed.
- The native arc is stronger than the forecast's: native commanded up to 7.9 deg and
  achieved 339 kN while the forecast stays near 6 deg. The frozen learned lift model
  underpredicts what the native plant delivers (the forecast cannot see in-flight lift
  learning, cf. the 3.8% / 7.3% drag deficit in `Final-Warmed-Tail`).

Hence the L7 residual is dominated by the arc overshoot into a push-over and
incidence-lag re-descent below 12 m, and by frozen lift learning. Both depend on the
unqualified attitude/lift plant, so a forecast cannot be certified without the
sim/live plant gap closed. This is a diagnosis of one simulated replay, not a
causal proof and not a live result.

## Next

No scalar correction. Candidate production-relevant lead (not implemented): the
push-over after arc overshoot below the shallow-glide sink with incidence lag. Test
offline whether limiting the push-over (or sizing the arc to avoid undershooting
3.5 m/s) removes the L7 re-descent, behind unchanged L5 golden and acceptance gates.
No production code, gate, golden or live setting changed in this experiment; the
aim/drogue live verification remains unapproved.
