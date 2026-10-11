# Experimental lateral heading-arrest projection

Original live L7 crosses the runway centreline while course is still 11.18°
away from runway heading. It then needs about 150 s to arrest that lateral
motion and drifts about 34.5 km. See the [measured chronology](../Entry-Lateral-History/README.md).

The candidate adds constant-lift/bank lateral stopping distance after the
existing roll-exchange projection. For local course angle theta, turn radius
R = V²/(lift × sin(bank)), the displacement is R(1−cos(theta)). In measured
crossrange-rate coordinates this is `signed(rate² / (lift × |sin(bank)| ×
(1 + cos(theta))))`. The candidate updates the rate through the existing
roll-response interval, then projects arrest. It applies only with finite
forward course alignment and available turning authority. Existing uncertainty,
leg capture, roll limits, dwell and azimuth/reversal gates still apply.

**Off by default and unqualified.** Set `KSP_LANDER_ENTRY_LATERAL_ARREST_TEST=1`
only for diagnostic replay. No live profile opts in. The runner records that
environment explicitly. No deorbit target, acquisition boundary or acceptance
gate changes. Existing focused lateral tests add a mirrored analytic stopping
case and zero/invalid authority cases; full tests retain baseline behavior.

## Paired MM304 results

`run.py` uses the measured MM304 inlets, unchanged current STS configuration,
and existing coefficients in either legacy or body-coordinate integration.
Both plants remain provisional. Deliberate TAEM stops exit 2; this is an
unfinished landing, not failed admission. `result.json` preserves commands,
provenance, final backend telemetry and simulator endpoint separately.

| Inlet / plant | Baseline cross at TAEM | Candidate cross | Candidate altitude / speed |
|---|---:|---:|---:|
| L5 / legacy | −2.340 km | −2.343 km | 24.452 km / 860.716 m/s |
| L5 / body | −2.200 km | −2.200 km | 24.427 km / 859.863 m/s |
| L7 / legacy | −41.662 km | −37.677 km | 24.718 km / 861.120 m/s |
| L7 / body | −43.179 km | −38.482 km | 24.394 km / 859.830 m/s |

L7 improves about 4–5 km; L5 is essentially unchanged. L7 is still poorly
aligned and does not reproduce the original live −32.53 km handoff.
Instantaneous lift held constant over a long arrest does not forecast rapidly
changing density, drag, flight-path rotation or energy allocation. This is not
a qualified entry guidance change. Full-chain comparison is recorded separately
in `chain-result.json`; do not infer landing success from a TAEM stop.
Adaptive flare remains off and ground drogue drag is not modeled.

## Full MM304-to-contact comparison

`run_chain.py` holds the corrected L7 MM304 inlet and existing-coefficient
body plant fixed, changing only the experimental flag. Both cases exit 2
and fail acceptance. Neither observes Final Approach or Flare.

| Case | Contact speed | Sink | Along | Cross | Gear | Backend failure |
|---|---:|---:|---:|---:|---|---|
| Baseline | 33.064 m/s | 24.203 m/s | −9794.148 m | +644.938 m | Up | MM305 crossed runway elevation before fixed-HAC exit |
| Arrest projection | 37.007 m/s | 14.784 m/s | −7066.021 m | −637.338 m | Up | Recovery exceeded 15 s |

The candidate is **not accepted for production**: a better TAEM crossrange
outlet did not produce the required downstream phase path or touchdown.
The simulator continues after backend failure, so contact is its subsequent
physical outcome, not a guidance success. These surrogate-chain failures
also differ sharply from the observed live MM305 continuations that reached
flare. Do not tune or rebaseline successful landing around them.

`make`, `make test` (104 tests, 44 subtests) and the native TAEM stack check
pass with the experiment off by default. The analytic mirror/zero/invalid
authority cases cover the new projection formula; the six retained replay
runs exercise the opt-in behavior. Native stack success does not qualify this
experimental full flight. No new live flight was authorized or performed.
