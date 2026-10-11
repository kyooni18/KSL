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
in `chain-result.json` when complete; do not infer landing success from a
TAEM stop. Adaptive flare remains off and ground drogue drag is not modeled.
