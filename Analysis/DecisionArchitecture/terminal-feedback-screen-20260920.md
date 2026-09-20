# Feedback-coupled terminal forecast screen — 2026-09-20

This is a simulator/model diagnostic, not a landing result. It compares the
terminal forecast with the live terminal law after making both use the same
bounded pitch-response model and force-derived angle-of-attack feedback.

## Representative state

The direct terminal-corridor probe used approximately:

```text
altitude       22.0 km
true speed     540 m/s
flight path    -16 deg
heading        0 deg
along-track    -8 km from the configured terminal station
cross-track    0 km
```

Before the predictor was feedback-coupled, the forecast reported a positive
live-energy margin of approximately `+2.8 kJ/kg` and committed a terminal path.
The plant then increased force demand as the flight-path angle steepened. After
the correction, the same state reports approximately `-7.8 kJ/kg` and does not
commit the path. This is a safety correction to a false-positive forecast, not
a performance regression.

## State-space screen

A bounded diagnostic screen over the direct corridor sampled altitudes
`20--23 km`, speeds `450--600 m/s`, and flight paths `-8--16 deg`. No sampled
state produced an executable committed terminal path under the coupled
feedback/energy model:

- the low and high altitude edges commonly failed to obtain the required
  handoff;
- the middle band could obtain MM304 ownership, but candidate live-energy
  margins remained negative or the candidate was not commit-ready; and
- no sample reached touchdown or rollout.

The result means the current terminal-corridor curriculum is not an executable
landing benchmark. The route/energy/topology contract must be made feasible or
the curriculum must be replaced with a physically valid post-handoff state
before RL progress can be interpreted as landing progress.
