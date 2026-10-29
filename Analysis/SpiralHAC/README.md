# Spiral HAC implementation checkpoint — NOT flight qualified

The active native route now uses `r(psi)=Rf+a*psi^4`, with
`a=(Ri-Rf)/sweep^4`. Remaining angle decreases in flight. Position, tangent,
signed curvature and its first two station derivatives are evaluated from
the polar curve; arc length uses Gaussian integration of `hypot(r,r')`.
The quartic has zero first three radial derivatives at exit. A quintic
Bezier acquisition matches endpoint position, tangent and curvature; a
cubic curvature unload joins a straight settling segment at the existing
Final station. The unload duration bounds nominal bank rate and acceleration
using the captured attitude response model. These bounds are not substitutes
for native propagation with changing speed and vertical lift demand.

Candidate search varies radius and unwrapped sweep and prices the actual
spiral route. Prediction and execution sample this same route through the
existing coupled native tracker. MM304 physical ownership now requires native
acquisition/Final evidence, including when the energy-law mode is configured.
No Final safety gate was relaxed.

## Reproduction and results

From the repository root:

```
make -C CLanding BUILD=build-spiral spiral-hac-test test
CLanding/build-spiral/spiral_hac_tests --native
KSP_LANDER_MODEL_PROFILE=sts-live-20261028 CLanding/build-spiral/spiral_hac_tests --native
make -C CLanding BUILD=build-spiral taem-native-stack-test
make -C CLanding BUILD=build-spiral mm305-feasibility-test
```

The first command passes, including the mathematical derivative, handedness,
arc-length and join tests and the default terminal safety regressions. The
last two commands FAIL; their logs are retained here. The native family
matrix has an obsolete specific-status expectation, but this must not obscure
the substantive non-vacuity failure: **zero qualified MM305 deliveries** in
the feasibility fixture. Tests have not been weakened to turn that into a pass.

The CSVs compare reference and actual position, altitude, course, station,
radius, curvature, bank, AoA, Mach, speed and energy margin at every half
second. Both directions physically follow the spiral in native propagation.
For the identified model the roughly 33 km, 4.7-radian route contracts from
12 km to 4.224 km radius; maximum lateral errors are approximately 76 m and
station never moves backwards. This is a deliberately tangent-seeded capture,
not proof of arbitrary displaced acquisition. Across 14–19.5 km starting
altitudes, profile generation rejects Final delivery for inadequate speed.
The STS model traces likewise remain unqualified. The `sts-polar` filename
means an STS aerodynamic-model run, NOT replay of the recorded polar entry.

## Outstanding acceptance gaps

The altitude/speed profile and trajectory optimization still do not produce a
qualified Final handoff in these fixtures. Changing starting altitude alone
does not cure the low delivery speed. This is evidence of an unresolved
coupled planning/control problem, not proof that a feasible shuttle trajectory
is physically impossible. Robust arbitrary-state acquisition, continuous
committed replanning, atmospheric/aerodynamic uncertainty coverage and live
command-response parity need further work and qualification.

The recorded 2026-10-29 72 km scenario has not been successfully revalidated
end to end here. Its reported low-energy, large-cross-track handoff must not
be labelled nominal: an instantaneous energy/range check cannot establish
acquisition reachability, and the new native-evidence ownership gate is meant
to reject that unsupported handoff. A recorded-state quantitative replay is
still required before claiming this regression closed.

No live KSP process was restarted or terminated. No live KSP landing was
demonstrated. Passing the separate Final touchdown test does not certify the
spiral-to-Final chain. This checkpoint does **not** meet all requested
acceptance criteria.
