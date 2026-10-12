# Guided deorbit opportunity search — polar STS, 2026-10-12

## Why this change exists

The preceding [Polaris overflight investigation](Polar-KSC-Overflight-Polaris-20261012.md) demonstrated that the first selected orbital approach carried the orbiter past KSC at too high a Mach number, altitude and unfavorable geometry for a qualified MM305/HAC-to-Final recovery. A small orbital ground-track miss at the Entry Interface is **not** a safe MM305 inlet, and a native MM304-to-MM305 mode transfer is **not** a commitment to a viable Final path.

This is a code and offline replay iteration. **No new live KSP flight or save-state change has occurred.**

## Implementation

- `CLanding/prediction/deorbit_planning.inc`: keep the existing 361-point coarse orbital search, nine-step timing refinement and normal-component search as a *seed*. Before locking in the deorbit, evaluate up to four nearby finite burns, two on each side of that seed. Spacing is the configured orbital capture distance divided by predicted Entry Interface airspeed (roughly 43 s for this polar case), not an arbitrary frame or geographic angle. Every candidate re-solves the physical finite-burn periapsis, simulates Entry, and runs the actual production MM304 executive shadow to its physical MM305 handoff.
- Opportunity selection prefers safe Entry, actual post-handoff proof when present, a qualified MM305 inlet, then physically reachable entry geometry: altitude window, configured HAC + Final path reserve, closing direction, and perpendicular runway approach. Candidates that have already crossed KSC (range increasing) do not count. When no inlet qualifies, the best *geometrically plausible but unqualified* opportunity remains diagnostic only; its negative certified energy is preserved rather than relabeled as a success.
- `CLanding/prediction/entry_planning.inc`: publishes MM305 admission energy, not the unrelated MM304 drag feedback, at a forced ownership handoff. The optional entry witness trace now includes Mach, runway relative coordinates, inlet range, admission veto and certified specific-energy margin.
- `CLanding/prediction/entry_planning.inc`: strict `deorbit_runway_capture_qualified()` now requires an actual qualified inlet, nonnegative certified energy, and committed native downstream HAC/Final path evidence. An inlet or ownership boundary alone cannot certify a runway delivery.
- `Configuration/missions/sts-polar.json`: `landing.requireStrictPlan=true`. The native backend may still present entry-stage-only plans for diagnosis, but the KSL **production polar landing mission refuses to engage one**. Explicit operator override for a separate experimental campaign remains available through `--set landing.requireStrictPlan=false`.
- `Configuration/sts-live.json`: minimum lead increased from 12 s to 60 s before the planned finite burn. The fourth flight used ~34 s between the nominal burn start and effective thrust acquisition. Logged body attitude began around heading 340 deg and rotated toward the target ~203 deg during that interval. Measured pitch/yaw torque and inertia imply a theoretical 180-deg bang-bang slew lower bound of ~25 s, before settling and RPC/control latency; 12 s could not contain that rotation. The earlier warp exit gives the actual orbiter time to settle at 1x before ignition. `CLanding/app/controller/runtime.inc` reports the *configured* lead, rather than a misleading fixed '100 seconds'. **60 s is a flight-specific preflight setting, not a universal aerodynamic law**, and has not yet been revalidated in live KSP.
- `Validation/DeorbitWindowTests.c`, `Validation/DeorbitStageAdmissionTests.c`, `tests/test_mission.py`: independently reject KSC-receding/too-close/unsafe inlet witnesses and unproven downstream route success; assert that the polar mission blocks unqualified deorbit plans.

## Repeatable recorded-state evidence

`Tools/benchmark_deorbit_planning.c --fourth-polar-plan` freezes the first fourth-flight orbital state and initial certified aero envelope from:

- `FlightLogs/2026-10-12T12-51-12Z-STS-vehicle.jsonl` (first orbital keyframe, UT 146949660.947894)
- `FlightLogs/2026-10-12T12-51-12Z-STS-planner.jsonl` (initial aerodynamic envelope)

It uses the project's native ShuttleSim atmosphere, not a copied full runtime physics store; this is a **reproducible offline proxy, not an exact reproduction of KSP's subsequent live atmospheric response**.

| Indicator | Original one-pass planner (2ed9d8e) | After guided timing search |
| --- | ---: | ---: |
| Planned burn center (UT) | 146957080.8 | 146957124.1 |
| Entry Interface range | 1019.7 km | 937.1 km |
| Native MM304 witness TAEM distance from KSC | 153.5 km | 71.3 km |
| KSC closure at MM305 boundary | closing | closing |
| Nominal MM305 geometric window 20–96.5 km | missed | inside |
| Native MM305 admission energy margin | ~-270 kJ/kg | ~-246 kJ/kg |
| Strict landing robustness | 0/15 | 0/15 |
| Entry-stage burn/atmosphere qualified | yes | yes |
| CPU time for planner | ~6.2 s | ~11.8 s |

Another recorded polar fixture (`--polar-plan`) changes its predicted inlet from ~168 km to ~87 km but also remains unqualified (around -252 kJ/kg). The exact outcome therefore is **a better-targeted MM305 inlet opportunity, not proven landing feasibility**. Varying the target post-burn periapsis in a separate offline sweep across 42, 46, 50, 54, 58 km did not produce positive certified MM305 energy in the sampled nearby candidate set. Do not solve this by weakening uncertainty/safety thresholds or by claiming proximity is adequate.

## Follow-up: native terminal integration (2026-10-13)

The next implementation phase now connects the same physical deorbit candidate to native MM305/HAC and Final continuation, with an explicit live-planet frame binding. See [Polar-Native-Terminal-Integration-20261013.md](Polar-Native-Terminal-Integration-20261013.md). The older blocker below records the state *before* that integration and is retained for provenance. The current remaining failure is certified inlet energy and failure to acquire Final, not the missing call path.

## Important next blocker

The existing production deorbit witness stops deliberately at the MM304-to-MM305 boundary (`predictor_simulate_entry_guidance_shadow` with `stop_at_taem=true`); it **does not run a native MM305 HAC/Final route**. Therefore a strict full-route plan cannot yet be certified by that witness, even when the nominal inlet becomes geometrically plausible. The new strict polar flight gate will intentionally hold until downstream route evidence is implemented and validated. Required next step: plumb an immutable, replay-validated `TerminalModel` into selected post-burn candidate evaluation, execute bounded native MM305/Final continuation from the predicted handoff, and certify that route under physical uncertainty before allowing a production landing. Do not weaken the existing 1 km physical MM304 capture contract, the MM305 energy budget or bounded flight controls simply to turn the status green.

Live runtime validation is still required for: (1) full attitude acquisition before the finite retro burn; (2) matching predicted vs realized 30/25-km kinematics under the new timing/taper; (3) downstream Final delivery. Do not infer any of these from an offline improvement alone.

## 2026-10-13 native-route integration

The next iteration integrated replay-validated native MM305/HAC/Final continuation and strict Final-stage proof. See [Polar-Native-Terminal-Integration-20261013.md](Polar-Native-Terminal-Integration-20261013.md). Both frozen polar-orbit fixtures still fail certified MM305 energy and 0/15 strict scenarios; the new model proves that a committed HAC route is not equivalent to Final approach arrival.

## Verification commands (offline only)

```
make -C CLanding deorbit-test entry-forecast-admission-test
make -C CLanding build/benchmark_deorbit_planning
KSP_LANDER_TRACE_DEORBIT_SEARCH=1 CLanding/build/benchmark_deorbit_planning --fourth-polar-plan
CLanding/build/benchmark_deorbit_planning --polar-plan
make test
```
