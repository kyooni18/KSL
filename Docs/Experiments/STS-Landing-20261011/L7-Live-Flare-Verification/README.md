# L7 MM305 live flare verification, 2026-10-11

The user explicitly approved loading
`ksl-20261011T064053Z-live-85km-10deg-landing-mm305` and flying from MM305.
The native guidance was unchanged from `8a42d22`. The worktree was
`~/Code/KSL-integration`, branch `integration/unified-flight-control`.

## Execution and evidence

`make` and `make test` passed (104 Python tests, 44 subtests, native checks).
The approved save was loaded with `Tools/ksp_test_guard.py load-save --name …`;
the guard confirmed STS at UT 6566.830, 26.917 km, both warp factors zero.
The CLI then ran:

```sh
KSL_DATA_ROOT=/Users/kyooni18/Code/KSL-integration \
KSP_LANDER_GATE_TRACE=1 KSP_LANDER_FINAL_TRACE=1 KSP_LANDER_TEL_TRACE=1 \
bin/ksl land --from mm305 -p sts-85km-10deg \
  --label live-l7-flare-verification --no-color
```

Before flight, the primary checkout's `Runtime/Physics` files were copied
read-only into the worktree to preserve prior observations. Runtime writes
and FlightLogs were directed to the worktree. No primary source files were
modified. This data-root override is material to reproducing the experiment.

Evidence retained locally:

- `runs/20261011T075525Z-live-l7-flare-verification/` (status, telemetry,
  events, backend stderr, configuration).
- `FlightLogs/2026-10-11T07-55-25Z-STS-vehicle.jsonl` and paired planner log.
- [result.json](result.json): native phase transitions, contact bracket,
  invariants, final sample, source-log SHA256 and terminal mission status.

The native vehicle stream was decoded by resetting state on `vehicleKeyframe`
and recursively merging `vehicleDelta.fields`, using the existing
`ShuttleSim.tools.identify_stsn_aero._merge`. `TEL_PLAN` includes diagnostic
forecast traces; use mission telemetry and live-only `TAEM_ENERGY` to establish
the flown trajectory. Tests produced separate simulator FlightLogs during
early TAEM; the live source log above is identified by its backend PID 86511
and empty (non-simulator) campaign identity.

## Outcome: flare verified, landing acceptance failed

| Event | UT | Speed | Along | Details |
|---|---:|---:|---:|---|
| Final handoff | 6930.950 | 94.21 m/s | −7690 m | `MM305 low-energy Final delivery`, 2235 m above runway; nominal preflare reject reason `none` |
| Flare | 7049.550 | 63.20 m/s | 1240 m | 6.46 m/s sink at phase entry; gear down |
| Last flying sample | 7054.530 | 52.77 m/s | 1531 m | **2.11 m/s downward**; gear down, cross +11.85 m |
| First landed sample | 7054.590 | 52.65 m/s | 1534 m | 1.25 m/s downward after contact; do not treat this as pre-contact sink |
| Stopped/aborted | 7130.250 | 0.024 m/s | 2646 m | **146 m beyond configured runway end**, cross +12.89 m |

The vehicle touched down on the runway but was slow and long: below the
63.75 m/s acceptance floor and about 1157 m past the 375 m target. Its mass
dropped by 230 kg at UT 7098.510, along 2581 m during rollout, consistent with
damage after leaving the runway. The damaged part was not identified.

All 8602 native samples reported SAS off, airbrakes off, zero current thrust
and zero commanded throttle. Independent kRPC observations at UT 6608.830
and 7095.230 confirmed RCS off and speedbrake action group 1 off; these are
sampled RCS checks, not a continuous RCS assertion. Gear was down at contact.
No flight warp was requested.

The universal terminal backstop was **not exercised**: the energy-law handoff
owned the transition to Final. Adaptive flare remained disabled. This single
MM305 continuation does not verify a full deorbit/MM304 mission or the other
TAEM paths.

## Newly exposed terminal-state problem

At the runway-end overrun the backend warned
`Touchdown occurred outside the runway 09 contact envelope.` It then remained
in Rollout with `Continuing bounded guidance while the rejected solution
replans.` even after stopping. `guidance_update_with_terminal_model` in
`CLanding/guidance/guidance.c` converts terminal Abort results into warning
recovery while automation is engaged. Grounded recovery keeps braking but
has no terminal failure/stop exit in this case.

The operator sent SIGINT to the verified CLI PID 86495 after the vehicle
stopped. The CLI sent backend abort, applied the safe-control path, disconnected
and exited 130. The mission status is **aborted**, not Complete. A fresh guard
probe confirmed STS landed, zero warp, KSP paused. Do not reinterpret the
absence of an automatic terminal result as successful landing acceptance.

Next work: distinguish ground landing failure from recoverable airborne
guidance rejection, then address the slow/long Final delivery and simulator
gap. Preserve the new live trace for plant refitting; do not tune offline
flare coefficients against the old L7 simulator short-landing alone.

## Upstream energy investigation corrections

Original L7 evidence is
`runs/20261011T064053Z-live-85km-10deg-landing/`. The plan predicted TAEM at
76.038 km with range error −20.462 km relative to the configured **96.5 km**
acquisition range. Its nominal shadow capture passed, but only 4/15 robust
cases passed; entry-stage execution was qualified/degraded. Actual TAEM began
at 86.937 km, Mach 2.85, 26.930 km. The negative plan range error alone does
not establish that moving entry farther away will improve runway energy.
MM304 sizes drag against `entry_taem_range_target`, which currently returns
the configured acquisition range (with a geometry floor), not a measured
maximum-range TAEM delivery contract. Reconcile that target with the range
the TAEM model can deliver before changing deorbit timing.

Original `TAEM_ENERGY` starts with `loop=0`, `k=0`; the first `loop=1` is at
UT 6605.4, `leg=2`, radius 7000 m. `tel_plan` can choose direct HAC intercepts
with `loop=true`; this does not prove an overhead circuit selected for excess
energy. `tel_plan_open` permits downwind replanning until the tangent is within
5 km. The blanket frozen-overhead-loop hypothesis is therefore unsupported.
