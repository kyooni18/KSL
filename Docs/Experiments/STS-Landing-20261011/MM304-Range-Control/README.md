# MM304 range-control investigation

Current guidance was replayed from the recorded L5/L7 MM304 inlet states,
with the MM305 acquisition range at 96.5 km or 60 km. This is an offline
diagnostic, not a deorbit qualification or landing acceptance run. The runner
intentionally stops at TAEM; exit code 2 for an unfinished landing is expected.

Reproduce in the integration worktree after `make`:

```sh
.venv/bin/python Docs/Experiments/STS-Landing-20261011/MM304-Range-Control/run.py
```

`live-frames.json` identifies the native source logs by SHA256 and retains the
first MM304/TAEM samples. INI fixtures use recorded latitude, longitude,
altitude, airspeed, FPA, mass, AoA and bank. Body heading approximates surface
course; this is not an exact kinematic/actuator-history restart. Both flights
use the current `sts-live.json` settings, `sts-live-20261010` aero and the
existing `att-live.ini` direct plant. Only `taemInterfaceRange` changes between
each flight's paired runs. Full model/build provenance and manifest locations
are in `result.json`; the raw replay streams are retained in ShuttleSim/runs.

## Measured live handoff differs in geometry, not only range

| Original live flight | MM304 inlet range | Inlet cross-track | TAEM range | TAEM cross-track | TAEM altitude | TAEM speed |
|---|---:|---:|---:|---:|---:|---:|
| L5 | 1162.6 km | −6.9 km | 59.8 km | −2.6 km | 27.09 km | 872.6 m/s |
| L7 | 1102.9 km | +101.4 km | 87.1 km | −32.5 km | 26.95 km | 872.0 m/s |

Both used a configured 96.5 km acquisition range. L7's crossrange passed
through the centreline and reached MM305 on the opposite side, with a much
larger remaining lateral problem. A scalar range error does not describe
that state. The original L7 plan's −20.5 km TAEM range error was relative to
96.5 km; it does not by itself prove that a longer entry range is needed.

## Current-backend replay sweep

| Inlet / acquisition setting | Outlet phase | Along | Cross | Altitude | Speed |
|---|---|---:|---:|---:|---:|
| L5 / 96.5 km | TAEM | −33.94 km | −2.34 km | 24.45 km | 860.25 m/s |
| L5 / 60 km | TAEM | −27.45 km | −2.44 km | 24.77 km | 861.28 m/s |
| L7 / 96.5 km | TAEM | −79.05 km | −40.78 km | 24.73 km | 860.75 m/s |
| L7 / 60 km | still MM304 at 900 s limit | −15.42 km | −18.20 km | 5.85 km | below Mach window |

TAEM rows are the final backend snapshot at the deliberate phase stop; the
last row uses the simulator's measured final state at the time limit. The
backend's final snapshot there preceded the simulator endpoint (6.01 km,
76.53 m/s); `result.json` preserves both instead of combining their timestamps.

The 60 km L7 case had four sampled `ENTRY_TRACE` states with **only range
veto bit 2 set**. Mach, altitude, nominal energy and closure were admissible,
but the smallest range in those samples was 73.226 km. At UT 6592.070 it was
22.51 km high, 692.56 m/s, along −60.57 km / cross −41.15 km, with nominal
energy margin +2587 J/kg. By the time range entered 60 km, the altitude and
Mach window had closed. Late veto 25 combined Mach (1), altitude (8), and
negative nominal energy (16). No gate was weakened.

## Decision and limits

**Do not adopt 60 km as the acquisition range.** `entry_taem_range_target`
feeds both DRAGREF's remaining-range sizing and the hard maximum-range
admission gate. Reducing it changes longitudinal flight and simultaneously
prevents the intended handoff. L7's feasible sampled state set never overlaps
that new range bound.

The L5 current-backend replay reaches TAEM about 26 km closer than the
original live flight, with roughly 2.6 km less altitude. Changes in guidance
since the live flight, the provisional aero/attitude plant, and the approximate
restart all remain possible contributors. These runs cannot certify deorbit
targeting or justify a production energy target numerically.

Also, the existing L5 **MM305** golden begins at `k=0`, just like L7. Its first
planned path is 77.179 km versus 76.194 km predicted maximum range; L7's is
78.454 versus 77.302 km. L5's much shorter geometric runway distance is spent
on a HAC path. Thus `k=0` or 87 km versus 60 km alone is not sufficient evidence
for the full cause; compare flown lateral path, energy loss, Final delivery,
and longitudinal model error. Next work should distinguish an energy-delivery
target from the maximum acquisition boundary and validate the lateral/range
history before changing the deorbit plan.

## Prediction/execution consistency correction

The TAEM range predictor called `tel_vertical(..., live=false)` and still
blended toward station speed at `k<=0.05`, while actual TAEM skipped that blend
after `8a42d22`. An experimental change used the same low-energy condition in both
calls. It addresses an internal command mismatch, not the upstream energy state.

Explicit replays preserve the numerical outcomes: L5 66.129 m/s, sink 1.844,
stop 1101.616 m, all gates pass; L7 55.499 m/s, sink 0.907, contact −225.778 m,
gear down, failed runway acceptance. Run directories:
`runs/20261011T082745Z-predictor-parity-l5` and
`runs/20261011T082757Z-predictor-parity-l7`. No golden was rebased, no plant or
production range setting changed. The candidate has not flown live and was
removed from the working source after `make -C CLanding qualification` failed
the admission fixture non-vacuity assertion (0/5 qualified routes). Unit tests
(104 plus 44 subtests) and native stack checks passed. The same standalone
`mm305-feasibility-test` fails on the unchanged baseline `d9881a1` with the
same 0/5 non-vacuity assertion. The candidate did not create this observed
failure, but remains set aside pending investigation of admission coverage.
No test gate was weakened. Logs: `/tmp/ksl-predictor-parity-qualification.log`
and `/tmp/ksl-baseline-feasibility.log`.
