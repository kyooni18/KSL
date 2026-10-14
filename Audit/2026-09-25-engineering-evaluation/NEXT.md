# Next work (after the 2026-09-25 remediation)

Prioritised to-do list that follows `REPORT.md` (the audit) and `REMEDIATION.md`
(what changed and the evidence). Items are ordered causally: an earlier item
either blocks the later ones or makes their evidence meaningless.

**Current state in one line:** all 7 unit/contract suites pass. MM304 and MM305
now fly the dispersed states into Final far more often than before. **No
ShuttleSim run lands on the runway at ≤3 m/s sink**, on any plant.

---

## 0. Decisions the owner has to make first

These are not guidance bugs. They set what "a landing" can physically mean, and
guidance work below depends on the answers.

| # | Decision | Why it matters | Evidence |
|---|---|---|---|
| D1 | **What is the real KSP STS-N subsonic aero?** Fit it from a flight log and replace `ShuttleSim/reference-model/stsn_aero_reference.csv`. | The reference plant (B) is synthetic. Its L/D is ≈2.9 at 12–15° and its 1-g speed at 15° is 74.7 m/s, equal to the configured touchdown speed, so every flare ends at α_max with speed decaying. Final cannot be tuned meaningfully against it. | `REMEDIATION.md` "Remaining limits"; `Docs/LatestFlight-2026-09-07-analysis.json` holds the only recorded KSP samples |
| D2 | **Is `terminalMaximumLiftAngleOfAttack = 15°` a hard vehicle limit** (tail strike or stall), or a guess? | At 20° the reference plant's 1-g speed drops to ≈66 m/s and a ≤3 m/s touchdown becomes plausible. | `Configuration/default.json`; `guidance/final/phases.inc` |
| D3 | **Is `finalGlideSlope = 28°` intended?** | A 28° outer glide on an L/D≈3 vehicle arrives at the HAC exit at ≈1.7 km / 3.2 km with ~55 m/s sink. Every flare starts from there. The Shuttle used 18–20°. | `Configuration/default.json`; MM305 sweep touchdowns |
| D4 | **Is a speedbrake (airbrake action group) allowed in approach?** The code currently forces it closed ("unpowered landing contract: no airbrakes"). | Without a drag device, Final's only energy lever is glide angle. That cannot shed excess energy in 3.5 km (plant C lands 2–3 km long). | `guidance/final/sequence.inc` (`terminal_speedbrake_closed`) |
| D5 | **Runner success window:** it now derives from `touchdownSpeed` (±15 %). Keep that, or restore the old 60–70 m/s? | The old window was below the physically reachable speed on plant B. | `ShuttleSim/scripts/run_guidance.py` |

---

## 1. Truth and tooling (do before any more guidance tuning)

1. **Fit and commit a KSP-anchored plant** (D1).
   - Refit the aero table and force book from recorded KSP flight samples, holding
     some samples out for validation.
   - Store the result under `ShuttleSim/data/fitted/` (preferred automatically) and
     update the tracked reference model when it is good enough.
   - *Done when:* the fit reproduces held-out samples to ≤10 % in L and D.
2. **Default the simulator to model error, not perfect knowledge.**
   - Make `run_guidance.py` give guidance a perturbed model by default: for example
     the plant table ×(1±15 %), via the existing `--terminal-*` options.
   - Keep a `--perfect-model` flag for debugging.
   - *Done when:* sweeps report both perfect-model and perturbed-model results.
3. **Put a closed-loop landing gate in CI**, even if it is red at first.
   - Add a `make campaign` target that runs `tools/sweep.sh` over the Final
     checkpoints and the MM305 scenarios and writes one summary file.
   - Track the pass count over time. Not part of `make test` (too slow), but
     scripted and reproducible.
4. **Move unit tests onto `Configuration/default.json`.**
   - The built-in terminal geometry (8 km / 20° / 12°) differs from the shipped
     JSON (3.2 km / 28° / 24°).
   - The `TAEMNativeStackTests` fixture has to be rebuilt around the JSON values.
   - *Files:* `CLanding/vehicle/models.c`, `Validation/TAEMNativeStackTests.c`.
5. **ShuttleSim ground model.**
   - Add gear contact, derotation (attitude dynamics on the ground), a crash-on-sink
     limit, and real off-runway terrain instead of the 70 m drop at the runway edge.
   - Until then touchdown outcomes past first contact are not trustworthy.
   - *Files:* `ShuttleSim/src/sim.c` (ground step).
6. **Identify the direct-control plant from KSP.**
   - `--direct-control` uses generic moment coefficients.
   - Fit pitch/roll control and restoring moments from recorded body rates and
     control inputs so FCS results mean something.
   - *Files:* `ShuttleSim/src/attitude.c`, `stsn_attitude_reference.ini`.

## 2. MM305 (TAEM) — the largest remaining guidance gap

7. **Energy-managing acquisition law** (highest guidance priority).
   - While no route is held, the vehicle flies straight at the HAC region and does
     not manage energy.
   - The 30°/60° cases then often reach 1 km with "no qualified route": N30, S30
     and W60 in the latest sweep.
   - Add Shuttle-TAEM-style energy control:
     - S-turns (bank away from the heading to the HAC) when energy is above the
       route requirement.
     - Direct flight at best-glide α when it is below.
     - Driven by the same energy-over-range comparison the planner uses.
   - *Files:* `guidance/guidance_taem.c` (`mm305_acquisition_command`).
   - *Done when:* no MM305 scenario ends with "no qualified route" on a plant that
     can glide the distance.
8. **Continuous energy loop inside a committed route.**
   - Today energy is corrected only by replanning (HAC radius and lead length).
   - Add a bounded S-turn or lead-stretch on the lead segment, and α modulation
     against the route's energy profile, so small errors do not require a route
     switch.
9. **Planning cost.**
   - A failing search still costs ~8 s wall on the fixture (tens of seconds on
     some dispersed states). Profiling showed ~55 % route construction and
     ~45 % profile generation.
   - Options:
     - Cache routes between attempts.
     - An analytic energy pre-screen before building lead variants.
     - Early exit when the first radius fails for energy reasons in the "too low"
       direction.
   - *Files:* `taem_candidate_search.c`, `terminal_solver.c`.
   - *Done when:* a failing search takes <1 s and a successful one <2 s on one core.
10. **Track the plan with the plan's model.**
    - Replans use a lift/drag-scaled model, but the tracker inverts the unscaled
      one.
    - Pass the scale factors into `taem_tracker_update`, or store the scaled model
      with the committed route.
11. **Remaining route releases.**
    - 0–5 per flight in the latest sweep.
    - Instrument why: the release message now carries cross-track and course
      error; add UT and a phase snapshot.
    - Separate genuine divergence from reference jumps at route adoption.
12. **Shadow forecasts pollute the MM305 trace.**
    - `KSP_LANDER_TAEM_DIAGNOSTICS=2` output mixes live and shadow-ensemble lines.
    - Tag or suppress shadow output (the guidance copy could carry a
      `diagnostic_shadow` flag).

## 3. Handoff (MM304 → MM305)

13. **Upper energy bound and planner-feasibility admission.**
    - Admission is still a lower bound plus a closure veto.
    - Admit only states the MM305 planner can route: a cheap reachability or
      energy test in the same model, or run the planner at the boundary.
    - Keep MM304 responsible for dumping excess energy.
    - *Files:* `prediction/decision/taem_capture.inc`.
14. **Align the advisory predictor's handoff with the live one.**
    - `prediction/entry_simulation.inc` still ends entry on the legacy
      `entry_taem_interface_capture`.
    - Use `entry_mm305_admission_envelope` so the forecast predicts the handoff
      the vehicle will actually make.
15. **End-to-end MM304 evidence.**
    - No full deorbit → entry → landing run exists on the current build.
    - The `mm304-{east,west}` scenarios were energy-infeasible.
    - Build energy-consistent entry states: propagate from `ksp86km-preburn`
      with the planned burn and save checkpoints.
    - Run production `--engage engage` over entry-azimuth and crossrange-sign
      dispersions on plants B and C.

## 4. Final

16. **Re-tune Final after D1–D4.**
    - The current law (energy-placed aim point, outer-glide speed closure,
      re-solved constant-deceleration preflare arc, measured-lift Newton step,
      authority-aware lateral) is structurally sound.
    - Its constants were exercised only on plants A/B/C (energy limited or
      energy rich), namely the shallow-glide segment (height and sink), the
      speed reference of 1.6 × V_1g, and the 0.25 g design pull.
    - Retune on the fitted plant.
17. **Final-admission fallback.**
    - It admits any aligned state steeper than 38°.
    - Restrict it to states the Final energy law can actually flare (a quick
      arc-height and speed check) and otherwise report "energy too high/low for
      Final" explicitly.
18. **Speedbrake in the energy law**, if D4 allows it: modulate drag to hold
    the outer-glide speed reference instead of steepening the glide.
19. **Rollout.**
    - Derotation and nose-gear logic, braking tied to nose-gear contact, and a
      documented wheel-steer sign convention.
    - Needs item 5 first.

## 5. FCS

20. **Gain scheduling and trim across the regime**, on the identified
    direct-control plant (item 6).
    - Schedule the pitch/roll bandwidth with q and Mach.
    - Pitch trim integrator with anti-windup outside approach.
    - Turn-coordination feed-forward.
    - *Files:* `control/flight_control.c`.

## 6. Tests to add

21. **Admission ⇒ MM305-feasible property test.** For states sampled inside the
    admission set, the planner finds a route. It is red until items 7 and 13 land.
22. **Final touchdown-quality test** on a fixed plant: sink, speed, along and
    cross limits. It is red until D1–D4 and item 16 land.
23. **Energy-law unit tests for MM304** (`entry_energy_control.c`):
    - The drag-profile solve is monotonic in range.
    - Saturation flags.
    - Integrator hold during reversals.

---

## How to reproduce the current numbers

```sh
make -C CLanding test                                   # 7 suites
T=Audit/2026-09-25-engineering-evaluation/tools
$T/sweep.sh fin ref engageFinalTest ShuttleSim/scenarios/final-mm305-ideal-3p5km-185mps*.ini
$T/sweep.sh mm305 ref engageHACTest Audit/2026-09-25-engineering-evaluation/scenarios/mm305-*.ini
$T/sweep.sh mm305C C engageHACTest Audit/2026-09-25-engineering-evaluation/scenarios/mm305-*.ini
$T/trajectory_table.py /tmp/ksl-sweeps/res/mm305-mm305-N60.json 2   # inspect one run
```

- `sweep.sh` snapshots the backend binary before running, so rebuilding during a
  sweep does not contaminate it.
- Add `-- --direct-control` to exercise the real FCS.
- Useful diagnostics environment variables:
  - `KSP_LANDER_FINAL_TRACE=1` (Final vertical law, per tick)
  - `KSP_LANDER_FINAL_LATERAL_DIAGNOSTICS=1`
  - `KSP_LANDER_TAEM_DIAGNOSTICS=2` (see item 12)
  - `KSP_LANDER_APPLY_DIAG=1` (phase and roll per applied command)
