# KSPShuttleLander — Independent Engineering Evaluation (2026-09-25)

Scope: MM304 entry, MM304→MM305 handoff, MM305/TAEM/HAC, Final/flare/touchdown/
rollout, the native flight-control system (FCS), ShuttleSim, tests and docs.
Method: code inspection of the production path, the maintained test gate, and
closed-loop ShuttleSim experiments. No implementation files were modified; all
audit artifacts live in this folder.

Evidence labels used below: **[D]** demonstrated (code + executed run),
**[C]** code-verified by reading, **[I]** strong inference, **[U]** unresolved.

---

## 1. Executive verdict

What the system is: a single-thread native C controller. At 10 Hz (30 Hz near
the ground) it polls kRPC telemetry, runs one of three guidance owners
(MM304 drag/energy feedback, MM305 plan-once native HAC route tracker, and a
geometric Final/flare law), then closes a per-axis PD attitude loop that writes
direct pitch/roll/yaw inputs.

Verdict:

- **Its current successes, if any, are not evidence of robust guidance.**
  Every layer that ShuttleSim can exercise failed in this audit. That covered
  3 plant variants and 33 closed-loop runs from the repo's own checkpoints and
  from states the program itself declares admissible. **0 of 33 runs landed.**
  Failures are layer-specific and reproduce across plants, so they are not
  artefacts of one plant table.
- **What is sound**
  - Frame and energy algebra (`taem_frames_energy`).
  - HAC circle geometry and its sign conventions.
  - The one-way ownership state machines.
  - The fail-closed MM305 commitment concept.
  - The measured-drag range integral as a building block.
  - The advisory-only boundary of the MM304 predictor.
- **What is not sound**
  - **MM304:** the bank (vertical-lift) law saturates at wings-level whenever
    lift is below roughly 30–50% of the equilibrium-glide requirement. It also
    multiplies measured force by a "confidence". Its lateral reversal law
    assumes eastbound flight.
  - **MM304→MM305 handoff:** it admits states that MM305 cannot fly,
    including vehicles already past and receding from the runway.
  - **MM305:** it plans once, synchronously (22.7 s on the control thread), and
    then flies a geometric/altitude profile with no energy feedback. Its only
    predictor is the simulator plant itself.
  - **Final:** a 10-s-lookahead flare latch fires kilometres too early on steep
    glides. There is no speed or energy closure. Every landing attempt
    touched down long, short or off-centreline at 13–37 m/s sink.
- **Validation is structurally unable to prove KSP behaviour.**
  - ShuttleSim bypasses the FCS entirely: it takes an ideal AoA/bank servo.
  - It supplies guidance with the plant's own response parameters and force
    book.
  - MM305 plans with the simulator's physics code.
  - The fitted data that defines the plant is git-ignored and absent, so
    `make test` fails on a clean checkout.

---

## 2. Architecture as found

```
kRPC C-Nano poll (fast items each tick) ──► read_live_sample ──► vessel_physics_observe
     ▲                                                       │ (overwrites confidences)
     │                                                       ▼
 FCS flight_control_step ◄── stabilized() ◄── guidance_update_impl (guidance_terminal.c)
 (PD per axis, same 10 Hz tick)                  ├─ deorbit/coast/entry-interface capture
     │                                           ├─ MM304 entry_program_guidance (contract.inc)
     ▼                                           │     drag ref (entry_drag_reference.c)
 direct pitch/roll/yaw, gear, brakes, wheel      │     alpha schedule (entry_alpha.c)
                                                 │     lateral/bank (entry_lateral.c)
                                                 │     executive (entry_exec.c)
                                                 ├─ MM305 taem_guidance_native (guidance_taem.c)
                                                 │     candidate search → profile → replay (terminal_solver.c)
                                                 │     tracker (taem_tracker.c)  [uses ShuttleSim physics]
                                                 └─ Final/flare/rollout (final/*.inc)
 prediction worker thread: MM304 forecast (advisory only), entry calibrator residuals
```

- Update rates [C]: guidance and FCS share one thread at `guidanceRate` (10 Hz),
  rising to 30 Hz below 100 m in Final/Flare/Touchdown/Rollout
  (`app/controller/runtime.inc::control_thread`). There is no separate fast
  inner loop, and KSP physics runs at 50 Hz.
- Ownership [C]:
  - `entry_exec` runs a strictly sequential phase machine.
  - The terminal owner (`guidance_terminal.c::terminal_guidance`) moves to
    MM305 only when `entry_complete && admission.ready`.
  - MM305 moves to Final when `mm305_hac_exit_reached` (≤700 m from the exit,
    course error ≤12°).
  - Final moves to the touchdown latch after 0.25 s of contact evidence.
- Dead code [C]: `guidance/entry/{bank_allocation,reference,sturn_planning}.inc`
  (~1,800 lines) are included nowhere. `ARCHITECTURE.md` still describes them.

---

## 3. Findings by layer

Severity: Critical, High, Medium, Low. Root causes are numbered R#; symptoms
cite their root cause.

### 3.1 Vehicle / simulator (ShuttleSim)

**R1 — Critical: ShuttleSim never exercises the flight-control system.** [D][C]
- Mechanism: `transport/krpc.c::krpc_apply` sends `target_aoa`/`target_roll`
  directly to the simulator when `s->simulator`. The plant (`ShuttleSim/src/attitude.c`) is
  a rate/acceleration-limited 2nd-order servo on AoA and bank. It has no
  moments, inertia, control surfaces, sideslip, trim or RCS.
- Why it matters: every inner-loop behaviour is unvalidated offline. That
  includes pitch/roll/yaw PD, authority estimation, trim, flare pitch, rollout
  yaw and the post-touchdown pitch release.
- Correction: add a 6-DOF rigid-body mode with surface moments, inertia and
  sideslip, driven by `flight_control_step` output. Otherwise, treat
  ShuttleSim results as guidance-only evidence.

**R2 — Critical: ShuttleSim hands guidance the plant's own model (inverse crime).** [C]
- Guidance-only telemetry fields: `sim_telemetry.c` publishes the servo's
  wn/ζ/rate/acceleration limits, and guidance and FCS consume them
  (`t->attitude_response`, `flight_control.c` lines 330-339).
- MM305 plans with plant code: `terminal_propagator.c` calls ShuttleSim's
  `flight_physics_airborne_step`, `aero_compute` and world/atmosphere, with the
  same data files the simulator loads (`run_guidance.py` sets
  `KSP_LANDER_TERMINAL_*` to the plant files).
- The "certified prior" is the plant table: the simulator session loads the
  plant force book as the guidance's certified aerodynamic prior
  (`krpc.c::sim_load_force_book`).
- Consequence: MM305 replay qualification and MM304 aero confidence are
  perfect-knowledge in simulation. They are only fitted approximations in KSP.

**R3 — High: the plant definition is not in the repository.** [D]
- `/ShuttleSim/data/` and `Runtime/` are git-ignored.
- `make test` fails at `TAEMNativeStackTests.c:126` (`terminal_model_capture`).
- The default MM305 model load fails in both the sim and live backends: it
  logs "MM305 native model unavailable" and aborts on the first MM305 tick.
- Nothing in the repo can reproduce the KSP-fitted plant. All
  "validated in ShuttleSim" claims are unverifiable from a checkout.

**Medium: simulator ground/touchdown model is too permissive to judge landing quality.** [C]
- Attitude is frozen on the ground (`ground_step` never calls
  `attitude_step`), so there is no derotation or nose-gear contact.
- There is no gear dynamics, no crash on sink rate, and a perfectly plastic
  vertical impact. Aero drag and lift are ignored on the ground.
- Terrain off the runway sits at sea-level radius, i.e. a 70 m "drop" at the
  runway edge.
- The runner's success check *does* enforce ≤3 m/s sink, so outcome
  classification is still sound.

**Medium: KSP parity of the audit plants.** [D]
- The recorded KSP force samples (`Docs/LatestFlight-2026-09-07-analysis.json`)
  imply M2.7/α23° CL≈0.23, CD≈0.37 (L/D≈0.64) and M0.5/α4° L/D≈1.65.
- The ShuttleSim built-in seed table is ~3× too lifting supersonic.
- Audit plants A/B/C bracket the recorded samples (see `tools/build_plants.py`).

**Low: build portability.**
- The code relies on macOS declaring `realpath` under `-std=c17`. The Linux
  build needs `-D_DEFAULT_SOURCE`.
- gcc fails `-Werror` on a snprintf truncation warning in `logging.inc`.

### 3.2 Navigation / state

- **High: measured force is multiplied by a confidence number (R4, see §3.3).**
- **Medium: guidance assumes perfect state.**
  - In the sim, position/velocity/AoA/bank/Mach/q are exact.
  - In KSP, kRPC gives exact positions, but AoA, sideslip and rates come from
    differentiated Euler/flight data at 10 Hz, with no filtering design
    document. Rates are finite-differenced per tick (`flight_control.c`
    318-325).
  - No bias or noise robustness testing exists. [C]
- **Medium: inconsistent `g_force` definitions.**
  - Sim: lift/(m·g₀) (`sim_telemetry.c:464`).
  - MM305 replay: |total aero force|/(m·g_local) (`terminal_solver.c:433`).
  - Live KSP: reported g-force.
  - The same limit (3.5 g) is therefore checked against three different
    quantities. [C]
- **Air vs ground velocity**
  - MM304 uses `true_air_speed` (air-relative) for Mach/q/energy
    (`rotating_specific_energy` includes the centrifugal term). This is
    correct. [C]
  - The MM305 tracker uses air-relative speed for lift and ground speed for
    curvature. This is correct for the windless sim. [C]

### 3.3 MM304 prediction / guidance

**R4 — Critical: confidence scales available lift.** [D]
- Code: `entry_lateral.c::bank_magnitude` computes
  `available_lift = measured_lift × min(authority_confidence, drag_confidence)`,
  with `authority_confidence = max(aerodynamic_confidence, physics_authority_confidence[1])`.
  These come from prior-flight history (`vessel_physics_derive_envelope`)
  and from torque/rate learning.
- Evidence: runs `mm304-auditA-base-*` and `mm304-auditB-base-*` without a
  force book.
  - `aerodynamicConfidence=0` and `physicsAuthorityConfidence=[0,0,0]` for the
    whole flight; commanded roll was 0° throughout.
  - The vehicles overflew the runway by 780 km and 239 km.
- Physics: a measured lift force does not become smaller because a prior model
  is uncertain. With confidence 0.25 (the live torque-seeded value), bank is
  computed against a 4× understated lift, which biases toward lift-up and
  overshoot.
- Scope: any fresh install, new craft, deleted physics DB, or simulator run
  without a force book.
- Correction: use measured lift directly. Let confidence gate or blend
  *model-predicted* quantities only, and fail explicitly (not silently
  wings-level) when measured lift is unavailable.

**R5 — Critical: the vertical-lift demand is equilibrium-relative with a capped
correction, so bank saturates at 0° when the vehicle is short of lift.** [D]
- Code: `entry_drag_reference.c` computes
  `required_vertical = (g − V²/r) × clamp(1 − correction, .10, 1.5)`, with the
  correction capped at 0.72. `bank = acos(required / available)` is clamped
  to [0, limit].
- Evidence: `mm304-auditB-base2-*`, UT 67521 (snapshot).
  - Required 1.71 m/s²; available 1.515 × 0.82 = 1.24 m/s², so bank was 0°
    with a predicted **overshoot of +212 km**.
  - Bank engaged only above q≈2,000 Pa. The vehicle passed over the runway at
    40 km altitude and aborted near the ground.
  - On Plant A (seed) the same thing happened in the thin upper atmosphere,
    followed by a 60 km lofted skip.
- Physics: when lift is below the equilibrium requirement, the vehicle
  cannot hold altitude. Guidance should still use bank to control descent rate
  and range. The Shuttle formulation commands L/D relative to *available*
  L/D, with drag-error and altitude-rate terms (Harpold & Graves 1978: "control
  laws that suppress oscillatory trajectory motion"), rather than an absolute
  acceleration capped against equilibrium.
- Scope: all early entry (q ≲ 100–1,000 Pa) on every plant, and most of entry
  for low-L/D vehicles like the recorded KSP STS-N (L/D ≈ 0.6 supersonic).
- Correction: formulate the command as (L/D)_cmd = (L/D)_ref + K_D·(D − D_ref)
  + K_ḣ·(ḣ_ref − ḣ). Compute cos φ = (L/D)_cmd / (L/D)_measured with explicit
  saturation handling (allow bank → limit, and report infeasibility), plus a
  bias/integral term with anti-windup.

**R6 — High: the drag reference collapses into a multiple of measured drag.** [D]
- Mechanism: `achievable_profile_drag_cap` takes 1.85 × current measured drag
  and projects it along a smoothstep current→TAEM altitude path. That cap binds
  for most of entry, so D_ref ≈ 1.85·D. The drag error is then always about
  +85% of measured drag, whatever the energy state.
- Evidence: at the Temperature-Control start, D_ref dropped 2.7→0.09 m/s² and
  predicted range jumped to 5,900 km (`mm304-auditB-base2` trace).
- Why it matters: the "reference" no longer encodes a D(V) corridor. The
  feedback loop chases its own measurement.
- Correction: build D_ref(V) from the terminal range requirement using
  analytic segments, as the Shuttle did. Keep feasibility bounds as
  constraints that *flag infeasibility*, not as the reference itself.

**High: no altitude-rate (phugoid) damping and no bias/integral term.** [C]
The vertical correction is only `0.28·drag_error_fraction + 0.22·range_fraction`.
There is no ḣ term and no persistent-bias estimate, so model or navigation bias
leaves a steady drag error.

**Medium: phases are fixed velocity gates.** [C]
- Temperature-Control, Equilibrium-Glide and Constant-Drag ends sit at fixed
  fractions (.30/.58/.82) of a velocity span anchored at 3.4×V_TAEM.
- The "equilibrium intercept" is a velocity test (`entry_drag_reference_exec_profile`).
- There is no range- or energy-driven phase skipping, and the handoff is
  accepted only in the Transition phase.

**R7 — Critical: the lateral reversal law assumes eastbound flight.** [D]
- Code: `entry_lateral.c` sets `demanded_sign = sign(projected crossrange
  error)`, where crossrange is the runway-09 centreline offset. The module
  receives no course input.
- Evidence (mirrored runs `mm304-mm304C-{east,west}`, both starting 10 km north
  of the centreline):
  - Eastbound converged (−10 → +1 km, two reversals).
  - **Westbound diverged monotonically (−10 → −26 km at q≈7,500 Pa) with zero
    reversals.** The same bank sign turns a westbound vehicle away.
- The unit test `EntryLateralFeedbackTests.c` encodes the same assumption.
- Correction: steer on target-relative delta-azimuth (heading to the
  HAC/runway point minus velocity azimuth), as in the Shuttle. Use a
  velocity-dependent deadband: Bump (1977) studied a ~12.5° lateral deadband
  with a minimum-bank schedule.

**Medium: the reversal deadband is a predictor residual.** [C][D]
- The "corridor" is `max(|trajectory_range_residual|,
  |predicted_raw_published_position_delta_30s|)`, i.e. an unrelated
  calibrator/predictor error. It was observed at 0 m, giving no deadband.
- Leg capture is the only anti-chatter mechanism, and the number of
  reversals is unbounded.

**Medium: no overflight / receding-target logic.** [D]
Range to go is the great-circle magnitude. After passing the site, MM304 keeps
"needing range" while flying away (both baseline failures).

**Medium: the alpha schedule is plausible.** [C]
The schedule has trim + 0.9·span above 1.85·V_trans, ramping to trim, with ±4°
drag modulation and asymmetric negative gain.

**Low: advisory predictor.**
- The MM304 predictor worker (~1,450 lines) is advisory only and cannot
  command [C].
- In the sim it still gates lockstep (`sim_plan_barrier`), and its residual
  feeds the reversal corridor.

### 3.4 MM304 control interaction

- **Medium: the attitude command path runs at 10 Hz with no inner loop.** Guidance
  commands pass through `stabilized()` jerk limiters and then the FCS in the
  same tick.
- **Medium: FCS gains are fixed across the regime.** Pitch wn 1.05/ζ 1.15 and
  roll wn 0.65 hold from q≈0 to 45 kPa, with authority normalised by an
  adaptive estimate [C].
- **Medium: no atmospheric attitude authority below aero authority.** RCS is
  permanently 0 in the atmosphere (`flight_control.c:553`), while the README
  advertises RCS blending [C]. Between q = 0.5 Pa (end of inertial
  RCS capture) and useful aerodynamic authority, only reaction wheels
  (if present) remain.
- All of this is untested offline (R1).

### 3.5 MM304 → MM305 handoff

**R8 — Critical: admission is a lower bound, not proof that MM305 can fly the state.** [D]
- Code: `taem_capture.inc::entry_mm305_admission_envelope` requires range ≤
  96.5 km, Mach 2.15–2.85, 22–30 km, descending, q/g, and energy >
  straight-line drag work. It has no upper energy bound, no heading/closure
  requirement and no FPA requirement, and it never calls the MM305 planner.
- Evidence:
  - `mm304-mm304C-east` handed over at **21.8 km past the runway, receding
    (course 84.5°), Mach 2.86**.
  - `mm304-mm304C-west` handed over with 32 km crossrange at −12° FPA.
  - MM305 rejected both on its first tick (`terminal_abort`, controls
    released).
- Correction: admission should require that the *same* MM305 planner (or a
  conservative reachability set derived from it) returns a qualified route.
  It should also require closure (receding states rejected) and an upper
  energy bound.

**High: missed-window behaviour.** [C]
If admission is never met, MM304 keeps flying its law (including below the
Transition velocity band) until the minimum-safe-speed or ground abort.
There is no forced or fallback transition.

**Medium: stale docs.** ARCHITECTURE.md still describes a retired contract
(−8 km station, Mach 0.7, 15–18 km, ±30° perpendicular).

### 3.6 MM305 / TAEM / HAC

**R9 — Critical: MM305 plans once, synchronously, and aborts if the plan fails.** [D]
- Evidence: the first MM305 tick took **22,720 ms of guidance compute** on the
  control thread (`mm305-hacC-m25-diag`, `guidanceComputeMilliseconds`). In
  KSP the vehicle would hold its last direct inputs for ~23 s at Mach 2.5.
- If no candidate qualifies, `terminal_abort` releases control immediately.
  This happened in 4/4 30-km admissible states, all Mach-2.5 checkpoints on
  Plants A/B/C, and both MM304 handoffs.
- There is no replanning after commitment, no retry, and no degraded mode.
- Correction: move the search to the worker. Keep flying a safe acquisition
  law (e.g., energy-neutral heading-to-HAC) while planning, and replan on
  divergence.

**R10 — Critical: no energy feedback during MM305.** [C][D]
- Code: the tracker inverts the frozen plant model each tick to hold the
  pre-generated altitude/FPA profile (proportional only, 15 s altitude time
  constant).
  - Airspeed and energy are not regulated.
  - The speedbrake is contractually forbidden.
  - The executive has no dissipation phase.
  - The HAC radius can only shrink.
- Evidence with model ≠ plant (`mismatchBmodelC`, real plant B, model C):
  - E90 "crossed runway elevation before reaching the fixed-HAC exit".
  - S60 diverged.
- Even with an exact model, W60 diverged live from a replay-qualified route.
  The route was qualified at dt 0.5 s and flown at 0.1 s; AoA saturated at
  15° and bank at 69° deep in a 4.3 km HAC at 108 m/s. So qualification carries
  no robustness margin.
- Physics: this is geometric trajectory replay, the opposite of the Shuttle
  principle of measuring, predicting and correcting. It survives in the sim
  only because the model *is* the plant.
- Correction: add closed-loop TAEM energy management, i.e. an energy-vs-range
  reference with feedback through the route length (HAC radius or S-turn
  lead), AoA/drag, and (if allowed) the speedbrake. Replan periodically from
  the measured state.

**High: misleading feasibility diagnostics.** [D]
"route is too long for the available energy" is reported when the propagated
vehicle cannot follow a 3–12 km HAC at Mach 2.5 and falls off the route. It is
a lateral infeasibility, not an energy shortfall.

**Medium: HAC-exit state is not conditioned for Final.** [C][D]
- Routes whose exit speed is below the final alignment speed are accepted
  (smallest deficit preferred).
- Exits were observed at FPA −32°, 129 m/s (E90).

**Low: naming.** The success status is named `TAEM_PLAN_UNQUALIFIED` /
`TERMINAL_SOLVER_UNQUALIFIED`.

**Sound** [C]:
- HAC geometry: the centre at (−final_distance, side·R), the tangent exit
  along the runway heading, the sweep sign = side, and the capture course. All
  signs were verified.
- Reciprocal-runway reframing.
- The route-lead/HAC-arc length is counted once (`route_finish`).

**Admissible-state sweep** [D] (Plant C, model = plant, 12 states: 25 km,
750 m/s, −5°, 30/60/90 km from W/S/N/E, heading at the site): **0/12
succeeded.**
- 30 km: rejected at tick 1 (4/4).
- 60/90 km (7 committed routes): 6 flew MM305 to Final and failed in
  Final/flare; W60 diverged in MM305.

### 3.7 Final (approach / preflare / flare)

**R11 — Critical: the flare latch is a fixed 10-s lookahead with no energy check, and Final has no speed/energy closure.** [D]
- Code (`final/sequence.inc`):
  - `terminal_pull_response_seconds()=10.0`. The comment says moving it by
    0.1 s caused terrain contact, which is evidence of brittle tuning.
  - The sink-vs-height profile `min(√(2·0.37g·h), 0.07·h)` is fixed.
  - The glide re-aims each tick at 75 m past the threshold, with FPA clamped
    to −4°…−25°.
  - There is no airspeed loop, no long-landing or go-around logic, and the
    gear offset (3.1535 m) is hard-coded.
- Evidence: every landing attempt (15/15 across the sweep and the Final
  checkpoints) failed at first contact (`results/touchdown_classification.txt`).
  - From the repo's "ideal" Final checkpoint (185 m/s, −20°, 1,274 m), the
    preflare latched at t=0 (1,271 m).
    - Plant A: crashed 191 m short at 37 m/s sink.
    - Plant B: touched down mid-runway at 26 m/s sink, then overran.
    - Plant C: pulled up to +2° FPA, crossed the threshold 945 m high and
      landed 2.6 km past the runway end.
  - MM305-delivered states landed 0.9–2.6 km past the runway end, 90–440 m
    off the centreline, at 13–21 m/s sink.
- Admission: Final admission ("contract is feasible") is computed with
  guidance's internal aero prior, not the plant, and passed in every case.
- Correction:
  - Preflare initiation from a predicted flare trajectory (energy, speed and
    pitch-authority based) rather than a constant lookahead.
  - An outer-glide speed/energy loop.
  - A touchdown aim point that moves with the energy state.
  - An explicit long-landing/abort decision.

**Medium: lateral alignment degrades in preflare.** [D]
Crosstrack grew from 6 m to 176 m during the pull-up (E90), and 577 m at the
runway end (S60). The bank limit shrinks with height (`3+27·h/60`), so late
corrections are impossible.

**Medium: abort reasons are misleading.** [D]
"Control departure: insufficient height" was reported after the vehicle had
already contacted terrain past the runway (S60 h=−64 m). The actual failure was
a long landing.

### 3.8 Touchdown / rollout

- **Defensible** [C]: pitch is released permanently at the first main-gear
  contact (`flight_control.c:508`), as intended. There is no attempt to "keep
  flying" after mains contact.
- **Medium: braking is not tied to nose-gear contact.** Brakes are commanded
  below 120 m/s, i.e. immediately at touchdown [C]. There is no nose-gear logic
  and no derotation law; nose lowering relies on KSP physics, and ShuttleSim
  cannot model it (§3.1).
- **Medium: rollout steering is a fixed-gain P-law.** Heading 0.09,
  cross 0.0033 and lateral-rate 0.036, clamped to ±0.4, with a single
  empirically chosen yaw sign ("pre-a397723 sign validated live") [C].
  Wheel-steer sign conventions between ShuttleSim (+ = right turn) and KSP are
  not documented [U].
- **Low: touchdown speed inconsistency.** Runner success requires a
  60–70 m/s touchdown, while the vehicle `touchdownSpeed` is 75 m/s.

### 3.9 Tests / validation

**R12 — High: the maintained tests cannot detect any defect above.** [C]
- `ShuttleArchitectureContractTests`: phase-machine bookkeeping only.
- `MM305AdmissionTests`: that veto bits fire. It does not check that
  admitted states are flyable.
- `EntryLateralFeedbackTests`: encodes the eastbound assumption.
- `TAEMNativeStackTests`: needs the absent data.
- No test covers MM304 closed-loop energy, Final touchdown quality, the FCS on
  a rigid body, or model-plant mismatch.

**Medium: docs and build targets are out of step with the code.**
- `make test` runs 7 targets (docs say 4). `qualification` is an alias of
  `test`, and `offline-acceptance` does not exist (VALIDATION.md claims both).
- `Validation/README.md` lists a nonexistent `TaemInterfaceCaptureEnergyTests.c`.
- The built-in defaults in `models.c` (final distance 8 km, final glide 20°,
  TAEM glide 12°) differ from `Configuration/default.json` (3.2 km, 28°, 24°).
  The unit tests use the built-ins while simulator runs use the JSON.
- The newest recorded MM304 evidence (`Validation/MM304OfflineReport.md`,
  2026-09-14) predates the current measured-feedback MM304 law (commit
  `634f89a`). No end-to-end evidence exists for the current law.

### 3.10 Obsolete / legacy architecture

- Dead entry slices (§2).
- `mm304_handoff_*` station parameters are still configured and used only by
  the non-live `entry_taem_interface_capture`.
- `Docs/CURRENT_SHUTTLE_LANDING_SYSTEM.md` describes a Python bridge and S-turn MPC that
  no longer exist.
- There are dozens of env-var overrides (`KSP_LANDER_*`) in production control
  code (FCS gains, diagnostics).

---

## 4. False positives / things that look wrong but are defensible

- **Rotating-frame specific energy with the centrifugal term** (`rotating_specific_energy`)
  is the correct energy for air-relative guidance.
- **One-way ownership and fail-closed commitment are correct principles.**
  The defects are the admission content and the missing fallback, not
  the one-way design.
- **Speedbrake disabled** is a stated mission contract, not a bug. It does,
  however, remove the Shuttle's main TAEM/Final energy effector, so something
  else must replace it.
- **Pitch released at main-gear contact** matches the stated intent.
- **The MM304 predictor is advisory only** (verified), so it cannot inject
  energy or fake admission.
- **HAC circle construction, signs and the reciprocal-runway frame are
  correct.**
- **The asymmetric negative alpha modulation** (smaller drag-shedding authority
  late in entry) is physically motivated (it avoids the density dive).
- **Semi-implicit Euler at 20 ms in ShuttleSim** deliberately mirrors
  PhysX/KSP ordering.
- **The stock Kerbin atmosphere in `world.c`** reproduces the KSP pressure,
  temperature and latitude/sun curves. This is a strength.

## 5. Historical Shuttle fidelity vs necessary KSP adaptation

Should mirror the Shuttle (principles):
- Drag-vs-velocity reference with measured-drag feedback, an altitude-rate
  damping term, and a bias/integral with hold logic.
- Bank magnitude from commanded vs available L/D.
- Bank sign from target-relative azimuth with a velocity-dependent deadband.
- Energy/range-driven phase routing.
- TAEM energy management with closed-loop feedback (energy vs range-to-go,
  S-turn or HAC adjustment).
- Guidance at seconds cadence with a fast stabilising inner loop.

Should *not* be copied:
- Earth numbers (2,500 ft/s / 82 kft / 60 nmi TAEM, 40° AoA, 33 ft/s² drag).
  Kerbin's 600 km radius, 70 km atmosphere and ~2.2 km/s orbital speed
  compress everything. The KSP STS-N's L/D (~0.6 supersonic, ~1.6–4 subsonic;
  uncertain) differs from the Orbiter's.
- The Orbiter's 19° outer glide, 1.5° inner glide and 300 KEAS approach speed.
  These must be derived from the KSP vehicle's subsonic L/D.
- RCS/aero blending thresholds (q = 2/10/40 psf). They must come from the KSP
  vehicle's measured control effectiveness.

Present implementation vs these principles:
- It keeps some Shuttle vocabulary (phase names, D(V), HAC), but the
  mechanisms differ.
  - Phases are velocity-fraction gates.
  - The drag reference is self-referential.
  - The bank law is equilibrium-relative and saturates.
  - Lateral steering is centreline-relative.
  - TAEM is plan-and-replay without energy feedback.
- Sources checked:
  - Harpold & Graves, "Shuttle entry guidance" (NTRS 19790037248): drag
    profile tracking to TAEM at 2,500 ft/s, oscillation-suppressing control
    laws.
  - Harpold & Hill, NASA TM-81034 (MCC formulation based on the Orbiter
    guidance).
  - Bump 1977 (NTRS 19770022264): lateral deadband and minimum-bank study.
- The full-text Shuttle PDFs could not be text-extracted in this container. The
  exact gains and I-loads are therefore not cited [U].

## 6. Validation gaps (claims the repo cannot currently prove)

1. Any KSP behaviour of the FCS (never executed offline).
2. That the ShuttleSim plant matches KSP: the fitted data is absent, and no
   held-out parity evidence is in the repo.
3. That MM304 converts a broad set of entry states into MM305-flyable states
   (current law: 0 end-to-end successes observed; admission ≠ feasibility).
4. That MM305 routes survive model error (planner model == plant in every sim
   run).
5. That Final lands from real MM305 exits (0/15 in this audit).
6. Real-time behaviour: the 22.7 s synchronous MM305 search, a 10 Hz inner loop,
   and RPC latency.
7. Robustness to navigation/AoA bias, mass, CG or atmosphere dispersions: there
   is no combined-dispersion campaign in the repo.
8. Live KSP: not evaluated. No kRPC endpoint or KSP instance was available in
   this environment [blocker].

## 7. Prioritised remediation sequence (causal order)

1. **Foundations and truth**
   - Commit (or deterministically regenerate) the plant data.
   - Make `make test` pass on a clean checkout.
   - Separate the MM305 planning model from the sim plant (inject calibrated
     model error).
   - Stop feeding guidance the sim's servo parameters and the plant force book.
   - Unify the `g_force` definition.
2. **ShuttleSim fidelity:** add a rigid-body/surface-moment mode that executes
   `flight_control_step`, plus a ground model with derotation, gear contact and
   crash criteria.
3. **MM304 guidance feasibility**
   - Rewrite the vertical channel as L/D command vs *measured* L/D with ḣ
     damping and a bias term (R4, R5).
   - Rebuild D_ref(V) from the range requirement (R6).
   - Replace centreline steering with delta-azimuth and a velocity-dependent
     deadband (R7).
   - Add overflight handling.
4. **Handoff contract:** admission must be the MM305 planner's own
   feasibility/reachability set, plus closure and an upper energy bound (R8).
5. **MM305:** plan asynchronously, keep a safe acquisition law while planning,
   add closed-loop energy management and periodic replanning, and qualify with
   model-error margins (R9, R10).
6. **Final:** add an energy/speed-closed outer glide and a predictive preflare,
   plus long-landing and abort logic (R11). Only then tune the flare and
   rollout.
7. **FCS:** gain scheduling by q and Mach, a pitch trim integrator with
   anti-windup across the regime, and turn-coordination feedforward. Remove
   env-var gain overrides from production.
8. **Tests:**
   - Closed-loop campaigns over combined dispersions (plant L/D ±50%, heading,
     crossrange sign, runway end, mass, atmosphere).
   - Admission-⇒-MM305-feasible property tests.
   - A heading-mirror test for lateral logic.
   - Touchdown-quality gates.
   - Reconcile docs and build targets.

---

## Appendix — experiment log

All raw run directories are under `ShuttleSim/runs/` (git-ignored). Compact
summaries are in `results/run_summaries.jsonl`, first-contact classification in
`results/touchdown_classification.txt`, and MM304 lateral traces in
`results/mm304-mm304C-*-entry-trace.txt`.

| Group | Runs | Outcome |
|---|---|---|
| MM304 from KSP post-burn, plants A/B, no force book | 2 | bank 0° all flight (confidence 0); overflight, abort |
| MM304 from KSP post-burn, plants A/B, with force book | 2 | bank 0° until q≈2 kPa (R5); overflight, abort |
| MM305 Mach-2.5 checkpoints, plants A/B/C | 5 | rejected at tick 1 (4); 1 committed on C |
| Final "ideal" checkpoint, plants A/B/C | 3 | short crash / hard long / 2.6 km past end |
| MM305 admissible sweep, plant C | 12 | 0 success (4 rejected, 6 Final failures, 1 MM305 divergence, 1 Final long) |
| MM304 east/west mirror, plant C | 2 | west crossrange diverges; both handoffs rejected by MM305 |
| MM305 model ≠ plant (plant B, model C) | 2 | ground contact before HAC exit; tracking divergence |
| End-to-end from KSP orbital states, plant C | 3 | all abort in MM304: post-burn and EI-70 hold bank 0° until q≈1.2 kPa (R5/R6) and pass 195 km beyond the runway; steep-3° falls ~560 km short with lift-up (likely physically infeasible, not counted against guidance) |
| Seed-plant first run with invalid audit atmosphere file (MM305 model unavailable) | 1 | same MM304 failure as plant A |
