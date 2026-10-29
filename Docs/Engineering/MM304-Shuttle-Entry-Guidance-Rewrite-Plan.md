# MM304 rewrite plan: Shuttle entry guidance (40° AoA, drag-velocity tracking)

Date: 2026-10-29. Status: plan only, nothing implemented yet.

## 1. Goal

Replace the current MM304 with a straight port of the real Space Shuttle entry
guidance (EGEXEC, Harpold & Graves 1979, flown STS-1 to STS-135), scaled to
Kerbin and this orbiter:

- Fly a **pre-planned angle-of-attack schedule against velocity**: hold 40°
  through the high-speed part of entry, then ramp it down to the max-L/D
  incidence at the TAEM interface (Mach 2.5).
- Steer **energy (downrange) with bank-angle magnitude only**. Track a
  drag-acceleration reference D_ref(V) built from the five Shuttle phases.
  Allow at most ±3° of AoA modulation, and only for small drag corrections.
- Steer **crossrange with bank sign only**, using roll reversals triggered by
  a velocity-dependent azimuth-error deadband.
- Run **one guidance code** both in flight and in the predictor/deorbit
  planner. The Shuttle used the same EG software in its ground trajectory
  design.

The MM304 to MM305 handoff contract (`entry_mm305_admission_envelope`) and
everything downstream (TAEM/HAC, Final) stay as they are.

## 2. How the real Shuttle did it (research summary)

### 2.1 Phases (`ISLECT`)

| # | Phase | Starts | Drag reference | Notes |
|---|-------|--------|----------------|-------|
| 1 | Pre-entry | EI (400 kft) | none | α = 40°, bank = 0 (I-load `MM304PH`/`MM304AL`), attitude hold. Ends when load factor > `ASTART` = 3 ft/s² (≈0.09 g). |
| 2 | Temperature control | drag > 3 ft/s² | Two **quadratics in V**, D = d23·(c0 + c1·V + c2·V²), with dD/dV slopes `AK`=-3.5, `AK1`=-5.5 | Shaped for a constant heating rate on the TPS. Forced to phase 3 below `VB1` = 19 000 ft/s. |
| 3 | Equilibrium glide | V < 19 000 ft/s, or when the profiles cross | D = (1 − V²/Vs²)/aldco, so lift balances gravity minus centrifugal | Vs = local circular speed (25 766 ft/s). |
| 4 | Constant drag | when D_ref reaches the constant-drag level | D = T2 ≈ `ALFM` 33 ft/s² (≈1 g) | T2 is re-solved each pass so the range flown down to `VQ` matches. |
| 5 | Transition | E/m < `ETRAN` (V ≈ 10 500 ft/s) | **Linear in energy**: D = DF + c1·(E − EEF4) | Ends at `DF` ≈ 19–21 ft/s² at TAEM energy (2 500 ft/s, ~82 kft, Mach 2.5). |

Range prediction is analytic for each phase: Simpson's rule on the quadratics,
logarithmic for equilibrium glide, V²/2D for constant drag, and logarithmic in
energy for transition. Each pass the guidance solves one parameter (`D23`, the
profile scale at VB1) by a second-order expansion, so that predicted range
equals range-to-go minus the transition-range bias `RPT`. In phase 4 it solves
`T2` instead, and in phase 5 it solves `DREFP`.

### 2.2 Vertical channel: the L/D command

```
L/D_v,cmd = L/D_ref + C16·(D − D_ref) + C17·(ḣ_ref − ḣ)        (+ integral ḣ term, ZK1)
L/D_ref   = g(1 − V²/Vs²)/D_ref + (2·ḣ_ref + C2·Hs)/V
ḣ_ref     = −Hs·V·(2·D_ref/V² − dD_ref/dV·…)   (exponential atmosphere, Hs scheduled with V)
bank      = acos( L/D_v,cmd / (L/D)_measured )
```

- Gains are scheduled on drag: `C16 = 0.27·D^-0.2` (clamped 0.025–0.36 s²/ft)
  and `C17 = 6.148e-3·D^-0.58` (clamped 0.0025–0.014 s/ft). This gives a damped
  drag-tracking loop whose bandwidth rises as drag builds.
- The guidance divides by **measured** L/D (`XLOD`, floor 0.5 on the Shuttle),
  so the bank absorbs any aero mismatch.
- Bank limits: `RLM` = 95° above 8 000 ft/s, then 70°. The minimum bank comes
  from `LMN` (L/D_v ≤ 0.966, or 0.80 outside the deadband), so some bank is
  always available for crossrange.

### 2.3 AoA schedule (3B "40-30" I-load, V in ft/s)

| Earth-relative V | α command |
|---|---|
| > 23 000 (and all of pre-entry) | **40°** |
| 23 000 → 22 000 | quadratic, 40 → 38.75° |
| 22 000 → 19 000 | linear, 38.75 → 31.25° |
| 19 000 → 18 000 | quadratic, 31.25 → 30° |
| 18 000 → 8 070 | **30° plateau** |
| 8 070 → 7 070 | quadratic, 30 → 28.5° |
| 7 070 → 500 | linear 7.285 + 0.003·V, giving **~14.8° at TAEM (2 500 ft/s)** |

Why 40°: below about Mach 14–20 heating on the upper surface and wing leading
edge drives the incidence. 40° is on the back side of the L/D curve but still
gives L/D ≈ 1 hypersonically. The ramp brings the orbiter to TAEM on the front
side of the L/D curve, at roughly 14° α, Mach 2.5 and 82 kft.

**AoA modulation** is active only below `VNOALP` (≈ 20 500–22 000 ft/s):
`Δα = CD·(D_ref/D − 1)/C20`, limited to ±3° (`DLAPLM`). It adds a roll bias
(`CRDEAF`) so bank and α do not fight each other. It is a fast, small drag
trim; bank carries the real energy control.

### 2.4 Lateral channel: roll reversals

- Bank sign `RK2ROL` flips when |ΔAz| exceeds the deadband
  `YL = clamp(−7.5° + 0.00625·V[ft/s], 10°, 17.5°)`. That is about 17.5° at
  high speed and 10° near TAEM. The first reversal uses a tighter `Y1` until
  it has happened (`IDBCHG`).
- Reversals are flown at limited roll rate. The ḣ feedback is locked out for
  `ACN1` = 50 s·D after each reversal (`VTRB`), so the drag loop does not chase
  the phugoid the reversal excites.
- Typical nominal entry: first roll near drag onset, then 4–5 reversals.

### 2.5 Interfaces

- EI is 400 kft, about 25 000 ft/s, at γ ≈ −1.2°.
- TAEM interface is V = 2 500 ft/s (Mach 2.5), about 82 kft, at 50–60 nmi.
- MM304 also ends early on range (`R_TAEM`) or on a high-energy range/speed
  combination.

## 3. Current KSL MM304: what is there and what is wrong

| Area | Today | Problem |
|---|---|---|
| Live command path | `guidance/entry/contract.inc::entry_program_guidance` (one ~660-line function) + DRAGREF (`entry_drag_reference.c`) + `entry_alpha.c` + `entry_lateral.c` | Months of patches: runway-end preview, HAC handoff targeting, yaw-reserve AoA ceiling, crossrange tolerance hacks, and more. The structure is not Shuttle-like. |
| Energy law | `entry_energy_control.c` is described in `ARCHITECTURE.md` as the energy law | **No production caller.** Only `Validation/EntryEnergyLawTests.c` (run by `make test`) calls `entry_energy_update`; the live path does not, so the doc is wrong. |
| AoA | `entry_alpha_schedule_default` holds **18° (trim) at all speeds**, with ±4° modulation | This was deliberately changed away from a 27° ramp (see the comment about the 2026-10-13 polar flights). It is not Shuttle-like at all. |
| AoA limits | `VehicleProfile.maximum_angle_of_attack = 28` (STS-N, `vehicle/models.c:25`) | A 40° hold needs ≥ 43° (40 + 3° modulation). Stall proxies (`vessel_physics.c`, onset = 0.75·max α) depend on this value. |
| Bank limit | `MM304_PLANNED_MAX_BANK_DEG = 40` | The Shuttle needed 70–95°. Low L/D makes this worse: at 40° bank only 23% of lift goes sideways. |
| Predictor / deorbit planner | `prediction/entry_simulation.inc::entry_bank` has its **own** entry law: AoA from `entry_angle_of_attack` plus a drag deficit, its own bank logic | The forecast flies a different MM304 than the live vehicle. Memory notes trace energy misjudgments to predictor/plant mismatch. |
| Yaw authority | `contract.inc` lowers the α ceiling as rudder use rises (live polar departure at Mach 3.6 with α 22→28°) | At 40° the rudder is blanked, as on the real orbiter. The Shuttle flew **RCS yaw jets** until about Mach 1. Our FCS turns RCS on above q 700 Pa and off below 500 Pa (`flight_control.c:52`), which is not the same thing. |

## 4. Can this orbiter fly 40°? Aero check

From `ShuttleSim/reference-model/sts_aero_cells20261028.csv` (L/D and CD):

| Mach | best L/D (α) | L/D @ 30° | L/D @ 40° | CD @ 40° vs @ 20° |
|---|---|---|---|---|
| ≥ 7.1 | 0.52 (28–35°) | 0.52 | **0.49** | 0.39 vs 0.28 |
| 6.6 | 0.58 (22°) | 0.55 | 0.52 | 0.37 vs 0.23 |
| 5.6 | 0.60 (18°) | 0.44 | 0.41 | 0.46 vs 0.22 |
| 4.5 | 0.64 (20–25°) | 0.50 | 0.47 | 0.42 vs 0.21 |
| 3.5 | 0.74 (22°) | 0.56 | 0.51 | 0.43 vs 0.20 |
| 2.5 | 0.82 (12°) | 0.77 | 0.66 | 0.38 vs 0.21 |

What the table shows:

1. **At entry Mach (≥ 7) 40° costs almost no L/D** (0.49 against 0.52), and
   CD is about 40% higher. This is the Shuttle's situation: back side of the
   curve, but cheap.
2. **Below about Mach 6 high α becomes expensive**, so the schedule has to be
   on the front side by roughly Mach 4–5. This is the Shuttle ramp. The
   earlier 27° failure happened because high α was held through Mach 4–7. It
   does not argue against 40° at the start.
3. **The rows at Mach 7.1 and above are identical copies.** Nothing above
   Mach 7.1 has been measured, and that is exactly where the 40° hold lives.
   The first deliverable is to measure the 30–45° cells at Mach 5–8.

## 5. Scaling to Kerbin (I-loads)

The Shuttle I-loads are in Earth ft/s. Kerbin's EI is about 2 290 m/s at
70–72 km, so V/V_circ ≈ 1.0 as on Earth. The TAEM interface is Mach 2.5 at
25 km (`mm305_target_mach = 2.5`, `taem_interface_altitude = 25000`), almost
exactly the Shuttle's Mach 2.5 at 82 kft. In V/V_circ terms, though, Kerbin's
TAEM sits at about 0.33 against the Shuttle's 0.10, because the atmosphere is
compressed. So:

- **High-speed breakpoints** (40° hold end, 30° start, phase 2/3 boundary
  `VB1`, `VNOALP`) are set as fractions of the local circular speed V_s. This
  is thermal and energy physics.
- **Low-speed breakpoints** (start of the final ramp, TAEM end) are set in
  **Mach**, because aero front side and control authority depend on Mach.
- Drag levels (`ALFM`, `DF`, the temperature-control scale) are computed at
  init from vehicle q, g and heating limits and the TAEM energy contract. They
  are not copied from Earth's 33 ft/s².

Proposed starting table for STS-N on Kerbin. It is an I-load table on the
vehicle profile, so it can be tuned without code changes.

| Breakpoint | Shuttle | Rule | Kerbin ≈ |
|---|---|---|---|
| 40° hold ends | 23 000 ft/s | 0.89·V_s | ~2 040 m/s |
| reach 30° | 18 000 ft/s | 0.70·V_s | ~1 600 m/s |
| 30° plateau ends | 8 070 ft/s (≈M8) | Mach 6 (aero: 30° still ≈ best L/D at M≥6.6) | ~Mach 6 |
| TAEM α | 14.8° @ M2.5 | max-L/D α at M2.5 (12–14°) | 14° @ M2.5 |
| α modulation on | 22 000 ft/s | 0.85·V_s | ~1 950 m/s |
| modulation limit | ±3° | same | ±3° |

The new rows rest on the aero data. The 30° "plateau" will be short on Kerbin
(Mach 7 down to 6) and may merge into the ramp. That is acceptable; it follows
from the shape of the atmosphere, not from a design choice.

## 6. Target architecture

New re-entrant module `guidance/entry_eg.c` with header `include/entry_eg.h`.
Its subroutines map one-to-one onto EGEXEC so the code can be read against
the Shuttle source:

```
eg_init        (EGINIT)   reset, I-load derived constants, RPT, VQ, D23 seed
eg_scale_ht    (EGSCALHT) Hs(V) — from the Kerbin atmosphere table, not the Earth fit
eg_common      (EGCOMN)   energy, T2, phase-5 look-ahead (DREFP5)
eg_alpha_cmd   (EGALPCMD) schedule table lookup (+ Δα from eg_lodv_cmd)
eg_range_pred  (EGRP)     phase 2/3 analytic range, D23 solve
eg_ref_23      (EGREF)    temp-control quadratics / equilibrium glide D_ref, ḣ_ref
eg_ref_4       (EGREF4)   constant drag
eg_tran        (EGTRAN)   linear-in-energy transition
eg_gains       (EGGNSLCT) C16/C17 schedule
eg_lodv_cmd    (EGLODVCMD) L/D command, α modulation, deadband, LMN
eg_roll_cmd    (EGROLCMD) bank = acos(L/D_v / L/D_meas), reversals, RLM, rate limit
```

Contract:
`EgOutput eg_step(EgState *, const EgInput *, const EgIloads *)`

- Input is SI and Earth-relative V, inertial V, altitude above the runway,
  ḣ, measured drag, measured L/D, load factor, range-to-go, ΔAz, current
  α and bank, and dt.
- Output: α command, bank command, phase, D_ref, ḣ_ref, reversal flag and the
  end-of-MM304 flag.
- No global state, no telemetry structs and no `getenv`, so the predictor can
  call it thousands of times.

Ownership after the rewrite:

| Keep | Replace/retire |
|---|---|
| `entry_mm305_admission_envelope` handoff gate; HAC-delivery target, which feeds range-to-go and the TAEM energy contract | `entry_drag_reference.c` (DRAGREF) becomes EG phases 2–5 |
| `entry_exec.c`, reduced to an ISLECT mirror for telemetry and checkpoint restart | `entry_alpha.c` becomes `eg_alpha_cmd` |
| FCS (`flight_control.c`), with changes from §7 | `entry_energy_control.c` and `Validation/EntryEnergyLawTests.c`: delete (no production caller) |
| Range-to-go geometry: alignment aimpoint, runway-end choice | `entry_lateral.c` bank-sign logic becomes the EG deadband. Keep only `entry_lateral_geometry_turn_sign`, and only if MM305 still needs it |
| | `contract.inc` shrinks to a thin adapter: telemetry to `EgInput`, `eg_step`, `EgOutput` to commands, handoff check |
| | `entry_simulation.inc::entry_bank` and its own AoA law: the predictor calls `eg_step` |

## 7. Flight control and vehicle changes needed for 40°

1. Raise STS-N `maximum_angle_of_attack` to 45°. Re-check every consumer:
   stall proxy onset, `entry_low_q_protective_aoa_floor`, calibration limits,
   and the aero certification clamp in `aero_certification.c:44`.
2. **Yaw jets as on the orbiter.** In the ENTRY profile, keep RCS yaw active
   from EI until Mach ≈ 1 (the Shuttle's yaw-jet cutoff), whatever q is. The
   rudder should take over only below about Mach 3.5. This replaces the
   yaw-reserve α ceiling in `contract.inc`.
3. **Bank about the velocity vector (stability axis).** At 40° α a body-axis
   roll converts into sideslip. Verify that the FCS bank command rolls about
   the velocity vector, i.e. body roll coordinated with yaw rate
   p·tanα. If it does not, add that coordination. The live departure at α 28°
   suggests it currently does not.
4. Raise `MM304_PLANNED_MAX_BANK_DEG` to the Shuttle `RLM` scheme: about 80°
   above Mach ~8, then 70°, with a minimum bank from `LMN`.

## 8. Implementation phases

Each phase ends with `make -C CLanding test` and a ShuttleSim run of the
72 km live-entry scenarios (`ShuttleSim/scenarios/sts-kerbin-2026102*-live-entry-72km.ini`).
Each phase is its own commit.

0. **Aero evidence** (gate): fly or replay pre-entry attitude holds at
   α = 30/35/40/45° from EI down to Mach 5. Fill the Mach 5–8 cells in
   `sts_aero_cells` and confirm yaw-jet authority at 40°. If 40° proves
   unflyable even with jets, fall back to the highest flyable hold (for
   example 35°) with the same schedule shape.
1. **`entry_eg.c` core plus I-loads**, offline. Port EGEXEC in SI units, with
   an I-load struct on `VehicleProfile`/`GuidanceSettings` and JSON keys in
   `models.c`. Unit-check it against a closed-form point-mass entry: phase
   sequence 1→2→3→4→5, D_ref continuity at the phase switches, range-to-go
   null at TAEM.
2. **Predictor on EG.** `entry_simulation.inc` calls `eg_step`. Then
   re-target the deorbit planner (`target_entry_range`, EI γ), because the
   40° hold changes the ballistic range.
3. **Live MM304 on EG.** Shrink `contract.inc` to the adapter, keep the
   handoff gate, apply the FCS changes from §7, and run ShuttleSim end to end.
4. **Cleanup.** Delete DRAGREF, `entry_alpha.c`, `entry_energy_control.c` and
   the dead `entry_reversal_*` fields. Rewrite the MM304 sections of
   `ARCHITECTURE.md`/`README.md` and drop tests that only covered deleted
   modules.
5. **Live KSP flight** with a full deorbit, then compare against the forecast.

Tests (kept small, per project policy):

- EG phase/sequence and range-null on a point-mass entry.
- α schedule table: 40° hold, monotone ramp, TAEM value.
- Sign checks of the L/D command (too little drag means bank increases; too
  much drag means bank decreases).
- A reversal triggers at the deadband.
- Predictor/live parity: the same `eg_step` output for the same input.

## 9. Risks and open decisions

- **Aero above Mach 7 is unmeasured.** The whole 40° hold rests on extrapolated
  cells. Phase 0 resolves this.
- **Yaw departure at high α** has already happened at 28°. Jets and
  stability-axis roll are mandatory, not optional.
- **Range change.** CD at 40° is about 40% higher, so energy is lost faster
  early and the deorbit targeting must move. The Shuttle handled this with a
  fixed EI-range I-load, and we should do the same.
- **Heating** is not modeled in guidance. The temperature-control phase is
  kept for structure and for its drag shape, not for a TPS model. A KSP
  heating limit could later replace the quadratic slopes.
- **Low-energy logic** (Shuttle `ILEFLG`, α to max L/D and range biasing) is
  left out of the first port. Add it if contingency entries need it.

## Sources

- Harpold, J.C. & Graves, C.A., "Shuttle Entry Guidance," *J. Astronautical Sciences* 27(3), 1979 — [ADS abstract](https://ui.adsabs.harvard.edu/abs/1978aans.confQ....H/abstract)
- NASA, *Descent Guidance and Mission Planning for Space Shuttle* — [NTRS 19850008593](https://ntrs.nasa.gov/api/citations/19850008593/downloads/19850008593.pdf)
- JSC-11542 *Flight Procedures Handbook — Entry* — [archive.org](https://archive.org/download/GandalfDDI-SpaceShuttleDocuments/JSC-11542%20-%20Flight%20Procedures%20Handbook%20Rev%20E%20200504.pdf)
- USA005512 *Entry, TAEM and Approach Workbook* — [ibiblio](https://www.ibiblio.org/apollo/Shuttle/Crew%20Training/Entry,%20TAEM%20and%20Approach%20Workbook.pdf)
- NASA TM-104744 *Space Shuttle Entry Terminal Area Energy Management* — [NTRS 19920010688](https://ntrs.nasa.gov/api/citations/19920010688/downloads/19920010688.pdf)
- JSC-14694 *Shuttle Entry Guidance analysis* (1979) — [Scribd](https://www.scribd.com/document/882254155/Shuttle-Entry-Guidance-TM-79949)
- giuliodondi, *kOS-Shuttle-OPS3*: a port of the Level-C EGEXEC to KSP/RSS. The I-load and gain values in §2 come from `src/entry_guid.ks`. [GitHub](https://github.com/giuliodondi/kOS-Shuttle-OPS3)
- FlightGear wiki, *Shuttle guidance — Entry guidance algorithm* (secondary) — [link](https://wiki.flightgear.org/Shuttle_guidance_-_Entry_guidance_algorithm)
