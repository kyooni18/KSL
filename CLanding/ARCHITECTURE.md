# CLanding architecture

CLanding is organized around explicit subsystem ownership. The public compatibility header remains `include/landing.h`, but new code should prefer the narrow domain headers when practical.

## Guidance ownership

`guidance/guidance.c` is the public update wrapper. Guidance implementation is divided by flight responsibility rather than accumulated history:

- `guidance_common.c`: stage-neutral response, runway-frame, and shared TAEM/Final guidance primitives.
- `guidance_core.c`: shared guidance state, stabilization, command construction, and common state-machine primitives.
- `guidance_entry.c`: MM304/Entry ownership. `entry/contract.inc` is the live command path; `entry/forecast.inc` is the advisory forecast. The energy (vertical) law is `entry_energy_control.c`, the lateral (bank-sign) law is `entry_lateral.c`, the alpha schedule is `entry_alpha.c`.
- `guidance_taem.c`: MM305/TAEM live ownership boundary. Route planning (`mm305_plan()`, `include/mm305_planning.h`) is re-entrant and runs on the prediction worker; the per-tick owner flies an acquisition law while no route is held, tracks a committed route, re-plans periodically and on divergence, and corrects the planning model with the measured lift/drag ratio. Fixed-HAC construction, native replay, reachability and tracking live in the `taem_*` / `terminal_*` modules.
- `guidance_final.c`: final-approach planning, phases, recovery, and sequence control.
- `guidance_terminal.c`: Entry→MM305 and MM305→Final ownership transitions and orchestration.
- `guidance_internal.h`: private interfaces shared only by guidance modules.

The `.inc` files under these owners are implementation slices, not independent translation units. They deliberately preserve existing static-symbol relationships and numerical behavior while keeping each responsibility navigable. Promote a slice to its own `.c` file only when its interface is intentionally stable.

## Prediction and decision ownership

`prediction/predictor.c` owns prediction and includes separate implementation slices for entry simulation, entry planning, deorbit planning, and Entry topology.

### MM304 (entry)

MM304 command ownership is single-path. `entry_program_guidance()` (`entry/contract.inc`) computes each cycle:

- **Vertical / energy** (`entry_energy_control.c`): a drag-acceleration reference D_ref(V) — constant level, then a linear transition to the TAEM drag — whose level is re-solved every cycle so that its predicted range equals the range still required. Measured drag is tracked as an altitude-equivalent error `Hs·ln(D_ref/D)` with ḣ damping and a bounded integral; the vertical-lift demand is converted to bank magnitude against the *measured* lift, `cos(bank) = a_v / L_measured`. When bank saturates and more drag is needed, alpha is raised toward its limit.
- **Lateral** (`entry_lateral.c`): bank sign from the target-relative azimuth error with a velocity-dependent deadband (Shuttle delta-azimuth logic), a leg-capture condition before reversal, and a turning-bank floor when the target is far off the nose. It is heading-independent (no eastbound/crossrange-sign assumption).
- **Alpha** (`entry_alpha.c`): velocity schedule plus the drag boost above.

The prediction worker's MM304 forecast is advisory; it never owns the command.

### MM304 → MM305 handoff

Release happens when the Entry executive reports completion and the measured state lies in the MM305 admissible set (`prediction/decision/taem_capture.inc::entry_mm305_admission_envelope`): range inside the TAEM range target, Mach and altitude bands, positive energy margin after the straight-line lower-bound path, descending, flying toward the site (|course-to-site error| ≤ 90°), and inside q/g/stall limits. MM305 then plans its own route from the measured state.

### MM305 → Final and Final

MM305 hands to Final at the HAC exit. If the priced terminal contract rejects an aligned vehicle that is geometrically inside Final's steep-glide envelope, Final takes it rather than aborting (an unpowered vehicle has no better option). Final (`final/sequence.inc`) flies an outer glide to an aim point moved by energy and the learned float L/D, holds a speed reference by glide angle, latches the preflare from the arc height the present sink and pull need plus the incidence-response height, flies a re-solved constant-deceleration arc to a shallow glide, and uses a Newton step on measured lift for incidence in the flare.

`prediction/decision_envelope.c` owns shared physical decision envelopes. Its slices separate basic margins, response timing, control authority, path geometry, path/energy feasibility, and TAEM capture. Decision envelopes are production policy and must not contain environment-variable bypasses for energy, authority, or safety gates.

## Application, transport, and persistence

`app/controller.c` owns `LandingController` state and threading. Configuration, logging, prediction workers, runtime loop behavior, and public API implementation are separate internal slices.

`transport/krpc_cnano_client.c` owns the C-Nano client ABI and state. Session setup, telemetry, controls, warp, and lifecycle code are separate internal slices.

`persistence/physics_store.c` owns SQLite physics persistence. Identity/canonicalization, schema/context management, history queries, sample reduction, and load/observe paths are separate internal slices.

## Public headers

`include/landing.h` is a compatibility umbrella over:

- `landing_types.h`
- `guidance_types.h`
- `vehicle_types.h`
- `landing_api.h`

Do not add new unrelated declarations back into the umbrella when a narrow domain header is sufficient.

## Validation policy

The maintained regression gate is intentionally small:

```
make -C CLanding test
```

It runs seven targets: architecture/ownership contracts, MM305 admissible-state tests, TAEM frame/energy algebra, the TAEM native stack (model, route descriptor, Final interface, replay gate), vessel physics, MM304 lateral feedback (including azimuth reversal), and the backend protocol. `qualification` is an alias of `test`. Maintained tests use supported headers and APIs; they do not include production `.c` files.

Historical white-box tests and campaign probes under `Validation/` are not part of the maintained gate. They may remain temporarily while an active validation campaign depends on them, but new regression coverage should be black-box or public-contract based rather than exposing file-local helpers.

## Refactoring rules

Preserve flight behavior while changing structure. Remove dead wrappers and temporary diagnostics instead of exporting private symbols to keep old tests alive. Generated compiler dependencies (`-MMD -MP`) must track every internal implementation slice. A structural change is complete only after the production backend builds and the maintained regression gate passes.
