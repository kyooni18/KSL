# CLanding architecture

CLanding is organized around explicit subsystem ownership. The public compatibility header remains `include/landing.h`, but new code should prefer the narrow domain headers when practical.

## Guidance ownership

`guidance/guidance.c` is the public update wrapper. Guidance implementation is divided by flight responsibility rather than accumulated history:

- `guidance_core.c`: shared guidance state, stabilization, command construction, and common primitives.
- `guidance_entry.c`: MM304/Entry ownership. Its internal slices separate reference generation, bank allocation, S-turn planning, topology control, and contract enforcement.
- `guidance_taem.c`: MM305/TAEM ownership. Its internal slices separate state, S-turn behavior, energy logic, candidate planning, preview, and committed control.
- `guidance_hac_path.c`: analytic HAC geometry, transitions, tracking, and vertical-profile support.
- `guidance_hac_planner.c`: HAC vertical profile, energy pricing, dynamic selection, Variant-B search/control, diagnostic preview, and finite-lead ownership.
- `guidance_final.c`: final-approach planning, phases, recovery, and sequence control.
- `guidance_terminal.c`: terminal ownership transitions and orchestration.
- `guidance_internal.h`: private interfaces shared only by guidance modules.

The `.inc` files under these owners are implementation slices, not independent translation units. They deliberately preserve existing static-symbol relationships and numerical behavior while keeping each responsibility navigable. Promote a slice to its own `.c` file only when its interface is intentionally stable.

## Prediction and decision ownership

`prediction/predictor.c` owns prediction and includes separate implementation slices for entry simulation, entry planning, deorbit planning, and Entry topology.

MM304 command ownership is intentionally single-path. `entry_program_guidance()` computes the live DRAGREF/ENTRYLATERAL/alpha demand and creates a deterministic provisional S-turn segment from that same demand. Prediction may propagate that executable segment and suggest reversal timing, while the asynchronous two-event topology search may replace it only after full evolving-guidance shadow validation and normal lineage/adoption checks. There is no separate free-bank/free-alpha Entry optimizer or predictor-side command supervisor in production.

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

It covers architecture/ownership, TAEM interface capture and energy, vessel physics, and the backend protocol. Maintained tests use supported headers and APIs; they do not include production `.c` files.

Historical white-box tests and campaign probes under `Validation/` are not part of the maintained gate. They may remain temporarily while an active validation campaign depends on them, but new regression coverage should be black-box or public-contract based rather than exposing file-local helpers.

## Refactoring rules

Preserve flight behavior while changing structure. Remove dead wrappers and temporary diagnostics instead of exporting private symbols to keep old tests alive. Generated compiler dependencies (`-MMD -MP`) must track every internal implementation slice. A structural change is complete only after the production backend builds and the maintained regression gate passes.
