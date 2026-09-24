# CLanding

`CLanding/` is the canonical native C implementation of the shuttle landing system. The old flat implementation under `../Legacy/CLanding` is reference material only; new flight logic belongs here.

## Design

The code is split by ownership rather than by historical development order.

- `app/` owns process startup and top-level controller orchestration.
- `control/` owns actuator-level flight-control command generation.
- `prediction/` owns trajectory prediction, decision envelopes, and asynchronous prediction policy.
- `vehicle/` owns vehicle/planet math, aerodynamics, and learned vessel physics.
- `calibration/` owns online/offline model calibration.
- `transport/` owns kRPC/C-Nano communication.
- `telemetry/` owns simulator telemetry, JSON, planner logs, and flight-log codecs.
- `persistence/` owns durable physics/model state.
- `include/` contains the supported public interface.

`include/landing.h` is deliberately only a compatibility umbrella. The actual public surface is divided into `landing_types.h`, `guidance_types.h`, `vehicle_types.h`, and `landing_api.h`. New code should include the narrowest header it needs.

## Guidance

Guidance is no longer a single source file.

- `guidance.c` is the public dispatcher.
- `guidance_core.c` owns shared guidance state/control machinery.
- `guidance_entry.c` owns MM304 entry orchestration. Its `entry/*.inc` slices separate reference construction, bank allocation, S-turn planning, topology, and handoff-contract logic.
- `guidance_taem.c` owns the live MM305/TAEM integration boundary and shared TAEM state/energy/executive helpers.
- `taem_frames_energy.c`, `taem_geometry.c`, `taem_reachability.c`, `taem_planner.c`, `taem_candidate_search.c`, `terminal_model.c`, `terminal_propagator.c`, `terminal_solver.c`, and `taem_tracker.c` form the rebuilt MM305 planning/prediction/tracking stack. The removed legacy HAC planner/path slices are not part of the compatibility surface.
- `guidance_final.c` owns outer final, preflare, flare, touchdown, rollout, and recovery. Its `final/*.inc` slices separate planning/gates, flight phases, sequencing, and recovery/invalidation.
- `guidance_terminal.c` owns terminal phase/ownership orchestration.
- `guidance_internal.h` is private implementation plumbing and is not a supported API.

The remaining `.inc` slices are private implementation modules for Entry, Final, Controller, and the small shared TAEM state/energy bridge. New MM305 reconstruction code uses explicit `.c`/header modules instead of reviving the deleted HAC planner/path slices.

Important behavioral invariants remain unchanged: MM304 hands ownership one-way to MM305/TAEM only after the terminal acquisition state is executable; MM305 has no S-turn or generic energy-dump phase; MM305 uses the fixed runway-anchored HAC geometry and must prove an executable path before commitment; Final/flare/rollout continue through the established public guidance state machine and its existing handoff gates.

See `ARCHITECTURE.md` for the flight-logic ownership and invariants that refactors must preserve.

## Build

    make -C CLanding

For isolated refactor/CI work, use a separate build directory:

    make -C CLanding BUILD=build-refactor

ShuttleSim's expert library consumes the same modular guidance sources through `ShuttleSim/CMakeLists.txt`; it does not maintain a second copy of flight logic.

## Tests

Normal development has one small supported regression gate:

    make -C CLanding test

It intentionally runs only four high-value tests: the MM304→MM305 architecture/ownership contract, TAEM interface energy/capture semantics, vessel physics, and the external backend protocol. These tests use supported headers/APIs and do not include production `.c` files to reach private helpers.

The older narrow contract/probe files in `Validation/` are historical investigation material. They are not the compatibility surface and must not drive production architecture back toward a monolith. Keep or use one only when a specific investigation still depends on it; otherwise prefer a small public-interface regression.

No default test connects to live KSP or starts a ShuttleSim campaign.
