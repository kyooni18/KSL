# CLanding

`CLanding/` is the build entry point and C ABI description of the shuttle landing backend. The backend itself is the Rust crate in [`../rust/landing`](../rust/landing/README.md); the former C implementation was replaced by it and is available from git history (the commit before the Rust rewrite) as the behaviour reference.

- `make -C CLanding` builds `rust/landing` with cargo and installs the executable at `CLanding/build/landing_backend`, where the Python mission layer, ShuttleSim runner and tools expect it. `BUILD=<dir>` selects another output directory.
- `include/` holds the headers describing the C ABI that `liblanding.a` (the crate's static library) exports. The regression tests in `../Validation` and ShuttleSim's optional `shuttlesim_expert` library use that interface.
- `guidance/guidance_internal.h` is the private guidance plumbing header used by white-box C tests.

`include/landing.h` is deliberately only a compatibility umbrella. The actual public surface is divided into `landing_types.h`, `guidance_types.h`, `vehicle_types.h`, and `landing_api.h`.

## Ownership (module layout of `rust/landing/src`)

- `app/` process startup and top-level controller orchestration.
- `control/` actuator-level flight-control command generation.
- `prediction/` trajectory prediction, decision envelopes, asynchronous prediction policy.
- `vehicle/` vehicle/planet math, aerodynamics, learned vessel physics.
- `calibration/` online/offline model calibration.
- `transport/` kRPC/C-Nano communication; `krpc_cnano/` and `nanopb/` are the protocol client.
- `telemetry/` simulator telemetry, JSON, planner logs, flight-log codecs.
- `persistence/` durable physics/model state (SQLite).
- `guidance/` MM304 entry, MM305/TAEM, Final, flare, touchdown, rollout, plus the top-level TAEM planner modules.
- `physics/` the ShuttleSim flight-physics model the backend shares with the simulator.

Important behavioral invariants remain unchanged: MM304 hands ownership one-way to MM305/TAEM only after the terminal acquisition state is executable; MM305 has no S-turn or generic energy-dump phase; MM305 uses the fixed runway-anchored HAC geometry and must prove an executable path before commitment; Final/flare/rollout continue through the established public guidance state machine and its existing handoff gates. See `ARCHITECTURE.md`.

## Build

    make -C CLanding
    make -C CLanding BUILD=build-refactor

Cargo features: `mm305-supersonic-active-uncertainty` replaces the former `make MM305_SUPERSONIC_ACTIVE_UNCERTAINTY=1` experiment switch (`cargo build --release --features ...` in `rust/landing`).

## Tests

    make -C CLanding test

runs the Rust white-box tests (`cargo test`) and the C-ABI regression gate: the MM304→MM305 architecture/ownership contract, MM305 admission and recovery, TAEM frames/energy and tracker allocation, vessel physics, flight control, entry laws, Final delivery parity and touchdown quality, deorbit admission, and the external backend protocol. Each C test links against `liblanding.a`.

`Validation/BackendParity.py REF_BUILD NEW_BUILD [scenario[:engage[:max-sim-s]]]` flies ShuttleSim scenarios with two backends and compares the deterministic simulator telemetry byte for byte. Build the reference backend from the pre-rewrite commit.

No default test connects to live KSP or starts a ShuttleSim campaign.
