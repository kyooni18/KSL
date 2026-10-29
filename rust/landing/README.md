# ksl-landing

The landing backend: deorbit planning, MM304 entry, MM305 TAEM, Final, flare,
touchdown, rollout, the atmospheric flight-control system and the kRPC/C-Nano
transport. It builds the `landing_backend` executable (NDJSON protocol v1 on
stdin/stdout, see `Docs/ARCHITECTURE.md`) and the `liblanding.a` static library
that exports the same C ABI the headers in `CLanding/include` describe.

```sh
make backend                     # cargo build --release, copy to CLanding/build/landing_backend
cargo test --release --manifest-path rust/landing/Cargo.toml   # white-box tests
make -C CLanding test            # C-ABI regression gate, linked against liblanding.a
```

Only a Rust toolchain and the system `sqlite3` library are required; the crate
has no crates.io dependencies.

## Provenance and the behaviour contract

This crate replaces the former C17 backend (`CLanding/**/*.c`, the vendored
C-Nano/nanopb sources it linked, and the ShuttleSim flight-physics sources it
embedded). The Rust was produced by a mechanical, semantics-preserving
translation of that C (c2rust 0.22, LLVM 18 front end) and then adapted to build on
stable Rust: opaque libc types, `_Thread_local` function statics (now
`thread_local!` cells with the same per-thread lifetime) and the two
variadic `set_error` helpers (now a macro over `snprintf`). Public symbols keep
their C names, layouts (`#[repr(C)]`) and calling conventions, which is why the
unchanged C regression tests in `Validation/` link against and pass on it.

Intentionally nothing was "improved": floating point evaluation order, integer
promotion, buffer sizes, truncation behaviour, thread structure, log formats and
JSON number formatting are the translation of the C, so the backend is expected
to behave identically. Evidence:

* every C test binary in `Validation/` prints byte-identical output linked
  against the C objects and against `liblanding.a`;
* `Validation/BackendParity.py` flies ShuttleSim scenarios closed-loop with a
  reference backend and this one and compares the simulator telemetry streams
  byte for byte (the simulator is deterministic and runs in lockstep);
* the NDJSON protocol test passes unchanged.

To reproduce the reference build, check out the commit before the Rust rewrite
(`git log -- CLanding/app/main.c`) in a separate worktree and run `make -C CLanding`
there; pass its `CLanding/build` directory to `BackendParity.py` as the
reference.

Known limits of the guarantee: closed-loop scenarios that use the asynchronous
MM305 planner threads are scheduled on wall-clock time and are not reproducible
even between two runs of the same C binary, so they can only be compared
statistically; and floating-point results are bit-identical only on targets
where the C compiler does not contract `a*b+c` into fused multiply-add (the
reference C build on x86-64 does not).

## Layout

`src/` mirrors the former C tree: `app` (controller, `main`), `guidance`,
`prediction`, `control`, `vehicle`, `calibration`, `transport`, `telemetry`,
`persistence`, the TAEM planner modules at the top level, `physics` (ShuttleSim
flight physics), `krpc_cnano` and `nanopb` (the protocol client).
