# Validation

All checked-in qualification paths are offline. They build and exercise the
native C guidance stack without opening KSP, a serial device, or a kRPC server.

- `make -C CLanding clean all` builds the C17 backend with the vendored kRPC
  C-Nano 0.6.0 and nanopb 0.4.9.1 sources plus SQLite.
- `make -C CLanding test` runs seven targets: architecture/ownership
  contracts, MM305 admissible-state tests, TAEM frame/energy algebra, the TAEM
  native stack (model capture, route descriptor, Final interface, replay gate)
  against the tracked reference plant in `ShuttleSim/reference-model/`, vessel
  physics, MM304 lateral feedback (crossrange and azimuth logic), and the
  backend NDJSON protocol. It passes on a clean checkout (fitted KSP data under
  the git-ignored `ShuttleSim/data/` is preferred when present).
- `make -C CLanding qualification` is an alias of `test`.
- Closed-loop evidence comes from ShuttleSim (`ShuttleSim/scripts/run_guidance.py`).
  Use `--terminal-atmosphere/--terminal-aero/--terminal-aero-book/--terminal-attitude`
  to give guidance a different model from the plant; without them guidance has
  perfect knowledge of the plant, which is not evidence of robustness.
  ShuttleSim's attitude plant is an ideal AoA/bank servo: the atmospheric FCS
  is not exercised offline.
- `python3 Validation/BackendProtocolTests.py` verifies the UI-to-C NDJSON
  protocol and the serial C-Nano configuration schema without connecting.

For memory/undefined-behavior checks, the full qualification gate also passes
when the backend is rebuilt with AddressSanitizer and
UndefinedBehaviorSanitizer.

Live KSP tests are a separate acceptance stage. `run_headless.command --live`
and the optional independent safety guard must only be used deliberately with a
configured kRPC serial-protocol endpoint. Offline qualification never loads a
quicksave or sends flight controls.
