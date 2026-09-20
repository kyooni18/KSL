# Validation

All checked-in qualification paths are offline. They build and exercise the
native C guidance stack without opening KSP, a serial device, or a kRPC server.

- `make -C CLanding clean all` builds the C17 backend with the vendored kRPC
  C-Nano 0.6.0 and nanopb 0.4.9.1 sources plus SQLite.
- `make -C CLanding test` runs the C-Nano framing/fault-injection transport
  tests, multi-call batch tests, the production C-Nano client against a stateful
  fake serial server, native atmospheric flight-control tests, native SQLite
  physics-store tests, vessel-physics tests, entry/TAEM executive tests, predictor
  supervision tests, and approach architecture contracts.
- `make -C CLanding qualification` extends the default gate with production
  guidance-integration and TAEM-integration contracts plus an offline runway-09
  final-to-rollout gate. The latter drives production terminal guidance through
  the native atmospheric FCS and sends every resulting actuator command through
  the native C-Nano client over the stateful fake serial transport before
  checking flare, runway contact, rollout, and stop completion. This is a
  deterministic terminal integration gate, not a substitute for the separate
  Entry/TAEM qualification contracts or for a live KSP landing.
- `make -C CLanding offline-acceptance` is the repeatable top-level offline gate:
  it rebuilds from clean, runs the full qualification suite, verifies the external
  NDJSON/configuration contract and native C-Nano architecture, and checks the
  existing physics database through the read-only compatibility path.
- `python3 Validation/BackendProtocolTests.py` verifies the UI-to-C NDJSON
  protocol and the serial C-Nano configuration schema without connecting.
- `python3 Validation/CNanoArchitectureTests.py` verifies that production
  source contains no Python bridge/TCP kRPC runtime dependency and that the
  vendored native components are wired into the build.

For memory/undefined-behavior checks, the full qualification gate also passes
when the backend is rebuilt with AddressSanitizer and
UndefinedBehaviorSanitizer.

Live KSP tests are a separate acceptance stage. `run_headless.command --live`
and the optional independent safety guard must only be used deliberately with a
configured kRPC serial-protocol endpoint. Offline qualification never loads a
quicksave or sends flight controls.
