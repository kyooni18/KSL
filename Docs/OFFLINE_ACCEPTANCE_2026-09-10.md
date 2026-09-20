# Offline acceptance — 2026-09-10

The native C runway-landing stack is accepted for the current offline engineering stage.
No live KSP session, kRPC server, serial endpoint, quicksave, or flight-control connection
was opened during this acceptance pass.

## Accepted production architecture

- The production runtime uses kRPC C-Nano 0.6.0 over the kRPC serial protocol.
- The retired Python kRPC bridge and TCP RPC/stream-port runtime are absent from the
  production path.
- Atmospheric guidance commands are closed by the native C flight-control law and sent
  through the native C-Nano actuator client.
- The native SQLite physics-history store is exercised against temporary databases and
  the existing project database through a read-only compatibility check.

## Repeatable gate

Run:

```bash
make -C CLanding offline-acceptance
```

The final pass completed successfully. It includes clean native build, C-Nano transport
and batching, stateful fake-serial client integration, native flight control, physics
store, vessel physics, Entry/MM304, TAEM, predictor supervision, terminal/approach
contracts, UI/backend protocol compatibility, and native-architecture checks.

The terminal integration gate completed three deterministic runway-09 scenarios and
sent 219 actuator-side C-Nano wire requests through the stateful fake serial transport.
It covers final capture, preflare, inner final, touchdown flare, runway contact, rollout,
and stop completion while checking bounded native FCS outputs.

AddressSanitizer and UndefinedBehaviorSanitizer qualification also completed with the
same three-scenario terminal gate.

Raw command output is retained in:

- `Docs/OFFLINE_ACCEPTANCE_2026-09-10.log`
- `Docs/OFFLINE_SANITIZER_2026-09-10.log`

## Remaining acceptance boundary

This is not proof of a successful physical KSP landing. The deterministic terminal gate
validates software integration and state-machine continuity, while Entry/TAEM behavior
is covered by the separate production contracts and predictor tests. The next acceptance
stage is an explicitly authorized live KSP campaign using a configured C-Nano serial
endpoint, beginning from the established quicksave/test procedure and retaining the
existing crash/quickload safety rules.
