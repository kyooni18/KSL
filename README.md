# KSP Shuttle Lander

KSP Shuttle Lander is a PyQt6 operator UI backed by a native C landing stack for
an autonomous Shuttle-style return to KSC Runway 09. C owns deorbit planning,
entry guidance, TAEM/terminal guidance, trajectory prediction, low-level
atmospheric flight control, physics-history persistence, safety handling, and
flight logging.

## Runtime architecture

The production path is native end to end:

```text
PyQt / headless operator
        │ NDJSON protocol v1
        ▼
CLanding native backend
        ├─ deorbit / entry / TAEM / terminal guidance
        ├─ native atmospheric FCS (flight_control.c)
        ├─ native SQLite flight-data store (physics_store.c)
        └─ kRPC C-Nano 0.6.0 client
                  │ ordinary kRPC RPC over TCP (live)
                  ▼
                KSP/kRPC
```

Live guidance, telemetry, and actuator control run in this native C backend;
there is no Python process in the kRPC control path. C-Nano uses the ordinary
kRPC RPC TCP endpoint for live KSP and does not support Streams or Events, so
telemetry is scheduled as synchronous RPC polling. Fast flight-state items are
sampled every control tick, medium-rate plant/control state is cached, and orbit
state is polled at a lower rate. ShuttleSim uses the same backend through its
serial/pseudo-terminal simulator transport. Actuator writes and atmospheric
pitch/roll/yaw commands are handled by the native client and C FCS.

The external UI-to-backend NDJSON protocol remains unchanged in purpose: UI
stalls are isolated from guidance, and the UI never talks directly to kRPC.
Flight-log schema v3 keeps scalar snapshots separate from trajectory deltas and
keyframes.

## Dependencies

The native backend vendors:

- `ThirdParty/krpc-cnano-0.6.0` — kRPC C-Nano 0.6.0, LGPL-3.0-or-later.
- `ThirdParty/nanopb-0.4.9.1` — nanopb 0.4.9.1.
- SQLite 3 — linked from the platform (`-lsqlite3`).

The UI uses PyQt6 from `PyQtApp/requirements.txt`. Python is only an operator/UI
or optional test-tool dependency; it is not in the kRPC control path.

## Build and regression gate

```bash
make -C CLanding
make -C CLanding test
```

`make test` is the maintained offline regression surface. It runs four focused checks only: MM304→MM305 ownership/architecture, TAEM interface energy/capture semantics, vessel physics, and the backend NDJSON protocol. It does not connect to live KSP or start a ShuttleSim campaign.

Historical investigation probes remain in `Validation/` as reference material for targeted debugging, but they are no longer Makefile targets or qualification requirements. Native implementation structure and the flight-logic invariants that refactors must preserve are documented in `CLanding/README.md` and `CLanding/ARCHITECTURE.md`.

## Native kRPC connection

The live backend speaks the ordinary kRPC RPC protocol directly over TCP from
the native C process. Set the game server's RPC endpoint in the live profile:

```json
{
  "connection": {
    "rpcHost": "127.0.0.1",
    "rpcPort": 50000,
    "timeoutMs": 2000,
    "clientName": "Spaceplane Landing Guidance"
  }
}
```

`Configuration/live-mm305.json` is the live MM305 profile. The backend uses
kRPC's RPC port (normally 50000); the separate stream port is not used. The
serial and pseudo-terminal transport remains for ShuttleSim's local simulator
interface. No Python serial-to-TCP bridge is required for live KSP.

The headless launcher accepts an explicit profile, for example
`./run_headless.command --live --config Configuration/live-mm305.json ...`.
The PyQt UI uses its saved operator configuration in
`Runtime/PyQtUI/config.json`; set the RPC host and port there before connecting.

## Running the UI

```bash
./run_pyqt.command
```

The launcher builds the native backend and creates only the PyQt UI virtual
environment when needed.

For deliberate headless flight testing:

```bash
./run_headless.command --live
```

The headless tooling contains strong save/abort safeguards, but a live run is a
separate acceptance activity and is not part of the offline build/test gate.
The optional independent `Tools/ksp_test_guard.py` uses a separate Python kRPC
installation. `KSP_LANDER_GUARD_PYTHON` is the preferred explicit interpreter;
the headless runner also checks known project/legacy guard environments and the
current Python, accepting a candidate only when it can actually import `krpc`.

## Guidance stack

The vehicle returns through deorbit planning, atmospheric entry/MM304-style
energy management, TAEM, an optional HAC or another dynamically feasible
terminal capture, final approach, preflare/flare, touchdown, and rollout.
Guidance emits `GuidanceCommand` targets. For atmospheric profiles,
`flight_control.c` closes the actuator loop using AoA/pitch state, physical body
rates, adaptive control-authority estimates, rate/impulse guards, bank shaping,
sideslip/yaw coordination, and trim/integral logic. RCS is not used in
atmospheric flight (mission constraint in `flight_control.c`). SAS is kept
off for direct atmospheric control. Orbital pointing may use the kRPC
AutoPilot before the atmospheric direct-control regime.

Prior-flight aerodynamic evidence is stored in
`Runtime/Physics/observations.sqlite3`. The native store keeps model/environment
identities, same-flight exclusion, quality metadata, airbrake-state separation,
and bounded history selection. Current-flight observations improve future
flights rather than rewriting the certified predictor from every sample.

## Safety and recovery

Pausing, aborting, disconnecting, or faulting calls the native safe-control path:
autopilot and SAS are disengaged, direct axes/throttle/wheel steering are
neutralized, RCS assist is released, independent engine-throttle overrides are
cleared, and engines activated by the backend are restored to their prior state.
The STS-N independent-throttle workaround is preserved for engines that ignore
vessel main-throttle commands.

Historical logs and repair/audit documents under `FlightLogs/`, `Runtime/`,
`Docs/`, and `AgentWork/` may mention the retired Python bridge. Those references
are forensic history, not the current runtime architecture.
