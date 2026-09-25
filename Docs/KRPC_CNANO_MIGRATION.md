# Native kRPC C-Nano migration

## Current runtime

The landing backend now uses kRPC C-Nano 0.6.0 directly from C. The retired
Python bridge, its `krpc==0.5.4` environment, installer, and protocol-v3 runtime
have been removed. The UI-to-C NDJSON protocol is independent of kRPC and is
still kept as the operator/backend boundary.

The live production transport is ordinary kRPC RPC over TCP, implemented by
the native C backend through `CLanding/krpc_cnano_transport.c`. The live profile
sets `connection.rpcHost` and `connection.rpcPort` (normally 127.0.0.1:50000).
The same C-Nano encoder and decoder are used in-process; no Python serial-to-TCP
adapter is needed. ShuttleSim retains its serial/pseudo-terminal transport for
the local simulator interface. C-Nano 0.6.0 has no Streams or Events, so the
native client polls telemetry synchronously.

## Dependency provenance

The kRPC C-Nano 0.6.0 source supplied for the migration had archive SHA-256
`4edd712b13680e8797ec5e61d01ce0e719631304e3a0100c78c9316e0aa57fb3` and is
vendored under `ThirdParty/krpc-cnano-0.6.0`. nanopb 0.4.9.1 is vendored from
upstream commit `cad3c18ef15a663e30e3e43e3a752b66378adec1`. See
`ThirdParty/PROVENANCE.md` for the local patch and license notes.

Upstream C-Nano configures nanopb for one procedure call/result per message to
minimize embedded memory. The host landing backend locally raises those fixed
arrays to 32 without changing the protobuf wire schema. This lets the client
send standard multi-call kRPC Requests while keeping all generated single-call
APIs intact.

## Polling and RPC budget

At the default 10 Hz guidance rate, the native client groups telemetry into
three tiers:

| Tier | Logical RPCs | Wire requests | Schedule |
| --- | ---: | ---: | --- |
| Fast flight/control state | 18 | 1 | every control tick |
| Plant/control/force/CoM state | 20 | 1 | every 0.25 s |
| Orbit state | 3 | 1 | every 1.0 s |
| Direct atmospheric axes | 3 steady-state | 1 | every applied control tick |

A steady 10 Hz atmospheric run therefore targets about 25 request/response
transactions per second: 10 fast reads, 4 medium reads, 1 slow read,
and 10 batched direct-control writes. Gear, brakes, airbrakes, speed mode,
throttle, wheel steering, RCS, SAS and AutoPilot state are cached or
change-driven, so unchanged state does not add steady-state transactions.

The stateful fake-server cadence test measures 49 physical request/response
transactions and 16,696 encoded bytes across two simulated seconds. At 921600
baud with 8N1 framing plus a deliberately pessimistic 2 ms turnaround charge
per transaction, that occupies about 14.0% of the available two-second window.
The test fails if request count exceeds 52 or this pessimistic occupancy reaches
50%, leaving substantial margin for real server execution jitter. Real KSP
latency still has to be measured before live acceptance.

The first telemetry update intentionally performs all three read tiers: 41
logical calls in three wire requests. Subsequent fast-only samples are 18
logical calls in one wire request. `Validation/CNanoClientTests.c` asserts both
of those budgets and also asserts that steady atmospheric pitch/roll/yaw writes
use one wire request. Runtime telemetry publishes logical-call and physical
wire-request counters alongside existing telemetry/apply latency measurements.

The medium tier also samples center of mass in the vessel reference frame, so
fuel-state CoM motion is persisted independently of the inertial CoM used by
trajectory reconstruction.


On macOS, the default 921600 baud rate is applied with `IOSSIOSPEED` when it is
not represented by a standard termios constant. The transport never silently
falls back to 115200; unsupported non-macOS rates fail connection setup.

The 71-point atmosphere table is loaded once per connection. Its 142
PressureAt/DensityAt calls are grouped into at most 32 calls per request, so the
full table uses five wire request/response transactions instead of 142.

## Native control and persistence

`CLanding/flight_control.c` now owns the atmospheric actuator loop that used to
live in Python. It preserves the physical body-rate sign boundary, pitch/AoA
feedback and damping, adaptive authority estimates, torque/inertia limits,
roll-rate shaping, trim/integral logic, sideslip/yaw coordination, RCS blending,
recovery behavior and low-speed terminal control.

`CLanding/physics_store.c` now owns the SQLite flight-data archive. The schema
is compatible with the accumulated archive, keeps model/environment identity,
quality metadata, same-flight exclusion and bounded history selection, and has
a compatibility path for legacy manual-model environments whose atmosphere
curve digest was produced by the retired Python serializer. The checked-in real
archive is only read during offline compatibility validation.

## Offline validation completed

The following gates run without opening KSP or a real serial link:

```bash
make -C CLanding test
make -C CLanding qualification
python3 Validation/BackendProtocolTests.py
python3 Validation/CNanoArchitectureTests.py
KSP_REAL_PHYSICS_DB="$PWD/Runtime/Physics/observations.sqlite3" \
  CLanding/build/physics_store_tests
```

The C test suite also passes with AddressSanitizer and UndefinedBehaviorSanitizer
when the whole backend and test objects are rebuilt with sanitizer flags.

`Validation/CNanoTransportTests.c` covers partial reads/writes, EOF, timeout,
malformed/truncated protobuf and reconnect behavior. `CNanoBatchTests.c` proves
multiple procedure calls are encoded and returned through one physical frame.
`CNanoClientTests.c` runs the actual production C-Nano client against a stateful
fake serial-protocol server, including connection setup, atmosphere-table
loading, fast/medium/slow telemetry decoding, axis mapping, force/torque/inertia
state and batched direct controls.
It also verifies that normal client shutdown sends the neutralizing safe-control
sequence as one bounded batch rather than waiting through a chain of individual
serial RPC timeouts.

## Remaining acceptance stage

No live KSP flight has been qualified as part of this migration. The remaining
runtime acceptance work is to connect the native client to the configured kRPC
RPC endpoint, measure request/response latency against the 10 Hz budget, and
then resume controlled flight testing. Historical logs may still mention
serial and Python adapters as forensic evidence; they are not part of the live
runtime.
