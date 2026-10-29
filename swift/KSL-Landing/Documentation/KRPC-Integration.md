# Native kRPC integration and operation

## Verified boundary

The client implements the request/response protocol described by the supplied kRPC 0.6.0 schema and C SDK declarations. The runtime is entirely Swift. Python is used only for independent test orchestration and a loopback wire fixture. No Python/C guidance process, project connector, or service participates in flight.

The supplied schema is preserved in [Evidence/krpc.proto](Evidence/krpc.proto). A small bounded Swift codec implements the used protobuf wire types and kRPC value encodings. This avoids a generated-client dependency while keeping transport, message framing, typed values, telemetry conversion, and dispatch separate. It is a supported subset client, not a general kRPC SDK: streams, events, arbitrary service generation, and every kRPC collection type are not implemented.

The fixture verifies that the Swift process communicates with an independently encoded TCP peer, including fragmented frames and error replies. This establishes local wire behavior against the supplied schema. No actual KSP instance was contacted during this delivery, so service-version compatibility, physical axis signs, airbrake mapping, and real timing remain unverified in the installed game.

## Build

Install Swift 6 with Swift Package Manager, then run from the project directory:

```sh
swift build -c release
swift test -c release
python3 Tools/verify_mock_krpc.py
python3 Tools/validate.py
```

There are no remote package dependencies. Foundation, Dispatch, and platform POSIX socket APIs are used. The delivered build/test evidence is from Swift 6.0.3 on Linux x86_64. macOS 13 is the declared minimum target, with Darwin socket branches implemented; no macOS compiler or device was available for verification. `Tools/run-validation.sh` runs the complete standard sequence. The chart-generation tool separately requires Matplotlib.

## Read-only discovery

```sh
.build/release/ksl-landing inspect \
  --host 127.0.0.1 --port 50000 --vessel STS
```

The host accepts numeric IPv4/IPv6 addresses or `localhost`. The client does not resolve arbitrary DNS names. Discovery checks the active vessel name, nonempty part list, Kerbin identity, equatorial radius, and gravitational parameter. It records the server version. `inspect` acquires one snapshot and closes the connection without arming or writing controls.

There is no automatic KSP launch, restart, save load, staging, ascent, OMS, or deorbit command. The landing runtime begins with the desired vessel already airborne after deorbit. The default vehicle is the archived 25 t STS; the much heavier STS-N final fixture is not a qualified interchangeable configuration.

## Configuration

```sh
.build/release/ksl-landing configuration --output configuration.json
```

Edit the JSON to set the surveyed runway, dimensions, envelope, and guidance parameters. Internal angles are radians and all other quantities follow SI units. The default Runway 09 threshold is latitude −0.0486111111°, longitude −74.7283333333°, elevation 68.44 m, heading 90°, length 2,500 m, width 70 m. A different runway on Kerbin can be configured. Changing planetary constants alone does not qualify a different celestial body or atmosphere.

The bundled atmosphere and aero data are SwiftPM resources. Changing vehicle identity/mass without supplying the correct polar, moments, inertia, and gear model does not establish suitability. See [Physics-and-Frames.md](Physics-and-Frames.md).

## Flight command

The following command takes control of the named active vessel. It is an execution instruction, not a record of a live test performed here:

```sh
.build/release/ksl-landing fly \
  --host 127.0.0.1 --port 50000 --vessel STS \
  --configuration configuration.json \
  --airbrake-group 1 --log live-flight.jsonl
```

Arming disengages the kRPC autopilot and commands zero throttle, SAS off, RCS off, and the configured airbrake action group off. Each subsequent control batch maintains these restrictions. The airbrake group is an explicit craft-specific mapping; the client cannot infer which parts another craft binds to an action group. Group numbers are checked against kRPC's 0–9 range.

The runtime writes a sidecar `live-flight.jsonl.configuration.json` for reproduction. `SIGINT` cancels the awaited flight task and runs bounded cleanup. Completion or abort also closes the recorder and session. On a usable connection, cleanup neutralizes pitch, roll, yaw, and wheel steering and preserves forbidden-system restrictions. It does not automatically restore the old autopilot or re-enable SAS/RCS. Wheel brake/gear state is not blindly reset during ground shutdown.

Exit status is zero for completed flight, 2 for a noncomplete runtime outcome, and 1 for thrown configuration, transport, telemetry, or file errors. A simulation returns success status even for a modeled abort unless `--require-success` is specified, so scripts can collect complete failure reports.

## Snapshot and command ordering

A snapshot is one ordered batch bracketed by two UT reads. Between them, the client reads geographic position, velocity in the surface reference frame, radar altitude, q, true airspeed, sound speed, pressure, temperature, lift/drag vectors, rotation, mass, situation, gear, throttle/thrust, SAS/RCS, the configured airbrake group, part list, active vessel, and wheel brakes.

The batch is rejected if its UT span exceeds 0.12 s, its request wall time exceeds 0.5 s, the vessel changes, or the part count changes. Invalid/nonfinite values, nonzero propulsion, SAS/RCS/airbrakes, and inconsistent configuration are also rejected. The protocol provides sequential RPC calls, not a physically atomic multi-property snapshot; the bracket is a bounded coherence check.

The common flight computer executes once per fresh UT. A command carries a monotonically increasing sequence and the exact source snapshot time. The adapter rejects a mismatched UT, an old sequence, or excessive wall age immediately before dispatch. Duplicate telemetry times do not repeat commands and eventually trigger a freshness error. Time moving backward, or an unsupported large state interval, is rejected. Guidance, actuation, and recording are ordered and do not race across independent tasks.

## Wire details

| Layer | Implemented behavior |
| --- | --- |
| TCP framing | Varint byte length followed by a protobuf message; maximum frame 4 MiB |
| Handshake | RPC connection type uses protobuf's zero default; name is field 2; returned client ID is field 3 |
| Request | Repeated calls, each carrying service, procedure, and indexed byte arguments |
| Response | Top-level and per-result errors are checked; result count must match request count |
| Scalars | IEEE little-endian 4-byte FLOAT and 8-byte DOUBLE; unsigned/bool varint; zigzag signed enum |
| Strings | Length-prefixed UTF-8 kRPC value representation |
| Vectors/quaternions | Tuple message with repeated element bytes; each component decoded at its declared scalar width |
| Socket behavior | Nonblocking descriptor, partial read/write loops, at most 50 ms poll slices, bounded deadline and cancellation checks |

The adapter converts both kRPC coordinate bases before constructing the canonical quaternion. It derives angular velocity from successive canonical attitudes. This choice avoids silently treating a left-handed axial vector as a right-handed ordinary vector. Lift/drag remain observed force vectors and are marked as measured, while simulator forces retain model provenance.

## Failures and reconnection

Transport or protocol failure closes the session. A command batch whose acknowledgement is lost is not automatically retried: the server may have executed some or all of it. Replaying it blindly would make ordering ambiguous. Cleanup can only send neutral commands when the connection is still usable; it cannot act through a physically lost socket.

Explicit reconnect repeats discovery in read-only state. A caller must reacquire coherent state and construct a fresh mission before arming again. The CLI does not silently reconnect and resume obsolete controller integrators or a stale HAC. Connection generations invalidate operations whose continuation returns after shutdown, so a late response cannot revive an armed session.

The independent fixture covers normal discovery, fragmented replies, wrong vessel, active-vessel change, part-count change, excessive UT span, delayed replies, malformed/truncated frames, timeout cleanup, failed command acknowledgement without replay, and cancellation cleanup. See [../Validation/krpc-mock.json](../Validation/krpc-mock.json). These checks do not replace a live flight.
