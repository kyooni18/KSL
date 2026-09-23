# ShuttleSim process protocol

Protocol version: `schema = 1`.

All network messages are one JSON object per UDP datagram. The simulator also writes the identical telemetry objects as JSONL to stdout and `--record` files.

## Ports

Defaults:

- `127.0.0.1:8795`: command input to ShuttleSim
- `127.0.0.1:8796`: telemetry output intended for Guidance
- `127.0.0.1:8797`: telemetry output intended for Telemetry Web

Each port can be disabled by setting it to `0`.

## Guidance -> ShuttleSim

Attitude command:

```json
{"type":"attitude_command","aoa_deg":35.0,"bank_deg":-68.0}
```

Gear command can be sent with the same object:

```json
{"type":"attitude_command","aoa_deg":12.0,"bank_deg":0.0,"gear_down":true}
```

Pause/resume are simulator-control messages, not flight Guidance:

```json
{"type":"pause"}
{"type":"resume"}
```

Guidance has no protocol operation for setting position, velocity, forces, aero coefficients, simulation time, or other internal state.

## ShuttleSim -> consumers

Representative telemetry:

```json
{
  "schema": 1,
  "type": "telemetry",
  "source": "sim",
  "scenario": "orbit86km-seed",
  "ut": 120.0,
  "sim_time": 120.0,
  "sim_rate": 4200.0,
  "position": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "lat_deg": 0.0,
    "lon_deg": 0.0,
    "altitude_m": 60000.0
  },
  "velocity": {
    "inertial_mps": 2100.0,
    "surface_mps": 1950.0,
    "air_mps": 1950.0,
    "vertical_mps": -80.0
  },
  "attitude": {
    "aoa_deg": 35.0,
    "bank_deg": -60.0,
    "heading_deg": 90.0,
    "cmd_aoa_deg": 35.0,
    "cmd_bank_deg": -60.36,
    "requested_aoa_deg": 35.0,
    "requested_bank_deg": -68.0,
    "q_w": 1.0,
    "q_x": 0.0,
    "q_y": 0.0,
    "q_z": 0.0
  },
  "aero": {
    "mach": 6.0,
    "q_pa": 10000.0,
    "lift_n": 1000000.0,
    "drag_n": 900000.0
  },
  "runway": {
    "along_m": -300000.0,
    "cross_m": 1000.0,
    "vertical_m": 59930.0
  },
  "ground": {
    "gear_down": false,
    "on_ground": false,
    "touchdown_seen": false,
    "on_runway_touchdown": false
  }
}
```

Guidance should depend on the semantic fields it needs rather than simulator internals. Telemetry Web can consume the same objects independently.

`requested_*` is the raw UDP/replay demand. `cmd_*` is the rate- and
acceleration-bounded target currently delivered to the attitude model. The
production guidance program applies the same response-aware bound before
sending commands; ShuttleSim keeps this second bound as a direct-input safety
guard.
