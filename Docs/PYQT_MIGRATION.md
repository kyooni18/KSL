# PyQt + C Migration

The desktop application surface is PyQt6 and the former Swift runtime has been replaced by a native C core. Python remains only where it is useful: the Qt frontend and the isolated kRPC worker.

## Boundary

`CLanding/build/landing_backend` owns one C `LandingController`, accepts operator commands as newline-delimited JSON and streams `LandingSnapshot` values back to the Qt process. The PyQt process owns presentation state, editable configuration, layout persistence and user input.

The C backend owns the flight-critical state: deorbit search, propagation, S-turn/HAC/TAEM/final/flare guidance, adaptive calibration, trajectory calibration, command shaping, safety states, flight logging and kRPC-worker supervision. There is no second implementation of these algorithms in the Qt layer.

## PyQt frontend

`PyQtApp/window.py` owns the former desktop-view responsibilities:

- connection, site, vehicle, calibration and guidance configuration;
- Plan, Engage, Pause/Resume and Abort commands;
- gear and brake direct controls;
- calibration start/stop/apply/reset;
- draggable control, center, telemetry, HUD and lower-instrument splitters;
- persistent layout and normalized configuration;
- guidance, navigation, flight, adaptive-model, deorbit-plan and low-level controller telemetry.

`PyQtApp/widgets.py` implements the instruments with `QPainter`:

- attitude horizon and pitch ladder;
- bank scale and measured-bank pointer;
- flight-path marker;
- aircraft symbol and commanded-attitude cue;
- speed, radar-altitude, Mach, q and navball-mode readouts;
- local runway/trajectory map;
- range/altitude trajectory profile.

`PyQtApp/ingame_hud.py` mirrors selected snapshots into KSP's own scene with kRPC Drawing: a sparse target-attitude director, flight-path reticle, planned/reference trajectory, recent actual trail, runway/extended centerline and a one-line guidance-mode annunciator. It intentionally avoids a text telemetry panel.

## Processes

```text
PyQtApp
  ↕ UI protocol v1
C landing_backend / LandingController
  ↕ kRPC bridge protocol v3
PythonBridge/krpc_bridge.py
  ↕ kRPC
KSP
```

The flight-critical kRPC transport remains isolated in the C backend. The UI environment contains PyQt6 plus a lightweight Python kRPC client used only by the in-game AR HUD renderer; that client consumes snapshots and owns no guidance or controls.

Headless live tests use the same renderer through `Tools/ingame_hud_stream.py`. `Tools/headless_flight.py` forwards the exact backend snapshots to that display-only child process, so target attitude, guidance mode, reference trajectory, flown trail and runway cues remain visible during automated reentry/TAEM/landing tests without putting kRPC drawing work on the flight-control process.

## Run

```bash
bash run_pyqt.command
```

For development with an already-created PyQt environment:

```bash
make -C CLanding
python -m PyQtApp.main --backend CLanding/build/landing_backend
```

The repository no longer requires Swift Package Manager or Xcode. C core regression tests are available through `make -C CLanding test`.
