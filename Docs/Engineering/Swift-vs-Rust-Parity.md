# Swift engine vs Rust backend: parity status (2026-10-30)

Offline, same ShuttleSim initial conditions (`Docs/Experiments/...mm305*.ini`). The two runs use different
plants: the Swift engine flies its own physics model, the Rust backend flies against ShuttleSim. Outcomes
are therefore indicative, not a like-for-like regression.

| Scenario | Swift (own model) | Rust + ShuttleSim |
| --- | --- | --- |
| l5-mm305 | complete, stopped, touchdown 75.7 m/s horizontal, sink 1.16 m/s, on runway, 2647 steps | complete, 66.1 m/s, sink 1.80 m/s, on runway |
| l7-mm305 | complete, stopped, 75.3 m/s, sink 1.05 m/s, on runway, 4295 steps | fails gates (known open gate) |

Live (KSP, 2026-10-30), Swift engine only:
- State decoding matches kRPC for speed, altitude, sideslip, bank (angle of attack is defined differently).
- Attitude through the kRPC AutoPilot tracked a target stably in vacuum (45 s).
- NDJSON backend protocol verified with `ksl`'s client (connect, reject createPlan, shutdown).

Not yet done: any live atmospheric flight, TAEM, final, flare, rollout, wheel steering; deorbit planning;
KSP checkpoints; a ShuttleSim-plant closed loop for the Swift engine.
