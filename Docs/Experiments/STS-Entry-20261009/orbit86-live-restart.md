# Live restart from the protected 86 km orbit

On 2026-10-09 the human stopped the 72 km attempt because of overshoot and explicitly requested a restart from `86km`. The interrupted run exited 130, released controls, and restored its initial 72 km checkpoint. It did not acquire MM305, touch down, or complete rollout.

The new normal orbital attempt restored original STS at UT 78.9 and altitude 86.7 km. It settled to mass 26.148 t and planned from current telemetry without the 72 km checkpoint physics profile. A private copy of the STS configuration disables reciprocal runway selection to target Runway 09. Atmospheric flight remains unpowered; the orbital deorbit burn is permitted.

The planner selected burn UT 13684.8 and delta-v 31.39 m/s. It reported entry-stage qualification 15/15, but downstream capture remained false, with recovery 1/15. These are predictions, not a demonstrated landing. Explicit warning continuation preserves rejected qualification fields and strict admission still rejects them.

Run manifest: `Runtime/Headless/run-manifests/20261009T063735081336Z-79338.json`.
Vehicle log: `FlightLogs/2026-10-09T06-37-36Z-STS-vehicle.jsonl`.
Console log: `/tmp/sts-live-orbit86-20261009.log`.

A one-second process sample at 15:40:23 KST confirms the control worker was inside `krpc_cnano_client_warp`, not stalled in planning. Independent read-only live telemetry subsequently reported UT 6952.85, rails factor 3, rate 50x, altitude 86750.2 m, and throttle zero. The coast is genuinely progressing; do not restart solely because normal snapshots are quiet during the blocking warp.

Protected `86km.sfs` SHA-256 after restart: `4ea194a07422eece54135f2f7586dc92b556b2c9380b785c4754ef868eb662a6`, unchanged. Never save over it.

Status at this checkpoint: live coast active; burn, MM305 acquisition, touchdown, and rollout remain unverified.
