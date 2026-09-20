# Guidance Upgrade Notes

This revision addresses deorbit depth, HUD/navball frame correctness, S-turn oscillation, aircraft-style command conditioning, live trajectory identification, physical atmospheric propagation, and adjustable UI layout.

## Deorbit and atmospheric interface

- Candidate search now scores interface range, interface flight-path angle, post-burn periapsis, TAEM range, runway miss, peak q, peak G, and capture validity together.
- Entry farther from the runway than the configured target is penalized more strongly than a modest late entry.
- A periapsis below the configured target receives an extra depth penalty.
- The conservative default burn uses 18% maximum throttle and a twelve-second smoothstep ramp.
- Final delta-v uses measured-thrust integration plus a live periapsis-closure taper; a valid planned periapsis must be captured before the burn can terminate, with a bounded reserve for residual error.
- When a valid periapsis target exists, planned delta-v is not an unconditional cutoff: a bounded 8 m/s closure reserve remains available until live periapsis is within 250 m of target.
- Coast guidance keeps the inertial retrograde AutoPilot engaged well before the burn window, giving the shuttle time to settle its attitude before throttle is permitted.
- Inertial AutoPilot leaves roll unconstrained; explicit roll targets resume in surface mode, avoiding the -180°/+180° roll singularity during orbital pointing.
- Surface roll values leave C in signed -180°…+180° form and are defensively normalized again at the Python/kRPC boundary, so a commanded -61° bank cannot leak downstream as a 299° long-way turn or flip predictor bank sign.
- Signed kRPC AutoPilot error is converted to an alignment magnitude before the burn gate, so a large error in either direction holds throttle.
- The burn gate is owned by C; the bridge no longer re-samples a transient AutoPilot error after re-issuing the inertial target and deadlocks an aligned burn at zero throttle.
- Exact zero throttle is written through to kRPC, while small positive taper commands remain available for the final periapsis closure.
- Pre-burn guidance cannot fall through into S-turn/HAC control while a future orbital burn is still pending.
- Burn execution continues after descent begins instead of falling through to Entry Interface prematurely. The default entry target is 300 km at -1.2 degrees with a 50 km post-burn periapsis and a -3.5 degree steepness boundary.
- After burn cutoff, entry capture uses an inertial prograde/up direction before switching to surface heading. This avoids the 180-degree Euler heading/roll ambiguity during the retrograde-to-entry turn.
- Surface-relative vertical, horizontal, total, and airspeed fallbacks are derived from the inertial position/velocity streams, so zero kRPC Flight-speed values cannot freeze the flight-path angle or mark an airborne vehicle stationary.

## Entry and S-turn

- Live range-energy error is fused with periodically regenerated TAEM prediction error.
- Periodic forecasts inherit the live S-turn leg age, requested bank side, and measured current bank. A leg clock begins only after the aircraft captures the requested side, so neither live guidance nor prediction consumes leg time during a slow reversal.
- Predictor authority grows with aerodynamic and trajectory-model confidence.
- S-turn corridors taper with range and use minimum leg time, maximum leg duration, and reversal hysteresis.
- Under-energy protection suppresses unnecessary reversals.
- C does not apply outer Euler-rate damping around kRPC AutoPilot. kRPC owns the inner angular-rate loop; C owns only attitude-trajectory shaping and guidance.
- Roll target changes are acceleration limited with shortest-angle wrapping once in C; Python does not apply a second attitude-target slew loop, so S-turn reversals are not delayed by competing command filters.
- Dynamic pressure, G-load, speed, and stall fraction continuously contract the bank envelope.
- Airbrake deploy and retract thresholds are separated.

## Python flight-control shaping

The Python bridge configures kRPC AutoPilot as a tight inner tracker rather than another trajectory shaper. Atmospheric profiles use roughly 1–2.5° attenuation regions and 1.8–2.8 second time-to-peak settings instead of the previous 10–20° attenuation and 6–10 second response. Roll threshold is 180°, keeping bank tracking active while pitch/heading converge.

C owns pitch, heading, roll and throttle command shaping. The bridge writes those targets directly to kRPC without a second throttle slew or deadband. Reference-frame changes are explicit release/reseed/re-engage transactions so an inertial target cannot be transiently reinterpreted in the surface frame.

Telemetry now carries vessel-frame angular velocity, actuator pitch/roll/yaw, available torque, moment of inertia and auto-tuned AutoPilot PID gains. The controller log also records axis command errors and telemetry/guidance/apply/whole-loop wall timing. Trajectory bank-effectiveness learning uses surface-course rate, while body angular velocity is used to identify rapid-attitude samples; differentiated Euler rates no longer close a control loop.

## Surface-relative instruments

- Atmospheric phases command kRPC Surface speed mode.
- Coast and deorbit phases command Orbit speed mode.
- Python reads the selected mode back into telemetry.
- The HUD artificial horizon rotates opposite aircraft roll.
- The bank command cue follows positive-right kRPC roll convention.
- Flight-path marker lateral displacement uses surface ground track minus nose heading.

## Live trajectory identification

The trajectory calibrator combines two evidence paths:

1. **Instantaneous force identification**
   - differentiates inertial velocity;
   - subtracts spherical gravity;
   - decomposes atmosphere-relative acceleration into drag and lift components;
   - gates powered, slipping, airbrake, and rapid-attitude samples;
   - updates density, drag, lift, and bank-effectiveness scales robustly.

2. **Forecast residual identification**
   - interpolates the current predicted trajectory at live UT;
   - compares predicted and actual altitude, airspeed, and remaining runway range;
   - accumulates robust residuals;
   - applies slow integrated-bias corrections rather than chasing noise.

## Atmospheric trajectory predictor

The numerical predictor integrates:

- spherical gravity;
- rotating atmosphere velocity;
- atmosphere-relative speed and Mach;
- Kerbin density versus altitude;
- Mach-regime lift-to-drag and ballistic coefficient;
- learned density, drag, and lift scales;
- scheduled AoA;
- banked lift-vector rotation;
- learned bank effectiveness;
- current vehicle mass and state vectors.

It reports the full path, entry range/angle/speed, closest runway distance, TAEM distance, peak q, and peak G. The forecast is regenerated in flight and shown on the map and range-altitude display.

After TAEM crossing, the predictor now leaves entry S-turn logic and follows the same HAC-side selection, L1 circle capture, final centerline capture, roll-rate limits, and final overspeed gate as flight guidance. Deorbit planning also records entry course error, TAEM speed and HAC capture quality, rejects unsafe load/periapsis solutions, and uses a multi-basin coarse/medium/fine search rather than refining a single coarse local minimum.

## Adjustable interface

The control and telemetry widths, HUD height/scale, lower-instrument height, map range, and panel visibility are adjustable. Values persist across launches and can be reset from the toolbar or UI Layout group.
