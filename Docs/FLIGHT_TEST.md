# Flight-Test and Tuning Procedure

Do not begin with a valuable crewed save. Use a duplicate save, quicksaves, and a test vehicle.

## Stage 1: Ground and connection test

- Start kRPC and connect while the craft is parked.
- Confirm that simply connecting does not change throttle, gear, brakes, or action groups.
- Check heading, pitch, roll, radar altitude, airspeed, and runway coordinates.
- Roll the parked or slowly moving craft a few degrees each way. For a right bank, the fixed bank pointer and magenta roll cue must move right while the artificial horizon rotates left.
- Test the direct gear and brake buttons.
- Press Abort and verify zero throttle and released attitude hold.

## Stage 2: Atmospheric handling calibration

Use a conventional atmospheric test flight before any deorbit attempt. Start high enough to recover manually after the complete AoA sweep. Five kilometres of radar altitude is a practical minimum for an unfamiliar craft; more is safer.

- Establish clean, stable flight above the configured minimum safe speed.
- Set the calibration altitude floor, maximum dynamic pressure, maximum G-load, maximum sink rate, and stall abort fraction conservatively.
- Press **Start Simple Glide**. The controller captures the current heading, commands zero throttle and wings level, and steps through the configured AoA values.
- During each AoA step, verify that the status changes from `Settling` to `Sampling`. Samples collected while the craft is oscillating, slipping, outside the pressure envelope, or otherwise invalid are rejected.
- During the final quarter of the middle-AoA sample, expect one short `Flight-test pulse` with the configured speedbrake action group deployed. This is permitted only with generous altitude and q margin. Confirm the action group affects only the intended speedbrake surfaces before running the sweep.
- Watch accepted/rejected sample counts, confidence, best-glide AoA, estimated stall speed, and recommended landing speeds.
- The mode must release AutoPilot automatically at its altitude, pressure, G-load, sink-rate, airspeed, or stall boundary. Press **Abort** at any time to release it manually.
- Treat the resulting aerodynamic profile as a shadow flight-test reconstruction. It does not rewrite the current session's nominal force data. Press **Stop & Apply** only when deliberately promoting a reviewed profile, and create a new deorbit plan afterward. Use **Stop** for ordinary flight-test collection.
- Repeat at different masses and, when practical, at transonic and supersonic conditions. The planner will not substitute low-speed samples for missing high-speed entry data.
- Disconnect/reconnect after a completed test flight before judging whether its aerodynamic observations improved predictor coverage; current-session samples become data-book evidence only on a later session.

Passive/shadow calibration can remain enabled during later flights. It may refine atmospheric density, speed-of-sound/control-response diagnostics and recommended safety speeds, but it does not recenter the certified aerodynamic force model in the same flight.

## Stage 3: Planner-only orbital test

- Use a 75–90 km near-circular Kerbin orbit.
- Connect and create a plan, but do not engage.
- Confirm burn time is in the future and predicted atmospheric entry exists.
- For the default profile, expect a late/shallow target near 300 km range, about -1.2 degrees at atmospheric interface, and a post-burn periapsis near 50 km. Reject a plan outside the configured -3.5 degree steepness boundary.
- Inspect the robust-case count, pass fraction, unsafe-case count, worst miss distance, worst entry angle, and worst post-burn periapsis. With the default settings, automation must remain blocked unless at least 75% of the deterministic stress cases pass and none crosses a hard unsafe boundary.
- Temporarily widen one uncertainty at a time, such as ignition timing, thrust, mass, navigation state, pointing, or density. Aerodynamic uncertainty is derived automatically from prior-flight data-book scatter and current-flight residual monitoring; a plan that becomes fragile should lose robust qualification rather than remaining engageable because its nominal trajectory still looks good. Restore the intended uncertainty settings before flight.
- Inspect the map for discontinuities or a path crossing the wrong hemisphere.
- Reject plans with low confidence or a capture miss.
- Change one vehicle parameter at a time and observe whether the predicted path changes in the expected direction.

## Stage 4: Burn-only test

- Quicksave before engagement.
- Engage and monitor burn setup.
- Verify the craft reaches retrograde before throttle opens.
- Confirm the default command remains below 18% throttle and rises gradually over roughly twelve seconds rather than snapping open.
- Confirm measured delta-v increases only while engines produce thrust.
- Watch live periapsis as the target approaches. Throttle should begin falling inside the final several kilometres and reach zero at the planned periapsis even if the integrated delta-v estimate differs slightly.
- During the burn, watch the cutoff-now result. Once the measured current state itself predicts a complete entry/TAEM capture, throttle may close before the nominal delta-v target; verify that it closes rather than continuing to chase the original plan blindly.
- After cutoff, confirm the plan changes from achieved-state pending to verified and records the measured post-burn periapsis. If the achieved state is outside the capture corridor, the UI must warn instead of continuing to label the original plan as successfully achieved.
- In a disposable rehearsal, remove engine availability or otherwise prevent thrust after the burn has clearly started. With substantial delta-v still remaining, the progress watchdog should abort after sustained no-progress, release AutoPilot, and command safe throttle rather than hanging in Deorbit Burn.
- Pause during a rehearsal orbit and verify AutoPilot releases and throttle goes to zero.

Reload after the rehearsal instead of continuing to entry.

## Stage 5: Entry and S-turn test

- Engage from a valid capture plan.
- Monitor dynamic pressure, G-load, stall fraction, bank command, roll rate, energy margin, predicted TAEM distance, and TAEM range error.
- Confirm the HUD and KSP navball both report **SURFACE** after atmospheric guidance begins. If KSP remains in Orbit mode, abort before judging any ground-track guidance.
- Verify bank reversals are separated, occur after crossing the tapered corridor, and do not chatter when TAEM prediction updates. Roll targets should ramp through a reversal rather than jump directly from one bank limit to the other.
- Compare the planned and actual tracks over several prediction updates. Density/bank-response and altitude/speed/range residuals may adapt gradually, but nominal lift/drag should remain frozen. Watch `physicsModelResidual` and its confidence: a sustained large value should widen robustness stress and raise a model-envelope warning instead of silently shifting the prediction.
- Abort for loss of control, persistent stall, thermal danger, or structural limit exceedance.
- Confirm RCS assistance decreases as measured aerodynamic authority takes ownership and stays off after the final transonic cutoff.
- Record whether the vehicle reaches TAEM long or short. Prefer another controlled flight-test/data-book iteration over hand-tuning L/D or ballistic coefficient from one trajectory.

## Stage 6: TAEM/HAC test

- Verify the selected HAC is on the practical side of the runway.
- Verify normal TAEM ownership occurs only inside the configured fast handoff speed band. A slower vehicle may still enter TAEM only when the retained future join has sufficient total energy, maneuver room and a predicted join inside the TAEM altitude shell; a late low-energy arrival should abort instead of being renamed TAEM.
- Confirm speedbrake deployment follows excess q/energy rather than raw airspeed alone, and that the predictor shows the same device-state transition.
- Confirm the map reference circle is recaptured from both inside and outside rather than merely followed on a parallel tangent.
- Check that altitude error converges while arc remaining decreases.
- If consistently high, enlarge HAC radius/TAEM range, increase airbrake authority, or steepen TAEM slope.
- If consistently low, reduce bank/drag, permit more powered assistance, or use a shallower TAEM slope.

## Stage 7: Final and landing test

- Require stable runway alignment before the final-approach point, then verify moderate cross-track disturbances do not switch the controller back to HAC.
- Confirm gear is down and locked well before flare.
- Confirm airspeed stays above configured minimum safe speed.
- Check that the height-scheduled flare reduces sink rate progressively without a pitch step or stall.
- Verify touchdown occurs inside the runway width and rollout steering has the correct sign.
- Increase braking only after directional control is proven.

## Abort criteria

Abort or take manual control when any of these persists:

- AutoPilot cannot hold the commanded attitude.
- The predicted miss grows rapidly after entry.
- Dynamic pressure or G-load remains above the configured limit.
- Stall fraction remains elevated or airspeed falls below the safe value.
- TAEM altitude error exceeds the vehicle's recoverable energy margin.
- Final cross-track exceeds 1 km inside 5 km to the aim point.
- Gear fails to deploy.
- Sink rate exceeds 5 m/s near flare.
- Any kRPC error places the controller in `Fault`.

## Data to retain after each run

Record `vehicle.modelId`, craft mass, fuel state, center-of-mass changes, installed aerodynamic mods, structural witness/environment identity, certified-cell count, certified aerodynamic uncertainty, live model residual/confidence, bank effectiveness, entry AoA, maximum bank, predicted-versus-actual entry range and angle, TAEM geometry, speedbrake-state coverage, final speeds, touchdown location, and observed long/short miss. Reusing one model ID after a major design change is an explicit operator decision; use a new ID when the new configuration should not inherit the old flight data book.
