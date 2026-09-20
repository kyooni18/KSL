# Guidance and Prediction Algorithm

## 1. Coordinate model

kRPC exposes KSP vectors in a left-handed coordinate system. `KRPCSession` converts each `(x, y, z)` tuple to the core's right-handed `(x, z, y)` representation before any cross products or orbital integration. Output direction vectors are converted back before being sent to kRPC.

The predictor constructs a body-fixed basis from the celestial body's non-rotating reference frame, its north-axis surface vector, its zero-longitude surface vector, rotational speed, and the current universal time. This allows inertial positions to be converted to and from latitude/longitude while Kerbin rotates beneath the trajectory.

## 2. Deorbit opportunity search

Planning requires both apoapsis and periapsis above 70 km.

The planner searches progressively through upcoming orbital opportunities, extending as far as three orbital periods when the first pass does not yield enough complete capture candidates. This matters on a rotating body because the next ground track is not the same as the following orbit's ground track.

1. Propagate the present state to each candidate burn time with RK4 two-body dynamics.
2. Apply a retrograde candidate delta-v.
3. Propagate through vacuum and atmosphere.
4. Tier candidates by recoverability before comparing numeric score: a complete capture-corridor solution outranks a trajectory that merely reaches TAEM, and any TAEM-reaching trajectory outranks an atmospheric fly-by/skip. Within each tier, score atmospheric-interface range and flight-path angle, post-burn periapsis, TAEM range/speed/HAC geometry, closest-site distance, peak dynamic pressure, peak G-load, capture-radius violation, and burn cost. Early/deep entry receives an asymmetric additional penalty.
5. Keep several diverse high-ranking coarse basins, refine all of them with medium time/delta-v increments, then run one narrow precision pass around the best refined basin.
6. Stress-test the precision solution and a small neighboring time/delta-v set with a deterministic execution envelope. The scenarios perturb ignition timing, available thrust, delivered delta-v, mass, initial radial position, initial along-track velocity, burn pointing, atmospheric density, and the certified aerodynamic data-book envelope. Aerodynamic stress includes drag-high/lift-low and drag-low/lift-high cases whose magnitude comes from prior-flight scatter and may be widened by a confident current-flight model residual. Each scenario propagates the full finite burn and entry rather than applying an instantaneous impulse.
7. Select the locally robust candidate before re-running it with trajectory recording enabled.

The stress set is an explicit deterministic uncertainty envelope, not a claim of formal covariance propagation or Monte Carlo certification. The default execution envelope is ±3 s ignition time, ±8% thrust, ±1.5 m/s delivered delta-v, ±3% mass, ±50 m radial position, ±0.5 m/s along-track velocity, ±2° pointing, and ±12% atmospheric-density scale. Aerodynamic uncertainty is not a fixed vehicle constant: it is derived from independent prior-session data-book scatter, bounded conservatively when coverage is sparse, and enlarged rather than re-centered when the current flight disagrees with the certified model.

The preferred engagement gate remains the strict capture corridor. By default at least 75% of stress scenarios must satisfy it and no stress scenario may cross a hard unsafe boundary such as incomplete finite-burn delivery, no atmospheric capture, post-burn periapsis below -8 km, entry steeper than the configured boundary by more than 0.5°, or predicted dynamic-pressure/G-load above 110% of the configured limit. If the strict geometric corridor is missed but the nominal trajectory and the same required fraction of stress cases remain inside a broader recoverable envelope, the plan is explicitly marked as a guarded-recovery execution rather than being silently presented as a completed-but-unexecutable plan. Hard-unsafe cases still block engagement. The plan exposes strict and recovery pass counts, unsafe-case count, worst miss distance, worst TAEM-range error, steepest entry, minimum periapsis, and maximum predicted q/G.

## 3. Flight-derived aerodynamic data book and calibration

The nominal aerodynamic model is frozen for one flight session. At connect time the bridge identifies the assembled vessel structure and atmospheric environment, then loads only matching observations from earlier sessions/timeline epochs. Raw force measurements from the current flight are persisted immediately for later reconstruction but are explicitly excluded from the current session's data book. This prevents a transient, upset, startup glitch, or one bad descent from redefining the vehicle while the same flight is relying on that model.

Prior observations are compressed into local q/Mach/AoA/sideslip cells with independent configuration dimensions for gear, wheel brakes and speedbrakes. Each cell carries the number of independent prior-session/timeline contributors. Clean speedbrake-retracted cells also derive a four-regime L/D/ballistic compatibility envelope for conditions without direct-force coverage. Confidence is based on repeatability across independent flight histories, not raw control-loop packet count.

Current-flight force samples are shadow flight-test evidence. The native model compares them with the matching certified cell and tracks a model-residual magnitude/confidence without changing nominal forces. A sustained discrepancy widens aerodynamic robustness stress and produces a warning. Atmospheric density, speed of sound and effective bank/control response may still adapt because they describe the environment or actuator response rather than replacing the vehicle aerodynamic database.

The Mach-binned adaptive calibrator continues to estimate a shadow profile and low-speed/stall margins. Automatic in-flight application is restricted to minimum-safe, approach and touchdown speed recommendations. Applying a reviewed aerodynamic profile is an explicit operator action and invalidates the current deorbit plan so it must be recomputed.

The simple-glide mode is a deliberate flight-test path. It holds the initial heading, commands wings level and zero throttle, then steps through the configured AoA range. Every step contains a settling period followed by a sampling period. During the last quarter of the middle-AoA sample, with generous altitude and dynamic-pressure margin, it performs one bounded speedbrake-on identification pulse so the next session can build speedbrake-specific force cells. AutoPilot is released immediately at the altitude floor or when dynamic pressure, G-load, sink rate, stall fraction, or recovery airspeed exceeds its configured boundary.

## 4. Entry prediction

The state derivative contains:

- spherical-body gravity;
- rotating-atmosphere relative velocity;
- drag acceleration derived from dynamic pressure and ballistic coefficient;
- lift acceleration derived from drag and lift-to-drag ratio;
- banked lift-vector rotation.

The body atmosphere is represented by the sampled pressure/density curve returned at connect time. The operational Mach-regime aerodynamic envelope comes from the prior-session data book and remains frozen during the flight. The trajectory calibrator may refine atmospheric density, local speed of sound and effective bank response, and it records forecast altitude/speed/range residuals for diagnosis. It does not use same-flight residuals to multiply nominal drag or lift. Current-flight force disagreement is instead represented as a model-residual uncertainty signal and as future-session flight-test evidence.

Direct-force prediction is configuration aware. The predictor queries separate prior cells for clean, gear, brake and speedbrake states, falls back to the certified Mach envelope when a local direct cell is missing, and applies explicit drag/lift stress multipliers only inside robustness scenarios. Nominal prediction never mutates the data book.

Entry guidance now uses the predictor as a receding-horizon controller rather than as a diagnostic overlay on a separate heuristic law. At each S-turn segment boundary it evaluates wings-level plus several candidate curvature radii derived from the currently achievable lift acceleration and learned bank effectiveness. Each candidate is propagated for a real 45–120 second control segment before the remainder of entry is predicted. The objective compares terminal/HAC geometry when it exists, or a smooth closest-approach surrogate before TAEM is physically reachable; it also scores short-horizon mechanical-energy dissipation, speed trend, q/G limits, curvature tightness, vertical-lift loss, control motion, and reversal count. The selected radius/AoA is committed until the segment boundary, while safety overrides may still unload immediately. This prevents a new prediction tick from becoming a new S-turn command.

The continuation law inside the predictor uses the same broad-radius family. It never falls back to the old direct score-to-maximum-bank mapping after the forced segment ends. Predicted bank and AoA pass through the same attitude rate/acceleration model as the live vehicle, and the live controller carries the chosen segment, current bank sign, and segment age forward on each forecast.

## 5. Burn execution

The vehicle points along instantaneous inertial retrograde. Inertial pointing leaves kRPC's roll target unset so the burn cannot jump between equivalent -180°/+180° roll representations; surface-mode guidance owns explicit roll after atmospheric entry. kRPC's signed AutoPilot angular error is normalized to an absolute alignment magnitude at the bridge/session boundary, and C inhibits throttle until that magnitude is below the configured gate. C emits signed surface-roll commands and the bridge normalizes them defensively before writing kRPC, without adding a competing transient-error throttle gate.

Retrograde capture begins during the coast phase rather than only at the burn-window boundary. When time warp is enabled, the controller exits warp with a lead of 180 seconds before burn start so the same attitude gate has time to settle even after a steep orbital attitude change.

After the burn, direct atmospheric control first captures the velocity-relative heading, wings-level roll, and the configured high-entry aerodynamic incidence. Entry pitch feedback is closed on measured angle of attack rather than surface Euler pitch, so large bank angles cannot project roll geometry into a false pitch error. RCS remains available throughout supersonic entry and is latched off at the transonic handoff; the rudder continues to provide sideslip coordination after that cutoff.

If guidance starts or resumes after the craft is already decisively descending below atmospheric interface, it abandons any stale orbital-burn state and enters atmospheric guidance. Delivered delta-v is an in-memory integral, so losing that accumulator during a process restart must not leave the orbiter holding retrograde through atmospheric flight.

Throttle is capped, follows a smoothstep startup ramp, and tapers as remaining delta-v decreases; exact zero is written through without suppressing small positive closure commands. Delivered delta-v is integrated from measured current thrust divided by current mass and projected by alignment cosine, not inferred from the throttle command. A second closure loop compares live orbital periapsis with the planned post-burn periapsis, progressively reducing authority over the final 8 km and cutting off within 250 m. When a valid target is available, burn completion requires that live periapsis capture or consumes at most an additional 8 m/s bounded closure reserve.

The burn also closes the planning/execution loop with a third signal: while thrusting, the controller periodically propagates the current measured vessel state as a `cutoff-now` trajectory. If stopping thrust at the current state already satisfies the complete entry/TAEM capture corridor, that achieved-state prediction can terminate the burn before the nominal delta-v or periapsis closure would do so. This prevents a small execution/model mismatch from turning into avoidable over-burn. The expensive full trajectory check is rate-limited during the burn rather than executed every control tick.

Immediately after burn completion, while the vehicle is still above the atmosphere, the controller performs a separate achieved-state verification from the measured inertial state. It records the actual osculating post-burn periapsis and predicts the resulting entry/TAEM path. A mismatch no longer silently inherits the original plan's success flag: the plan records whether the achieved state is still inside the capture corridor, lowers confidence when it is not, and surfaces a warning while guidance continues from live state.

Burn execution also has a progress watchdog for a partial or failed maneuver. Once the burn has started and retrograde alignment is valid, the controller tracks measured delivered delta-v progress. If significant delta-v remains but the measured thrust/progress stays effectively stalled for more than six seconds, guidance aborts instead of holding a stale burn indefinitely. The controller immediately invokes the kRPC safe path so throttle and AutoPilot state are released rather than relying on the previous command to expire.

The conservative defaults remain 18% maximum throttle, a twelve-second ramp, a 300 km entry-interface target at -1.2 degrees, and a 50 km post-burn periapsis. A started burn remains in the burn phase even when vertical speed first becomes negative, avoiding premature entry-mode transition. Before burn completion, the state machine cannot fall through to atmospheric guidance on a steep orbital descent.

## 6. Entry S-turn guidance

S-turn geometry is expressed as turn radius, not as a bank-angle schedule. The current aerodynamic model supplies lift acceleration and bank effectiveness; from those values the planner derives the minimum physically achievable radius and evaluates a family of deliberately wider arcs. In very thin air the optimal segment may be wings-level because banking cannot yet produce useful curvature and only discards vertical lift. As dynamic pressure builds, the same radius family naturally maps to larger bank commands and progressively smaller physical arcs.

Every selected leg is a precomputed trajectory segment. The controller commits its radius, bank side, and AoA for 45–120 seconds, then resolves the next segment from the measured state. Before dynamic TAEM capture exists, the objective keeps closest approach near the largest terminal circle the current architecture could eventually accept and uses short-horizon mechanical specific-energy loss to distinguish otherwise similar candidates. Mechanical energy is used instead of raw speed loss so descending through the gravity field cannot masquerade as aerodynamic braking. Airbrakes remain prohibited during Entry/S-turn; energy is managed by incidence, atmospheric dwell time, and banked lift.

Entry pitch is controlled directly in AoA space in the Python inner loop. Ground-track heading is guidance metadata during an S-turn, not a nose-heading servo target. Rudder feedback drives sideslip toward zero using a learned yaw-to-beta coupling. RCS is blended by axis according to measured aerodynamic ownership: jets fill only the pitch/roll/yaw authority that the surfaces have not yet demonstrated, then a final transonic latch prevents late re-arming. Dynamic-pressure, G-load, stall, and low-speed protection may override a planned segment immediately without changing the nominal trajectory logic.

## 7. TAEM and heading alignment

TAEM is no longer entered at a fixed altitude or fixed range shell. On every entry update the controller asks whether the current aerodynamic state can support a terminal circle at no more than the allowed terminal footprint. It derives the minimum turn radius from live/adaptive lift and bank effectiveness, expands it by capture margin, and checks both left and right HAC geometries. A candidate is accepted only when runway-local geometry, altitude corridor, and drag-derived braking distance can all be satisfied on the remaining HAC plus final path. Passing near or even beyond KSC while still hypersonic is therefore not itself a terminal failure.

Once a terminal solution is accepted, its HAC side and radius are latched. The reference trajectory, predictor, and live guidance all use that same dynamic radius. Remaining arc is integrated continuously through angle wrap; a transient reduction in instantaneous authority does not throw the vehicle back into Entry/S-turn. Bank demand is calculated from the lateral acceleration required for base circle curvature plus radial/course capture, divided by current lift authority. This replaces the old fixed-radius coordinated-turn formula.

The longitudinal TAEM channel computes the speed/energy schedule that can still be dissipated over the remaining HAC and final-approach distance. Live guidance and prediction share one binary approximation of the Shuttle-style speedbrake energy channel: a q-bar PI term plus specific-energy bias drives deploy/retract hysteresis, while AoA and flight-path command track the vertical profile. The KSP speedbrake remains retracted during Entry/S-turn because a full binary action-group deployment is not a faithful high-Mach trim surface. HAC completion is held at a small radius-relative exit window until radial, heading, energy, and turn-authority capture are simultaneously valid; only then may Final Approach latch.

TAEM phase entry has an explicit envelope. The normal fast handoff requires speed below the configured upper threshold but still inside the normal TAEM speed band. Slower states are not accepted merely because they are below that threshold; they must have a complete retained terminal candidate, sufficient total specific energy to reach its future join, enough lead time/cross-track maneuver room, a finite measured turn envelope, and a predicted join inside the configured TAEM altitude shell. Conversely, the current aircraft may still be above that shell if the retained future join is inside it. An exhausted HAC deadline never force-publishes an unconverged candidate.

## 8. Final approach

The final controller uses runway-local along-track and cross-track coordinates.

- Desired altitude is final-distance remaining multiplied by the configured glide slope.
- Glide-path error and sink-rate error jointly adjust the commanded flight-path angle.
- A speed-scaled L1 centerline law converts cross-track and actual ground-course error into lateral acceleration and bank.
- A simplified TECS law separates total-energy demand from height-versus-speed balance.
- Target speed decreases toward the calibrated approach speed.
- Optional propulsion is limited by `maximumApproachThrottle`; airbrakes use hysteresis for excess energy.
- Final capture and release use different thresholds so TAEM and Final cannot chatter at the boundary; capture is also withheld while airspeed is still above the recoverable final-approach energy gate.
- Gear deployment is based on altitude and distance.

## 9. Flare, touchdown and rollout

Flare begins at the configured radar altitude or, within a bounded low-altitude gate, when time-to-ground becomes short. Desired sink rate follows a height-dependent curve from the approach sink rate to the configured touchdown sink rate. Target speed now decays continuously from approach speed toward touchdown speed with height, so the controller does not dump energy at flare entry; airbrake and powered-assist thresholds follow that scheduled speed. A filtered sink-rate controller, low-speed AoA protection, and an increasing flare AoA schedule produce a progressive round-out while the short-look-ahead centerline law remains active.

At ground contact, AutoPilot is disengaged. Throttle is zero, airbrakes remain deployed, wheel-steering authority is reduced at high speed, and brakes are applied only after speed falls below a vehicle-relative threshold. The run completes when surface speed is below 1.5 m/s.

## 10. Phase state machine

`Idle → Calibration Glide` is an independent atmospheric test path. The landing sequence remains `Idle → Planning → Coast → Burn Setup → Deorbit Burn → Entry Interface → Entry / S-Turn → TAEM Acquisition → Heading Alignment → Final Approach → Flare → Touchdown → Rollout → Complete`

`Paused`, `Abort`, and `Fault` are explicit safety states. The controller does not write controls while idle or paused.

## 11. Command conditioning and envelope protection

Every atmospheric phase uses independent pitch, roll, heading, and throttle slew limits in C. Heading limiting follows the shortest wraparound direction. On a phase transition, limiters initialize from the measured aircraft attitude and throttle, avoiding a step command caused by different phase trim values. The Python bridge does not add another throttle or attitude slew layer.

kRPC AutoPilot is treated as the inner attitude/rate loop. Atmospheric profiles use narrow attenuation regions and shorter response times so small residual errors are not deliberately suppressed, and roll tracking remains enabled through large pitch/heading errors because bank is a primary flight-path control. Switching between inertial and surface reference frames explicitly disengages AutoPilot, seeds the complete target in the new frame, and then re-engages it, preventing one control cycle from interpreting an old target in a new frame.

Low-level diagnostics record body-frame pitch/roll/yaw angular velocity, actual pitch/roll/yaw actuator output, available torque, moment of inertia, auto-tuned kRPC PID gains, axis command errors, surface-course rate, telemetry/apply latency, guidance-compute time, and complete control-loop wall time. Euler-coordinate derivatives remain available for display/backward compatibility but are not used as stabilizing feedback. Bank-effectiveness learning uses surface-course rate instead of Euler heading rate.

The bank envelope is recomputed from calibrated minimum speed, dynamic-pressure ratio, G-load ratio, and stall fraction. This envelope is applied after the guidance law, so even a large path error cannot command a bank that violates the current low-speed or structural margin. Robust PID controllers use filtered derivatives and conditional integration to recover quickly after throttle, bank, or path-control saturation.

## 12. Instrument and interface model

Atmospheric commands explicitly request kRPC Surface speed mode; orbital coast and burn commands request Orbit mode. The bridge reads the selected mode back for the HUD. The artificial horizon rotates opposite measured aircraft roll, while the fixed bank pointer and command cue retain kRPC's positive-right-bank convention. Ground-track and flight-path markers use surface-relative velocity.

UI pane widths, HUD height and scale, lower-instrument height, map range, and map/profile/telemetry visibility are persisted independently of flight configuration. Split views remain draggable, and stored layout values are normalized to safe minimum and maximum dimensions.

## Latest flight corrections (September 7)

The 08:36:52Z flight crossed the final-entry and runway stations without HAC
acquisition. See [the measured incident and corrections](LatestFlight-2026-09-07-fixes.md).
S-turn dwell now starts at measured bank capture; committed events are replayed
by the predictor and canceled when their policy is replaced. Local energy
scoring preserves signed inbound progress after an overshoot. Steady elevator
trim is retained across delayed updates, and recovery release requires pitch
capture as well as low angular rates.
