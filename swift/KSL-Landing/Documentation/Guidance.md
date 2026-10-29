# Guidance, energy, and control

## Mission responsibilities

`LandingMissionController` coordinates entry, acquisition, heading alignment, approach, preflare, flare, rollout, completion, and abort. It owns phase and one committed terminal plan. Entry forecasting, path search, energy assessment, path following, approach capture, touchdown observation, and surface control belong to separate objects. No old MM304/MM305 class hierarchy or interprocess guidance bridge is required.

A phase is a description of the current control task, not evidence that the aircraft completed it successfully. Completion requires observed airborne flight, acceptable contact inside the runway, and a confirmed stop. Simulation reports and live reports carry different evidence labels.

## Entry prediction

The entry engine performs bounded receding-horizon shooting. It searches a lift-allocation parameter and, when reversal timing permits, both initial bank signs. Negative allocation moves alpha toward the polar's best lift-to-drag ratio; positive allocation increases bank magnitude and redirects lift from vertical support into lateral turning. Alpha is constrained by the moment-derived trim envelope. A speed-dependent heading deadband and minimum reversal interval suppress rapid bank-sign switching.

Each candidate integrates rotating-frame translation with the common atmosphere and polar. The candidate's lateral law is evaluated throughout the forecast. The same reversal hysteresis is simulated inside each candidate; it is not represented as an instantaneous sign switch. Alpha/bank response has bounded acceleration and rate, and the alpha rate cannot exceed the guidance slew limit. The usual horizon stops when the vehicle descends into a terminal-speed region, with a maximum duration of 1,600 s.

When the vehicle is already inside that speed region but terminal planning has not accepted a path, forecasts must still evolve. The entry solver enforces a minimum forecast duration of four replan intervals in this condition. Without it, every candidate would return the current state immediately and the search would cease to distinguish control actions. A regression test explicitly verifies that opposite bank commands produce different future states after a rejected handoff.

The inexpensive candidate score estimates distance, cross-range, remaining energy, and the turn still needed toward the long-range reference. The heading term is a soft cost, not a mandatory inlet orientation. The three leading valid forecasts are passed to the actual terminal planner, using its complete nominal radius/angle family. A feasible complete terminal route takes priority over a purely geometric score. Forecasts that violate q, load, or terrain constraints receive penalties. These are finite-grid searches; they are not globally optimal nonlinear programming or a proof of reachability.

The steering aim approximately 20 km before the threshold is a soft long-range reference. It does not define a mandatory acquisition point. Terminal planning starts from the actual measured position, velocity, bank, altitude, and flight-path angle. A feasible terminal route can take over before that reference is reached, and can use either side of the runway.

## Variable-radius spiral HAC

The HAC is an analytic spiral with remaining turn angle `theta` decreasing to zero:

\[
r(\theta)=R_f+k\theta^2,\quad k>0.
\]

In along-runway/right-of-runway coordinates, for side `s` equal to ±1,

\[
x=x_e-r\sin\theta,\qquad y=s(R_f-r\cos\theta).
\]

The endpoint lies on the centerline and its flown tangent is runway-forward. The planner samples final radial parameters from 3 to 40 km and turns of 60°, 100°, 150°, 210°, 270°, 330°, 390°, and 450°, subject to configured bounds and route length. The geometry supports turns up to 540°. The growth coefficient uses the speed-based turn-radius estimate and bounded positive growth. Both mirrored sides are evaluated.

`R_f` is a radial geometry parameter, not the local curvature radius. Curvature is calculated from first and second derivatives along the direction of flight. The local turn radius is `1/abs(curvature)` and varies over the route. Unit tests compare analytic curvature against finite changes of tangent, check mirroring, and include a 450° wraparound.

A quintic acquisition curve matches horizontal position, tangent, and curvature at the measured start and spiral inlet. Initial curvature comes from observed aerodynamic acceleration plus rotating-frame terms, projected across the ground track. Using `g*tan(bank)` would incorrectly assume a coordinated level turn during a steep entry and could demand several times the available lift. Another quintic unloads spiral curvature to zero over the transition to the straight final. C2 endpoint continuity is tested. The complete route includes acquisition, the spiral, the unload segment, final descent, preflare, and the touchdown reference. A direct terminal route may also be selected when feasible; `--spiral-only` disables that option for dedicated HAC validation.

The planner chooses geometry using a complete energy assessment, rather than selecting a visually plausible circle and assuming the rest of the landing will work. Vertical terminal geometry is a cubic matching current height/slope and final height/slope. This is a deliberately limited family: a rejected search is not proof that every possible trajectory is impossible.

## Energy assessment and constraints

Specific mechanical energy is derived from spherical gravitational potential and kinetic energy. Along every path station, the assessor calculates required normal and lateral lift, bank, q, trim-limited alpha, load factor, drag work, and remaining speed. The vertical normal curvature term is `V²*kappa`, without an extra projection by cosine; gravity is projected into the normal direction separately. An analytical pullout regression protects this distinction.

Lift inversion examines the polar's alpha knots, including nonmonotone regions. It does not assume an unlimited alpha-to-lift relationship. Required bank rate is compared with both the identified roll-rate limit and a reserve below the commanded slew limit. The final checks include touchdown speed and a runway braking-distance margin. Energy exhausted before touchdown is an infeasible route, even when horizontal geometry is excellent.

Nominal candidate rejection classifications are preserved. The selected route is additionally assessed at two opposing force corners: 0.8 lift/1.2 drag and 1.2 lift/0.8 drag by default. The results accompany the plan. If a corner fails, guidance explicitly emits `modelUncertainty`; it does not call the route robustly feasible. The present implementation still permits nominal execution of such a route. Requiring uncertainty-feasible plans is future work and would shrink the present operational envelope substantially.

When no route exists, entry prediction continues. At low speed, limited altitude, and negligible wind, a deliberately optimistic still-air range screen uses the largest polar L/D, favorable uncertainty, and an additional kinetic/ballistic reserve. A runway outside that range aborts the modeled mission. This check is conditional on the model and assumptions; it is not a universal impossibility theorem. Abort neutralizes surface commands and reports failure. It does not create an alternate runway or guarantee a survivable emergency landing.

## Path tracking and authority

Terminal tracking advances through a short monotonic station window. This prevents a nearest-point search from jumping to a later overlapping revolution. Reference values are interpolated along path distance, rather than abruptly switching at coarse samples. Lateral curvature feedforward is combined with heading/cross-track feedback. Vertical curvature feedforward is combined with altitude/sink feedback. Their combined acceleration demand determines bank and alpha through the shared lift model.

A bounded, filtered observation/model force ratio estimates current lift effectiveness. Tracking and approach lift inversion use it. The nominal polar remains immutable, and the estimator does not retroactively certify a plan. Large or sustained tracking error can release a committed plan for a new search while adequate altitude remains.

Targets pass through bank and alpha slew limits before reaching the attitude controller. The target slew limit and physical rate/acceleration limits have different roles: one constrains requested motion; the other constrains what surfaces can achieve.

## Final capture, preflare, and flare

The approach profile has a steep outer descent, a smooth preflare, and a short shallow segment. Defaults are 24° outer glide, 120 m/s design speed, 0.6 g preflare acceleration reserve, a 1.8° shallow glide, a 100 m shallow segment, and a 400 m touchdown reference. These values describe the supplied STS model, not a generic shuttle recipe.

An arriving aircraft rarely matches the nominal curve exactly. `ApproachCapture` constructs a quintic correction matching measured height and flight-path slope and rejoins the profile continuously. The same corrected curve is energy-priced and flown. This avoids assessing a nominal curve while commanding a different capture maneuver. Capture requires bounded position, heading, bank, and energy error. A committed HAC is not bypassed merely because a loose proximity test passes.

Preflare includes curvature feedforward with a short preview to account for attitude response. The final flare begins near the configured gear-height surface and targets a bounded descending sink rate instead of commanding a zero-sink hover. Roll authority narrows close to the ground. Low lift authority is reported, not hidden by an unattainable alpha request. Actual contact quality is checked independently of guidance phase.

## Surface control and ground control

`FlightController` uses q-dependent trim/dynamic inversion, angular-rate feedback, filtered target-rate feedforward, and bounded conditional integration. It reports pitch/roll/yaw saturation and rate-limits surface commands. In vacuum, the plant has no aerodynamic authority; the controller does not receive an artificial simulator-only torque.

Only pitch, roll, yaw, wheel steering, gear, and wheel brakes are available as commanded outputs. Throttle is always zero, SAS/RCS/airbrakes are always false, and airborne wheel braking is rejected. Rollout applies a short braking ramp and centerline steering. Both live and simulated brakes use the same boolean threshold.

## Where the method remains limited

The entry predictor is a reduced attitude model. The terminal vertical family and finite radius/turn search can miss viable routes. Worst-case force corners are reported but not required for commitment. Some archived and perturbed entry states still fail, as listed without omission in [Validation.md](Validation.md). Atmospheric heating, detailed terrain avoidance, damage recovery, multi-runway diversion, and online moment-system identification are outside this implementation. These are concrete development boundaries, not capabilities implied by a successful nominal simulation.
