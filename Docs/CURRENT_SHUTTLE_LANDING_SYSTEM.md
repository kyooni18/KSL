# KSP Shuttle Landing Guidance System

Technical description of the current KSPShuttleLander implementation, including the flight-dynamics model, adaptive aerodynamics, deorbit planning, entry energy management, S-turn guidance, TAEM/HAC capture, final approach, flare, and direct atmospheric control loops. This document describes the implementation present in the codebase on 7 September 2026 rather than an idealized Space Shuttle guidance system.

## 1. Purpose and system architecture

KSPShuttleLander is an autonomous deorbit, atmospheric-entry, terminal-area-energy-management, approach, and landing system for a shuttle-like vehicle in Kerbal Space Program. Its main design goal is not to replay a fixed trajectory. Instead, it continuously estimates the vehicle's actual aerodynamic behavior, predicts the future trajectory from the measured state, and changes the commanded bank angle and angle of attack so that the vehicle reaches a dynamically feasible terminal-energy state from which a Heading Alignment Circle (HAC), final approach, flare, and runway landing can be completed.

The implementation is split into two principal control layers. The native C core owns navigation, trajectory propagation, aerodynamic estimation, deorbit planning, guidance, phase logic, safety gates, and the receding-horizon entry optimizer. A Python kRPC bridge owns the actual KSP connection and the low-level atmospheric control loop. The two communicate using an NDJSON protocol. Orbital attitude commands may use an inertial target, but after atmospheric entry the system releases the ordinary kRPC AutoPilot/SAS attitude controller and directly commands pitch, roll, and yaw control inputs. The C core therefore specifies the aerodynamic state that should be flown, while the Python bridge turns that request into actuator commands using measured angular-rate response.

The normal control thread runs at the configured guidance rate, currently 10 Hz. Atmospheric forecast propagation normally runs at the configured prediction interval, currently 1.5 s. A full entry-MPC candidate re-optimization is intentionally slower: while an accepted segment remains valid, the controller normally re-solves the candidate family every approximately 3 to 10 s, with the interval shortened as measured aerodynamic acceleration rises, as the terminal boundary approaches, or as the retained segment nears its end. The entry optimizer commits the S-turn side and reversal events for stability, while bank magnitude and angle of attack may therefore be re-solved from the current measured state without allowing left/right branch chatter.

The principal sequence is

\[
\text{Orbit} \rightarrow \text{Deorbit Burn} \rightarrow \text{Entry Interface}
\rightarrow \text{Entry/S-turn} \rightarrow \text{TAEM/HAC}
\rightarrow \text{Final} \rightarrow \text{Flare} \rightarrow \text{Rollout}.
\]

The implementation also contains attitude-recovery, guarded-recovery, pause, abort, fault, glide-calibration, reentry-only, and HAC-only paths.

## 2. Coordinate systems, surface navigation, and rotating Kerbin

The predictor is propagated in an inertial Cartesian frame. kRPC vector conventions are converted at the interface so that all core vector algebra uses one internally consistent right-handed frame before cross products, rotations, and numerical integration are performed.

For a body with angular-velocity vector \(\boldsymbol{\omega}\), inertial position \(\mathbf r\), and inertial velocity \(\mathbf v\), the velocity relative to the rotating atmosphere is

\[
\mathbf v_a = \mathbf v - \boldsymbol{\omega}\times\mathbf r,
\qquad
V = \|\mathbf v_a\|.
\]

This air-relative velocity, not inertial orbital velocity, is used for Mach number, dynamic pressure, aerodynamic force, ground-track course, and entry mechanical energy.

The body-fixed longitude basis is reconstructed from a stored prime-meridian vector at an epoch. If \(t_0\) is the epoch and \(\hat{\mathbf n}\) the north-pole axis, the prime-meridian basis at universal time \(t\) is obtained by Rodrigues rotation,

\[
\mathbf a' = \mathbf a\cos\theta
 + (\hat{\mathbf n}\times\mathbf a)\sin\theta
 + \hat{\mathbf n}(\hat{\mathbf n}\cdot\mathbf a)(1-\cos\theta),
\qquad
\theta = \omega(t-t_0).
\]

This lets the propagator convert inertial positions back into latitude and longitude while Kerbin rotates underneath the trajectory.

Great-circle range between geographic points \((\phi_1,\lambda_1)\) and \((\phi_2,\lambda_2)\) is evaluated with the haversine relation

\[
h = \sin^2\!\left(\frac{\Delta\phi}{2}\right)
 + \cos\phi_1\cos\phi_2\sin^2\!\left(\frac{\Delta\lambda}{2}\right),
\]

\[
d = 2R\operatorname{atan2}(\sqrt h,\sqrt{1-h}).
\]

The initial great-circle bearing is

\[
\chi = \operatorname{atan2}\!\left(
\sin\Delta\lambda\cos\phi_2,
\cos\phi_1\sin\phi_2-\sin\phi_1\cos\phi_2\cos\Delta\lambda
\right).
\]

Near the runway, surface geometry is converted into local east/north coordinates. With runway heading \(\psi_r\), east offset \(e\), and north offset \(n\), the runway-local coordinates are

\[
x_{\parallel}=e\sin\psi_r+n\cos\psi_r,
\qquad
x_{\perp}=e\cos\psi_r-n\sin\psi_r.
\]

Negative \(x_{\parallel}\) denotes a point before the runway threshold. These coordinates are used by HAC exit validation, centerline capture, final approach, flare, and rollout.

## 3. Atmospheric and aerodynamic model

### 3.1 Atmosphere

When connected to KSP, the body model contains a live altitude table of atmospheric density and pressure. Density and pressure are interpolated logarithmically between altitude samples because both vary approximately exponentially over much of the atmosphere. The current predictor therefore does not rely on a single exponential scale height for normal connected flight.

The local speed of sound is evaluated using KSP's pressure/density relation,

\[
a = \sqrt{\gamma\frac{p}{\rho}},
\]

where the implementation converts KSP pressure from kPa to Pa, giving

\[
a = \sqrt{\gamma\frac{1000p_{\mathrm{kPa}}}{\rho}}.
\]

The trajectory calibrator may apply a bounded learned speed-of-sound scale. Mach number and dynamic pressure are then

\[
M = \frac{V}{a},
\qquad
q = \frac12\rho V^2.
\]

The live density used by the predictor is

\[
\rho_{\mathrm{model}}(h)=s_\rho\,\rho_{\mathrm{KSP}}(h),
\]

where \(s_\rho\) is an online trajectory-calibration factor. The estimator stores it over the range 0.35 to 2.8; atmospheric propagation applies the slightly tighter operational clamp

\[
s_\rho^{prop}=\operatorname{clip}(s_\rho,0.35,2.5).
\]

### 3.2 Reduced-order vehicle model

The predictor currently uses a reduced-order shuttle aerodynamic model rather than reconstructing every KSP part, drag cube, center of mass, and control surface from the `.craft` file. Vehicle-specific behavior is represented by a Mach-dependent lift-to-drag ratio, a Mach-dependent ballistic coefficient, an angle-of-attack polar, and learned correction scales. The live KSP lift and drag force streams are used to continually correct this reduced-order model.

The aerodynamic envelope contains four models centered at Mach anchors

\[
M_i \in \{0.35,\ 1.05,\ 2.6,\ 6.0\},
\]

and \(L/D\), ballistic coefficient, and confidence are linearly interpolated between adjacent anchors.

For a baseline ballistic coefficient \(\beta\), the predictor first forms

\[
a_0 = \frac{q}{\max(20,\beta)}s_D,
\]

where \(s_D\) is the learned drag scale. Angle-of-attack and Mach factors then give

\[
a_D = a_0 f_D(M,\alpha),
\]

\[
a_L = a_0\left(\frac{L}{D}\right)s_L f_L(M,\alpha),
\]

with learned lift scale \(s_L\).

The stock-KSP reference polar used to keep the model well conditioned before enough flight data have been learned is normalized at the configured entry angle of attack \(\alpha_r\). For \(|\alpha|\le45^\circ\),

\[
D_b(\alpha)=0.125+0.00066\alpha^2,
\qquad
L_b(\alpha)=\sin(2\alpha),
\]

where the trigonometric argument is converted to radians. The transonic factor is

\[
T(M)=\exp\!\left[-\left(\frac{\max(0,M)-1}{0.38}\right)^2\right].
\]

The normalized force factors are approximately

\[
f_D = \frac{D_b(\alpha)}{D_b(\alpha_r)}\left(1+0.22T\right),
\]

\[
f_L = \frac{L_b(\alpha)}{L_b(\alpha_r)}\left(1-0.05T\right).
\]

They are bounded in the implementation, with \(f_D\in[0.02,3.2]\) and the magnitude of \(f_L\) bounded by 3.2. At incidence above \(45^\circ\), a small additional drag growth and lift decay are applied, primarily as a numerical safety extension rather than a calibrated normal operating model.

### 3.3 Aerodynamic force direction

Let

\[
\hat{\mathbf v}_a=\frac{\mathbf v_a}{V}
\]

and let \(\hat{\mathbf u}=\mathbf r/\|\mathbf r\|\) be local up. A nominal lift-up axis is formed by projecting local up into the plane normal to the airflow,

\[
\hat{\mathbf l}_0 =
\frac{\hat{\mathbf u}-(\hat{\mathbf u}\cdot\hat{\mathbf v}_a)\hat{\mathbf v}_a}
{\left\|\hat{\mathbf u}-(\hat{\mathbf u}\cdot\hat{\mathbf v}_a)\hat{\mathbf v}_a\right\|},
\]

with lateral lift axis

\[
\hat{\mathbf l}_s=\hat{\mathbf v}_a\times\hat{\mathbf l}_0.
\]

The learned bank effectiveness \(k_\phi\) modifies the commanded bank,

\[
\phi_{\mathrm{eff}}=k_\phi\phi,
\]

The turn-radius and live terminal calculations accept the learned range

\[
k_\phi\in[0.35,1.8].
\]

For the translational force-vector propagation itself, the factor is conservatively clipped to

\[
k_\phi^{prop}=\operatorname{clip}(k_\phi,0.55,1.35),
\]

so an immature calibration estimate cannot rotate the modeled lift vector by an extreme amount.

and the lift direction is

\[
\hat{\mathbf l}=
\hat{\mathbf l}_0\cos\phi_{\mathrm{eff}}
+\hat{\mathbf l}_s\sin\phi_{\mathrm{eff}}.
\]

The predictor's aerodynamic acceleration is therefore

\[
\mathbf a_{\mathrm{aero}}=-a_D\hat{\mathbf v}_a+a_L\hat{\mathbf l}.
\]

## 4. Translational trajectory predictor

The gravitational acceleration is spherical two-body gravity,

\[
\mathbf a_g=-\mu\frac{\mathbf r}{r^3}.
\]

Atmospheric propagation uses

\[
\dot{\mathbf r}=\mathbf v,
\qquad
\dot{\mathbf v}=\mathbf a_g+\mathbf a_{\mathrm{aero}}.
\]

Both vacuum and atmospheric trajectories are integrated with classical fourth-order Runge-Kutta. The atmospheric time step is adaptive. It is reduced in the transonic region, at high dynamic pressure, when non-gravitational acceleration is large, when density changes rapidly along a steep descent, and near event surfaces such as atmospheric entry. This is important because a single coarse fixed step can move TAEM acquisition, peak dynamic pressure, or an S-turn reversal by several kilometres.

The predictor is not a complete six-degree-of-freedom rigid-body simulator. Translation is propagated physically, while bank and angle-of-attack response are represented by rate- and acceleration-limited surrogate attitude states. This is intentional: the live KSP vehicle supplies the actual rigid-body dynamics, and the predictor only needs an attitude response model accurate enough to forecast the energy and ground-track consequences of guidance decisions.

For bank, the predictor uses a first-order rate demand with acceleration limiting. In simplified form,

\[
e_\phi=\phi_c-\phi,
\qquad
\dot\phi_c^*=\operatorname{clip}(0.72e_\phi,-\dot\phi_{\max},\dot\phi_{\max}),
\]

\[
\ddot\phi=\operatorname{clip}
\left(\frac{\dot\phi_c^*-\dot\phi}{\Delta t},
-\ddot\phi_{\max},\ddot\phi_{\max}\right).
\]

Angle of attack uses the same idea with a 0.62 error gain, approximately \(4.2^\circ/\mathrm{s}\) maximum rate and \(2.4^\circ/\mathrm{s^2}\) maximum acceleration inside the predictor.

## 5. Mechanical energy model

Entry guidance treats bank and angle of attack primarily as energy-path actuators. Because the atmosphere rotates with Kerbin, the predictor uses an air-relative rotating-frame specific mechanical energy rather than simply \(V^2/2+gh\). At latitude \(\phi\), radius \(r=R+h\), and air-relative speed \(V\),

\[
\varepsilon(\phi,h,V)
=\frac12V^2-\frac{\mu}{r}
-\frac12\omega^2(r\cos\phi)^2.
\]

The energy remaining to a target state is

\[
\Delta\varepsilon
=\varepsilon(\phi,h,V)
-\varepsilon(\phi_t,h_t,V_t).
\]

The TAEM target speed is generated from the configured final-approach speed and nominal TAEM range,

\[
V_{\mathrm{TAEM}}
=\operatorname{clip}\!\left(
1.22V_f+\frac{R_{\mathrm{TAEM}}}{70},
1.22V_f,
\max(6.3V_f,700)
\right).
\]

With the current defaults \(V_f=115\ \mathrm{m/s}\) and \(R_{\mathrm{TAEM}}=35\ \mathrm{km}\), this produces a target of approximately \(640\ \mathrm{m/s}\). The accepted TAEM speed envelope is

\[
V_{\min}=\max(420,3.5V_f),
\qquad
V_{\max}=\max(820,7V_f),
\]

which is currently 420 to 820 m/s.

Entry guidance estimates the average drag acceleration still required per metre of useful downrange,

\[
\bar a_{D,\mathrm{req}}
=\frac{\Delta\varepsilon_{\mathrm{TAEM}}}
{\max\left(5000,\ d-\max(R_{\mathrm{TAEM}},3R_{\mathrm{HAC,base}})\right)}.
\]

The ratio

\[
\eta_D=\frac{a_D}{\bar a_{D,\mathrm{req}}}
\]

is one of the central indicators used by the S-turn policy. If \(\eta_D<1\), the trajectory is not dissipating energy quickly enough for the downrange being consumed. The controller may bank more deeply, reducing vertical lift and allowing the shuttle to descend into denser air, or use lateral path length to preserve useful downrange while drag work accumulates.

## 6. Prior-flight aerodynamic data book and shadow identification

The nominal aerodynamic model is immutable for one connected flight session. At connect time, the Python bridge hashes the assembled vessel structure and the atmospheric environment, then selects only matching observations from earlier sessions and quickload timeline epochs. The current session is explicitly excluded from its own predictor history. This prevents one upset, bad derivative, startup fallback, or unusual trajectory from redefining the model while the same vehicle is relying on it for entry guidance.

The persistent SQLite archive keeps raw q, Mach, signed AoA, sideslip, mass, force vectors, device state and structural metadata. Prior observations are reduced to a bounded local data book in \((\log q,M,\alpha,\beta)\) space, with independent configuration dimensions for landing gear, wheel brakes and speedbrakes. Each local cell also records the number of independent prior-session/timeline contributors represented by the selected sample. Support confidence therefore reflects repeatability across flight histories instead of the number of correlated 10 Hz packets collected in one run.

For a prior cell with measured wind-axis force \(\mathbf F_w\) and dynamic pressure \(q\), the stored quantity is

\[
\mathbf f_q=\frac{\mathbf F_w}{q}.
\]

At a predicted state with dynamic pressure \(q_p\) and mass \(m_p\), the local direct-force model returns specific aerodynamic acceleration

\[
\mathbf a_{aero}\approx \frac{q_p}{m_p}\,\operatorname{interp}(\mathbf f_q).
\]

This preserves the measured force shape while retaining the physical inverse-mass scaling. The local interpolation support is deliberately compact so a high-Mach/high-incidence cell cannot silently extrapolate into a low-speed landing condition.

Clean speedbrake-retracted cells are also reduced into subsonic, transonic, supersonic and hypersonic compatibility values of effective \(L/D\) and ballistic coefficient. These values are used only when no local direct-force cell supports the query. L/D is averaged linearly; ballistic coefficient is averaged in log space. Weighted prior scatter becomes the certified aerodynamic uncertainty rather than an online correction to the nominal model.

Current-flight direct-force samples are shadow flight-test evidence. They are archived for a future session, but the C core does not merge them into the current data-book cells. Instead it compares measured specific force with the matching certified prediction and forms a bounded residual

\[
r_F=\frac{\|\mathbf a_{\mathrm{meas}}-\mathbf a_{\mathrm{book}}\|}
{\max(0.05,\|\mathbf a_{\mathrm{meas}}\|)}.
\]

Only independently spaced valid samples increase residual confidence. A sustained residual outside the prior-flight scatter raises a model-envelope warning and widens the drag/lift stress cases used for robustness analysis; it does not recenter the nominal force model during the same entry.

The trajectory calibrator still adapts quantities that are properly environmental or response variables. Density scale is learned from measured density versus the live body table, speed-of-sound scale from the measured local speed of sound or \(V/M\), and bank effectiveness from actual course-rate response. Same-flight drag and lift multipliers remain unity.

The shadow Mach-binned estimator can still derive recommended low-speed/stall margins. Automatic in-flight application is limited to minimum-safe, approach and touchdown speeds. Promoting a reviewed aerodynamic profile is an explicit operator action and marks the current deorbit plan stale so it must be recomputed.

The simple-glide calibration path is a deliberate test maneuver rather than an online certification mode. It steps through the configured AoA values in clean, wings-level, zero-throttle flight. During the last quarter of the middle-AoA sample, and only with generous altitude and dynamic-pressure margin, it performs one short speedbrake-on identification pulse. Those force measurements are intended to populate speedbrake-specific cells on the next session.

Bank effectiveness is learned from actual course-rate response. The nominal coordinated-turn course rate is estimated as

\[
\dot\chi_{\mathrm{exp}}
=\frac{a_L\sin\phi}{V_h},
\]

and a sample of bank effectiveness is approximately

\[
k_{\phi,s}=\frac{\dot\chi_{\mathrm{meas}}}{\dot\chi_{\mathrm{exp}}}.
\]

The learned value is bounded to 0.35 through 1.8.

## 7. Deorbit planning

The deorbit planner searches a future time/delta-v space instead of assuming one fixed burn. It normally searches up to three orbital periods. The first coarse pass samples future coast times at roughly 22 to 40 s spacing and candidate retrograde delta-v from 5 to 420 m/s in 10 m/s increments, retains a set of geographically and energetically distinct candidates, then performs successively finer time and delta-v refinements around promising solutions. A high-resolution rescue scan is available when a broad recovery corridor exists but the ordinary grid may have stepped over a narrow strict-entry window.

The burn itself is propagated as a finite burn rather than as an impulsive velocity subtraction. The commanded direction is nominally retrograde, with stress-test cases adding radial and normal pointing errors. If maximum available acceleration at the configured throttle is \(a_{\max}\), remaining delta-v is \(\Delta v_r\), elapsed burn time is \(t_b\), and ramp duration is \(t_r\), the throttle-shaping function is

\[
x=\operatorname{clip}\left(\frac{t_b}{t_r},0,1\right),
\qquad
s(x)=x^2(3-2x),
\]

\[
\Delta v_t=\max(1,1.25a_{\max}),
\]

\[
f_{\mathrm{taper}}=
\begin{cases}
1, & \Delta v_r\ge\Delta v_t,\\
\max\left(0.18,\sqrt{\Delta v_r/\Delta v_t}\right), & \Delta v_r<\Delta v_t,
\end{cases}
\]

\[
f_{\mathrm{burn}}=\operatorname{clip}(\min(s,f_{\mathrm{taper}}),0,1).
\]

After the finite burn, predicted osculating periapsis is calculated from specific orbital energy and angular momentum,

\[
\mathbf h=\mathbf r\times\mathbf v,
\qquad
E=\frac12v^2-\frac{\mu}{r},
\]

\[
e=\sqrt{1+\frac{2E\|\mathbf h\|^2}{\mu^2}},
\qquad
r_p=\frac{\|\mathbf h\|^2}{\mu(1+e)},
\qquad
h_p=r_p-R.
\]

Each candidate is then propagated through atmospheric entry and terminal acquisition. Fixed entry range is no longer treated as the principal deorbit target. The configured 1030 km `targetEntryRange` remains a nominal geometric scale inside entry guidance, but deorbit selection is dominated by whether the predicted trajectory actually reaches the dynamic TAEM/HAC boundary with acceptable energy, speed, loads, and geometry.

The implementation's coarse deorbit objective is dimensionful and empirical. In compact notation it is

\[
\begin{aligned}
J_{\mathrm{deorbit}}={}&1.8|e_R|+650|e_\chi|
+5000\max(|e_\chi|-120,0)\\
&+40000|\gamma_e-\gamma_t|
+320000\max(\gamma_{\min}-\gamma_e,0)^2\\
&+0.45|h_p-h_{p,t}|+0.8\max(h_{p,t}-h_p,0)
+28\max(h_{p,\mathrm{floor}}-h_p,0)\\
&+3.2\max(d_{\min}-R_{\mathrm{cap}},0)+0.08d_{\min}
+J_V+J_{\mathrm{HAC}}+J_{\mathrm{noTAEM}}+J_{q,g}+8\Delta v,
\end{aligned}
\]

where \(e_R\) is the TAEM radial/range error, \(e_\chi\) is entry course error, \(\gamma_e\) is entry flight-path angle, and the remaining terms penalize TAEM speed violation, poor HAC capture score, failure to reach TAEM, excess dynamic pressure, and excess g-load. Candidate tier is considered before the scalar score, so a physically qualified capture candidate cannot be displaced merely by a numerically smaller score from a less useful class.

### 7.1 Robustness testing

The selected burn is evaluated with a deterministic 13-case stress matrix. Current configurable uncertainties include ignition timing, thrust, delta-v delivery, mass, initial position, initial velocity, pointing, and atmospheric density. With the current default configuration these include approximately ±0.5 s timing, ±8% thrust, ±0.5 m/s delta-v, ±3% mass, ±50 m position, ±0.5 m/s velocity, ±2 deg pointing, and ±12% atmospheric density. The required strict or recovery pass fraction is 75%, and no stress case may violate the hard unsafe envelope.

The default strict capture gate requires atmospheric entry, dynamic TAEM acquisition, closest approach no worse than 100 km, an entry flight-path angle no steeper than -3.0 deg, entry course error within 120 deg, TAEM radial error within \(\max(20\ \mathrm{km},0.7R_{\mathrm{TAEM}})\), TAEM speed inside the accepted bounds, HAC capture score no worse than 6, peak dynamic pressure below 1.05 times the vehicle limit, peak g-load below 1.05 times the vehicle limit, and sufficient post-burn periapsis margin. With the current 50 km target periapsis, the strict periapsis floor is 40 km.

The guarded-recovery gate is deliberately wider. It still requires a shallow-enough entry and a 100 km closest-approach bound, but permits a much wider TAEM error and speed range, up to 1.10 times the configured q and g limits, and the lower 35 km execution periapsis floor. A plan that satisfies only this broader envelope is marked degraded rather than being presented as a nominal strict solution.

## 8. Live deorbit-burn execution

The live burn begins at

\[
t_{\mathrm{start}}=t_{\mathrm{plan}}-\frac12t_{\mathrm{burn,est}}.
\]

The vehicle is aligned to inertial retrograde. Throttle is inhibited outside a 10 deg alignment gate. Delivered delta-v is not assumed from command duration; it is integrated from measured thrust and mass,

\[
\Delta v_{\mathrm{del}}
\leftarrow\Delta v_{\mathrm{del}}
+\frac{T_{\mathrm{meas}}}{m}\cos e_{\mathrm{align}}\,\Delta t.
\]

Alignment authority is tapered as

\[
f_{\mathrm{align}}
=\operatorname{clip}\left(\frac{10^\circ-|e_{\mathrm{align}}|}{6^\circ},0,1\right),
\]

and, when live periapsis closure is available, the controller forms the periapsis residual against the predicted post-burn periapsis of the accepted plan,

\[
e_p=h_{p,\mathrm{live}}-h_{p,\mathrm{plan}},
\]

then applies the terminal burn factor

\[
f_p=\operatorname{clip}\left(\frac{e_p-250\ \mathrm{m}}{8000\ \mathrm{m}},0,1\right).
\]

Thus throttle tapers continuously as the measured osculating periapsis closes to within 250 m of the planned post-burn value. Effective throttle is therefore approximately

\[
u_T=u_{T,\max}\min(f_{\mathrm{burn}},f_p)f_{\mathrm{align}}.
\]

The burn may terminate from integrated delta-v, periapsis closure, or a live cutoff-now trajectory that already satisfies the capture gate. A thrust/delta-v progress watchdog aborts the burn if meaningful thrust is commanded but measured progress disappears for a sustained interval.

## 9. Entry interface and aerodynamic capture

After the deorbit burn, the system captures a controlled entry attitude before allowing S-turn guidance to take over. For the default Kerbin atmosphere, the current entry-guidance start is approximately 69 km altitude because

\[
h_{\mathrm{entry}}
=\max\left(h_{\mathrm{TAEM}}+1000,
h_{\mathrm{atm}}-m_{\mathrm{entry}}-500\right).
\]

With \(h_{\mathrm{atm}}=70\ \mathrm{km}\), margin \(m_{\mathrm{entry}}=0.5\ \mathrm{km}\), and \(h_{\mathrm{TAEM}}=25\ \mathrm{km}\), this gives 69 km.

The entry-interface command is wings level, aligned with ground-track/prograde direction, and commands the maximum configured entry-capture angle of attack. At the actual entry boundary, S-turn guidance is inhibited until the measured state satisfies

\[
|\phi|\le18^\circ,
\qquad
|\alpha-\alpha_{max}|\le4^\circ,
\]

\[
|\psi-\chi_g|\le15^\circ,
\qquad
|\dot\phi|,|\dot\theta|,|\dot\psi|\le8^\circ/\mathrm{s}.
\]

This prevents the first S-turn from beginning while the craft is still correcting an arbitrary orbital attitude.

## 10. S-turn geometry and entry MPC

### 10.1 Turn radius from measured lift

The S-turn system is radius-based rather than a table of fixed bank angles. For effective bank \(\phi_{\mathrm{eff}}=k_\phi\phi\), measured or modeled lift acceleration \(a_L\), and true airspeed \(V\), lateral acceleration is

\[
a_{lat}=a_L|\sin\phi_{\mathrm{eff}}|,
\]

and the turn radius is

\[
R_{turn}=\frac{V^2}{a_{lat}}.
\]

The inverse relation used to convert a desired radius to bank is

\[
\phi
=\frac{1}{k_\phi}
\sin^{-1}\!\left(\frac{V^2}{R_{turn}a_L}\right),
\]

subject to current bank, load, stall, and authority limits.

The instantaneous live bank envelope begins at the configured vehicle limit \(\phi_{max}\). Let

\[
s_V=\frac{V}{V_{minsafe}}.
\]

At low speed the preliminary limit is

\[
\phi_{lim}\leftarrow
\begin{cases}
\min\left(\phi_{lim},18^\circ+30^\circ\max(0,s_V-0.9)\right),&s_V<1.15,\\
\min\left(\phi_{lim},32^\circ+40^\circ(s_V-1.15)\right),&1.15\le s_V<1.4.
\end{cases}
\]

For dynamic-pressure ratio \(r_q=q/q_{max}\),

\[
r_q>0.92\quad\Rightarrow\quad
\phi_{lim}\leftarrow\phi_{lim}\operatorname{clip}(1.25-r_q,0.35,1),
\]

and for load ratio \(r_g=n/n_{max}\),

\[
r_g>0.90\quad\Rightarrow\quad
\phi_{lim}\leftarrow\phi_{lim}\operatorname{clip}(1.20-r_g,0.30,1).
\]

The live controller additionally caps bank at 24 deg when stall fraction exceeds 0.10, then clamps the final result between 5 deg and \(\phi_{max}\). These limits are continuous guards on a physically selected trajectory rather than the primary guidance law.

### 10.2 Candidate family

At an MPC solve, the minimum physically achievable radius is computed from the current lift and bounded bank capability. Candidate radii are then generated approximately as

\[
R_c\in\{\infty,\ 8R_{\min},\ 4R_{\min},\ 2R_{\min},\ 1.2R_{\min}\}.
\]

The angle-of-attack candidates are

\[
\alpha_c\in
\left\{\alpha_{entry},\frac{\alpha_{entry}+\alpha_{max}}2,\alpha_{max}\right\}.
\]

With the current defaults this corresponds to approximately 18 deg, 23 deg, and 28 deg. Left and right S-turn signs are evaluated unless the current side or a planned reversal is committed.

Candidate segment duration is tied to actual curvature rather than a fixed arbitrary pulse. If \(\dot\chi\approx V/R_c\), the nominal segment duration is

\[
t_s=\operatorname{clip}
\left(\max(t_{\min\,leg},0.45/\dot\chi),35,75\right)\ \mathrm{s},
\]

and it cannot be shortened below an already committed remainder.

Each candidate is propagated through the same atmosphere, aerodynamic envelope, attitude-response model, S-turn reversal logic, and dynamic TAEM/HAC acquisition used by the main predictor. A candidate is therefore a future policy, not merely a short bank pulse followed by an unrealistically optimistic fallback.

### 10.3 Entry MPC cost

When a forecast reaches TAEM, normalized terminal errors are

\[
r_n=\frac{e_{R,\mathrm{TAEM}}}{\max(20\ \mathrm{km},R_{\mathrm{TAEM}})},
\]

\[
e_n=\frac{\Delta\varepsilon_{\mathrm{TAEM}}}
{\max(50\ \mathrm{kJ/kg},\frac12V_{\mathrm{TAEM}}^2)}.
\]

The core MPC cost is

\[
\begin{aligned}
J={}&4r_n^2+J_E+0.18H^2+1.15P+3.2Q^2
+J_{short}+J_{transition}\\
&+60q_o^2+60g_o^2+0.08N_{rev}
+0.45T^2+0.35L_v^2+0.025\Delta\phi_n^2+0.015\Delta\alpha_n^2,
\end{aligned}
\]

where

\[
J_E=
\begin{cases}
4.5e_n^2,&e_n<0,\\
2.2e_n^2,&e_n\ge0,
\end{cases}
\]

\(H\) is the normalized HAC capture score, \(P\) is the pending-terminal soft cost when TAEM has not yet been reached, \(Q\) penalizes penetration inside the nominal TAEM shell without proper capture, \(q_o\) and \(g_o\) are normalized load-limit violations, \(N_{rev}\) is reversal count, \(T\) measures how close the candidate is to the minimum achievable turn radius, and

\[
L_v=1-\cos|\phi_c|
\]

penalizes unnecessary loss of vertical lift.

The short-horizon term explicitly compares energy work with useful downrange consumption. Let

\[
W=\max(0,\varepsilon-\varepsilon_{TAEM}),
\qquad
U=\frac{W}{d_{useful}}.
\]

More exactly, define

\[
U_b=\frac{W_b}{d_b},
\qquad
U_a=\frac{W_a}{d_a},
\qquad
G=\frac{\max(0,U_a-U_b)}{\max(1,U_b)},
\]

where \(b\) and \(a\) denote the beginning and control-horizon states. Let aerodynamic mechanical-energy removal over the horizon be

\[
\Delta E_{loss}=\max(0,\varepsilon_b-\varepsilon_a),
\]

with normalized useful loss

\[
L_E=\frac{\Delta E_{loss}}
{\max(50\ \mathrm{kJ/kg},0.04V_b^2)}.
\]

The primary short-horizon cost is

\[
J_{short,0}
=7.5G^2
+1.35\left[\frac{U_a}{\max(2.5,0.85U_b)}\right]^2
-0.45\operatorname{clip}(L_E,0,3).
\]

If useful downrange consumption

\[
\Delta d=\max(0,d_b-d_a)
\]

exceeds 1 km while \(W_b>50\ \mathrm{kJ/kg}\), the controller estimates the energy loss that should have accompanied that downrange consumption,

\[
\Delta E_{req}=U_b\Delta d,
\qquad
p_E=\frac{\Delta E_{loss}}{\max(\Delta E_{req},1000)},
\]

and, when meaningful, adds the deficit penalty

\[
J_{def}=8
\left[\frac{\max(0,0.88-p_E)}{0.88}\right]^2
\operatorname{clip}\left(
\frac{\Delta d}{\max(0.08d_{initial},15\ \mathrm{km})},
0.35,1.5\right).
\]

A further late-consumption term is applied if less than 35% of the previous useful distance remains while more than 55% of the previous energy work remains,

\[
J_{late}=4\left(\frac{0.35d_b-d_a}{d_b}\right)^2.
\]

Near the terminal footprint, if the forecast has still not captured TAEM and has passed the runway-alignment station, an additional signed progress penalty is

\[
J_{pass}=12\left[
\frac{\max(0,-d_{align,a})}{\max(20\ \mathrm{km},R_{TAEM})}
\right]^2.
\]

Thus

\[
J_{short}=J_{short,0}+J_{def}+J_{late}+J_{pass},
\]

with inactive conditional terms equal to zero. If the vehicle consumes a large amount of downrange but removes too little mechanical energy, the cost rises even when a far-future fallback could still mathematically recover. Conversely, a broad S-turn can be rewarded because it preserves downrange while real drag work accumulates. This term is what prevents receding-horizon procrastination in which every short solve decides that the next solve can perform the hard energy correction later.

If any candidate produces a safe TAEM solution while staying within 1.02 times the configured q and g limits, terminal feasibility becomes a constraint: the optimizer chooses from terminal-feasible candidates instead of allowing a cheaper near-miss trajectory to win.

### 10.4 Reversal logic

An S-turn reversal is only treated as established after the measured bank has actually captured the commanded sign and sufficient magnitude. The minimum leg timer, currently 24 s, starts from this measured bank capture, not from the moment the reversal was requested. This distinction avoids counting roll-through time as useful energy-management dwell.

The predicted reversal corridor tightens as the terminal region approaches. The predictor estimates how much range will be consumed while rolling through the reversal. A rough reversal-time model is

\[
t_{rev}\approx
\frac{|\phi_{current}|+|\phi_{next}|}{\dot\phi_{lim}}
+\frac{\dot\phi_{lim}}{\ddot\phi_{lim}}.
\]

It then evaluates the heading corridor using the projected range after that roll rather than the present range.

The final reversal is additionally energy-gated. Let \(R_{reserve}\) be the terminal join range plus the distance needed to reverse and establish the new bank. The final reversal is permitted only when the remaining TAEM energy can plausibly be removed inside the reserved path,

\[
\Delta\varepsilon_{TAEM}
\le \max(0.03,a_D)R_{reserve}(1.35).
\]

This prevents the shuttle from leaving S-turn energy management merely because it has geometrically approached KSC while still carrying too much energy for the HAC.

Once the final reversal is committed, heading metadata points to a fixed point 500 m before the runway threshold. The bank command, however, is not a direct straight-line point-capture bank. It is generated from the same dynamic HAC intercept geometry that will be used by TAEM. This avoids flying a chord through the HAC and consuming the terminal path length that was reserved for energy dissipation.

Airbrakes are explicitly prohibited during Entry/S-turn. Entry energy management is performed with bank and angle of attack so that the predictor's drag polar does not change discontinuously underneath the optimized trajectory.

## 11. Dynamic TAEM and Heading Alignment Circle

TAEM is not entered simply because the shuttle crosses 25 km altitude or 35 km range. Those values are target/reference parameters. The actual handoff occurs only when the measured aerodynamic state can support a physically feasible HAC and the remaining energy can be absorbed along that terminal path.

### 11.1 Dynamic HAC radius

The system first computes the minimum turn radius available with no more than 55 deg nominal bank and all live safety limits applied. If this minimum radius is \(R_{min}\), the terminal radius is approximately

\[
R_{HAC}=\operatorname{clip}
\left(\max(R_{base},1.30R_{min}),R_{base},R_{max}\right),
\]

where

\[
R_{max}=\max\left(
R_{base},
\min\left(0.20R_{planet},\max(8R_{base},2.5R_{TAEM})\right)
\right).
\]

If

\[
1.20R_{min}>R_{max},
\]

the current state is considered incapable of constructing a valid terminal circle and Entry/S-turn guidance continues.

### 11.2 HAC runway geometry

Let the runway threshold be the local origin, runway unit vector

\[
\hat{\mathbf a}=(\sin\psi_r,\cos\psi_r),
\]

and runway-right vector

\[
\hat{\mathbf r}=(\cos\psi_r,-\sin\psi_r).
\]

The final-approach start point is

\[
\mathbf F=-d_f\hat{\mathbf a},
\]

and for HAC side \(s\in\{-1,+1\}\), the circle center is

\[
\mathbf C=\mathbf F+sR_{HAC}\hat{\mathbf r}.
\]

For current position \(\mathbf P\), radial error is

\[
e_R=\|\mathbf P-\mathbf C\|-R_{HAC}.
\]

The remaining arc is found from the signed angular distance between the current circle angle and the final tangent angle, multiplied by \(R_{HAC}\). Live guidance integrates signed angular progress through the `atan2` branch cut, so once an exit is passed the remaining arc becomes negative rather than wrapping back to another full circle.

The desired HAC altitude is

\[
h_d=h_{rwy}+d_f\tan\gamma_f
+\left(s_{arc}+0.65e_{R,c}\right)\tan\gamma_{TAEM},
\]

with

\[
e_{R,c}=\min(|e_R|,1.5R_{HAC}).
\]

The current default final glide slope is 3.5 deg and the TAEM/HAC slope is 12 deg.

For candidate-side comparison, the HAC geometry score used by the predictor is

\[
S_{HAC}
=2.4\frac{|e_R|}{R_{HAC}}
+1.4\frac{|e_\chi|}{45^\circ}
+0.45\frac{s_{arc}}{R_{HAC}(300^\circ\pi/180)}
+0.15\min\left(\frac{d_F}{3R_{HAC}},2\right),
\]

where \(d_F\) is distance to the final-approach start point. This score is used for comparing capture quality; it is distinct from the hard TAEM feasibility gate.

### 11.3 TAEM capture gate

The geometric gate requires range to lie in a terminal annulus, HAC radial error within approximately

\[
|e_R|\le\max(2.5\ \mathrm{km},0.30R_{HAC}),
\]

and course error within 45 deg. The energy gate computes remaining energy to the desired HAC altitude and TAEM target speed. If path length is approximately

\[
s_p=s_{arc}+d_f,
\]

then removable energy is estimated as

\[
\Delta\varepsilon_{rem}\approx\max(0.03,a_D)s_p(1.35).
\]

TAEM capture requires

\[
-0.04V_{TAEM}^2\le\Delta\varepsilon\le\Delta\varepsilon_{rem},
\]

TAEM speed inside its accepted range, and altitude roughly between 6.5 km below and 12 km above the modeled HAC path. Both positive and negative HAC sides are evaluated, and the feasible side with the better capture score is latched.

### 11.4 HAC bank law

During the HAC, the look distance is

\[
L=\max(7V,0.20R_{HAC}).
\]

Nominal lateral acceleration for the circle is

\[
a_{circle}=s\frac{V^2}{R_{HAC}},
\]

and course-error capture adds

\[
a_{capture}=\frac{2V^2}{L}\sin e_\chi.
\]

Thus

\[
a_{lat}=a_{circle}+a_{capture}.
\]

Using measured lift acceleration whenever available, the bank demand is

\[
\phi_c
=\frac{1}{k_\phi}
\sin^{-1}\!\left(\frac{|a_{lat}|}{a_L}\right)
\operatorname{sgn}(a_{lat}),
\]

bounded by the live bank envelope.

### 11.5 TAEM longitudinal energy management

TAEM owns the deceleration from TAEM energy toward final-approach energy. Let

\[
V_{term}=\max(1.12V_f,1.08V_{minsafe}).
\]

The target terminal specific energy is \(\varepsilon_t\) at runway altitude and \(V_{term}\). Let \(\varepsilon_0\) be the current-location specific energy evaluated at zero airspeed. With predicted remaining drag work \(a_Ds_p\), the allowable speed is

\[
V_{allow}=\max\left[
V_{term},
\sqrt{\max\left(0,
2\left(\varepsilon_t+0.95\max(0.03,a_D)s_p-\varepsilon_0\right)
\right)}
\right].
\]

An altitude PID corrects the 12 deg nominal TAEM glide path. Angle-of-attack trim is based on learned best-glide AoA when available and is increased slightly when actual speed exceeds \(V_{allow}\). Unlike Entry/S-turn, TAEM may deploy the airbrakes when the shuttle is more than 12 m/s above allowable speed and dynamic pressure is below 90% of its limit; they are retracted when no longer needed or when q exceeds 95% of the limit.

## 12. Final-approach capture and guidance

Final approach is only accepted after the HAC exit lies inside a runway-local validity envelope. With

\[
d=\max(0,-x_{\parallel}),
\qquad
h_d=h_{rwy}+d\tan\gamma_f,
\qquad
\dot h_d=-V_h\tan\gamma_f,
\]

the exact capture gate is

\[
d_{site}<1.25d_f,
\qquad
-1.18d_f\le x_{\parallel}<L_{rwy},
\]

\[
|x_\perp|<\max\left(\frac{W_{rwy}}2,\min(350\ \mathrm{m},0.06d)\right),
\qquad
|\psi_r-\chi|<12^\circ,
\]

\[
V_{minsafe}\le V\le1.5V_f,
\qquad
|h-h_d|<\max(120\ \mathrm{m},0.06d),
\]

\[
-12^\circ<\gamma<3^\circ,
\qquad
-\max(12\ \mathrm{m/s},|\dot h_d|+6\ \mathrm{m/s})<\dot h<3\ \mathrm{m/s}.
\]

The HAC cannot silently transition to final after crossing the exit in an invalid state; an invalid exit is treated as a terminal failure.

Altitude error \(e_h=h_d-h\) is passed through a robust PID and supplemented by sink-rate error. The result modifies commanded flight-path angle around the nominal \(-\gamma_f\).

Lateral guidance is an L1-like centerline capture. The look distance is

\[
L_1=\operatorname{clip}(5.5V,450,2800)\ \mathrm{m}.
\]

The intercept angle is

\[
\delta_\chi
=\operatorname{clip}\left[
\tan^{-1}\left(\frac{-x_\perp}{L_1}\right),-28^\circ,28^\circ
\right],
\]

so target course is

\[
\chi_c=\psi_r+\delta_\chi.
\]

With course error \(e_\chi=\chi_c-\chi\), lateral acceleration demand is

\[
a_{lat}=\frac{2V_h^2}{L_1}\sin e_\chi,
\]

and final-approach bank is approximately

\[
\phi_c=\tan^{-1}\left(\frac{a_{lat}}{g}\right),
\]

with a bank limit that decreases toward the runway and decreases further near flare height.

The speed target is

\[
V_c=V_f+\operatorname{clip}\left(\frac d{420},0,24\right).
\]

The longitudinal controller is a simplified TECS-like energy balance. Define

\[
E_V=\frac12(V_c^2-V^2),
\qquad
E_h=ge_h,
\]

\[
E_T=E_h+E_V,
\qquad
B=\operatorname{clip}\left(\frac{E_h-E_V}{500g},-4,4\right).
\]

The balance term \(B\) biases pitch/path allocation between altitude and speed. Airbrakes are controlled from total energy and excess speed. If powered approach is enabled, throttle is generated by a speed PID with a small altitude-error bias. The final pitch request is the sum of commanded flight-path angle and a bounded best-glide/airspeed trim.

## 13. Flare and touchdown

Let normalized flare height be

\[
f_h=\operatorname{clip}\left(\frac{h_{radar}}{h_{flare}},0,1\right).
\]

The pre-flare approach sink target is

\[
\dot h_{app}=-\max(2.5,V_h\tan\gamma_f),
\]

and desired sink is blended toward touchdown sink rate with

\[
\dot h_c=\dot h_{TD}+(\dot h_{app}-\dot h_{TD})\sqrt{f_h}.
\]

With the current configuration, \(\dot h_{TD}=-1.5\ \mathrm{m/s}\) and nominal flare height is 55 m. A dedicated sink-rate PID supplies flare correction.

Flare speed is scheduled between touchdown speed and the top-of-flare speed,

\[
V_{top}=\max(V_f,1.05V_{minsafe}),
\]

\[
V_c=V_{TD}+(V_{top}-V_{TD})f_h^{0.65}.
\]

The commanded flare angle of attack is based on learned best-glide AoA plus a height-dependent nose-up increment, with a low-speed reduction that prevents an attempted flare from driving the vehicle below its survivable speed floor.

Centerline control continues with a shorter look distance,

\[
L_f=\operatorname{clip}(3.5V,220,900)\ \mathrm{m},
\]

and bank authority reduces to approximately

\[
|\phi|\le4^\circ+6^\circ\sqrt{f_h}.
\]

After ground contact, wheel steering uses runway heading error and cross-track error,

\[
A_s=\operatorname{clip}\left(\frac{55}{\max(V_g,15)},0.25,1\right),
\]

\[
u_{steer}=\operatorname{clip}
\left[(0.03e_\psi-0.0011x_\perp)A_s,-1,1\right].
\]

Airbrakes are deployed for rollout, wheel brakes are enabled below the configured high-speed threshold, and landing is considered complete below approximately 1.5 m/s surface speed.

## 14. Command shaping and robust PID implementation

Guidance outputs are not sent as instantaneous attitude steps. Pitch/AoA, roll, heading, and throttle pass through rate, acceleration, and slew limiters. The angular jerk-limiter computes

\[
\dot x_{stop}=\sqrt{2a_{max}|e|},
\]

\[
\dot x^*=\operatorname{sgn}(e)
\min\left(v_{max},\dot x_{stop},\frac{|e|}{\Delta t}\right),
\]

then limits the change of rate,

\[
\dot x\leftarrow\dot x+
\operatorname{clip}(\dot x^*-\dot x,-a_{max}\Delta t,a_{max}\Delta t),
\]

\[
x\leftarrow x+\dot x\Delta t.
\]

For atmospheric phases, pitch shaping is performed in angle-of-attack space rather than raw surface Euler pitch. If the C guidance command transports target pitch as

\[
\theta_{transport}=\gamma+\alpha_c,
\]

the command shaper extracts

\[
\alpha_c=\theta_{transport}-\gamma,
\]

limits \(\alpha_c\), then reconstructs the transport field. This avoids a high-bank Euler projection error that previously caused commanded aerodynamic incidence to diverge badly from the predicted AoA.

The robust PID implementation uses a filtered derivative,

\[
D_{raw}=\frac{e_k-e_{k-1}}{\Delta t},
\qquad
\alpha_D=\frac{\Delta t}{\Delta t+\tau_D},
\]

\[
D_k=D_{k-1}+\alpha_D(D_{raw}-D_{k-1}),
\]

and

\[
u=K_Pe+K_II+K_DD_k.
\]

The integrator is bounded and conditionally updated so that saturation does not continue winding it in the wrong direction.

Current principal PID gains are approximately: TAEM altitude \((K_P,K_I,K_D)=(0.0026,0.000035,0.005)\), final altitude \((0.005,0.00006,0.012)\), powered-approach speed \((0.02,0.0018,0.006)\), and flare sink \((0.75,0.08,0.22)\). These are outer-loop corrections; low-level attitude control is separate.

## 15. Direct atmospheric flight-control loop

After atmospheric interface, the Python bridge directly drives `control.pitch`, `control.roll`, and `control.yaw`. The loop is deliberately authority-aware. It learns how much angular acceleration one unit of control currently produces rather than imposing a fixed Mach/q gain schedule.

For each axis, available angular acceleration is modeled as

\[
A(q)=A_0+k_q q,
\]

where \(A_0\) is non-aerodynamic authority and \(k_q\) is learned aerodynamic authority per unit dynamic pressure. The estimator observes measured rate change,

\[
a_{obs}=\frac{\omega_k-\omega_{k-1}}{\Delta t},
\]

subtracts slowly learned drift acceleration, and divides by the previous normalized control input. In near vacuum, meaningful control excitation updates \(A_0\). In atmosphere, authority above the baseline is attributed to the \(k_q q\) component. An overoptimistic baseline can be corrected downward from loaded flight data.

### 15.1 Adaptive rate command

Given attitude error \(e\), measured angular rate \(\omega\), authority \(A\), capture factor \(c\), and response time \(\tau\), the low-level loop computes a time-optimal-like rate bound

\[
\omega_{TO}=c\sqrt{2A|e|},
\]

and a linear small-error settling rate

\[
\omega_{lin}=\frac{|e|}{1.5\tau}.
\]

The target rate is

\[
\omega^*=\operatorname{sgn}(e)\min(\omega_{TO},\omega_{lin}).
\]

With optional steady bias acceleration \(a_b\),

\[
a^*=a_b+\frac{\omega^*-\omega}{\tau},
\qquad
u=\operatorname{clip}\left(\frac{a^*}{A},-1,1\right).
\]

Separate response profiles are used for entry, TAEM, approach, flare, and recovery, but authority comes from the measured vehicle rather than from a hard-coded pressure switch.

### 15.2 AoA control and trim learning

For Entry, TAEM, Approach, and Flare, the bridge closes the pitch loop directly on measured AoA. The commanded aerodynamic target is

\[
\alpha_c=\theta_{transport}-\gamma.
\]

An integral trim is learned in normalized control space,

\[
u_{trim}\leftarrow
\operatorname{clip}
\left(u_{trim}+0.018w_\alpha e_\alpha\Delta t,-0.95,0.95\right),
\]

where \(w_\alpha\) rises with measured aerodynamic ownership or with dynamic pressure. Keeping trim in normalized actuator space is important: if it were stored as an angular acceleration, a changing online authority estimate would unintentionally change the steady elevator command needed to hold the same AoA.

At high bank angle the bridge increasingly blends away from surface Euler pitch/roll rates toward measured body-axis rates. This avoids the near-knife-edge Euler singularity in which ordinary course motion can appear as an enormous roll-rate signal even when body roll is quiet.

### 15.3 Sideslip and yaw controller

In loaded atmospheric flight, yaw is a coordination actuator rather than the main heading-steering actuator. Bank controls the flight path; yaw primarily damps sideslip. The bridge learns the local relation

\[
\dot\beta\approx g_y\,r+g_r\,p,
\]

where \(r\) is body yaw rate and \(p\) body roll rate. It maintains centered exponentially weighted covariance terms and solves the regularized two-variable least-squares system

\[
\begin{bmatrix}
S_{yy}+\lambda & S_{yr}\\
S_{yr} & S_{rr}+\lambda
\end{bmatrix}
\begin{bmatrix}g_y\\g_r\end{bmatrix}
=
\begin{bmatrix}S_{y\beta}\\S_{r\beta}\end{bmatrix}.
\]

This lets the controller discover the actual sign of yaw-to-beta response instead of assuming that a particular rudder sign always reduces sideslip.

The desired beta rate is approximately

\[
\dot\beta_c
=\operatorname{clip}\left(-\frac{\beta}{\tau_\beta},-10,10\right),
\]

so desired yaw rate is

\[
r_c=\frac{\dot\beta_c-g_rp}{g_y}.
\]

The resulting yaw-acceleration request is normalized by yaw authority and blended by regression confidence, aerodynamic surface ownership, and current sideslip/yaw magnitude. During thin-air entry, heading/RCS control owns direction; as aerodynamic surfaces gain authority, heading-yaw authority fades and beta/rudder authority takes over. During TAEM and final, yaw does not chase runway heading directly; bank does that job.

### 15.4 RCS handoff

RCS remains available through hypersonic and supersonic entry. For atmospheric control profiles it is deliberately kept on until the transonic handoff. The bridge latches RCS off at

\[
M\le1.2
\]

and resets the latch only if a later/restarted test returns to

\[
M\ge1.5.
\]

This hysteresis prevents TAEM or approach from accidentally re-enabling RCS after the intended transonic cutoff.

### 15.5 Physics-warp and delayed-frame compensation

Direct controls are sample-and-hold. A command that remains active for a longer simulation-time interval produces more angular impulse. The bridge therefore scales the transient portion of pitch/roll/yaw by

\[
s_h=\operatorname{clip}
\left(\frac{\Delta t_{nom}}
{\max(\Delta t_{meas},\Delta t_{expected\ warp})},0,1\right).
\]

Steady aerodynamic pitch trim is not scaled, because trim balances a continuous aerodynamic moment rather than a transient command impulse.

## 16. Attitude-departure detection and recovery

The terminal controller monitors actual body motion, control saturation, bank excess, sideslip growth, and pitch/yaw departure. It deliberately does not treat generic kRPC AutoPilot error or large commanded bank error as a departure, because those are normal during an S-turn reversal.

Euler roll rate is geometrically down-weighted near knife-edge and body roll rate is preferred when available. Sideslip magnitude alone is not enough to trigger recovery; it must be associated with growing beta, body yaw motion, saturation, or another genuine departure indicator.

When recovery is triggered, normal terminal progression is inhibited. The command becomes approximately wings level, zero throttle, prograde/ground-track heading, and a moderate entry AoA. Recovery is not released merely because angular rates become small. Pitch must also be recaptured near the intended recovery target. The stable-release conditions include pitch within about 5 deg of target, roll and pitch rates below about 5 deg/s, bank below about 10 deg, quiet yaw/beta behavior when aerodynamically loaded, and roughly 2 s of continuous stability. Recovery is aborted to manual control if it persists about 15 s or if insufficient height remains.

## 17. Current default reference configuration

The current default landing site is KSC Runway 09 with the runway threshold at latitude -0.0486111111 deg, longitude -74.7283333333 deg, altitude 70 m, heading 090 deg, length 2500 m, and modeled width 70 m.

The default shuttle profile uses an initial estimated \(L/D=0.4\), ballistic coefficient 700, entry AoA 18 deg, maximum AoA 28 deg, maximum bank 70 deg, maximum dynamic pressure 45 kPa, maximum g-load 3.5 g, minimum safe speed 85 m/s, final-approach speed 115 m/s, and touchdown speed 75 m/s. Powered approach is permitted with maximum approach throttle 0.45.

Guidance defaults include a 100 km deorbit capture radius, nominal 1030 km entry-range scale, target entry flight-path angle -2.0 deg, maximum accepted entry steepness -3.0 deg, target post-burn periapsis 50 km, 18% maximum deorbit throttle, 12 s throttle ramp, nominal TAEM reference altitude 25 km, nominal TAEM range 35 km, 12 km base HAC radius, 8 km final-approach distance, 12 deg TAEM glide slope, 3.5 deg final glide slope, 1500 m gear-deployment altitude, 55 m flare altitude, and -1.5 m/s touchdown sink target.

The nominal guidance loop is 10 Hz, the atmospheric prediction interval is 1.5 s, minimum established S-turn leg duration is 24 s, entry roll rate limit is 4.5 deg/s with 1.8 deg/s^2 roll acceleration, TAEM roll rate is 6 deg/s, and final-approach roll rate is 3.5 deg/s.

## 18. Important implementation limits

The current system is a model-predictive guidance and adaptive-control implementation for KSP, not a high-fidelity reproduction of NASA Shuttle GNC. Its predictor uses spherical gravity and a reduced-order aerodynamic model; the actual game still supplies the full part-level KSP physics. The current implementation does not yet parse the `.craft` file to derive exact geometry, fuel-dependent center of mass, per-part lift/drag, or control derivatives. Those effects enter indirectly through live measured forces, learned Mach-envelope parameters, trajectory residuals, bank effectiveness, and low-level control-authority identification.

The default atmosphere and calibration assumptions are centered on stock Kerbin. The connected predictor uses the body's live pressure/density table, so ordinary atmospheric variation is handled correctly, but a rescaled planet, strongly altered aerodynamics, FAR-like replacement aerodynamics, or a materially different shuttle configuration should be recalibrated and revalidated rather than assumed compatible with the default coefficients.

The configured TAEM altitude and range are reference energy/geometry parameters, not fixed phase-switch triggers. Likewise, `targetEntryRange` is no longer a hard landing objective. The current architecture is deliberately based on predicted energy dissipation and dynamically achievable terminal geometry because a shuttle can pass close to KSC while still being far too fast, too high, too low, or physically unable to turn onto the runway.

## 19. Summary of the guidance philosophy

The current landing program can be summarized as a hierarchy of physical feasibility checks. The deorbit planner chooses a burn that survives a deterministic uncertainty envelope. The atmospheric predictor propagates the actual rotating-atmosphere energy state using a continuously calibrated reduced-order aerodynamic model. Entry MPC chooses a broad S-turn radius and AoA based on how much mechanical energy must still be removed per metre of useful downrange. S-turn reversals are propagated as committed trajectory events rather than improvised heading-PID sign changes. The final reversal is allowed only when a dynamic HAC can absorb the remaining energy. TAEM begins only when that HAC is geometrically, aerodynamically, and energetically feasible. Final approach is accepted only from a valid runway-local envelope, and flare is accepted only when centerline, descent, and speed remain survivable.

The low-level controller follows the same philosophy. It does not assume fixed pitch/roll/yaw effectiveness. It learns angular authority from measured vehicle response, closes atmospheric pitch on AoA instead of Euler pitch, uses bank as the principal steering actuator, learns the sign of yaw-to-sideslip coupling, keeps RCS available until the transonic handoff, and supervises the vehicle for genuine control departure. The result is a system in which the trajectory model, terminal geometry, and actuator behavior are all tied back to the state KSP is actually producing rather than to a single precomputed nominal descent profile.
