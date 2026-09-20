# STS-N Launcher Thrust-Vector / Asymmetry Audit

This audit supersedes the centered-Cougar trim numbers in the first launcher draft. It uses the patched GameData values, saved STS-N mass distribution, live kRPC measurement of the J-N500 thrust axis, the exact PhotonCorp RSRB-5 thrust curve, and a pressure sweep from vacuum to 1 atm. It is a propulsion-only audit: aerodynamic moments are deliberately not credited as control authority.

## Coordinate model and hard assumptions

The vehicle longitudinal axis is the nominal +thrust axis. The orbiter/ET offset lies in the pitch plane. The two RSRB-5s are mounted at equal and opposite lateral coordinates, so equal booster thrust cannot create yaw. The J-N500 gimbal is locked and its force is treated as a fixed vector. The launch-prepared STS-N thrust line is about 0.1116 m off its own launch-prepared CoM in the pitch plane. The ET/orbiter centerline spacing is modeled at 4.55 m pending exact VAB measurement.

The equilibrium condition at every fuel state is the full moment balance

`sum((r_engine - r_CoM) x F_engine) = 0`.

No reaction wheel, RCS, or aerodynamic torque is counted in the nominal launcher balance.

## Important design correction: move the Cougar 0.14 m away from the orbiter

A centered Cougar is safe, but it is not the best thrust-line geometry. With the physically reachable post-SRB ET fuel range (171.44 t down to empty), a centered Cougar needs about -3.25 to +2.18 degrees of pitch gimbal and tilts the resultant thrust vector as much as 2.34 degrees from the stack axis.

Shifting the Cougar centerline only **0.141 m away from STS-N**, while keeping zero permanent cant, centers the required engine-vector range almost perfectly:

- required Cougar pitch direction: approximately **-2.76 to +2.75 deg** over the complete reachable post-SRB fuel range and 0..1 atm pressure sweep;
- maximum resultant-thrust tilt: approximately **1.99 deg**;
- worst axial-thrust loss from trim: approximately **0.084%**;
- available Cougar gimbal: +/-6 deg, so at least about **3.24 deg** of nominal pitch authority remains.

The optimum offset is not fragile. For plausible final ET/orbiter centerline spacing, the minimax offset is approximately:

| ET-orbiter centerline | optimal Cougar offset away from orbiter | max required Cougar angle |
|---:|---:|---:|
| 3.80 m | 0.125 m | 2.30 deg |
| 4.00 m | 0.130 m | 2.42 deg |
| 4.25 m | 0.135 m | 2.57 deg |
| 4.55 m | 0.142 m | 2.76 deg |
| 4.75 m | 0.146 m | 2.88 deg |
| 5.00 m | 0.151 m | 3.03 deg |
| 5.25 m | 0.157 m | 3.18 deg |
| 5.50 m | 0.162 m | 3.33 deg |

Therefore the VAB procedure should attach the Cougar normally, then use the offset gizmo to move it about 0.14 m on the ET pitch axis **away from the orbiter**. If the exact VAB spacing differs, use the table/interpolated value and rerun the final measurement. If node-offset constraints prevent this, a centered Cougar is still within authority; it simply has less margin.

## Nominal time-varying thrust vector

With the 0.141 m Cougar offset and RSRB gimbal limiter at 60%, representative robust pressure-endpoint values are:

| elapsed time | state | Cougar pitch | RSRB pitch | resultant thrust tilt | max axial loss |
|---:|---|---:|---:|---:|---:|
| 0 s | liftoff | -0.34..-0.40 deg | -0.26..-0.30 deg | -0.25..-0.29 deg | 0.0015% |
| 40 s | boost | -0.50..-0.56 deg | -0.37..-0.42 deg | -0.37..-0.41 deg | 0.0029% |
| 80 s | boost | +0.05..+0.11 deg | +0.04..+0.08 deg | +0.04..+0.08 deg | 0.0001% |
| 120 s | late boost | -0.26..-0.32 deg | -0.20..-0.24 deg | -0.19..-0.24 deg | 0.0010% |
| 130 s | SRB tail | -1.14..-1.18 deg | -0.85..-0.88 deg | -0.83..-0.86 deg | 0.0142% |
| 135 s | SRB tail | -1.28..-1.31 deg | -0.96..-0.98 deg | -0.93..-0.96 deg | 0.0177% |
| just post-SRB | powered ET | -2.71..-2.76 deg | n/a | -1.96..-1.99 deg | 0.0836% |
| 200 s | powered ET | -2.07..-2.12 deg | n/a | -1.50..-1.53 deg | 0.0494% |
| 280 s | powered ET | -0.64..-0.69 deg | n/a | -0.46..-0.50 deg | 0.0052% |
| 300 s | powered ET | -0.05..-0.10 deg | n/a | -0.04..-0.07 deg | 0.0001% |
| 320 s | 2g-capped | +0.63..+0.68 deg | n/a | +0.46..+0.49 deg | 0.0050% |
| 340 s | 2g-capped | +1.50..+1.54 deg | n/a | +1.08..+1.12 deg | 0.0261% |
| 360 s | near empty | +2.52..+2.57 deg | n/a | +1.82..+1.86 deg | 0.0726% |
| 363.24 s | theoretical ET empty | +2.71..+2.75 deg | n/a | +1.96..+1.99 deg | 0.0834% |

The resultant vector is not parallel to the airframe even when torque is exactly zero. Guidance must therefore steer the **resultant thrust vector**, not blindly assume the body longitudinal axis equals the acceleration vector. The required body/thrust bias remains below about 2 degrees in the modeled geometry.

The late 2g throttle cap changes elapsed time but not trim at a given fuel state because Cougar and J-N500 are scaled by the same throttle. Full-throttle-equivalent ET depletion is 355.190 s, but with the specified 2g cap beginning at about 304.496 s, theoretical ET depletion moves to about **363.242 s**.

## Booster symmetry and direct thrust mismatch

With the RSRBs placed at +/-4.45 m laterally, equal thrust and equal pitch gimbal cancel yaw and roll exactly in the ideal geometry. Differential axial thrust primarily produces **yaw**, not pitch. The roll component created by differential thrust while the boosters are pitch-gimballed is smaller by approximately `tan(|booster pitch|)`; with nominal booster pitch around or below 1 degree before separation, roll asymmetry is only on the order of 1-2% of the yaw asymmetry.

Using the 60% RSRB gimbal limiter and conservatively reserving gimbal cone for simultaneous nominal pitch trim, a direct pair thrust/limiter mismatch produces approximately:

- 1% pair mismatch at liftoff: 0.154 MNm yaw disturbance, about 1.4% shared yaw command;
- 5% pair mismatch: about 6.8% shared yaw command;
- 10% pair mismatch: about 13.5% shared yaw command.

The sensitivity becomes smaller in the SRB tail as thrust decays. A huge radial placement error of about 1.64 m would be required to consume 50% yaw command at liftoff; a realistic 0.05 m left/right radius error corresponds to only about 1.5% yaw command. Normal VAB symmetry placement is therefore more than adequate.

## The dangerous asymmetry is unequal SRB burnout

The PhotonCorp thrust curve remains at roughly 15% of nominal thrust immediately before fuel reaches zero. If one booster flames out while the other still has even a tiny residual, the surviving booster instantly leaves about 545 kN of off-axis vacuum thrust at the tail. That is much more severe than an ordinary few-percent limiter mismatch.

If one booster is full and the other is deliberately underfilled, retaining both dry casings until separation, the exact curve integration gives:

| initial fuel mismatch | burnout-time split | worst shared yaw command, 60% RSRB gimbal |
|---:|---:|---:|
| 0.10% | 0.094 s | about 63% |
| 0.50% | 0.466 s | about 63% |
| 1.00% | 0.932 s | about 63% |
| 2.00% | 1.854 s | about 64% |
| 5.00% | 4.546 s | about 69% |

Those cases are statically trimmable, but waiting for a one-sided flameout is unnecessary. A conservative point-mass inertia model, with **no corrective gimbal at all**, bounds a 0.10% mismatch tail at about 0.10 deg yaw excursion; 0.5% can already reach a few degrees before correction, and 1% can become severe. The actual distributed ET/SRB inertia is higher than the point-mass model, so these are conservative transient angles.

**Staging rule:** do not wait for flameout. Separate both RSRBs together when the lower of the two booster fuel masses reaches about **0.27 t**, corresponding to about T+134.0 s in the symmetric nominal case. At that point each nominal booster still has only about 0.22% of its initial fuel and about 0.7 MN*s of residual ideal impulse; discarding it is a negligible performance penalty and removes the one-sided-burnout hazard. In automated guidance, trigger on `min(left_solid_fuel, right_solid_fuel) <= 0.27 t`, not on an average or total fuel quantity.

The Cougar's pitch trim jumps by about 1.42 deg when the boosters are dropped at T+134 s. Its installed gimbal response speed is 8 deg/s, so the commanded transition takes about 0.18 s. A conservative point-mass inertia estimate gives only about 0.076 deg of pitch excursion during that slew. This separation transient is acceptable.

The RSRB gimbal response speed is 15 deg/s. A 60% limiter remains preferred: increasing it to 80-100% only improves the worst unequal-burnout yaw command from roughly 63% to roughly 60-57%, while materially increasing booster control authority and potential high-q over-control. The 60% setting is sufficient for realistic mismatch while keeping better control resolution.

A **complete early one-SRB thrust loss is not an accepted engine-out case**. With the failed booster still carrying its fuel mass, a single remaining full/thrust-curve SRB cannot be trimmed by the Cougar + remaining RSRB through most of the burn. At 60% RSRB gimbal it only becomes statically recoverable in approximately the last 9-10 seconds of the burn. Therefore an early RSRB failure requires immediate thrust cut/abort logic rather than continued nominal ascent.

## Liquid-engine ratio asymmetry

The J-N500 ClosedCycle and Cougar must stay on the same effective throttle ratio. Their throttle response is effectively immediate in the installed configs; only the airbreathing J-N mode has spool response. Therefore the launcher must start and remain in **ClosedCycle** throughout powered ascent and must not expose a mode-toggle action to normal guidance commands.

With the Cougar shifted 0.141 m away from the orbiter, the nominal fixed-J-N balance is +/-about 2.75 deg. If J-N thrust degrades while the Cougar remains nominal, the Cougar can compensate for substantial ratio error, but not a late total J-N flameout. With J-N completely off, required Cougar gimbal reaches its 6 deg hard limit at approximately **T+319.97 s**, when about **27.93 t of ET propellant** remains under the 2g-capped timeline. At theoretical empty ET it would require about 7.94 deg and is impossible with the Cougar.

Contingency: a J-N flameout after about T+320 s must command common throttle to zero and transition to separation/coast/abort logic; do not continue Cougar-only thrust with the ET attached. A 50% J-N underthrust remains barely trimmable through ET empty (about 5.35 deg required at the end), while 80% nominal remains comfortably inside the envelope.

A Cougar failure while J-N remains at full thrust is also not a continuable case: the fixed orbiter engine is far off the attached stack CoM. The correct response is immediate throttle zero, not an attempt to fly on the J-N alone with the ET attached.

## ET separation and the fixed J-N500

The currently installed J-N500 actually has a 2 deg ModuleGimbal, but this launcher intentionally locks it to satisfy the zero-TVC requirement. Live kRPC measured the current un-gimballed thrust line about 0.101 m from the current orbiting vehicle CoM; the launch-prepared saved craft gives about 0.112 m offset. At 820 kN that is about **91 kN*m** of pitch moment for the launch-prepared orbiter if it fires alone at full vacuum thrust without TVC.

The orbiter's reaction wheel is only 20 kN*m pitch/yaw, and the live combined reaction-wheel + RCS attitude torque is only about 25 kN*m in the stronger measured direction. That is not enough to statically cancel ~91 kN*m at full J-N thrust.

Therefore **ET separation must occur at zero throttle if the J-N500 is kept truly non-gimballed**. Complete orbital insertion with the powered ET, throttle both liquid engines to zero, separate the ET, and only then hand over to the orbiter's own propulsion/control strategy. Do not separate the ET while the fixed J-N is still producing full thrust and assume the orbiter will remain torque-balanced.

This is a separate STS-N propulsion-alignment issue, not a flaw in the powered-ET launcher. If later full-thrust post-ET J-N burns are required with TVC prohibited, STS-N itself needs its fixed engine line/CoM aligned or an RCS/auxiliary trim system with substantially more than the present ~25 kN*m authority.

## Final acceptance criteria

The launch stack is acceptable only if the actual VAB geometry reproduces these properties with J-N gimbal locked:

1. Cougar shifted about 0.14 m away from STS-N (adjust to measured centerline spacing); no permanent cant required.
2. Nominal Cougar pitch trim stays below 4 deg at every fuel snapshot; target from this model is <=2.8 deg.
3. Nominal resultant-thrust/body-axis offset stays below 2.5 deg; target is <=2.0 deg.
4. Left/right RSRB fuel, thrust limiter, and gimbal limiter are identical and edited through symmetry.
5. RSRB separation is triggered before first flameout, preferably at min-side SolidFuel mass <=0.27 t (~T+134 s nominal).
6. ET separation is performed at zero throttle when J-N TVC is locked.
7. Any J-N loss after ~T+320 s or any early complete RSRB loss causes abort/cutoff logic rather than continued nominal thrust.
