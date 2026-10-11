# KSP STS Thrust-Vector and Engine-Throttle Analysis

**Stock shuttle + external tank + dual Pollux boosters**

- **Vehicle source:** \`STS.craft\` / \`STS.loadmeta\`
- **KSP version:** 1.12.5
- **Loaded vehicle:** 60 parts, 459.45401 t editor mass

## Purpose

Determine how the Skiff, four Bobcats, and Cheetah should be throttled so that the combined thrust line follows the moving vehicle center of mass (CoM), while both Pollux boosters remain at fixed thrust until burnout and the shuttle propellant load is kept full by transferring propellant from the external tank (ET).

> **Key result:** this is a constrained thrust-allocation problem, not a single fixed throttle ratio. At each instant, engine throttles should be solved from the current CoM and current available thrust. The time schedule below is a useful reference curve, but the recommended flight implementation is closed-loop CoM-based allocation.

## 1. Vehicle Configuration

**Shuttle:** one Skiff main engine. Shuttle propellant mass is treated as constant because ET propellant is transferred to keep the shuttle tank full.

**External tank:** four Bobcats arranged as two engines on the shuttle side and two on the opposite side, plus one Cheetah close to the ET centerline.

**Boosters:** two Pollux solid boosters mounted on opposite sides of the ET. Their thrust is fixed until burnout.

**Control grouping:** the two shuttle-side Bobcats share one throttle command; the two opposite-side Bobcats share another. This preserves left/right symmetry while allowing the two Bobcat rows to generate pitch moment.

### Engine-line geometry extracted from \`STS.craft\`

| Engine group | Count | Craft z (m) | Arm from z0 (m) | Throttle variable |
|---|---:|---:|---:|---|
| Skiff | 1 | -2.55042505 | -3.80000793475 | \`uS\` |
| Near Bobcats | 2 | 0.40958166 / 0.40958214 | -0.84000098525 | \`uN\` |
| Far Bobcats | 2 | 2.08958387 / 2.08958387 | +0.84000098525 | \`uF\` |
| Cheetah | 1 | 1.24958229 | -0.00000059475 | \`uC\` |
| Pollux | 2 | 1.24957943 / 1.24957991 | -0.00000321475 mean | fixed |

The reference centerline is

\[
z_0 = 1.24958288475\ \text{m}
\]

which is the midpoint between the two Bobcat rows. Define

\[
c = z_{\mathrm{CoM}} - z_0
\]

Negative \(c\) means the CoM lies toward the shuttle.

## 2. Exact Thrust-Moment Model

Let \(S\), \(B\), \(C\), and \(P_b\) denote the 100% available thrust of one Skiff, one Bobcat, one Cheetah, and one Pollux at the current atmospheric condition.

For the paired engines,

\[
N = 2B,\qquad F = 2B,\qquad P = 2P_b
\]

and the thrust-line offsets are

\[
s=-3.80000793475
\]

\[
n=-0.84000098525
\]

\[
f=+0.84000098525
\]

\[
h=-5.9475\times 10^{-7}
\]

\[
p=-3.21475\times 10^{-6}
\]

with all distances in metres relative to \(z_0\).

With parallel thrust axes and no gimbal contribution, the pitch-axis moment is

\[
\tau_x
=
(s-c)Su_S
+
(n-c)Nu_N
+
(f-c)Fu_F
+
(h-c)Cu_C
+
(p-c)P
\]

For zero commanded pitch moment,

\[
\tau_x = 0
\]

For a deliberate pitch command,

\[
\tau_x = \tau_{\mathrm{cmd}}
\]

A first-order angular-acceleration command can use

\[
\tau_{\mathrm{cmd}} = I_x\alpha_{\mathrm{cmd}}
\]

with the sign chosen consistently with the vehicle coordinate convention.

The equivalent thrust-center expression is

\[
z_{\mathrm{thrust}}-z_0
=
\frac{
sSu_S+nNu_N+fFu_F+hCu_C+pP
}{
Su_S+Nu_N+Fu_F+Cu_C+P
}
\]

The steady-thrust-line condition is therefore

\[
z_{\mathrm{thrust}}-z_0=c
\]

There are four liquid-engine throttle groups but only one pitch-moment equality constraint, so the solution is not unique until an allocation objective is chosen. The allocation used here is to **maximize total liquid-engine thrust** subject to the moment constraint and

\[
0\le u_i\le1
\]

## 3. Sea-Level Piecewise Throttle Solution

Using the stock sea-level thrust values used in the calculation:

- Skiff: \(S=240.909\ \text{kN}\)
- One Bobcat: \(B=374.194\ \text{kN}\)
- Cheetah: \(C=52.817\ \text{kN}\)
- One Pollux: \(P_b=1155.556\ \text{kN}\)

Therefore

\[
N=F=748.388\ \text{kN}
\]

\[
P=2311.112\ \text{kN}
\]

The sea-level branch boundaries are

\[
c_0=-0.4678541958\ \text{m}
\]

\[
c_1=-0.4604849867\ \text{m}
\]

\[
c_2=-0.2231959352\ \text{m}
\]

\[
c_3=-0.0000019326\ \text{m}
\]

### Region 1: \(c_0\le c\le c_1\)

\[
u_S=1,\qquad u_N=1,\qquad u_F=0
\]

\[
u_C=
-\frac{(s-c)S+(n-c)N+(p-c)P}{(h-c)C}
\]

### Region 2: \(c_1\le c\le c_2\)

\[
u_S=u_N=u_C=1
\]

\[
u_F=
-\frac{(s-c)S+(n-c)N+(h-c)C+(p-c)P}{(f-c)F}
\]

### Region 3: \(c_2\le c\le c_3\)

\[
u_N=u_F=u_C=1
\]

\[
u_S=
-\frac{(n-c)N+(f-c)F+(h-c)C+(p-c)P}{(s-c)S}
\]

> **Feasibility limit while Pollux is burning:** if \(c<-0.467854\) m at sea level, throttle allocation alone cannot move the thrust center far enough toward the shuttle. The minimum-moment saturation is Skiff 100%, near Bobcats 100%, far Bobcats 0%, Cheetah 0%. Remaining pitch moment must be handled by gimbal, lower booster thrust, revised engine placement, or a different mass distribution.

At \(c=-0.400\) m, for example:

- Skiff: 100%
- Near Bobcat pair: 100% each
- Far Bobcat pair: approximately 21.86% each
- Cheetah: 100%

## 4. After Pollux Burnout

At burnout, set

\[
P=0
\]

Booster dry mass still affects the CoM until separation, but it contributes no thrust.

One additional low-\(c\) branch is

\[
u_S=1,\qquad u_F=0,\qquad u_C=0
\]

\[
u_N=
-\frac{(s-c)S}{(n-c)N}
\]

The remaining branches use the same equations as above with \(P=0\).

In a real controller, branch selection should come from instantaneous CoM and available-thrust values rather than from time alone.

## 5. Reference Throttle Schedule vs. Launch Time

This is the first-order time-domain reference curve developed from the simplified mass-depletion model used in the analysis.

Assumptions used for this reference schedule:

- Shuttle propellant mass remains constant.
- ET propellant supplies both the shuttle top-off flow and ET liquid engines.
- Pollux thrust remains fixed until approximately 73.83 s.
- Liquid-engine burn modelling uses the stock vacuum-class thrust / Isp values used in the estimate.
- The curve is a reference, not a replacement for closed-loop CoM allocation.

### Reference graph

\`\`\`mermaid
xychart-beta
    title "STS Engine Target Throttle vs Launch Time"
    x-axis "Elapsed time from launch (s)" [0, 73.83, 100, 150, 200, 246.32, 260, 279.252, 300, 350, 400, 455.242]
    y-axis "Throttle (%)" 0 --> 100
    line "Skiff" [100,100,100,100,100,100,100,100,100,100,100,100]
    line "Near Bobcat pair, each" [100,100,100,100,100,100,100,100,83.42,55.43,37.81,23.69]
    line "Far Bobcat pair, each" [0,52.06,44.66,29.93,14.59,0,0,0,0,0,0,0]
    line "Cheetah" [0,100,100,100,100,100,57.29,0,0,0,0,0]
    line "Pollux pair" [100,0,0,0,0,0,0,0,0,0,0,0]
\`\`\`

The discontinuity at 73.83 s is Pollux burnout.

### Piecewise reference expressions

Skiff:

\[
u_S(t)=1.0,\qquad 0\le t\le455.242
\]

Pollux:

\[
u_P(t)=
\begin{cases}
1,&t\le73.83\\
0,&t>73.83
\end{cases}
\]

Near Bobcats:

\[
u_N(t)=1,\qquad t\le279.252
\]

Define

\[
q=\frac{t-279.252}{455.242-279.252}
\]

Then

\[
u_N(t)
=
-0.40470789q^3
+
1.12997148q^2
-
1.48229471q
+
0.99391264
\]

for \(279.252\le t\le455.242\).

Far Bobcats:

\[
u_F(t)=0
\]

for \(t<73.83\) and \(t>246.32\).

Define

\[
r=\frac{t-73.83}{246.32-73.83}
\]

Then

\[
u_F(t)
=
0.01277464r^3
-
0.05335746r^2
-
0.48008335r
+
0.52060889
\]

for \(73.83\le t\le246.32\).

Cheetah:

\[
u_C(t)=0,\qquad t<73.83
\]

\[
u_C(t)=1,\qquad 73.83\le t\le246.32
\]

Define

\[
v=\frac{t-246.32}{279.252-246.32}
\]

Then

\[
u_C(t)
=
0.04805025v^2
-
1.04786244v
+
0.99986307
\]

for \(246.32\le t\le279.252\), and

\[
u_C(t)=0
\]

afterward.

### Selected schedule points

| Time | Skiff | Near Bobcat | Far Bobcat | Cheetah | Pollux |
|---:|---:|---:|---:|---:|---:|
| 0 s | 100% | 100% | 0% | 0% | 100% |
| 73.83 s - | 100% | 100% | 0% | 0% | 100% |
| 73.83 s + | 100% | 100% | 52.06% | 100% | 0% |
| 100 s | 100% | 100% | 44.66% | 100% | 0% |
| 150 s | 100% | 100% | 29.93% | 100% | 0% |
| 200 s | 100% | 100% | 14.59% | 100% | 0% |
| 246.32 s | 100% | 100% | 0% | 100% | 0% |
| 260 s | 100% | 100% | 0% | 57.29% | 0% |
| 279.252 s | 100% | 100% | 0% | 0% | 0% |
| 300 s | 100% | 83.42% | 0% | 0% | 0% |
| 350 s | 100% | 55.43% | 0% | 0% | 0% |
| 400 s | 100% | 37.81% | 0% | 0% | 0% |
| 455.242 s | 100% | 23.69% | 0% | 0% | 0% |

## 6. Recommended Flight-Control Implementation

The time schedule should not be implemented as a rigid lookup table. It is better used as an expected trend while throttle allocation is solved every physics frame.

Atmospheric pressure changes each engine's available thrust, exact CoM changes as resources move and burn, and gimbal or aerodynamic forces may introduce additional moments.

Recommended control loop:

1. Read the current vessel CoM in the vessel reference frame.
2. Read each engine group's current available 100% thrust at the present pressure.
3. Compute \(c=z_{\mathrm{CoM}}-z_0\) and the lever arms relative to \(c\).
4. Keep Pollux thrust fixed while burning; set its thrust contribution to zero after burnout.
5. Solve the bounded allocation problem that maximizes useful liquid-engine thrust while satisfying the requested pitch moment and \(0\le u_i\le1\).
6. Apply the same command to both engines in each Bobcat pair.
7. Use gimbal for residual moment and attitude control, especially during the pre-burnout infeasible region.

The per-frame constraint is

\[
(s-c)Su_S
+
(n-c)Nu_N
+
(f-c)Fu_F
+
(h-c)Cu_C
+
(p-c)P
=
\tau_{\mathrm{cmd}}
\]

For a steady thrust line,

\[
\tau_{\mathrm{cmd}}=0
\]

For constant pitch-up or another attitude command, replace zero with the requested pitch moment.

This is cleaner and more stable than hard-coding one Bobcat pair to shut down for pitch control.

## 7. Assumptions and Limits

- The exact static equations and lever arms come from the uploaded \`STS.craft\` geometry.
- The time-domain schedule is a simplified reference model. It approximates the changing CoM instead of reconstructing the exact dry-mass and resource-mass distribution of every stock part at every instant.
- Shuttle fuel mass is assumed to remain constant through transfer from the ET.
- The static equations assume nominally parallel thrust axes. Engine gimbal changes force direction and therefore the moment equation.
- Aerodynamic pitching moment is not included. During atmospheric ascent the controller should balance engine moment together with aerodynamic and attitude-control effects.
- Maximum-thrust allocation is one sensible objective, not the only possible one. A controller could instead minimize propellant use, preserve engine margin, limit dynamic pressure, or track a commanded total thrust / TWR.

> **Bottom line:** use the exact CoM/thrust moment equation as the flight-control law. The plotted schedule is a good preflight expectation: Skiff stays at 100%; after booster burnout the far Bobcat pair is progressively reduced to zero, then the Cheetah is reduced to zero, then the near Bobcat pair is reduced as the ET empties.

## Appendix A. Source Notes

The uploaded source files were \`STS.craft\` and \`STS.loadmeta\`.

\`STS.loadmeta\` reports:

- KSP 1.12.5
- 60 parts
- Editor mass: 459.45401 t
- Vehicle dimensions: approximately 13.193 x 25.448 x 13.482 m

Engine positions used in the lever-arm calculation were read directly from \`STS.craft\`.

The exact geometry/moment equations are intentionally separated from the approximate time-domain schedule so that the reference curve is not mistaken for an exact open-loop flight law.

