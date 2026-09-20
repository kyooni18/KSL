# STS-N Launch Platform — Flight Design Baseline

> Thrust-vector revision: `THRUST_VECTOR_AUDIT.md` is authoritative for engine offset, asymmetry, SRB pre-flameout separation, and ET-separation constraints.

## Configuration

Use the existing STS-N orbiter unchanged structurally, except for launch preparation: retract all landing gear, force the J-N500 to ClosedCycle, set its gimbal limiter to 0%, and unload all 902 units of onboard Oxidizer. The orbiter's only installed rocket mode is LiquidFuel-only, so that Oxidizer is dead ascent mass. Full-as-saved launch-prepared STS-N mass becomes 40.155 t instead of 44.665 t, a 4.510 t reduction. Preserve onboard LiquidFuel and MonoPropellant.

The launcher is a powered external tank with two gimballed SRBs:

- 1 x `STS_ET_decoupler` — ET-OV 100-series radial decoupler, 0.250 t. Crossfeed ON.
- 1 x `DIRECT_STS_tank` — Olympus S4-JUMBO 5 m external tank, LFO subtype. Effective dry mass in the installed B9 configuration: 62.700370187 t.
- 1 x `nflv-nflv-fueltank-adapter-5-375-4` — NR-AD-CAP 5 m -> 3.75 m adapter, 0.300 t.
- 1 x `nflv-engine-rd701-1` — KR-701 Cougar, 8.500 t, 2,150 kN vacuum, 345 s vacuum / 295 s sea-level Isp, 6 deg gimbal.
- 2 x `PC_5Seg_RSRM` — PhotonCorp RSRB-5, each 7.000 t casing + 122.925 t SolidFuel, 3,632 kN vacuum max, 267/241 s Isp, 7.5 deg gimbal.
- 2 x `PC_RSRM_RadialDecoupler` — 0.250 t each.
- 2 x `PC_Nose` — 0.075 t each + 3 units SolidFuel for separation.
- 4 x `launchClamp1` — TT18-A launch stability enhancers; pad hardware only, released at ignition.

## Geometry

Treat the ET centerline as the launcher reference axis. Attach STS-N to the ET through the purpose-built ET-OV decoupler at its native nodes; do not hand-cant the orbiter. Mount the Cougar on the ET aft assembly with zero fixed cant, then shift its centerline about 0.14 m away from STS-N in the orbiter/ET pitch plane. This is a thrust-line optimization; the exact final value should be adjusted after measuring the true VAB centerline spacing. Mount the two RSRB-5 boosters symmetrically in the plane perpendicular to the orbiter/ET offset plane so their static thrust does not add a pitch moment toward or away from the orbiter.

The ET rendered width is about 5.245 m and the RSRB-5 rendered width is about 3.220 m. Geometric tangency therefore occurs at a booster centerline radius of about 4.233 m. Use approximately 4.40-4.50 m centerline radius after the radial separator is fitted, giving physical clearance without needlessly increasing bending load. Align both booster longitudinal centers with the ET longitudinal center; the RSRB-5 is 26.704 m node-to-node and the ET is about 27.55 m tall, so this also puts their nozzle planes close naturally.

Use rigid attachment and autostrut-to-heaviest/root on the ET/booster interfaces. Avoid asymmetric struts or surface hardware in the orbiter/ET pitch plane.

## Propellant load

Set the ET to the LFO subtype, then use:

- LiquidFuel: 30,555.587421 units = 152.777937105 t
- Oxidizer: 24,828.564223 units = 124.142821117 t
- Oxidizer fill fraction: 66.4830283% of the LFO subtype's nominal Oxidizer capacity
- total ET propellant: 276.920758222 t

This mixture is intentionally fuel-rich relative to stock LFO because the J-N500 simultaneously consumes LiquidFuel without Oxidizer. At equal throttle, the Cougar consumes 0.2859638945 t/s LF + 0.3495114266 t/s Ox and the J-N500 consumes 0.1441667749 t/s LF. Both ET resources therefore reach zero together after 355.189592 s at continuous full throttle.

Set ET resource priority above every STS-N tank so neither the Cougar nor the J-N500 drains orbiter propellant before ET depletion.

## Mass and thrust

Launch mass excluding clamps: 649.371128408 t.

At t=0 the installed RSRB thrust curve begins at 0.95. With Kerbin sea-level engine performance, total initial TWR is 1.378. The exact installed RSRB cubic-Hermite thrust curve integrates to 135.294 s burn duration at 100% limiter.

Representative propulsion-balance states after the thrust-vector audit, with the Cougar shifted 0.141 m away from the orbiter and RSRB gimbal limiter at 60%:

| t | Cougar pitch | RSRB pitch | resultant thrust tilt | note |
|---:|---:|---:|---:|---|
| 0 s | -0.34..-0.40 deg | -0.26..-0.30 deg | -0.25..-0.29 deg | liftoff |
| 40 s | -0.50..-0.56 deg | -0.37..-0.42 deg | -0.37..-0.41 deg | boost |
| 80 s | +0.05..+0.11 deg | +0.04..+0.08 deg | +0.04..+0.08 deg | boost |
| 120 s | -0.26..-0.32 deg | -0.20..-0.24 deg | -0.19..-0.24 deg | late boost |
| 130 s | -1.14..-1.18 deg | -0.85..-0.88 deg | -0.83..-0.86 deg | RSRB tail |
| 134 s | about -1.30 deg | about -0.98 deg | about -0.95 deg | recommended pre-flameout separation point |

Do not wait for mathematical RSRB burnout at 135.294 s. The PhotonCorp curve still produces roughly 545 kN vacuum thrust per booster immediately before flameout, so even a very small left/right fuel mismatch can create an abrupt one-sided tail. Separate both boosters when the lower of the two SolidFuel masses reaches about 0.27 t, nominally near T+134.0 s.

Immediately after the RSRBs leave, Cougar trim steps to about -2.72 deg. The installed 8 deg/s Cougar gimbal response traverses the roughly 1.42 deg trim step in about 0.18 s; a conservative point-mass inertia model predicts only about 0.076 deg pitch excursion during the slew.

Post-SRB Cougar trim with the 0.141 m engine offset and J-N500 gimbal locked:

| actual elapsed t | ET propellant | Cougar trim range, 0..1 atm | resultant tilt | throttle state |
|---:|---:|---:|---:|---|
| ~134 s | ~172.45 t | about -2.72 deg | about -1.97 deg | 100% |
| 160 s | 152.18 t | -2.50..-2.55 deg | -1.81..-1.84 deg | 100% |
| 200 s | 120.99 t | -2.07..-2.12 deg | -1.50..-1.53 deg | 100% |
| 240 s | 89.81 t | -1.49..-1.53 deg | -1.08..-1.11 deg | 100% |
| 280 s | 58.62 t | -0.64..-0.69 deg | -0.46..-0.50 deg | 100% |
| 300 s | 43.03 t | -0.05..-0.10 deg | -0.04..-0.07 deg | 100% |
| 304.496 s | 39.52 t | near zero | about -0.3 deg | 2g cap starts |
| 320 s | 27.91 t | +0.63..+0.68 deg | +0.46..+0.49 deg | ~92% |
| 340 s | 14.22 t | +1.50..+1.54 deg | +1.08..+1.12 deg | ~83% |
| 360 s | 1.88 t | +2.52..+2.57 deg | +1.82..+1.86 deg | ~75% |
| 363.242 s | 0 t | +2.71..+2.75 deg | +1.96..+1.99 deg | ~74% |

The centered-Cougar draft was already controllable, but the 0.141 m away-from-orbiter offset improves the reachable post-SRB trim envelope from about -3.25/+2.18 deg to about -2.76/+2.75 deg and reduces worst resultant-vector/body-axis offset from about 2.34 deg to about 1.99 deg. Keep zero permanent cant.

## Gimbal and control settings

- J-N500: 0% gimbal limiter / locked. Treat as fixed thrust.
- Cougar: 100% gimbal limiter. Static balance consumes at most about 2.76 deg in the modeled reachable envelope, leaving at least about 3.24 deg of nominal pitch authority.
- RSRB-5: start with 60% gimbal limiter, giving +/-4.5 deg. Static balance requires less than about 1.8 deg before separation, leaving at least about 2.7 deg for steering while reducing over-control from the extremely powerful boosters.
- SAS/guidance should command the stack through the Cougar + RSRBs only; the orbiter engine must not be relied upon for TVC.

## Throttle program

Run Cougar + J-N500 at 100% from ignition through RSRB separation. Keep full liquid throttle after separation until approximately T+304.5 s, when the modeled vacuum-equivalent liquid-stack TWR reaches 2.0. Thereafter cap longitudinal acceleration at about 2.0 g by reducing common vessel throttle. Because both liquid engines are throttled by the same ratio, this preserves their thrust ratio and therefore preserves the calculated trim balance for a given fuel state.

Approximate 2.0-g cap commands are about 92% at T+320 s, 83% at T+340 s, and 74% near theoretical ET depletion. The cap moves theoretical ET depletion from the 355.190 s full-throttle equivalent to about T+363.242 s. In actual ascent, ET separation should still be driven by orbit/energy target and residual propellant, not elapsed time alone.

## Staging

1. Prelaunch: gear up, J-N500 ClosedCycle, J-N500 gimbal locked, ET crossfeed ON, ET priority highest, RSRB gimbals 60%, Cougar gimbal 100%.
2. Ignition stage: ignite J-N500 + Cougar + both RSRB-5s and release all four TT18-A clamps in the same staging event.
3. RSRB separation: do not wait for one-sided flameout. Fire both PC_Nose separation motors and both PC radial separators together when the lower of the two booster SolidFuel masses reaches about 0.27 t, nominally near T+134.0 s. The mathematical full-burn time remains 135.294 s.
4. Powered-ET phase: continue Cougar + fixed J-N500. Apply 2-g throttle cap late in ascent.
5. ET separation: when the orbital energy target is met, command throttle to zero first. With J-N500 TVC locked, do not separate the ET under full J-N thrust. Shut down/cut thrust, decouple ET-OV, and only then hand over to the orbiter propulsion/control strategy.

## VAB acceptance checks

The final VAB assembly is accepted only after checking CoM/CoT and RCS Build Aid with the J-N500 gimbal locked at these resource snapshots: full stack; 80%, 60%, 40%, 20% RSRB fuel; immediately before and after RSRB separation; 75%, 50%, 25%, 10%, and near-empty ET. No snapshot may require more than 5 deg Cougar trim in the actual assembled geometry; the revised target is <=3 deg, with the model predicting <=2.8 deg and <=2.0 deg resultant-thrust tilt. If the measured trim exceeds that threshold, change ET/orbiter spacing or booster alignment first rather than unlocking the J-N500.
