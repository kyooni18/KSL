# STS-N Powered External-Tank Launcher - Offline Engineering Design

This design is based on the final patched KSP 1.12.5 GameData database (`ModuleManager.ConfigCache`), the saved `STS-N.craft`, KSP `PartDatabase.cfg`, and a live kRPC geometry measurement of the orbiting STS-N test article. The patched database contains 1,439 PART definitions and 184 resource definitions. Full machine-readable and flat inventories are in this directory.

## 1. STS-N baseline

Saved craft: `saves/JUSTLANDTHEFREAKINGSHUTTLE/Ships/SPH/STS-N.craft`.

The static craft/config calculation gives 20 parts, 30.335 t nominal dry mass and 44.665 t nominal wet mass. The saved Rockomax tank is deliberately overfilled (`amount=800`, `maxAmount=360`), and KSP preserves the over-capacity amount; therefore launcher sizing retains the 44.665 t conservative craft-file mass rather than clamping to nominal capacity.

Live kRPC measurement of the currently orbiting quicksave article: 20 parts, 40.675 t at the measured instant, 30.053 t dry, 4 crew. KSP `Physics.cfg` uses `kerbalCrewMass=0.045 t`. The live closed-cycle engine is the J-N500 Project Eeloo: 820 kN vacuum, 580 s vacuum Isp, 500 s sea-level Isp, LiquidFuel only. Its initial thrust direction is essentially exactly along the orbiter longitudinal axis. kRPC measured the fixed thrust line 0.10096 m from the instantaneous vessel CoM in the belly/dorsal axis. The installed config advertises a 2 degree gimbal and the live article currently has it unlocked, but this launch design intentionally locks that gimbal and treats the STS-N engine as zero-TVC, per the launcher requirement.

## 2. Selected launcher hardware

Orbiter-to-ET attachment: `STS_ET_decoupler` (ET-OV 100-series radial decoupler), 0.250 t. Crossfeed must be explicitly ON. Set external-tank resource priority above every STS-N LiquidFuel tank so ascent consumes ET LiquidFuel first and preserves orbiter internal fuel.

External tank: `DIRECT_STS_tank`, Olympus S4-JUMBO 5 m External Cryogenic Tank, B9 fuel type LFO. BaseVolume = 67,901.305380. The LFO tank type adds 0.000346096 t per volume, giving 23.500370187 t switch mass and 62.700370187 t total dry ET mass. Rendered size from KSP PartDatabase is approximately 5.142 x 27.55 x 5.245 m.

ET aft adapter: `nflv-nflv-fueltank-adapter-5-375-4`, NR-AD-CAP, 0.300 t, 5 m to 3.75 m.

ET engine: `nflv-engine-rd701-1`, KR-701 Cougar, 8.500 t, 2,150 kN vacuum, 345 s vacuum / 295 s sea-level Isp, 6 degree gimbal. This is deliberately selected over the 4,500 kN Manatee. A second optimization allowed both lateral engine offset and a permanent mechanical cant on the Manatee. With a physically plausible offset of 0.50 m, the best static cant still needs about +/-2.294 degrees of dynamic gimbal, exceeding the Manatee's 2 degree limit. Getting below 2 degrees requires an absurd multi-meter lateral offset and roughly ten degrees of permanent cant, which is physically unsuitable under the 5 m ET and wastes axial thrust. The Cougar can trim the stack through full ET depletion with substantial TVC margin while remaining centered and uncanted.

Boosters: two `PC_5Seg_RSRM` PhotonCorp RSRB-5 motors, each 7.000 t dry plus 16,390 SolidFuel units = 122.925 t propellant, 3,632 kN vacuum maximum thrust, 267 s vacuum / 241 s sea-level Isp, 7.5 degree gimbal. Each side also uses one `PC_Nose` (0.075 t plus 3 SolidFuel units) and one `PC_RSRM_RadialDecoupler` (0.250 t). Complete per-side initial booster mass is 130.2725 t. Both sides drop 14.695 t of dry hardware at separation.

Use symmetry about the ET plane. Put the two SRBs on the left/right sides of the ET, perpendicular to the orbiter-ET offset plane. Keep both SRB nozzle planes approximately level with the ET engine nozzle plane. Use autostrut/rigid attachment instead of adding flight mass unless structural testing proves a physical strut is required.

## 3. Exact ET load for simultaneous Cougar + fixed STS-N burn

Installed resource density: LiquidFuel = 0.005 t/unit and Oxidizer = 0.005 t/unit.

The Cougar consumes, at full throttle in vacuum:

- total mass flow = T/(Isp*g0) = 2150/(345*9.80665) = 0.6354753211 t/s
- LiquidFuel mass flow = 0.45 of total = 0.2859638945 t/s
- Oxidizer mass flow = 0.55 of total = 0.3495114266 t/s

The fixed STS-N J-N500 consumes LiquidFuel at:

- 820/(580*9.80665) = 0.1441667749 t/s

Combined LiquidFuel demand = 0.4301306694 t/s. To exhaust ET LiquidFuel and Oxidizer together while both liquid engines remain at the same throttle ratio, use:

- LiquidFuel = 30,555.587421 units = 152.777937105 t (100% of the LFO tank's LF allocation)
- Oxidizer = 24,828.564223 units = 124.142821117 t
- Oxidizer slider = 66.4830283% of the tank's normal 37,345.717959-unit LFO Oxidizer capacity
- total ET propellant = 276.920758222 t
- full-throttle ET burn time = 355.189592273 s

This nonstandard oxidizer load is intentional: a stock 45:55 LFO fill would leave excess Oxidizer because the orbiter adds an extra LiquidFuel-only flow path.

## 4. Launch mass and thrust

Conservative launch mass model, excluding launch clamps and assuming the 44.665 t saved-craft STS-N mass:

- STS-N: 44.665 t
- LFO ET dry + selected propellant: 339.621128408 t
- Cougar: 8.500 t
- NR-AD-CAP: 0.300 t
- ET-OV decoupler: 0.250 t
- two complete RSRB-5 assemblies: 260.545 t
- total: 653.881128408 t

At ignition, the PhotonCorp thrust curve begins at multiplier 0.95. One RSRB-5 therefore produces about 3,114.41 kN at Kerbin sea level at t=0. Cougar sea-level thrust is 1,838.41 kN and the J-N500 sea-level thrust is 706.90 kN. Total ignition sea-level TWR is 1.368. The corresponding vacuum-equivalent t=0 TWR is 1.539.

The exact PhotonCorp cubic-Hermite thrust curve was numerically integrated. At 100% thrust limiter, SRB burnout occurs at 135.294 s. Average thrust-curve multiplier over the burn is 0.65501.

## 5. CoM and thrust-vector design

The authoritative condition is zero pitch moment from propulsion at each fuel state:

`sum((r_engine - r_CoM) x F_engine) = 0`.

The STS-N engine is held fixed. The Cougar and the two RSRB gimbals provide trim and steering.

The exact ET/orbiter attachment-node transforms live inside Unity `.mu` models and are not represented numerically in ModuleManager config, so the only geometric approximation in the offline model is ET-to-orbiter centerline spacing. KSP PartDatabase gives an ET width of 5.245 m and the live aft fuselage belly envelope is about 1.52 m from the orbiter reference axis. A 4.55 m centerline spacing provides roughly 0.4 m additional physical/decoupler clearance. The model is insensitive to normal VAB spacing error: earlier sensitivity sweeps from 3.8 to 5.5 m kept the required Cougar trim far inside its 6 degree range.

With 4.55 m centerline spacing and the Cougar centered on the ET axis, the modeled full-stack transverse CoM moves from 0.312 m toward the orbiter at liftoff to 0.673 m immediately before SRB burnout. Required common trim remains small during the booster burn. Sea-level-endpoint calculations (conservative for low altitude) are:

| t (s) | mass (t) | ET prop (t) | one SRB solid (t) | SRB curve | transverse CoM (m) | TWR, SL endpoint | Cougar gimbal | SRB gimbal |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 653.88 | 276.9 | 122.9 | 0.950 | 0.312 | 1.368 | -0.22 deg | -0.28 deg |
| 20 | 586.49 | 261.3 | 97.0 | 0.819 | 0.347 | 1.376 | -0.24 deg | -0.30 deg |
| 40 | 531.44 | 245.7 | 77.3 | 0.660 | 0.383 | 1.319 | -0.35 deg | -0.43 deg |
| 60 | 476.43 | 230.1 | 57.6 | 0.767 | 0.428 | 1.620 | -0.02 deg | -0.03 deg |
| 80 | 418.11 | 214.5 | 36.2 | 0.717 | 0.487 | 1.768 | +0.13 deg | +0.16 deg |
| 100 | 366.63 | 199.0 | 18.3 | 0.574 | 0.556 | 1.754 | +0.13 deg | +0.17 deg |
| 120 | 324.36 | 183.4 | 4.9 | 0.358 | 0.628 | 1.539 | -0.17 deg | -0.21 deg |
| 130 | 309.03 | 175.6 | 1.2 | 0.176 | 0.659 | 1.221 | -0.98 deg | -1.23 deg |
| 135 | 302.90 | 171.7 | 0.1 | 0.150 | 0.673 | 1.188 | -1.13 deg | -1.41 deg |

At nominal RSRB burnout and immediate booster jettison, attached mass is 287.855 t and ET propellant remaining is 171.440 t. Vacuum TWR of Cougar + J-N500 is 1.052 at that instant; actual atmospheric thrust at the expected high-altitude separation lies between the sea-level and vacuum endpoints. This is why the 5-segment RSRB was selected instead of a smaller booster.

Post-SRB, the required Cougar trim through complete ET depletion is:

| elapsed t (s) | attached mass (t) | ET prop (t) | transverse CoM (m) | Cougar trim | vacuum TWR |
|---:|---:|---:|---:|---:|---:|
| 135.294 | 287.855 | 171.440 | 0.708 | -2.903 deg | 1.052 |
| 160 | 268.593 | 152.178 | 0.759 | -2.674 deg | 1.128 |
| 180 | 253.001 | 136.585 | 0.806 | -2.459 deg | 1.197 |
| 200 | 237.408 | 120.992 | 0.858 | -2.210 deg | 1.276 |
| 220 | 221.815 | 105.399 | 0.919 | -1.919 deg | 1.365 |
| 240 | 206.222 | 89.807 | 0.988 | -1.574 deg | 1.469 |
| 260 | 190.629 | 74.214 | 1.069 | -1.161 deg | 1.589 |
| 280 | 175.036 | 58.621 | 1.164 | -0.654 deg | 1.730 |
| 300 | 159.443 | 43.028 | 1.278 | -0.019 deg | 1.899 |
| 320 | 143.851 | 27.435 | 1.417 | +0.800 deg | 2.105 |
| 340 | 128.258 | 11.842 | 1.589 | +1.895 deg | 2.361 |
| 355.190 | 116.415 | 0 | 1.751 | +3.010 deg | 2.602 |

Peak modeled trim is about 3.01 degrees versus a 6 degree Cougar limit. The sign reversal near t=300 s is expected: as ET propellant drains, the stack CoM migrates toward the orbiter. The available margin is large enough that the final VAB geometry can be adjusted by several decimeters without exhausting TVC.

## 6. Delta-v and stage timeline

With the exact RSRB thrust curve integrated and using vacuum/sea-level endpoint bounds rather than inventing an ascent atmosphere profile:

- SRB burnout: 135.294 s
- mass immediately before SRB jettison: 302.550 t
- booster hardware jettisoned: 14.695 t
- mass immediately after SRB jettison: 287.855 t
- stage-1 ideal delta-v: 2,306.61 m/s vacuum endpoint; 2,039.88 m/s if sea-level performance were unrealistically held for the entire stage
- remaining Cougar + J-N500 burn: 219.896 s
- effective combined vacuum Isp of Cougar + J-N500 at the selected flow ratio: 388.455 s
- stage-2 ideal delta-v: 3,448.66 m/s vacuum endpoint; 2,955.52 m/s sea-level-all-the-way lower endpoint
- total ideal delta-v: 5,755.27 m/s vacuum endpoint; 4,995.40 m/s sea-level-all-the-way endpoint
- ET depletion: 355.190 s after ignition at continuous full throttle
- attached mass just before ET separation: 116.415 t

The real flight lies between the atmospheric endpoints and incurs gravity/drag/steering losses. The margin is adequate for a Kerbin ascent. Late in the burn, throttle down to limit longitudinal acceleration; because Cougar and J-N500 share the same vessel throttle, reducing both together preserves their thrust ratio and therefore largely preserves the balance solution at a given ET fuel state.

## 7. VAB implementation and verification

1. Load STS-N from SPH in the VAB, rotate the whole orbiter upright, retract all three landing gears, and lock the J-N500 gimbal / set gimbal limiter to zero.
2. Fit the ET-OV radial decoupler on the orbiter belly and attach the 5 m S4-JUMBO ET. Turn ET crossfeed ON. Set external fuel priority above all orbiter LF tanks.
3. Fit NR-AD-CAP to the tank aft node and center one Cougar on the ET axis. Do not cant it mechanically; use the 6 degree gimbal for trim and steering.
4. Mount two PhotonCorp RSRB-5 boosters symmetrically on the ET left/right axis using PC radial separators, each with a PC nose/separation unit. Align booster nozzles approximately with the Cougar nozzle plane.
5. Set ET to LFO. LF = 30,555.587421 units; Oxidizer = 24,828.564223 units (66.4830283%).
6. Stage 0: ignite Cougar + J-N500 closed cycle + both RSRB-5s. Stage 1 at SRB exhaustion: PC nose separation motors + radial separators. Continue Cougar + fixed J-N500. Stage 2: ET-OV separation when ET LF/Ox are nearly exhausted or the target orbit condition is met. Continue on STS-N internal LiquidFuel.
7. In VAB, turn on CoM/CoT markers and RCS Build Aid. Check at least: full, 80%, 60%, 40%, 20%, SRB-empty, post-SRB with ~171 t ET prop remaining, 50% ET, 20% ET, and nearly empty ET. The final acceptance criterion is no large static torque with J-N500 gimbal locked and required Cougar trim comfortably below 6 degrees throughout. If actual centerline spacing differs materially from the 4.55 m model, update the geometry and rerun the calculation rather than eyeballing cant angles.

Launch clamps are deliberately omitted from flight-mass calculations. Crew/cargo changes should be rerun through the same mass/torque model before flight if the mission payload changes appreciably.
