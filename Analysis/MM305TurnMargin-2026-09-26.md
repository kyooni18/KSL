# MM305 turn-margin and command-continuity experiment

All runs used ShuttleSim `engageHACTest`, the simulator transport, fresh simulator UDP ports, `--no-mirror`, and no replay archive. No live KSP session or kRPC endpoint was touched. The four cardinal cases start 8 km from the runway threshold at 10 km altitude, 180 m/s airspeed and −8° flight-path angle, heading toward the threshold. The nominal case starts about 60 km west at 26 km and Mach 2.6. These are synthetic handoff probes, not recorded MM304 states.

## Mechanism and change

The tracker already imposed a Mach-dependent AoA cap (28° at high Mach, 15° subsonic), but route ranking did not price control reserve. Native replay now records commanded AoA, lateral authority use, combined bank/AoA turn burden, and bank-target reversals. Candidate comparison uses tracking and turn burden where predicted exit-speed deficits differ by at most 1 m/s, or where both candidates reach the configured speed target. A larger deficit remains significant to an unpowered vehicle. No geometry, admission, or Final gate was weakened.

The largest observed control discontinuity was in the **bank target**, not the simulated attitude: at one 90° inlet tick it fell from 63.2° to 8.9° while measured bank stayed near 64°. The shared MM305 tracker now slews its bank target at five times the identified maximum physical roll rate. Native replay carries the prior requested bank in its propagated attitude state; live guidance carries the prior issued bank target in MM305 state. Actual roll remains bounded by ShuttleSim's normal attitude dynamics.

## Committed-code ShuttleSim sweep

The maximum consecutive 0.1 s bank-target jump fell from 33.8°–69.5° in the earlier five runs to 6.23° in every committed-code run. Measured heading changed by at most 0.91° per 0.1 s in the committed-code sweep. The table evaluates **MM305 HAC and runway alignment**, not touchdown or rollout.

| Entry heading | MM305 aligned exit: along / cross | Exit altitude / airspeed | Exit bank | Commanded-bank reversals | MM305 route adoptions |
| --- | ---: | ---: | ---: | ---: | ---: |
| Nominal 90° high energy | −2982 / +14 m | 1585 m / 148.2 m/s | +0.04° | 2 | 1 |
| 0° | −2959 / −21 m | 1567 m / 141.8 m/s | −0.64° | 1 | 1 |
| 180° | −3056 / −17 m | 1632 m / 140.2 m/s | −2.89° | 1 | 1 |
| 90° low energy | −3055 / +18 m | 1653 m / 117.6 m/s | +1.03° | 2 | 2 |
| 270° | −3018 / −13 m | 1611 m / 129.1 m/s | −0.74° | 0 | 1 |

The 90° low-energy inlet still releases its initial route and adopts another. It also has two bank-sign reversals. This remains a guidance/route-selection limitation; the smooth bank target alone does not prove smooth route ownership. All five states reached a measured runway-line alignment state with bank under 3°. The 90° exit airspeed is materially below the configured 160 m/s preference. Final approach and landing results are intentionally excluded from MM305 acceptance in this experiment.

A slower target slew at twice the physical roll rate was rejected: the 0° case diverged and spent too much energy before alignment. Replaying candidate routes at 0.1 s instead of 0.5 s was also rejected after the 90° case selected a different route and needed more recovery. A fully weighted speed/turn score preferred a lower-energy 90° route but still needed repeated replanning. The committed comparison keeps predicted exit-energy differences above 1 m/s decisive.

Focused `taem-native-stack-test` and `mm305-recovery-monitor-test` pass, including a new tracker command-continuity assertion. Relevant commits: `2cc4cba`, `33b4933`.

## Remaining 90° route divergence

Detailed replay/live traces agree closely through about 60 s. At the lead/HAC join around 70 s, the cubic lead still asks for roughly 0.001 rad/m curvature; the analytic 3 km circle asks for 0.000333 rad/m. The vehicle reaches that C1-only join about one second later than the 0.5 s replay and is still banked near 66°. The tracker then commands an opposite bank to recover the overshoot; measured course error grows to about 15°. Replay at the same stage has already reduced its bank target. The 90° route replacement is therefore rooted in a curvature/roll-timing mismatch, not a sudden actual heading jump.

Two geometry experiments were rejected. Globally preferring a lower curvature jump in compact lead construction removed the 90° route replacement, but the 180° probe adopted an extra route and lost 4.6 m/s at aligned exit; the 270° probe lost 5.5 m/s and the nominal route changed topology. Adding a separate smooth-join candidate to the ordinary profile search caused the 90° probe to commit a poor route and eventually abort with no qualified route. Both experiments were restored. The next geometry change needs to preserve the original candidate set and compare a smooth join against it with reliable replay of the transient.

## Surface-moment control check

The same 10 km, 180 m/s, −8° / 90° handoff was run with `--direct-control`, which makes the backend flight controller command ShuttleSim pitch, roll, and yaw surface moments rather than the ideal AoA/bank servo. MM305 sets `heading_control_enabled=false`; yaw only damps sideslip. This was an explicit simulator transport with fresh UDP ports, no replay mirror, and no live kRPC connection.

The direct-control run **failed MM305**. The initial route was the same 3 km, 180° HAC chosen in ideal-servo replay, but by 4 s measured bank was 19.7° against a 37.6° request, versus 42.8° measured against a 52.8° request in the ideal-servo run. At 20 s direct-control bank was 35.2° against 38.7° requested, while the ideal-servo vehicle had already banked, rolled back, and reached 30.7°. The direct-control run adopted two replacement routes, then at 232.6 s aborted for lack of a qualified route; airspeed had fallen to 78 m/s. Peak measured sideslip was only 0.70°, so this was a roll/pitch response and energy/route-feasibility problem, not a yaw-induced spin. Peak measured AoA was 14.1°; the 85 m/s minimum-safe-speed constraint was eventually violated during the failure.

An experiment reducing the guidance replay attitude-rate limits to 4°/s for pitch and roll did not rescue the direct-control case: it found no viable route and aborted at 240.4 s. This is useful evidence that the 90°/180 m/s fixture may be unrecoverable with the current generic surface-moment plant and controller, but it does not certify that the real shuttle has those exact roll coefficients. The reference-model file explicitly says its direct roll/yaw coefficients were not identified from KSP data. The existing replay model should not be treated as proof of control margin for the direct-control plant.

A shared AoA-target slew was also tried, with an exception when immediate vertical lift required a faster target change. Focused tests passed, but the nominal ideal-servo 90° run then lost its route and aborted at 228.8 s. The change was rejected and restored. This shows that target smoothing cannot be applied independently of the vertical and energy trajectory; it must be planned and replay-qualified with the actual transient response. The current committed MM305 code still has large AoA target changes (up to 7° per 0.1 s in the prior ideal run), while actual pitch motion is rate-limited by the plant. It is not yet a robust direct-control HAC solution.

The 26 km/M2.6 nominal 90° entry also failed with the direct-control plant. It adopted three successive left-side routes and exhausted runway-alignment distance at 203.1 s. At 120 s it was 3.1 km south of the runway line, then commanded a 39° right bank; at 140 s it had reversed to a 38° left-bank request while measured bank was still −22°. At 200 s it was only 514 m above the runway, 864 m short of the threshold, and still 49 m off line. Peak sideslip remained below 0.60°. The first important concern is route/attitude-response mismatch and late reversal, not yaw authority. The 90° low-energy fixture and the nominal high-energy fixture both require a route-qualification model that represents the surface-moment control response before claiming robust MM305 safety.
