# Planner and Final arrival contract audit

The planner's nominal speed band and the native Final admission gate are not
interchangeable. Synthetic arrival snapshots show that some speeds outside the
planner band pass Final, while many speeds inside it fail Final energy or load
checks. This does not justify lowering the planner speed target: none of the
125–130 m/s arrivals tested here passes all native Final gates.

Run `.venv/bin/python Docs/Experiments/STS-Landing-20261011/Final-Arrival-Contract-Audit/run.py`
from the integration checkout. The runner builds a temporary fixture and
passively instrumented Final object/planning include, retaining production
commands and equations. Native `-Werror` compilation, snapshot counts and
arrival/gate key pairing are checked. Input/source/binary/log hashes and rows
are saved in `result.json`; `arrivals.csv`, `gates.csv`, and `energy.csv` retain
the observations. Raw logs are ignored. No kRPC connection or flight occurs.

## Contract distinction

For configured STS sample 0 (nominal 26 km, Mach 2.5, 30 km before the runway),
`mm305_final_alignment_distance` resolves 9,000 m. Applying the shared planning
model produces 20° Final slope and 165 m/s target, whose ±12% floor is 145.2 m/s.
The prior zero-turn diagnostic deliberately used the raw configured 8 km /
160 m/s contract, with 140.8 m/s floor. It did not test the full production
resolved contract. Its best constrained speed, 127.814 m/s, is below either
floor. This audit corrects the interpretation without changing that artifact
or treating either diagnostic as route qualification.

`terminal_solver.c` first requires smooth reference/altitude/endpoint feasibility
and then checks ±12% speed. Native candidate replay subsequently calls
`terminal_native_delivery_admissible`, which constructs native-force telemetry,
uses the shared live runway/preflare capture gate, and evaluates the shared
terminal delivery contract. The replay wrapper does not authorize emergency
low-energy delivery. Live low-energy delivery is a separate, latched degraded
ownership transition after a persistent energy shortfall at minimum drag;
it is not evidence that a candidate or landing was qualified.

The nominal Final gate checks projected preflare/touchdown energy, available
response, physical runway capture, pressure, measured load and vertical reserve.
Its specific-energy margin takes the minimum of lower preflare kinetic margin,
upper preflare energy margin and projected touchdown margin. Thus a speed
inside a station target band is neither sufficient nor equivalent to this gate.

## Synthetic snapshot matrix

Use the actual configured STS October 10 native aero/atmosphere/servo and
25,123.15 kg mass. At each 8/9 km station, set geometric altitude to the 20°
Final line and course along the runway with zero cross track. Sweep speeds
110, 125, 127.814225, 130, 140.8, 145.2, 160, 165, 180 m/s; measured FPA −20°
or −25.7°; measured AoA 4°, 8°, 12°, 13°, 14°, 15°. These are 216 independent
snapshots with cold guidance history and one native-force observation. They
are not actual arrivals from the zero-turn runs: incidence, altitude history,
velocity direction and learned estimator history must be preserved in any
future trajectory-level comparison.

Six snapshots pass the complete native delivery gate; five of them are outside
the corresponding planner speed band. Of the snapshots classified inside the
band by the native floating-point comparison, 83 fail delivery. Accepted cases:

| Station m | Speed m/s | FPA ° | AoA ° | Inside planner speed band |
|---|---:|---:|---:|---|
| 8,000 | 110 | −25.7 | 8 | No |
| 8,000 | 110 | −25.7 | 12 | No |
| 8,000 | 180 | −20 | 8 | No |
| 9,000 | 110 | −20 | 8 | No |
| 9,000 | 110 | −25.7 | 12 | No |
| 9,000 | 180 | −20 | 8 | Yes |

No snapshot at 125, 127.814225 or 130 m/s passes. At 8 km / 127.814225 m/s /
−20° FPA, lower incidences meet the outer capture gate but fail upper energy;
higher incidences have a feasible energy verdict but fail measured load:

| AoA ° | Outer capture | Specific-energy margin J/kg | Measured-load margin g |
|---|---|---:|---:|
| 4 | Pass | −1,888.73 | +2.291 |
| 8 | Pass | −1,547.32 | +1.156 |
| 12 | Pass | −484.81 | +0.020 |
| 13 | Fail | +9.92 | −0.263 |
| 14 | Fail | 0.00 | −0.547 |
| 15 | Fail | 0.00 | −0.831 |

For 4° AoA, lower preflare kinetic margin is +4,888.73 J/kg, while the upper
margin is −1,888.73 and touchdown margin +464.87. This is an excess-energy
forecast rejection, not simply the planner's minimum speed rejection. At 13°,
the energy margins pass, but the native measured force/load exceeds the
configured 3.5 g ceiling. A feasible terminal-energy verdict alone must not be
reported as Final admission when the separate outer capture predicate fails.
All snapshot preflare plans are feasible; that too is not complete delivery.

No production contract or gate was changed, and no snapshot establishes a
trajectory, Final tail, landing, rollout, plant qualification or full five-state
MM305 feasibility. Future work should replay the actual propagated exit state
and compare shared energy forecasts with its flown Final/preflare behavior,
including force/load and learned history. Do not replace that work with a
lowered scalar speed threshold or infer physical reachability from this grid.
