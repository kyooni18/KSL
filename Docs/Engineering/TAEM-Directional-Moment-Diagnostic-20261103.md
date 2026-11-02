# Recorded-state directional pitch-moment check

The native rotational model uses the minimum positive/negative available angular acceleration for both command directions. The actual first supported pitch-balance seed has +0.286325 and -0.149064 rad/s² pitch authority (positive/negative ratio 1.920823), so this symmetric approximation merits examination. Available actuator torque is not a measured local control derivative; substituting it into a force model without checking the response would be premature.

The narrow diagnostic compares the existing symmetric-minimum formula against a directional-available-torque hypothesis on the identical archived state/axis history. Both freeze the seed inertia, torque values and prebranch balance input 0.799357593. The directional hypothesis subtracts the modeled moment at that balancing input from the moment at the applied equivalent pitch input. It does not select the gain from the sign of the *difference* between those inputs. Rigid-body gyroscopic and planet-frame terms are included identically. No postbranch coefficient is fit.

Observed body-rate derivatives and actual postbranch axis readbacks are used for this one-step closure check. This is not an open-loop forecast. There is no actual individual surface-deflection history in this archive, and no negative-pitch excitation: equivalent pitch remains 0.802847–0.824468 over the diagnostic interval. Thus the two physical torque branches are not independently identified.

| Interval | Symmetric minimum RMS | Directional available torque RMS |
| --- | ---: | ---: |
| Before recorded moment-support exit, 18 intervals / 1.82 s | 0.004625 rad/s² | 0.006336 rad/s² |
| Entire diagnostic span, 59 intervals / 6.06 s | 0.004063 rad/s² | 0.006748 rad/s² |

Both residuals have positive bias. The full diagnostic span extends beyond original moment support and is explicitly not a validated regime. The result rejects simply substituting directional available torque as a demonstrated fix; it does not validate the symmetric model or identify the true aerodynamic moment law. Production dynamics, authority, controller gains and evidence-support bounds are unchanged.

Reproduce with the standard-library-only `Validation/TAEM-Recovery-20261103/diagnose_directional_moment.py`, passing the original live.jsonl, its effective configuration JSON, and first-supported-balance-current.json. The adjacent directional-moment-diagnostic.json retains the exact epoch/record sequence, input hashes and limitations. Source prefix hashing includes bytes consumed through the last diagnostic record. This was one actual recorded-state check, without synthetic capability changes or trajectory campaigns.

A separate standard-kRPC read-only reconnect attempt (MCP run 2) again failed during bootstrap with ConnectionRefusedError errno 61. It replaced no active program and terminated with code 1 before the query body executed. No game state/control change or new measurement occurred; the safe summary is retained in krpc-read-only-recheck.json. The configured endpoint remains unavailable; no conclusion about whether KSP itself is running follows from that error.

Next useful evidence remains actual surface state and measured pitch response at known actuator history, followed by matched-command validation and supported uncertainty. The existing pause/load/flight authorization requirements still apply. Command search, regenerated spiral and landing readiness remain unfinished; this report makes no live landing claim.
