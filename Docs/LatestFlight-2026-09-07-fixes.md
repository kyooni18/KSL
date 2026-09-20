# Latest flight: S-turn, control and missed HAC alignment

Analyzed `FlightLogs/2026-09-07T08-36-52Z-STS-N.jsonl`: 7,301 snapshots,
native build `Sep 7 2026 16:52:56`. The run resumed from a checkpoint explicitly
marked unqualified. It never entered TAEM or acquired either HAC. It ended in
Abort, 47.4 km beyond the runway threshold and 5.5 km off the runway axis,
at 5.0 km altitude. The log's final abort message reports manual control
restored; it is not evidence of an automatic overshoot abort or a crash.

The extracted measurements and phase transitions are in
[LatestFlight-2026-09-07-analysis.json](LatestFlight-2026-09-07-analysis.json).

| Runway-relative station crossed | Snapshot | UT | Speed | Altitude | Cross-track |
| --- | ---: | ---: | ---: | ---: | ---: |
| 8 km final-entry station | 5714 | 21896.95 | 880 m/s | 24.83 km | 4.37 km |
| 500 m pre-threshold aimpoint station | 5772 | 21906.33 | 788 m/s | 24.15 km | 4.43 km |
| Runway threshold station | 5776 | 21907.13 | 781 m/s | 24.09 km | 4.44 km |

These are crossings of planes perpendicular to the runway axis, not successful
point captures. The 8 km station is the HAC exit/final-approach start in this
code; the 500 m aimpoint is heading metadata. Neither is a separate upstream
HAC entry gate. HAC entry itself is a dynamic circle capture tube with radial,
course, altitude, energy and turn-authority conditions. At the threshold the
measured lift supported a roughly 100 km turn radius at 55 degrees bank, before
capture margin. That is already outside the bounded terminal-circle allowance
once margin is included. The important failure occurred before the runway
crossing: the shuttle spent its inbound range without acquiring a feasible HAC.

## Confirmed defects corrected

- **Premature repeated reversals:** two legs reversed after only 8.96 and 9.28
  seconds beyond 12-degree bank capture, despite the configured 24-second
  minimum. Roll-through age had been passed to the predictor as established
  leg time. Predictions now start that clock from actual capture, and live
  execution independently enforces the minimum dwell.
- **Stale reversal events:** replacement wings-level/no-reversal policies could
  inherit an old scheduled bank kick or final-exit classification. Event state
  now belongs to the accepted policy. Identical policies retain their deadline;
  changed policies replace it, and invalid/no-event policies cancel it.
- **Forecast/execution divergence:** forecast replay ignored the committed
  reversal timestamp and recomputed its own event. It now consumes the retained
  event. Final-exit classification also requires a feasible bounded HAC radius,
  matching the live demotion to an ordinary energy leg when authority is absent.
- **Stale optimization comparisons:** new candidate costs were compared against
  costs and terminal flags from older aircraft/model states. This could freeze
  a policy as authority, range and terminal feasibility changed. Current-state
  candidate selection and its existing attitude-change penalty now decide.
- **Outbound range credited as useful range:** increasing radial distance after
  passing KSC could reduce the optimizer's required work per remaining metre.
  The terminal-region objective now caps useful inbound distance with signed
  progress to the final-entry station and penalizes outbound forecasts that
  never reach TAEM. Live guidance explicitly warns when that station has been
  crossed without HAC acquisition; crossing alone does not prove that every
  possible return is impossible, so it is not an unconditional abort trigger.
- **HAC-side eligibility:** the predictor picked the lowest geometric score
  before testing feasibility, potentially rejecting the other feasible side.
  Both sides are now tested first, matching live guidance.
- **AoA tracking:** the median requested-minus-measured AoA error was 4.38 degrees
  during entry at 500–7,000 Pa. The bridge scaled steady elevator trim with
  sample interval and could disable trim learning when the estimated aerodynamic
  authority fraction vanished. Steady trim now survives latency derating, and
  loaded-flight integral correction does not depend on estimator excitation.
- **AoA transport and interface continuity:** clamping Euler target pitch before
  subtracting flight-path angle changed requested AoA during steep descent.
  Entry now shapes AoA directly, and reseeds its AoA limiter at interface handoff.
  The predictor's pre-entry incidence now matches live maximum-entry AoA.
- **Recovery cycling:** recovery could release at low angular rates while pitch
  remained more than 10 degrees below its target. Release now also requires
  pitch capture and nonnegative AoA.
- **TAEM reporting:** terminal readiness and telemetry follow the actual retained
  event rollout. Missing TAEM states remain unavailable in recovery and other
  prediction phases as well as entry, rather than displaying a later endpoint
  as a TAEM arrival.

## Verification and limits

Passed `make -C CLanding test`, `python3 Validation/PythonBridgeTests.py`, and
`python3 Validation/BackendProtocolTests.py`. Added regressions cover event
replacement/cancellation, immediate events, actual-bank dwell, committed-event
forecast timing, signed outbound progress, missed-station reporting, steep-FPA
AoA transport, and recovery capture. Bridge tests include delayed-update trim
and a closed-loop synthetic pitch plant with a constant aerodynamic moment and
zero learned aerodynamic fraction. Both new trim regressions fail against the
saved original bridge and pass against the repaired bridge.

The backend was rebuilt. No KSP save was reloaded, no running flight process was
restarted, and no new live landing was flown. These checks establish the repaired
code contracts, not successful KSC capture from this unqualified checkpoint.
The predictor remains an approximate aerodynamic model; late-flight return
feasibility and full closed-loop landing performance require a fresh flight.
Original edited sources were preserved under
`Runtime/Repairs/2026-09-07-latest-log/` because this directory has no Git repository.
