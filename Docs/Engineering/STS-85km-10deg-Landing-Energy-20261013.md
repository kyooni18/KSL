# STS 85 km / 10° landing energy failure

The 2026-10-13 flight record is split between `runs/20261013T133312Z-sts-85km-10deg` (deorbit and MM304) and `runs/20261013T140448Z-sts-85km-10deg` (reentry continuation). These are recorded flight logs, not a new flight test.

## Evidence and cause

- The selected 29.8 m/s deorbit to a 50 km periapsis predicted an MM305 inlet only 47.6 km from KSC against a 96.5 km target. Its native terminal energy margin was −242 kJ/kg, boundary capture failed, and robustness was 0/15. `executionQualified=true` described safe entry-stage execution; it did not certify landing. Sources: the first run's `backend.stderr.log` and `deorbit-plan.json`.
- MM304 carried large lateral displacement: about +103.5 km at 70 km altitude, changing to −42.3 km by 30.7 km. The previous reversal gate could reject projected cross-track arrest when the point azimuth was inside its deadband. The drag reference also priced spherical distance to an alignment point without charging the bank-limited turn to reach it. Source: first run's `telemetry.jsonl`; old logic in `guidance/entry_lateral.c` and `guidance/entry/contract.inc`.
- The continuation began at 24.1 km and about Mach 3.7, yet the restart path gave MM305 ownership based on altitude and range alone. At 20.5 km it was about Mach 2.85, below the nominal 22 km lower inlet altitude. MM305 was already at its maximum-range setting (`k=0`), with about 42.9 km to go versus 31.1 km predicted reach; it could not manufacture energy. The vehicle fell to about 77 m/s and splashed about 7.45 km from the runway without Final capture. Sources: second run's `backend.stderr.log`, `telemetry.jsonl`, and `status.json`.
- The STS live configuration disabled reciprocal runway use even though KSC 09 and 27 are both valid landing directions. That excluded a potentially shorter terminal route. Source: `Configuration/sts-live.json` before this change.

## Corrections

1. The 85 km / 10° acceptance mission requires strict full-route qualification before deorbit. Other mission profiles retain the existing best-effort default.
2. Deorbit candidate ranking now values a captured MM305 ownership boundary and a smaller energy deficit before a close ground endpoint. A completed, equally qualified approach still uses endpoint miss to break a tie.
3. MM304 includes a bounded turn to the Final alignment point in its near-terminal drag budget, warns when its maximum-range profile still falls short, and can reverse bank to arrest measured cross-track motion inside the azimuth deadband.
4. Reentry continuation enters MM305 directly only after crossing the same upper altitude and Mach boundary used by live MM304 handoff.
5. STS live guidance enables both runway directions and previews the shorter reachable path before MM304 hands off, retaining the chosen runway frame through MM305. The deorbit passive overflight reserve can be satisfied by either threshold.

Offline unit and configuration tests cover these changes. No live KSP test or successful landing has been observed after this patch. A route that still cannot satisfy the native energy and robustness proof will be rejected by the acceptance mission before its deorbit burn.
