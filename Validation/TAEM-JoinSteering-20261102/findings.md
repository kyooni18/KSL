# Committed HAC join and recovery steering fixes — 2026-11-02

The b182029 live investigation established a geometry jump immediately after commitment and zero-bank fallback commands during most of acquisition.

Changes: flight updates no longer resize a committed spiral using the unrelated legacy q-scheduled minimum line. Offline interface placement retains its own line-based sizing. The selected geometry and tangent therefore remain consistent across the following tick. Best-L/D recovery now allocates lateral acceleration up to L*sin(configured maximumBank), with vertical acceleration derived from that same total lift vector. It no longer requires constant-flight-path support before allowing any banking. Existing structural, trim, bank, q and target-limiter protections are retained.

Seven focused tests pass, including genuine consecutive b182029 live handoff states with reconstructed observed force gains: radius/growth/turn stay fixed, remaining distance decreases and the recovery target commands a left turn. The recorded initial-state regression also confirms best-L/D incidence is preserved. This is a narrow verification of the demonstrated defects, not evidence of successful capture or landing. No additional full continuation was run. Prior offline delivery remained unsuccessful.

User explicitly requested fixing and restarting. New live experiment uses the same original72km checkpoint and prior configuration, an archived executable/resources and source provenance. Stop the prior native guidance owner before loading. Verify startup only and do not monitor afterward, per the user's instruction.
