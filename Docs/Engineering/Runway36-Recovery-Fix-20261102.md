# MM304 delivery and TAEM fallback repair — 2026-11-02

This change addresses the decisions demonstrated in `Runway36-Restart-Investigation-20261102.md`. It does not establish landing qualification. The user explicitly authorized fixing and restarting from the existing orbital checkpoint.

## Behavior changed

MM304 now compares the full propagated arrival: along/cross position, altitude, Mach and runway-heading error. Each component is normalized by its existing configured capture tolerance; no new tuning gains or vehicle limits were introduced. A bounded golden-section refinement searches the existing best-glide-to-trim incidence family and retains endpoint candidates. Trials retain the same atmosphere, polar, slew limits, attitude-response approximation and support-preserving bank law. The arrival remains advisory; this is not a proof of downstream recovery. The legacy bracket flag is true only when the along bracket and returned along tolerance actually support it. New diagnostics retain endpoint cross, heading and total capture residual.

A fixed-speed turn-radius penalty was considered and rejected during the narrow diagnostic: it continued to prefer the large cross miss because it incorrectly held the inlet's instantaneous speed/turn scale throughout recovery. The committed residual uses the explicit mission tolerances instead. It is an arrival ranking, not a fabricated complete HAC work price.

The configured `captureTargetRange` (5 km in this flight) now supplies MM304's before-threshold delivery objective and the common deorbit/runway comparison objective. The unrelated outer `terminalTargetRange` had silently supplied the 35.023 km objective. The delivery aim does not gate terminal search or ownership; a return HAC remains allowed. The existing outer interface/search configuration remains available separately.

TAEM now computes support-preserving lateral authority from current q, Mach, trim ceiling, observed lift effectiveness, configured uncertainty, load and bank bounds. Acquisition radius is horizontal-speed squared divided by that authority. With no supported turn, the radius is absent and recovery is explicitly unqualified. A radius/path exceeding the configured search envelope is reported rather than clipped. Every complete descent check still applies. When all candidates fail, the fallback compares their common geometric lower bounds and chooses the shortest recovery instead of the longest overhead. A geometric lower bound does not become an attainable path or a passed assessment. Actual bounded vertical/lateral allocation remains authoritative.

The selected MM304 screen is retained in `DeorbitPlan.entryScreen`; failed reciprocal-runway comparisons now retain downstream altitude/Mach rejection and capture miss ahead of orbital heading/miss tie-breakers. Optional encoding preserves old plan decoding. Existing OMS execution, normal-first capability, RCS configuration, target limiter, 40° incidence/70° bank limits and ten-second RPC reconnect behavior are unchanged.

## Focused evidence

Two new regression methods use genuine restart snapshots in `Fixtures/runway36-recovery-20261102.json`; no simulation capability is altered.

- At UT 216350.051, the old scalar fallback selected along +32,649 m/cross +37,903 m. The new ranking selects the existing maximum-incidence endpoint along +34,111 m/cross +96.89 m. Both remain outside the historical objective; the returned reference correctly stays unbracketed. This check deliberately retains the historical −35,023 m objective to isolate ranking from the separate 5 km mapping repair.
- At the measured TAEM handoff, with reconstructed lift/drag gains, the code correctly reports no current support-preserving turn, rather than claiming a 25.8 km radius is attainable. It selects a 48.92 km geometric recovery lower bound with about 180.5° remaining HAC turn instead of the failed 145.2 km/536.6° overhead. Its force/geometry assessment remains explicitly unqualified.
- The configured 5 km aim is checked, together with permission to search for a terminal plan from the genuine missed handoff state.
- The existing recorded-entry incidence-root check and downstream deorbit-ranking check pass. The latter's expected distance now derives from the configured delivery coordinate instead of the stale hardcoded −8 km target.

Final focused run: four methods passed (two new recorded-state checks, one existing entry-root regression, one deorbit-ranking regression). Release build passed. Earlier outputs preserve development failures rather than presenting them as passes.

One exploratory invocation also ran the older `GuidanceTests.testHighCloseCaptureObjectiveProducesAnAdmissibleHAC`. Its initial delivery assertions predate the required 5 km aim; after temporarily updating those assertions, it reached an unrelated existing expectation that acquisition requests an external terminal plan. That expectation conflicts with unchanged `terminalSearchDue`, which only schedules external terminal previews in entry. The exploratory test edits were reverted; its failure is retained in `arrival-checks.txt`. No physical limits or admission checks were relaxed to obtain the focused results.

No new full-route campaign or synthetic force/atmosphere scaling was run. The authorized live restart is the next actual flight validation and will retain a frozen executable, source/configuration identity and continuous logs.
