# Rejected-reference flight control fix — 2026-11-02

Commit b182029 fixes a control ownership error left after 635a285: the same-profile check rejected an impossible Hermite descent, but flight control still consumed its height/slope/curvature and commanded the zero-incidence dive. A rejected or energy-insufficient reference now cannot drive altitude tracking. TAEM flies an efficient best-L/D lift vector, bounded by the existing trim/structural limits, with turning lift allocated after vertical support. Maximum-q protection remains active. This recovery is explicitly unqualified. References are regenerated from the measured state at the configured replan interval, on the preserved ground track, and adopted only after the existing force/energy screen passes. S-turns cannot spend energy against an unusable reference.

Also fixed measured HAC progress: near the center, polar angle can close faster than flown arclength. Angular reference progress is bounded by measured ground travel divided by the spiral metric. This preserves winding and prevents spending more descent-reference distance than the vehicle actually travels. The bound is geometric, not a tuned cap.

## Evidence
- The actual recorded first TAEM state now commands 12 degrees best-L/D incidence and zero bank, rather than the rejected curve's approximately 2.2 degrees followed by sustained zero incidence.
- Matched initial-state/controller-history continuation, before the final arclength-progress change: at 40.1 s altitude is22.09km and speed469.35m/s, compared with17.31km/741.52m/s under the ballistic command. The rejected reference is not tracked. The new profile is eventually adopted when the screen passes; no landing certification follows from that.
- This continuation still FAILS terminal delivery: unqualified Final recovery at479.9s, height961.21m, along-4532.46m, cross+5849.61m, speed65.52m/s. No touchdown/full-stop success is claimed.
- Six focused tests pass, including the real recorded handoff's fallback command and the demonstrated near-center HAC progress defect. The final progress fix has focused geometry verification; the quoted full continuation predates it.
- The fixture embeds genuine state/configuration from runs/20261102T040424-mm304-shooting-live-72km. No vehicle-strength or atmosphere perturbations were introduced.

## Live experiment
User explicitly requested fixing the issue and starting a run from the prior72km checkpoint. The failed offline delivery was reported before launch. Use the archived binary/config/source/hash and launch manifest of the new run for provenance. Startup verification only; respect the user's request not to monitor after launch. Do not describe the experiment as a demonstrated landing.

Remaining work includes generation of continuously attainable, energy-consistent descent and actual HAC join/capture validation. Best-L/D recovery is a bounded physical fallback, not a claim that the committed nominal profile is repaired or that the full landing problem is solved.
