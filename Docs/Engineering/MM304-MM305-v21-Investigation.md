# MM304 → MM305: v21 runway-27 investigation

Investigation of `runs/live-swift-v21-rwy27-crossrange-checkpoint72-20261031/`.
Status: **not fixed; no landing demonstrated**. This is a forensic checkpoint,
not a request to change flight guidance based on a successful build.

**Causal summary:** the reciprocal threshold and right-bank sign are correct.
MM304's requested crossrange acceleration greatly exceeds its lift-limited
response; it delivers an eastbound vehicle 25 km north of the threshold.
MM305 does take over *before* the five-kilometre target plane is crossed.
However, its survey is delayed past earlier priced HAC opportunities, its
ranking can prefer partially priced bad candidates over completed good ones,
and its selected reference commands a push-over which unloads lift while
banking steeply. The measured turn consequently collapses. Replanning preserves
winding but not attainable force/command continuity or short actual rejoins.
There is also a substantial, independently measured predictor pitch-authority
error. This is a combination, not another demonstrated geometry-sign error.

## Evidence and reproduction

- Baseline `36c4c99` **plus** the run's `source.patch`. At investigation start,
  `git diff` was byte-identical to that patch. All 13 pre-existing modified
  source/test files are preserved, not included in this investigation commit.
- Configuration: `live.jsonl.configuration.json`, runway 27, spherical
  reciprocal threshold, `captureTargetRange=5000`, `maximumBank=70°`, RCS
  enabled by the vehicle configuration's default. No release rebuild or live
  operation was performed. Debug-only offline diagnostics are used.
- Foundation Memory was searched for **KSL TAEM capture**, then original
  sources on reciprocal geometry, the energy investigation, and HAC continuity
  were read. The old remote endpoint is stale; the existing local MCP client
  credential works with the local Foundation endpoint. No credentials belong
  in this report.
- Reconstruct the logged spherical frame and decoded proposals:

  ```sh
  python3 swift/KSL-Landing/Tools/investigate_transition.py \
    runs/live-swift-v21-rwy27-crossrange-checkpoint72-20261031 \
    --output runs/mm304-mm305-v21-investigation
  ```

  Outputs include exact event states, per-sample geometry/energy/acceleration,
  decoded proposals, and input SHA-256 hashes. Run artifacts remain local
  under ignored `runs/`; this report and analysis tools are tracked.
- Build the **debug** diagnostic executable and the standalone native probe:

  ```sh
  swift build --package-path swift/KSL-Landing -c debug --product ksl-landing
  B=swift/KSL-Landing/.build/out/Products/Debug
  swiftc -parse-as-library -I "$B" -I swift/KSL-Landing/Sources/CSQLite \
    swift/KSL-Landing/Tools/probe_transition.swift \
    "$B"/KSLCore.o "$B"/KSLControl.o "$B"/KSLPhysics.o \
    "$B"/KSLGuidance.o "$B"/KSLDiagnostics.o -lsqlite3 \
    -o "$B"/ksl-transition-probe
  "$B"/ksl-transition-probe \
    runs/live-swift-v21-rwy27-crossrange-checkpoint72-20261031 \
    runs/mm304-mm305-v21-investigation --dense-entry
  ```

  The module-object layout above is this machine's Xcode SwiftPM backend.
  Keep the executable beside `KSL-Landing_KSLPhysics.bundle`. Rerun the Python
  reconstruction afterward to include the survey opportunity in `events.json`.
  `probe_transition.swift` warms the FCS from measured states, applies dynamics
  invalidations and proposal epoch rejection, replays the native tracker, and
  compares identical initial states and recorded commands with the plant.
  It also surveys every recorded entry state inside the configured terminal
  range, plus a five-second grid outside it. Outputs: `matched-warm-plant.csv`,
  `tracking.csv`, `survey.json`, and `probe-summary.txt` (stdout redirect).
- Existing `entry-energy-probe` was also run against this run, producing
  `matched-command-energy.csv`. Its direct `pitchAcceleration` column does
  **not** warm trim or model `observe`'s replacement of measured authority;
  use the standalone warm probe for that comparison.
- Numerical cross-checks: 93 forecast initial chart points agree with the
  Python reconstruction to **3.5e-10 m**; warmed recorded FCS commands reproduce
  to **9.9e-14** maximum control-input error. Thus the diagnostic replay is
  using the actual flown law/commands, not merely a plausible HEAD build.
- SHA-256 of `live.jsonl`:
  `evidence hashes are in source-hashes.json` (all three input files are hashed
  on each reconstruction). No compile-time regression tests were added: there
  is no production-guidance change in these investigation checkpoints.

## Geometry and conventions

Elapsed time is from UT **196014.14879776392**. Crossrange is positive right
of selected runway 27, i.e. approximately **north**, not south.

- `KSLCore/Geometry/RunwayGeometry.swift:15-24,35-85` defines the reciprocal
  and azimuthal-equidistant spherical chart. Recomputing the reciprocal in
  native Swift gives latitude **-0.048610689129°**, longitude
  **-74.489600832741°**, heading **269.999797454241°**, exactly matching the
  logged runway after angle wrapping. Along=+2500 returns the 09 threshold,
  **-0.048611111100°, -74.728333333300°**, separated by **2500.000000 m**.
  This is the actual reciprocal threshold, not the old threshold with a
  changed label/heading.
- MM304's objective is **along=-5000, cross=0**, approximately
  **-0.048607313405°, -74.012135831646°**: east of the 27 threshold.
  `handoffAlong=-8000` is still in the configuration but does not override
  `terminalCaptureRange=5000` in this objective. Altitude **25000 m is ASL**;
  `RunwayFrame.height` and forecast capture/track heights are runway-relative
  and differ by **68.44 m**. The objective subtracts elevation before using
  `frame.position`; no 68 m datum mistake explains the miss.
- Initially along/cross are **+798212.05/-9121.74 m**: approaching from the
  **west, slightly south**, almost reciprocal to runway 27. It drifts north
  before meaningful aerodynamic turn authority develops. Maximum crossrange
  is **+40782.73 m at +343.08 s**. It passes **north**, moving east-southeast,
  not down the runway-27 approach centerline.
- The log has **no separate KSC map-marker coordinate or KSC distance**.
  `runway09ReferenceRange/Bearing` in the reconstruction explicitly use the
  old 09 threshold as a *site-reference proxy*, not as a measured KSC marker.
  At closest threshold approach these distances are: 09 proxy **24180.81 m**,
  selected 27 threshold **23074.63 m**, MM304 objective **21572.60 m**.
  Substituting any of these for another is wrong. A UI/map distance to some
  other KSC point cannot be established from this log alone.
- Heading errors below are **transported runway-forward minus velocity
  course**, and **bearing to the MM304 objective minus course**, respectively.
  At first MM305 request, course is **109.066°**, threshold bearing
  **179.670°** (bearing error **70.603°**), objective bearing **168.322°**
  (error **59.255°**), but runway-forward error is **160.933°**. These are
  different objectives; a modest bearing error is not runway alignment.
- Chart derivatives are measured from successive positions, not identified
  with orthonormal velocity components. At +402.94, along/cross derivatives
  are **-1355.23/-128.91 m/s**; at +463.92, **-777.19/-268.60 m/s**.
  At +400.02 the controller's `crossRate=-v·right` is **+128.21 m/s**, whereas
  actual `d(cross)/dt=-122.05 m/s`: opposite sign by its *error-rate*
  definition, and a different magnitude because of chart/radial metrics.

## Event timeline and geometry/energy table

The first two columns below extend the original checkpoint with earlier
model opportunities and later physical bounds. Crossing means the first
**recorded** sample past the plane: the bracket is **+470.92..471.00 s**, with
linear interpolation **+470.9671 s**. Closest distances are recorded minima.

| Event | Elapsed s | ASL m | Mach | Along m | Cross m | Threshold distance m |
|---|---:|---:|---:|---:|---:|---:|
| First in-region priced HAC opportunity found | 402.94 | 31713.78 | 4.528 | 66694.19 | 37267.34 | 76400.06 |
| Last such entry-survey opportunity found | 432.32 | 27493.78 | 3.882 | 30244.71 | 32386.92 | 44313.15 |
| First MM305 request | 463.92 | 23959.05 | 2.850 | 143.37 | 24892.37 | 24892.78 |
| First result / acquisition owns controls | 464.02 | 23950.10 | 2.846 | 65.70 | 24865.50 | 24865.59 |
| Delivered fresh-epoch result incorrectly discarded | 469.04 | 23527.93 | 2.638 | — | 23494 approximately | 23777 approximately |
| First `headingAlignment` label | 470.00 | 23446.44 | 2.609 | -4330.55 | 23223.73 | 23624.04 |
| First crossing of objective along=-5000 | 471.00 | 23358.37 | 2.589 | -5022.67 | 22939.11 | 23482.55 |
| Closest threshold | 477.18 | 22689.09 | 2.544 | -9205.12 | 21159.02 | 23074.63 |
| Closest objective | 483.94 | 21686.54 | 2.524 | -13645.55 | 19188.07 | 23545.34 |
| 176 km rejoin substituted | 494.18 | 19562.78 | 2.530 | — | 16238 approximately | 25920 approximately |
| Optimistic still-air range bound first exhausted | 595.00 | 6718.12 | 0.400 | -36335.65 | -3654.68 | 36518.98 |

Additional state at each event (V = measured airspeed, vz = radial vertical
speed; degrees for all angles). **ΔE** is rotating-frame mechanical energy
minus the flown MM304 objective's energy, in **kJ/kg**. After handover the last
MM304 objective is held only as a diagnostic reference; this is **not** a HAC
reachable-energy margin. A positive value does not prove a return HAC fits.

| Elapsed s | V m/s | vz m/s | ΔE kJ/kg | Objective distance km | Runway / objective heading errors ° | Bank target / actual ° | AoA target / actual ° |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 402.94 | 1440.39 | -150.22 | +810.50 | 80.801 | +174.57 / +21.97 | +34.09 / +35.13 | +21.80 / +20.59 |
| 432.32 | 1193.22 | -131.34 | +447.64 | 47.865 | +169.93 / +32.48 | +37.89 / +37.98 | +18.46 / +15.67 |
| 463.92 | 859.69 | -89.51 | +73.58 | 25.418 | +160.93 / +59.26 | +32.51 / +34.72 | +20.80 / +17.00 |
| 464.02 | 858.66 | -89.48 | +72.61 | 25.376 | +160.90 / +59.38 | +32.51 / +34.71 | +20.80 / +17.47 |
| 469.04 | 794.98 | -83.47 | +16.14 | 23.532 | +158.42 / +65.14 | +58.57 / +62.52 | +15.84 / +15.96 |
| 470.00 | 785.84 | -86.09 | +8.18 | 23.233 | +157.87 / +66.21 | +57.70 / +62.81 | +13.92 / +14.49 |
| 471.00 | 779.69 | -90.31 | +2.57 | 22.939 | +157.48 / +67.53 | +57.36 / +61.81 | +11.92 / +8.15 |
| 477.18 | 764.74 | -126.85 | -15.03 | 21.573 | +156.46 / +77.70 | +59.88 / +60.77 | +0.61 / +3.47 |
| 483.94 | 756.75 | -170.77 | -30.21 | 21.046 | +155.81 / +90.07 | +65.09 / +64.96 | -0.31 / +0.48 |
| 494.18 | 754.03 | -243.35 | -51.62 | 22.244 | +155.79 / +108.91 | +69.66 / +69.00 | -0.31 / -0.19 |
| 595.00 | 125.80 | -20.15 | -448.01 | 31.548 | +47.34 / +53.98 | +12.56 / +12.97 | +6.78 / +6.57 |

Thus MM304 itself did **not** fly past the along=-5000 plane before MM305
owned control. This is not v18's missing-search failure. The closest objective
distance is **21045.64 m**, not the distance to KSC, the threshold, or an along
coordinate. Passing was not by itself disallowed or evidence of failure.

The first terminal result examined 54 candidates, with **zero completed,
zero feasible** candidates. It nevertheless supplied the advisory route, as
required by the non-veto policy. Selected reference: final radius 3000 m,
remaining winding 3.52628 rad, first spiral station 8944.14 m along the path.
Its full advisory pricing reports peak load **31.222 g**, peak bank **89.981°**,
and minimum lift margin **-292.111 m/s²**. Those are properties of the
reference, not loads or bank actually achieved by the vehicle. Acceptance
and the `headingAlignment` phase label are not capture certificates.

## MM304: why lateral delivery fails

Paths below are under `swift/KSL-Landing/Sources/`.

1. **Not the reciprocal sign defect again.**
   `KSLGuidance/Entry/EntryCommandLaw.swift:159-167` constructs the local
   runway-forward/right frame and passes an absolute along separation.
   `EntryLateralController.swift:54-67` applies the signed alignment. A positive
   right bank on this eastbound reciprocal approach accelerates **south**, i.e.
   toward decreasing runway-27 crossrange, exactly as required. Projecting
   measured aerodynamic force plus Coriolis/centrifugal acceleration onto
   ground-track right matches the Cartesian measured velocity derivative with
   RMS residual **0.0579 m/s² over +400..464**, and **0.0583 over +464..490**.
   There are **zero MM304 bank reversals** in the examined budget samples;
   no reversal-dwell delay explains this run.

2. **The law is a local cross-coordinate intercept, not a complete reciprocal
   arrival/HAC solution.** In a fixed orthonormal chart, the cubic's initial
   acceleration `6e/T²+4eDot/T` is correct for terminal cross position and
   cross velocity zero. It does not specify **positive along velocity** or
   runway-270 arrival heading: an eastbound centerline crossing also satisfies
   that lateral endpoint. The projection uses `alignment*demand`, not inversion
   by alignment; the resulting runway-right contribution scales with
   `alignment²` away from parallel flight. Bearing pursuit fills the singular
   perpendicular case, but is not an exact endpoint solve. Its horizon ignores
   chart Jacobians, acceleration, speed loss, roll/pitch lag and turn-path
   length. Chart transport matters strongly far away and by several percent
   near capture. Those approximations are real, but cannot explain the
   **orders-of-magnitude** authority gap below as a sign error.

   The six-second floor avoids division by zero near the along plane. After
   passing it, absolute separation grows again, so this is not a finite-time
   arrival solution or an explicit return-HAC policy. **That post-pass MM304
   behavior was not flown here:** MM305 already owned controls.

3. **Early bank exists, but useful aerodynamic lateral force does not.**
   At +119.98, target/actual bank are 70.00/70.21°, but lift is only
   **0.0651 m/s²**. Northward chart drift remains **+182.66 m/s**.
   Later the altitude/energy support allocation caps the turn well below
   maximumBank; merely increasing a lateral gain cannot overcome it.

| Elapsed s | Intercept horizon s | Signed lateral request m/s² | L/m m/s² | Required vertical lift m/s² | Energy bank ° | Target-lift lateral* m/s² | Measured aero-right m/s² | Measured total-right m/s² |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 299.98 | 123.74 | +17.371 | 1.073 | 0.855 | 37.19 | 0.649 | 0.706 | 0.759 |
| 340.00 | 93.20 | +28.383 | 2.111 | 1.688 | 36.92 | 1.268 | 1.312 | 1.354 |
| 400.02 | 52.27 | +72.697 | 5.325 | 4.132 | 39.11 | 3.048 | 3.224 | 3.248 |
| 440.04 | 24.81 | +259.081 | 8.931 | 6.607 | 42.29 | 5.316 | 5.453 | 5.465 |
| 460.02 | 9.66 | +1493.220 | 7.216 | 8.274 | 0.00 | 4.180 | 4.377 | 4.235 |
| 463.92 | 6.36 | +3343.416 | 7.692 | 8.700 | 0.00 | 4.134 | 4.386 | 4.364 |

*`(measured L/m)*sin(limited target bank)`, not a prediction of extra lift from
a new AoA. The much larger unsaturated intercept request is not the command
actually realized. Total-right includes rotating-frame terms; finite-difference
noise and changing controls account for the remaining small residual.*

4. **Vertical protection is a material trade, not a hidden sign gate.**
   `EntryCommandLaw.swift:96-108,120-151` chooses least-drag vertical support
   (or maximum lift when support is unavailable), with the configured 20%
   aerodynamic uncertainty. `EntryLateralController.swift:52,68-77` permits
   only lift left after vertical demand. At +340 the ratio vertical/lift is
   about 0.8, hence the familiar **acos(0.8)≈36.87°** bank, despite a huge
   lateral miss. By +460 even wings-level measured lift is below requested
   vertical recovery. The shaped bank and outer target limiter cannot unload
   instantaneously; their lag explains nonzero actual bank with energyBank=0.
   There is no q/load-protection diagnosis in these samples.

5. **Energy protection is exhausted; the old bypass diagnosis must not simply
   be reapplied to this new working tree.** `EntryDragReference.swift:108-132`
   solves its range integral, but `EntryCommandLaw.swift:137-151` applies the
   actual drag-work budget and makes the single AoA decision. At +400.02,
   observed/reference drag are **8.160/8.392 m/s²**. At +440.04 they are
   **11.667/6.199** with `bestRangeRecovery=true`; at +463.92 **11.171/0**,
   again in the recovery fallback. Reserve-adjusted capture energy first
   becomes negative at **+461.90**, before raw objective ΔE becomes negative
   at **+471.68**. At +463.92 raw margin is **+73578.74 J/kg**, reserve
   **90597.13 J/kg**, adjusted margin **-17018.39 J/kg**. Zero reference drag
   is not evidence of zero achievable drag while trying to maintain altitude.
   Current recovery/turn/drag objectives are mutually strained, not proof of
   an unflown old modulation/drag-boost bypass.

## MM305: permission, ranking, ownership and execution

### Survey is still late, but worker latency is not the primary cause

- `LandingMissionController.swift:75-104` retains altitude and **Mach≤2.85**
  gates unless a forecast arms it first. Range permission passes at **+381.96**;
  altitude-plus-five-second-lead passes at **+409.50**; Mach only at **+463.92**.
- The offline survey finds the first **in-region, priced-family** opportunity
  at **UT 196417.088797553 (+402.94)**: one completed feasible path of
  **124970.84 m**, priced touchdown **87.205 m/s**, energy-height margins
  **+286.55 m excess / +140.04 m deficit**. It is an advisory model opportunity,
  **not** closed-loop certification. There are 188 feasible-family states in
  the dense in-range survey, last at **+432.32**. For example, **+419.76** is
  already inside the altitude/range gates, but Mach **4.188** prevents search.
  Actual search begins **60.98 s** after the first found opportunity.
- This does **not** prove an earlier takeover would succeed: *none* of the
  1176 surveyed states selects a feasible plan, even where its family contains
  one. This is an independently important ranking defect described below.
- `EntryCaptureForecaster.swift:129-143` also waits for altitude≤30 km,
  Mach≤2.85, and descent. It then sets `capture.feasible=false` outside
  **terminalCaptureRange (5 km)+five seconds' travel**. That still confuses
  delivery accuracy with a permitted return HAC. This does not directly gate
  live fallback search anymore, but makes forecasts unsuitable for expressing
  the requested permitted overshoot/return behavior.
- 93 delivered entry proposals have maximum simulated age **1.88 s**;
  74 delivered terminal proposals have maximum age **0.12 s**. The first
  terminal request is **UT 196478.068797521**, result recorded/committed/control
  acquisition at **UT 196478.168797521**: **0.10 s** later. The log records
  delivery, not an independently timestamped worker finish; actual finish is
  between request and delivery. No telemetry gap exceeds **0.24 s** during
  this run. Late-Mach permission, not a minute-long calculation, explains
  the first-search timing.
- `MissionPlanning.swift:47-59` prioritizes terminal work; `FlightRuntime.swift:
  85-135` cancels entry work when terminal search takes priority and invalidates
  stale generations after configuration changes. Request/cancel wall-time
  events themselves are not logged, so all canceled/never-delivered requests
  cannot be reconstructed from these files.

### A delivered proposal need not be accepted

Eight delivered proposals are discarded by `KSLRuntime/FlightComputer.swift:
53-58`: `proposal.time <= dynamics.lastChange.time`. All eight are requests
stamped **at** the new dynamics epoch, rather than old requests with large
latency. In particular, the **+468.92** terminal request, delivered **+469.04**,
is rejected after the center-of-mass revision. The active plan remains the
**+463.92** plan until **+474.14**; the apparent +469.04 replacement in a naive
proposal decode was never flown. Two rejected proposals are entry, six
terminal, so there are **68 actual terminal replacements**, not 74. This is a
fresh-request/same-UT invalidation bug, separate from RPC recovery.

`LandingMissionController.swift:145-154` accepts any returned candidate and
resets the tracker; `MM305Admission.swift:12-35` returns `accepted=true` and
retains physical/qualification violations as advisories. That correctly keeps
flying, but **is not** a reachability claim. Control ownership really changes
at +464.02, by `LandingMissionController.swift:281-305`, not merely when a
forecast or admission flag changes.

### Selection and reference generation explain why the return does not work

1. **Inconsistent pricing/ranking.** `HACPlanningEngine.swift:174-204` stores
   both complete feasible candidates and failed candidates whose energy
   pricing stopped at their first violation. Failed prefixes are extended
   with *unpriced* stations. `fitCost` at **339-372** compares prefix-only
   deficits and roll travel with fully priced routes. The selected bad route
   alone is then completed in advisory mode at **377-384**, with **no rerank**.
   At +402.94 the family contains a feasible path, yet the completed selected
   advisory has **31.571 g**, minimum lift margin **-302.377 m/s²**. Non-veto
   behavior does not require preferring this over a better candidate.

2. **The first accepted reference is not physically reachable as priced.**
   Its 3 km *final* radius is not its measured-state dynamic turning radius:
   `fit.turningRadius=40000 m`, already clipped at the configured maximum,
   and local model turn rate **0.0152493 rad/s**. The actual unclipped radius
   at that speed/rate is larger. The selected 47.835 km route instead asks for
   the enormous loads quoted above. Geometric proximity/plan acceptance does
   not establish that a gentle return spiral is fitted.

3. **Direct command mechanism: unload AoA while maintaining a steep bank.**
   The cubic/reference curvature in `HACPlanningEngine.swift:138-170` and its
   regeneration yields a large *negative* vertical-normal-force demand.
   `EnergyManagementEngine.swift:182-195` prices its magnitude using `hypot`
   while banking against `max(0.1,vertical)`; this is not an attainable
   positive-lift, ≤70°-bank force vector for negative vertical demand.
   The runtime tracker, `PathTrackingEngine.swift:62-94`, uses maximum available
   lift to allow bank but clamps required lift from `vertical/cos(bank)` to
   **zero** when vertical is negative. Its bank and AoA therefore describe
   incompatible force allocations.

   The native replay shows at **+464.02** reference vertical curvature
   **-4.4502e-5 m⁻¹**, vertical demand **-24.95 m/s²**, raw AoA **-0.229°**,
   raw bank **+58.58°**. The target limiter initially retains AoA **20.80°**
   and bank **32.51°**, then slews toward the new commands. At **+474.14**, a
   regenerated reference demands **-30.90 m/s²**, raw AoA **-0.301°**, bank
   **57.95°**. At **+483.94**, limited/actual AoA **-0.310/0.480°** with bank
   **65.09/64.96°** leaves measured lift only **0.424 m/s²**, aero-right
   acceleration **0.384 m/s²**, total turn acceleration **0.364 m/s²**.
   Sink increases from **-89.48** at takeover to **-170.77 m/s**. The vehicle
   passes the objective bearing with almost no useful turn force, despite
   having followed the bank command. This is not simply sluggish roll response.

4. **Short acquisition labels conceal tracking loss and long rejoins.**
   The initial geometric join ends at path distance **5407.5 m**; the first
   station *tagged* spiral is **8944.1 m** because of coarse path sampling.
   At +470 the nearest-reference projection jumps from an acquisition branch
   to a spiral-interpolated station, with **3140.63 m tracking error**.
   `headingAlignment` after only 5.98 seconds is not confirmed acquisition.
   At +474 onward `joinSegment = continuing ? .spiral : .acquisition`
   (`HACPlanningEngine.swift:312-335`) labels the new acquisition curve as
   spiral. Its cost consequently counts that work as spiral too.

   The clearest counterexample is the accepted **+494.18** plan: total length
   **203880.87 m**, actual rejoin to the analytic spiral **175949.7 m**, but
   fitted response/join diagnostic only **6242.1 m**. The 176 km acquisition
   is counted as spiral, defeating the short-acquisition preference. Winding
   does **not** renew in any accepted continuing plan (3.5263 rad initially,
   0.00943 rad at the end), but that is insufficient for command continuity.
   At +489.18 raw bank jumps from about **+69.85° to -69.94°**; at +494.18
   raw AoA jumps from near **-0.31° to +14.45°**. The limiter remains continuous
   and bounded; the moving reference underneath it does not.

   Later, actual bank reaches **147.50° at +519.42**, while limited target
   is **48.46°** and all logged bank targets stay within **±70°**. The plant/
   FCS coupling during that upset needs separate validation; it is not an
   authorized increase of maximumBank, and occurs *after* the miss is evident.

## Forecasts and model validation

All 93 delivered entry forecasts are infeasible. Their actual geometry remains
**(-5000,0), 25000 m ASL, terminal slope -24°**, rangeMultiplier **0.8**.
Accepted objective speeds change **748.859 → 757.222 → 757.118 m/s** across
the dynamics invalidations/environment reseeding, not by moving the aim point.
Representative forecast ends (heights converted from logged runway-relative
to ASL):

| Request elapsed s | End elapsed s | Forecast end ASL m | Mach | Along / cross km | Deficit |
|---:|---:|---:|---:|---:|---|
| 0.00 | 515.60 | 21683.73 | 0.977 | -26.543 / +13.917 | insufficientAltitude |
| 121.46 | 513.36 | 21637.94 | 0.979 | -21.530 / +12.095 | insufficientAltitude |
| 362.00 | 517.10 | 21563.08 | 0.930 | -16.184 / +13.555 | insufficientAltitude |
| 398.92 | 511.02 | 21964.80 | 0.994 | -18.366 / +12.529 | insufficientAltitude |
| 459.48 | 508.38 | 21977.85 | 1.039 | -20.005 / +13.474 | insufficientAltitude |

These forecasts do warn of failure, but their final state is not the first
MM305 opportunity. It is the result of continuing through the Mach/range/
lower-altitude restrictions. `analyze_capture_run.py:43-45` replaces all
search-envelope tests with **threshold distance≤5 km+travel** whenever
`captureTargetRange` exists; its `headingError` is threshold-bearing error,
not runway-heading error. Its gate labels must not be used for this code.
The existing v21 analysis also predates the tail of the live log; the forensic
reconstruction uses all **10587** samples through **+823.62**.

### Identical-state, identical-command measurements

The pre-existing working changes remove the unsupported high-Mach pitch
stiffness, retain a measured total-actuator anchor and RCS flag, simplify the
AoA energy budget, and pass snapshots into predictors. They are preserved.
They improve important prior problems, but do **not** yet validate this plant.

- Over **+400..464**, measured rotating mechanical energy loss is
  **771556.73 J/kg**, integrated measured drag work **772221.00 J/kg**
  (difference **0.086%**), one-step plant energy loss **813169.04 J/kg**.
  The loss is genuine drag work, not merely altitude traded for speed.
  Median measured/model force ratios at the same measured AoA/q/Mach are
  **0.997 lift / 0.951 drag**. Over +464..490 they are **0.946 / 0.882**.
  The gross collapse in lateral force follows commanded AoA unloading, not
  an order-of-magnitude aero-force discrepancy at identical AoA.
- The warm probe uses actual actuator initial controls and recorded commands,
  both with the identified trim anchor and with it disabled. Over +100..300,
  pitch-acceleration RMS error improves from **0.01337 to 0.00351 rad/s²**
  with the anchor. But over **+400..464**, it is still **0.42418 rad/s²**
  (without trim **0.40737**), one-step AoA RMS **0.09277°**, turn-acceleration
  RMS **0.46126 m/s²**. Small per-tick angle errors do not certify a
  100-second forecast.
- Remaining concrete model error: `ShuttlePhysicsEngine.swift:40-61` replaces
  measured total authority with **max(measured, provisional q-only authority)**
  and marks it identified. At +400.02, measured pitch acceleration authority
  is **0.07904 rad/s²**, inherited q-fit **0.89940**; at +463.92 **0.15018**
  versus **0.99126**. That overwrite can amplify pitch response by factors
  of 6–11 despite removing stiffness in `RotationalDynamicsEngine.swift:31-46`.
  At +400.02 the exact-command warm plant predicts pitch acceleration
  **+0.11523 rad/s²**, while measured next-step acceleration is **-0.05657**.
- This is not RCS being disabled: every issued command requests RCS, and every
  measured state after the initial pre-command sample has it on. The probe
  retains it. Measured total torque is not isolated RCS torque, so RCS versus
  wheel/surface contributions cannot be separately identified from this log.
- Roll/bank one-step errors are much smaller before the upset: +400..464
  roll-acceleration RMS **0.02013 rad/s²**, bank RMS **0.00537°**. Over
  +490..540 these grow to **7.363 rad/s² / 1.467°**. No high-confidence
  assertion about late-upset roll/yaw/RCS dynamics is justified by the current
  provisional plant. Low-q total-authority preservation must not be removed
  just because its high-q extrapolation is wrong.

## When recovery is lost, and what is not established

- At takeover, there is no priced feasible candidate in the *actual searched*
  family; it is wrong to call the accepted route captured. Conversely that
  finite, flawed family failing does **not** prove all physical return HACs
  impossible at +464, or prove the fixed objective globally unreachable.
- The local MM304 endpoint demand is already far outside available turning
  force hundreds of seconds earlier, but failure of that fixed-horizon
  intercept is not proof that a longer, state-fitted return is impossible.
- A defensible **latest** loss bound uses the existing optimistic still-air
  screening formula from `LandingMissionController.swift:233-255`, applied
  diagnostically even while a plan exists. The table polar maximum L/D is
  **3.1092126**; favorable ±20% uncertainty gives **4.6638189**. With its
  additional ballistic reserve, the bound is
  `L/D*h + (L/D/2+1)*V²/g`, compared with the nearest usable pavement after
  braking reserve, ignoring all turn cost. At **+595.00** this generous range
  is **36509.49 m**, versus pavement distance **36518.98 m**, and thereafter
  the vehicle cannot return within that still-air modeled envelope. At +600.02
  the shortfall is **773.32 m**. This is a model-derived upper bound, **not**
  an exact global time of physical irrecoverability, and must never veto flight.
- The earlier exact point of physical no-return remains unestablished because
  control/force models and candidate completeness are insufficient. The
  intermittent +402.94..432.32 priced opportunities likewise are not verified
  captures. The missing independent KSC marker and unlogged canceled-worker
  timing cannot be invented.
- Last log sample: **+823.62**, ASL **1974.34 m**, airspeed **33.61 m/s**,
  sink **-27.33 m/s**, still airborne. `stdout.txt` ends in disconnection/
  recovery failure. There is no verified runway touchdown, rollout or stop.

## Smallest justified next work and discriminating validation

**Do not start with another reciprocal sign/gain patch.** No production
guidance change was made here. There is no evidenced one-line fix for the
whole flight. The smallest bounded next experiment is MM305 force/reference
consistency, with the following separately testable defects kept separate:

1. **Repair/validate the measured-state plant before using its verdicts to
   tune guidance.** Keep the existing low-q measured-total/RCS anchor, but do
   not let an unvalidated inherited q-fit overwrite measured total authority
   as identified evidence. Re-run matched commands at +300, +400.02, +440.04,
   +463.92 and the late upset; compare actuator-state/trim-aware torque,
   pitch/roll response and force residuals against measurement confidence and
   the configured aerodynamic uncertainty. A short open-loop command replay
   from each identical state separates plant error from changing guidance.

2. **Make advisory MM305 references physically realizable as commands.**
   Replay +464.02 and +474.14 with the same path/state/feedback and inspect
   the demanded/realized normal-force vector. Generate the closest available
   lift/bank allocation or a bounded-curvature vertical reference when a
   positive-lift, ≤maximumBank vehicle cannot supply the reference push-over.
   Do not permit a large-bank/near-zero-lift pair to masquerade as a turn.
   Limits must come from the polar, measured authority/response, load/bank
   configuration and pitch/roll dynamics, not a newly guessed bank/altitude.
   This must still fly an advisory fallback rather than reject the vehicle.

3. **Reprice comparable candidates before ranking; preserve actual rejoin
   semantics.** Use the +402.94 feasible-family/selected-infeasible case to
   test selection with consistent pricing and a retained all-bad fallback.
   Include actual join length/curvature independent of a `.spiral` label;
   replay +489.18 and +494.18 to prevent 176 km rejoins or raw command flips
   from winning a short-acquisition preference. Winding continuity alone is
   not enough. Add only the specific selection/force-allocation regressions
   needed when those production changes are made.

4. **Separate survey permission from admission/ownership and the 5 km aim.**
   Compare an earlier advisory survey using the *existing* terminal-range
   lead (not a new guessed Mach threshold) against the baseline at +402.94,
   +419.76 and +463.92. Remove the 5 km forecast-feasibility restriction for
   return HAC assessment, without moving MM304's centerline objective.
   Use the same repaired plant and bounded reference in both branches.
   This distinguishes late permission from bad planning/execution; merely
   opening search earlier is not a validated cure. Tag requests with dynamics
   revision/generation so a post-change request at the same UT is not mistaken
   for pre-change work. That is distinct from timeout/reconnect recovery.

5. **Only then conduct one controlled live comparison**, if needed, using
   the explicit logged runway-27 configuration and checkpoint. Stop the old
   fly process before load/restart, never rebuild a binary in use, capture
   `source.patch`, configuration and request/revision/completion/acceptance
   timestamps. Compare measured side acceleration, retained energy, actual
   rejoin time/distance and reference continuity, not just phase flags.

All of these must preserve **maximumBank**, the limiter's **no-overshoot**
step, configured **RCS availability**, **10 s per-RPC deadline**
(`KSLKRPC/Transport/TCPTransport.swift:13`, `KRPCSession.swift:55-69`), and
reconnect/resume handling (`KSLRuntime/FlightRuntime.swift:178-190,218-234`).
An advisory never authorizes stopping guidance. A successful landing claim
still requires verified runway touchdown and rollout through full stop.
