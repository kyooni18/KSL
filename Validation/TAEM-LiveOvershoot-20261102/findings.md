# Live HAC overshoot investigation — 2026-11-02

Run: `runs/20261102T020820Z-taem-reference-control-72km`, archived source b182029, empty source.patch. This investigation reads existing telemetry only; it neither commands nor restarts KSP. No production guidance changes were made.

## Finding
The aircraft was not commanded to acquire the HAC effectively. Two guidance defects interact: the selected join moves after commitment, and the new best-L/D fallback suppresses lateral force whenever efficient lift is below the support term. This is not evidence of the aircraft failing to follow a substantial bank demand.

### 1. HAC geometry changed after the join was latched
At TAEM entry UT210227.89256597302, measured speed738.33m/s, runway-relative height26388.09m, along-35711m/cross+1084m. MM304 delivery is near the configured35km interface; the older85km handoff problem is not reproduced.

At t=0 the selected turn is361.883834deg, final radius3000m, growth0.0759909m/rad², range52299.04m. At t=0.1s growth becomes975.097373m/rad² while the SAME turn stays latched, and range becomes164431.71m. This is112.13km of added route in0.1s, not vehicle travel.

Using the actual spiral equations, the entry radius changes3003.03→41899.24m. The latched join moves from along-2384.72/cross-1.41m to along-3663.36/cross-38876.59m. Distance to the join jumps33.34→51.18km. The new position is no longer the previously selected tangent.

Source: TAEMGuidanceEngine.prepareGeometry (lines168–174 in b182029) still derives growth from the legacy minimum q-scheduled line, despite same-profile admission being introduced. On the following tick, selection reuses captureTurn (lines562–563) on the changed spiral. Growth is then frozen. Neither prior altitude/reference latching nor the angular progress fix makes this a consistent join. The old energy line continues to change actual geometry even though it was removed from direct route admission.

### 2. The new fallback consumes all turning allowance
At b182029 lines792–801, efficient lift is computed at best-L/D and lateral allowance is sqrt(max(0, efficientLift²-support²)). Support is approximately gravity*cos(gamma), with curvature correction; it is not derived from a planned descending turn. When efficientLift<support, this forces bank demand to zero. An aircraft already descending must not be treated as having no possible turn solely because efficient lift cannot maintain constant flight-path angle; a jointly planned descent can trade vertical acceleration for lateral force. This does NOT justify arbitrary bank or dropping safety limits.

A read-only replay of the actual authority-estimator history reconstructs the following (m/s²):

| t after handoff | Best-L/D lift | Reserved support | Lateral allowance | Recorded bank command |
|---|---:|---:|---:|---:|
|0s|3.685|8.076|0|0deg|
|30s|6.698|8.487|0|0deg|
|60s|4.650|8.377|0|0deg|
|90s|7.333|7.608|0|0deg|
|120s|7.493|8.885|0|0deg|
|150s|9.327|9.093|2.072|-6.36deg|

The force reconstruction uses the shipped polar/configured maximum incidence and observed estimator history; it is not a full mission replay or an independent actuator-authority certification. The zero bank demands themselves are directly recorded telemetry. Across the first4974 logged TAEM samples analyzed (~484s),4442 (89.3%) commanded exactly zero bank. Actual bank generally remains small. The mission remains acquisition throughout this analyzed interval; no HAC phase transition is recorded.

### Consequence
At t85.24s the aircraft crosses the runway threshold station at274.57m/s, height16497.08m, cross+2225.49m. At t480.1s it is39.23km beyond the threshold, still acquisition, referenceTracking=false. This is sustained loss of capture, not a touchdown or completed landing.

## Why previous offline checks did not establish readiness
Earlier offline continuations already failed capture. They warmed the flight-controller/trim history but initialized a new mission and new guidance authority estimator at handoff. Their geometry remained near the small initial spiral; they did not reproduce this live geometry jump.

A narrow replay with the recorded authority history reproduces the geometry-expansion mechanism but NOT the exact live value: it gives growth2138.29 (before the existing spiral clamp) and range233.82km instead of live975.10/164.43km. Adding recorded incidence-monitor history did not remove the discrepancy (ceiling is nil at these samples). Thus full mission/planner/NEP-state provenance still needs matching; do not call this an exact replay or claim estimator initialization alone quantitatively explains the difference. Telemetry and the flown source independently establish the actual jump and its code path.

## Required correction
1. Select geometry, actual tangent/join and descent together. Do not resize a committed spiral through a legacy line while retaining its old tangent angle. Regeneration must preserve actual join/control continuity and price the resulting geometry consistently.
2. Allocate lift jointly to an attainable descending turn; the fallback must preserve recovery steering instead of making constant-gamma support an unconditional prerequisite for banking.
3. Reproduce the observed first-two-tick geometry and zero-bank interval with the complete guidance state before another live comparison. Validate a recorded-state capture attempt, not merely builds, phase labels, or the disappearance of the prior dive.

Artifacts: `samples.json`, first handoff/geometry records, `authority-replay.txt`, `geometry-replay.txt`, `complete-history-replay.txt`, archived diagnostic harness. The temporary test was removed. Its passing result only means the read-only diagnostic executed, not that flight behavior passed. No new live run was started.
