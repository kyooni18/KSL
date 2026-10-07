# STS Variant Adaptation

## Status

The STS craft at `/Applications/.Misc/LMPKSP/saves/LunaMultiplayer/Ships/SPH/STS.craft` has **not flown a real flight**. This profile is setup work only. It does not establish aerodynamic, attitude-control, deorbit, or landing performance for STS.

## Profile isolation

Use `Configuration/sts.json` for this craft. Its `vehicle.modelId` is `STS`, which gives it a separate persistent physics history from `STS-N`. Keep this ID for later STS flights so observations accumulate in the same STS data book. Do not change the STS-N profile to switch vehicles.

The initial fallback values in `vehicle` were copied from the STS-N default profile because STS has no flight-derived values yet. In particular, estimated L/D (0.4), ballistic coefficient (700), AoA, speed limits, bank limits, and airbrake action group are **unverified starting values**, not STS measurements. Replace them as craft inspection and safe flight data justify. Passive and trajectory calibration are enabled, but learned values still require meaningful and safe flight coverage.

## Propulsion constraint

The saved craft contains three aft SSMEs and two AJ10 OMS engines. The OMS pair is mirrored about the same lateral line as the SSMEs in the saved part positions, so equal pair thrust should cancel most lateral thrust moment. This is a static geometry observation; loaded-vessel CoM and actual engine state must be checked in KSP.

The deorbit controller commands vessel throttle and plans from aggregate live vessel thrust. It does not select OMS engines or balance thrust moments engine by engine. For an OMS deorbit, configure the craft so the paired OMS engines are active and the SSMEs are shut down before planning. Confirm live `availableThrust`, `currentThrust`, mass, and attitude rates before and during a cautious burn. The inherited 18% deorbit throttle limit is not yet validated for this craft and should be reviewed against the live OMS-only thrust and planned burn duration.

## Before first guidance flight

1. Load `Configuration/sts.json` into the PyQt UI's saved configuration (`Runtime/PyQtUI/config.json`) before connecting, or enter the same profile values in the UI and apply them. The UI loads its saved configuration over the compiled default.
2. Confirm `modelId` is exactly `STS`, the KSC runway/site is correct, and the configured airbrake action group matches the craft.
3. In KSP, confirm both OMS engines are enabled together, the SSMEs are off for the deorbit burn, and measured thrust agrees with the expected OMS-only mode.
4. Start with a controlled OMS-only deorbit/attitude evaluation. Record mass, thrust, fuel state, attitude rates, and post-burn state. Stop if one OMS is unavailable or the craft develops sustained rotation.
5. Build STS-specific aero and control confidence from safe, staged flights before relying on full entry-to-runway guidance. Review certified model coverage and residuals after each flight.

## Current software boundary

The main landing and entry guidance is shared across craft through configuration and online physics identification. The separate model ID already isolates learned data. There is no craft-file importer, per-engine controller, or engine-moment estimator in the flight program; those are not required to establish a distinct calibration profile, but engine asymmetry must remain an explicit flight qualification item.
