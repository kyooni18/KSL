# MM305 Stage 0 baseline

Captured from `/Users/kyooni18/Code/KSPShuttleLander` at HEAD `a4b8f9994beb4a1feeb5a237f9f9f40103633f11`. The source-derived effective-model export is [effective_model_manifest.json](effective_model_manifest.json).

## Fixture and provenance

- Scenario: `ShuttleSim/scenarios/mm305-tf-hac-inlet-20km.ini` (SHA-256 `376a15c1daf00999fc51008b1bad6bd4a63be7cc6f57afe83885d27debb8dcfa`).
- Inputs: Cartesian MM305 inlet state and scenario runway overrides. The launch recipe uses seeded world, STS-N aero, and attitude models, with no atmosphere/aero/aero-book/attitude override, at 0.02 s; no gear-down start.
- Git working tree is dirty; all paths and the source hashes are recorded in the manifest. No unrelated edits were changed.
- No persistent ShuttleSim or live KSP process was active. The exporter launches the existing binary in lockstep, records its first telemetry at simulator time zero with UDP disabled, then terminates it. Binary freshness is `stale by mtime` relative to the pinned model source files.
- The captured telemetry is the effective initialized state of that binary artifact. It is stale by mtime relative to one or more pinned model source files, so it does not establish loaded-model parity with the current source snapshot; the source-derived section separately records current checked-out defaults and overrides.

## Effective model boundary

The manifest separates source-derived native world, atmosphere, aero, attitude response, vehicle initialization, simulator defaults, and scenario runway from the full application Final/vehicle configuration. The initial binary telemetry provides loaded world, atmosphere sample, aero forces, attitude response, vehicle state, and runway geometry. The application Final configuration is policy input only; this baseline does not establish its joint delivery envelope or learned runtime state.

## Build and validation evidence

- Build: not run. The shared build tree and unrelated ShuttleSim changes were preserved; rebuilding would replace shared artifacts.
- Focused tests: not run. No pre-existing independent Stage 0 test report was available, and candidate tests would compile shared outputs.
- Supplied analytic checks: reference only. Their algebraic identities do not establish native plant parity, runtime model loading, trajectory feasibility, or flight success.

This baseline is provenance and initialization evidence only. It makes no current-source plant-parity, trajectory, touchdown, or flight-success claim.
