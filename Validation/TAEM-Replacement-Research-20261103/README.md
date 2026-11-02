# TAEM/MM305 replacement research evidence — 2026-11-03

The engineering report is `Docs/Engineering/TAEM-MM305-Replacement-Research-20261103.md`.

This directory contains deterministic reductions of the genuine recorded run
`runs/20261102T152517Z-dessert-150ms-live/` and source/provenance fingerprints.
It contains no new flight, alternative trajectory, plant scaling or simulation
campaign. Original flight files were read only and remain in their run directory.

## Contents

| File | Purpose |
|---|---|
| `extract_ksl_taem_flight.py` | Standard-library-only, read-only log/provenance extraction. Does not import KSL or contact KSP/kRPC. Writes only its explicit output directory. |
| `flight-evidence.json` | Exact hashes, effective configuration, source/resource checks, initial state and first-minute/all-TAEM reductions, and reconstruction limitations. |
| `flight-seed.json` | The last recorded pre-TAEM row and first TAEM row, retaining actual attitude, actuators, trim/authority diagnostics and previous target. **Not** a complete serialized private controller/limiter/estimator snapshot. |
| `flight-first-minute.csv` | All 591 TAEM observations in the closed first 60 simulation seconds; no interpolation or resampling. |
| `source-index.json` | Fingerprints of 36 referenced backend, model, document, instruction and UI files; identifies current-worktree-only UI/instructions separately from the flown backend revision. |
| `research-preservation.json` | Before/after SHA-256 of the 15 pre-existing modified tracked files, and the absence of a backend diff. |

## Reproduce the data reduction

From `/Users/kyooni18/Code/KSL`:

```bash
python3 Validation/TAEM-Replacement-Research-20261103/extract_ksl_taem_flight.py \
  --run runs/20261102T152517Z-dessert-150ms-live \
  --output /tmp/ksl-taem-research-recheck
```

The alternate output avoids overwriting committed evidence. The script hashes
the frozen executable, configurations, resource bundle and log, checks source
identity against the run manifest, and checks that the log did not change during
extraction. It derives the same spherical runway chart and the support expression
present in the inspected TAEM source. It does not reconstruct hidden online
estimator state or declare observed lift to be maximum lift.

The sample table in the report uses first actual observations at or after 0, 10,
30 and 60 seconds. The +60.06 s table sample is just outside the closed first-minute
CSV window, which ends at +59.96 s. JSONL lines are one-based; `sequence` is a
different logged counter. `ut` is KSP simulation time, not UTC.

## Verified findings

All 591 first-minute samples command exactly zero bank, remain in acquisition,
report `acquisitionTurnUnavailable`, and have reference tracking off. Measured
bank ranges from −0.355° to +1.597°; no pitch/roll saturation is reported. The
first nonzero TAEM bank target is +69.28 s and only 0.01282°. All 3,742 TAEM
diagnostic records remain acquisition with reference tracking false.

The effective runway is Dessert Airfield Runway 18, 1,750 m by 42.6 m, elevation
820.95 m ASL. Crossrange is positive right. The effective control interval is
0.1 s; the `150ms` run label refers to a 150 m/s deorbit delta-v limit. RCS is
observed on in the handoff sample and off in the next sample 0.08 s later;
actual thrust and throttle are zero throughout the first-minute window.

These findings demonstrate the recorded command behavior and support the
source-level failure mechanism. They do not establish an alternative runway
solution, a fully reconstructed warm controller state, a validated prediction
model, or live flight readiness.
