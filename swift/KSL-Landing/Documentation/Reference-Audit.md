# Supplied-project investigation and evidence audit

## Scope and preservation

The original KSL archive was extracted into a separate reference tree. The new package is independently buildable and has no source dependency on that tree. No ascent, OMS, deorbit, existing landing source, KSP process, or unrelated project infrastructure was modified. The original archive remains the comparison baseline; the delivery contains only the independent rewrite and the small reference datasets needed to reproduce it.

The supplied narrative describes C/Python ancestry, but the actual latest landing tree includes a Rust implementation with C-compatible interfaces. The rewrite was designed from the files that exist, rather than assuming the narrative described the current source language exactly. The separate MM304/MM305 reference pack was read as design background; its equations and recommendations were checked against the supplied vehicle and simulator instead of treated as proof of craft capability. No external research sources or project connectors were used to infer flight performance.

## Investigated areas

Paths below are relative to the original `KSL` archive.

| Reference | Finding and design consequence |
| --- | --- |
| `rust/landing/src/mm304/law.rs`, `lateral.rs`, `predict.rs`, `plan.rs` | A continuous sigma variable blends scheduled alpha toward max L/D or increases bank. This useful physical idea is retained as one search parameter, without preserving the FFI/mode architecture. |
| `rust/landing/src/mm304/plan.rs` | The existing scalar solver describes a monotone terminal-energy relationship and uses bracketing/Illinois false position. Lateral reversals, alternate terminal routes, and discrete feasibility can break a globally monotone interpretation. The rewrite evaluates a bounded candidate set and then prices full terminal trajectories. |
| `rust/landing/src/mm304/terminal.rs` | The latest original already calls MM305 spiral-energy and tangent functions and admits states by energy and altitude bands. It is inaccurate to characterize all of the original as a fixed-radius circle or a fixed geometric point. The new design broadens admission into an explicit complete-path feasibility search from the actual state. |
| `rust/landing/src/taem_*`, `terminal_model.rs`, `terminal_solver.rs` | Geometry, energy, attitude models, and runtime interfaces are spread through legacy bindings and large data structures. The rewrite separates path geometry, force/trim assessment, stateful tracking, and surface control. |
| `ShuttleSim/src/world.c`, `attitude.c`, associated physics code | Kerbin FloatCurves, rotating-frame terms, separate attitude/control modes, and provisional roll/yaw behavior must be preserved as explicit models rather than flattened into a fitted point-mass approximation. |
| `ShuttleSim/reference-model/sts_aero_cells20261029.*` | This is the latest supplied STS polar and update metadata. It remains provisional. Its reference area and observed-cell coverage are recorded below. |
| `ShuttleSim/reference-model/sts_attitude_live20261028.ini` | Direct-input pitch parameters and the separate identified autopilot response parameters are different models. Mixing a surrogate roll acceleration with a direct surface-input equation would be unjustified. |
| `ThirdParty/krpc-cnano-0.6.0/protobuf/krpc.proto` and SDK declarations | kRPC scalars, tuple encoding, handshake fields, enum representation, procedure names, and surface/vessel frame bases were checked before the native client was written. |
| `FlightLogs/*-vehicle.jsonl`, `ShuttleSim/runs/*`, historical INI fixtures | The latest vehicle streams contain two interleaved sessions per file and damaged lines. Some records come from simulator runs. A phase label alone cannot establish a live touchdown. |
| `Runtime/Physics/observations.sqlite3` | Read-only access reports `database disk image is malformed`. The original was left unchanged. A live calibration/holdout error claim cannot be reconstructed from this database. |

These are source findings and architectural conclusions. They are not a claim that every original flight failure has been uniquely attributed to one defect. Corrupted records and missing raw live files prevent that level of certainty.

## Aerodynamic provenance

The polar contains 294 cells, covering Mach 0–25 and alpha −10°–45°, normalized by 250 m². Its update metadata labels the model `provisional; not certified`, records 53 observed cells, and lists 19 live sessions and 283 simulator files. None of the 19 named raw live files is present at its referenced archive path.

The update's ratio to the earlier polar is:

| Coefficient | Minimum ratio | Median ratio | Maximum ratio |
| --- | ---: | ---: | ---: |
| CL | 0.509820 | 0.948381 | 1.182550 |
| CD | 0.499073 | 1.026495 | 2.123868 |

These numbers measure changes between model versions. They are **not** prediction residuals, holdout accuracy, or a certified error bound. The ±20% force variations used by the new validation suite are sensitivity experiments. The data do not support interpreting that interval as a measured confidence interval.

Source bytes and SHA-256 values are preserved in [Evidence/source-manifest.json](Evidence/source-manifest.json). The runtime polar is a copy, not an edited original. The original metadata and direct-input pitch identification are also included as evidence.

## Direct control and vehicle identity

The default model uses the supplied STS nominal mass of 25,000 kg. The pitch acceleration coefficient is 12.6211°/s²/kPa, capped at 56.7948°/s², with stiffness 0.61037 per kPa and trim alpha 2.445°. The diagonal inertia estimate and roll/yaw equations are archival/provisional inputs, not newly identified live measurements.

An archived final scenario named `final-ideal-3p5km-v190-fpa20.ini` has mass approximately 40,252.9 kg and belongs to the STS-N family. Its name does not make it a nominal validation case for this STS polar. It is retained as an explicit model/vehicle mismatch stress case. Passing the 25 t final does not qualify that heavier vehicle.

## Historical log integrity

The native Swift importer audited all five latest vehicle JSONL files. The explicit salvage mode drops malformed records, invalidates affected delta chains, separates sessions, and waits for complete keyframes. Its default mode rejects malformed data. The audit does not silently splice sessions or invent missing telemetry.

| Vehicle log time, 2026-10-29 UTC | Lines | Malformed | Orphan deltas | Invalid snapshots | Accepted snapshots | Sessions |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 11:43:41 | 916 | 19 | 107 | 0 | 784 | 2 |
| 13:04:47 | 36 | 5 | 31 | 0 | 0 | 2 |
| 13:05:05 | 712 | 10 | 98 | 0 | 599 | 2 |
| 13:20:37 | 487 | 7 | 63 | 1 | 416 | 2 |
| 13:26:18 | 695 | 2 | 25 | 0 | 625 | 2 |

Headers, duplicate times, and other non-snapshot records mean the categories are not a simple partition of line count. Full session IDs, original file hashes, duplicate counts, and phase counts are in [../Validation/reference-audit.json](../Validation/reference-audit.json). Recovery of an `Abort`, `TAEM`, or `Rollout` label does not prove an acceptable touchdown or a stop on the runway. Neither the importer nor this report promotes simulator-provenance records to live evidence.

The provided historical INI files are usable initial conditions for new closed-loop experiments. Their new results are labeled offline model results. This is a reproduction of recorded starting conditions under the new model and controller, not replay of an observed live landing with identical controls.

## Defects corrected during rewrite validation

The validation process found and corrected defects in the new implementation as well. They are recorded here so they are not misattributed to the original code:

1. Planet-fixed angular velocity was initially used directly in inertial Euler coupling. The corrected equation uses inertial body rate and transforms its derivative back; the rotating-frame angular-momentum test now checks the result.
2. Angular-rate integration was upgraded to a coupled midpoint step; translation remains RK4 within the small plant substep.
3. A terminal-speed stop condition initially collapsed all entry forecasts to zero duration after a rejected handoff. A minimum control-dependent forecast duration now prevents that loss of optimization authority.
4. Initial HAC curvature initially used the coordinated-level-turn relation despite a steep descending state. It now derives from observed force and rotating-frame acceleration.
5. A normal-curvature term in terminal pricing had an extra cosine projection. The corrected acceleration matches the analytical pullout test.
6. A coarse nearest-station search could skip an overlapping spiral wrap. Tracking now uses a short monotonic window and continuous interpolation.
7. Approach assessment and execution initially used different capture shapes. The same C2 capture curve is now priced and flown.
8. Ground brake actuation is quantized to match kRPC's boolean brakes, and touchdown assessment uses pre-contact observations rather than trusting zeroed post-impact velocity.
9. Corrupt archival delta chains, oversized JSONL lines, duplicate telemetry times, cancellation cleanup, and stale asynchronous continuations now have explicit failure behavior and regression coverage.

The complete final case matrix is retained even when a case fails. Corrections improve the engineering basis; they do not justify claiming live flight qualification or global robustness.
