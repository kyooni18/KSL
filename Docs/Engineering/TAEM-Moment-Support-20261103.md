# Preserve actual moment-evidence support across forecasts

2026-11-03 (Asia/Seoul), following `abbe21c4` and frame correction `8b4713b4`.

## Broader recorded-source inventory

`Validation/TAEM-Recovery-20261103/inventory_native_response.py` inventories 92
existing native-format candidate record streams under `runs` (79 distinct
decompressed SHA-256 values, about 6 GB of source files). This is a read-only
source query, not a flight, plant perturbation or simulation campaign. The
complete searchable result is `native-response-inventory.json`.

The anchor/part-count filter is explicitly only a partial identity. Its few
high-Mach RCS-off hits include startup/low-q records; they do not establish joint
handoff coverage. The inventory also exposes high-Mach leads with other anchor
IDs. A cockpit handle is not a structural model ID, so those sources must not be
automatically discarded or considered compatible. For example, two inspected
26-part runs near Mach 2.9 have approximately 21-degree incidence versus the
research seed's 29.5 degrees and different measured mass/CoM. Their actuator
configuration, source patches and reference frames need reconciliation before
moment transfer. Do not count duplicate exports as independent flights or
interpret this inventory as exhausting every possible source of model evidence.

## The existing first measured balance and calibrated force model

The response audit now accepts `--first-supported-balance` to select the first
chronological TAEM state whose **preceding** controller snapshot already supports
a pitch balance. Selection uses no held-out error. In the research flight this is
UT 216412.69235289673, 6.66 seconds after handoff, with observed input 0.79936.
The prior sample at UT 216412.59235289678 already met the existing strict observer
gate; this balance does not depend on the new off-target observation path below.

`--identified-forces` reconstructs the existing live AerodynamicAuthorityEstimator
from prior measured records, including recorded dynamics resets. It freezes those
identified lift/drag corrections at the branch, and uses the recorded environment
calibration available at the seed. No subsequent measured force/calibration state
is assimilated into the plant. This is an actual observed model correction, not
an arbitrary lift/drag/density capability multiplier.

| Same 6.06-second recorded-command interval | Nominal resources | Prior observed force correction |
|---|---:|---:|
| Speed error | +5.199 m/s | -0.361 m/s |
| Rotating energy-change error | +3,975 J/kg | -262 J/kg |
| AoA error | 4.55 degrees | 4.61 degrees |
| Wrapped bank error | -0.598 degrees | -0.599 degrees |

The correction uses lift effectiveness 0.98723, drag effectiveness 1.10662,
density factor 0.999675 and sound-speed factor 0.999963, all derived from prebranch
flight evidence. Command reconstruction remains 2.96e-14 maximum before the
branch and exactly zero mismatch in the held-out interval. This separates the
remaining attitude error from an avoidable nominal-force-model mismatch. It does
not repair the original handoff's missing moment anchor.

Evidence files preserve the comparison:

- `first-supported-balance-baseline.json`: nominal resources, existing observer.
- `first-supported-balance-identified-forces.json`: observed force correction.
- `first-supported-balance-current.json`: same corrected interval, with explicit
  original moment-support extent and the current observer implementation.

Reproduce the current diagnostic with a development build:

```
swift/KSL-Landing/.build/out/Products/Debug/ksl-landing taem-response-audit \
  --input runs/20261102T152517Z-dessert-150ms-live/live.jsonl \
  --configuration runs/20261102T152517Z-dessert-150ms-live/live.jsonl.configuration.json \
  --first-supported-balance --identified-forces
```

The audit intentionally also reports the full interval after support expires to
expose error. That exploratory tail is not an admitted, qualified forecast.

## Correct support and prediction-only observation

The existing AdaptiveTrim query restricts a local observation by two-second
freshness, Mach difference 0.35, pressure ratio 0.5–2 and one-degree local
incidence (or an independently supported regression). These existing observer
bounds are retained; no new flight tuning limit is introduced.

Previously native continuation retained a scalar moment balance indefinitely.
A nonnil seed balance was sufficient even after the underlying evidence query
would have rejected the predicted time/regime. The native checkpoint also carried
an evolving controller snapshot, which must not be mistaken for new real moment
evidence after the prediction branches.

NativeCommandSeed now preserves the original measured-vehicle origin and moment
evidence independently of the evolving predicted controller. A checkpoint inherits
that provenance. Current predicted-state coverage is explicit as NativeMomentSupport:
configured provisional plant, zero-q physical case, local balance, missing
authority/balance, expired or out-of-regime evidence, a different required balance,
or unvalidated diagnostic residual. The constant-bias plant cannot claim a
state where its evidenced moment function requests another value.

Entry forecasts now stop propagation at missing/out-of-support measured moment
evidence and report model uncertainty. Terminal evaluation applies the same
current-state check. Entry capture retains `evaluationUnassessed` instead of
calling this an envelope or signed-energy failure. These are forecast advisories;
the live guidance owner and fallback keep commanding the aircraft. This reduces
forecast/admission coverage where the old code extrapolated unsupported dynamics;
it must not be sold as improved flight feasibility.

The observer additionally admits a **prediction-only** measured equilibrium in
entry/acquisition/heading alignment when actual force/authority data, unchanged
RCS-off history, small measured pitch acceleration, small input change and the
existing incidence-rate gate support it. The 3% equivalent-input reserve and
0.3-degree/s rate are the observer's existing criteria, not tuned additions.
Target proximity still governs the original feedforward regression. Off-target
prediction evidence does not update regression weight/freshness, feedforward,
trim ceiling or actuator capability. Modeled observations cannot qualify for this
new actual-measurement path. A real off-target eligible row occurs at +7.38 s;
its existence is not evidence of a global restoring-moment slope.

## What the support extent actually proves

The first chronological calibrated balance replay exits original moment support
at +1.94 s. Its last fully sampled supported prefix is 1.82 s (18 recorded
intervals), with AoA error 0.428 degrees, bank error -0.227 degrees, speed error
+0.502 m/s and altitude error -0.094 m. Its energy-change residual is +400 J/kg.
These residuals are recorded, not a newly invented acceptance tolerance or a
six-second response certificate. The longer interval still has 4.61-degree AoA
error and cannot certify a large descending turn or reversal.

## Minimal checks

Nine focused checks pass; only two moment-evidence regressions were added, plus
the separately committed frame regression:

```
swift test --package-path swift/KSL-Landing --filter 'PredictionPitchBalanceTests|TAEMPredictionEvidenceTests|testAnchorOffsetIsBodyRelativeAndInvariantToWorldAttitude'
```

The new history fixture adds three unchanged genuine posthandoff states/targets.
Their actual 0.12-second and 0.10-second intervals matter; substituting nominal
cadence initially missed the equilibrium gate, and that test setup was corrected.
Checks cover prediction-only versus feedforward evidence, persistence/expiry through
native checkpoints, rejection before an unsupported entry command, existing
saturation/regime behavior and CoM-frame invariance. The 21-tick native software
check tests bookkeeping with the real configuration; it is not flight-performance
or counterfactual recovery evidence. No physical capability is perturbed.
Evidence: `Validation/TAEM-Moment-Support-20261103/focused-tests.txt`.

No live connection, flight, KSP load/unpause, original-save operation, release
replacement or broad simulation campaign ran. Unrelated work is preserved.

## Remaining requirements

The report's dynamically propagated topology search, reachable spiral joins,
complete all-bad recovery ranking, rebasing and actual recovery-policy replacement
remain outstanding. The zero-bank policy is not fixed. These changes establish
which observed model components and local prefixes can be investigated without
pretending that a local trim anchor predicts an entire landing. Further recorded
actuator/model evidence or separately authorized measurements are still needed
before extrapolating the attitude plant. Full-chain landing readiness and the
user's complete goal remain unestablished.
