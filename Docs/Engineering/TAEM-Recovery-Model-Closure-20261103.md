# TAEM response closure and honest outcome accounting

2026-11-03 (Asia/Seoul). Continues `2628bb2` and the supplied replacement research.

## Recorded check and decision

The decisive question was whether the native predictor can reproduce the real
STS's first TAEM attitude response under the **actual issued commands**, before
using that predictor to select a different turn. `taem-response-audit` reconstructs
the FCS from all 4,781 preceding atmospheric records, applies the three recorded
dynamics changes, and then holds out the first 6.02 seconds of TAEM (59 intervals).
The horizon is the greater of configured lookahead and reconstructed pitch/roll
response, not a run-specific time. Plant replay assimilates no measurements after
the branch. Later observations are comparison data only.

Input: `runs/20261102T152517Z-dessert-150ms-live/live.jsonl`, effective
`live.jsonl.configuration.json`, flown source `63b0eb3`. Run and source hashes are
already preserved in `Validation/TAEM-Replacement-Research-20261103/flight-evidence.json`.
The new check reads the original record stream; no flight is operated.

Results (`Validation/TAEM-Recovery-20261103/`):

| Quantity | Result |
|---|---:|
| Maximum prebranch reconstructed axis-command error | 2.96e-14 |
| Maximum held-out reconstructed axis-command error | 0 |
| Original model alpha error at interval end | 165.10 degrees |
| Original model wrapped bank error | -179.94 degrees |
| Original model speed error | -16.96 m/s |
| Original model altitude error | -14.18 m |
| Measured rotating specific energy change | -53,108 J/kg |
| Original model energy change | -67,014 J/kg |

The FCS command reconstruction closes numerically. The finite-response plant
**does not close**. This is substantially stronger evidence than a successful
build or software-equivalence check. The moment plant interprets measured
actuator authority without an observed balancing moment as zero restoring bias;
the RCS-on handoff has no supported RCS-off trim anchor. It then predicts an
inversion absent from the real flight. Its seed force residuals are also nonzero
(+0.205 m/s² lift, -1.859 m/s² drag with nominal frozen resources).

A single additional, explicitly measured local-residual experiment subtracts the
native control/gyro acceleration from the seed's measured body-rate derivative
using only the preceding 0.12-second interval. This does not change actuator
capability, fit future observations, or tune a gain. Holding that residual fixed
reduces the model inversion, but still leaves **25.25 degrees alpha error,
7.78 degrees bank error and -10.63 m/s speed error** at the held-out endpoint.
It is therefore **insufficient** as a replacement moment law. It is exposed only
as an explicit optional model input used by this diagnostic, with zero default;
no flight guidance/planner enables it. It does not satisfy the terminal seed
balance check or confer qualification. Its q/Mach/incidence dependence and
measurement uncertainty remain unknown.

`response-audit.json` preserves the original baseline;
`response-audit-current.json` includes RCS exclusion and the local-residual branch.
The latter's nominal branch still fails closure (-17.21 m/s, 164.64 degrees alpha,
-179.96 degrees bank). No alternative recovery trajectory was run or interpreted.

## Implemented corrections

1. Native continuation checkpoints now retain exact surface integration,
   authority anchor and its RCS mode, touchdown history and moment-balance input.
   They retain configuration and immutable atmosphere/aero instance identities;
   a mismatched continuation is rejected before issuing a forecast command.
   Entry-to-terminal forecasts carry this checkpoint instead of reconstructing
   a plant from an observed row. This is exact native continuation, not exact
   reconstruction of unmeasured live surfaces.
2. RCS-off propagation uses the seed's existing measured `excludingRCS` torque
   decomposition, refreshed before the first rotational substep. A native
   command branch with no applicable decomposition reports unknown rather than
   borrowing enabled-RCS torque. No torque is scaled or invented.
3. Terminal evaluations explicitly report complete/incomplete/invalid status,
   termination cause, ending state, rotating energy/height changes, aerodynamic
   work and peak/integrated pressure/load excess. Nonterminal q/load violations
   stay disqualifying while bounded propagation continues; they no longer erase
   their downstream consequence. A horizon/cancellation keeps its missing-tail
   timeout even when earlier violations exist. Numerical end residue uses the
   same sub-step resolution guard as entry propagation.
4. A measured-aircraft seed with no pitch balance is invalid for terminal
   qualification instead of being evaluated with an implicit zero balance.
   This is a necessary seed check, not a certificate of extrapolated trim/model
   accuracy. Another geometric drawing cannot supply the missing evidence, so
   the qualification loop stops retrying drawings for that same model failure.
5. Explicit incomplete/invalid evaluation data overrides optimistic feasibility
   flags in qualification. Capture assessments expose `evaluationUnassessed`,
   distinct from a modeled physical runway failure. MM304 keeps a neutral energy
   reference for unassessed outcomes; no signed energy remedy is invented.

No live controller ownership rule, vertical-first MM305 allocation, bank/AoA/load
limit, RCS configuration or transport/reconnect setting was relaxed. Unknown
qualification remains advisory; guidance retains its existing command owner.

## Minimal verification

Six focused checks pass:

```
swift test --package-path swift/KSL-Landing --filter 'NativeCommandRolloutTests|TAEMPredictionEvidenceTests|CaptureFailureRegressionTests'
```

Only two new tests were added. Existing native-continuation checks were extended.
The new fixture is the exact effective configuration, first-TAEM state and target
from the research flight. Tests check RCS exclusion, explicit missing-decomposition
handling, unchanged-state continuation/model identity, unsupported moment evidence,
missing-suffix classification, serialization and neutral recovery semantics.
Removing optional measurement fields tests unknown evidence; it does not create a
weaker vehicle. The zero prediction horizon tests search coverage only. No physical
capability parameter is perturbed. Complete-contact/stop behavior and accumulated
violation accounting have not been validated by a new end-to-end flight.

Reproduce the one recorded response audit with a development build:

```
swift/KSL-Landing/.build/out/Products/Debug/ksl-landing taem-response-audit \
  --input runs/20261102T152517Z-dessert-150ms-live/live.jsonl \
  --configuration runs/20261102T152517Z-dessert-150ms-live/live.jsonl.configuration.json
```

No release executable was replaced, KSP loaded/unpaused, save changed, kRPC
connection opened, live command issued or broad campaign run. Unrelated work is
excluded from this checkpoint.

## Outstanding work

The model-closure failure blocks interpreting the report's paired alternative-turn
comparison. A local constant moment correction demonstrably does not solve it.
Next identify the actual state-dependent aerodynamic moment/response model from
available recorded telemetry, checking excitation and support before fitting or
extrapolating. Applied axis values are not individual surface-deflection
measurements, and the single first-minute zero-bank command history cannot validate
large-bank/reversal response. No claim is made that all useful evidence has been
exhausted or that new live data is already required.

Topology-seeded shooting, dynamically reached spiral joins, all-bad full-outcome
ranking, measured-state rebasing and coherent tracking display remain outstanding.
The known zero-bank fallback remains unfixed. Landing readiness is unknown; this
checkpoint must not be described as a successful recovery or a perfected landing
system.

## Follow-on prebranch identifiability check

`identify_pitch_support.py` performs one local affine moment-residual fit using
only the six-second prehandoff response window (60 samples). Its four features
are dynamic-pressure-scaled constant, incidence, Mach and pitch-rate/airspeed.
The measured body acceleration has the same native actuator/gyro terms removed;
QR uses column normalization and reorthogonalization, without external packages.
No coefficient is selected from held-out observations or installed in guidance.

The design is numerically full-rank, but numerical rank is not physical support.
Training residual RMS is 0.00106 rad/s²; held-out measured-state residual RMS is
**0.05315 rad/s²**, worse than simply retaining the seed residual (0.02459 rad/s²).
All 59 held-out Mach samples and 53 of 59 incidence samples lie outside the
training interval. This directly rejects extrapolating that apparently good local
fit across handoff. The check uses measured future states only as comparison
inputs and is not a new trajectory rollout. It cannot establish surface-response
identifiability from axis input telemetry.

Evidence: `Validation/TAEM-Recovery-20261103/pitch-identifiability.json`. Reproduce:

```
python3 Validation/TAEM-Recovery-20261103/identify_pitch_support.py \
  --input runs/20261102T152517Z-dessert-150ms-live/live.jsonl \
  --configuration runs/20261102T152517Z-dessert-150ms-live/live.jsonl.configuration.json
```

This is an identified rejection, not evidence that all recorded data is
insufficient. Investigate the recorded actuator-mode transition and wider genuine
moment/deflection evidence before proposing another fitted production model.
