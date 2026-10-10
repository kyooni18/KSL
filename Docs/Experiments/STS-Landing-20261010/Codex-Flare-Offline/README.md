# Offline flare continuation

Baseline: `b3dfb34`. All variants are rejected unless a later LOG.md entry says otherwise.
Experiments ran from an isolated `/tmp/sts-codex-flare/CLanding` source snapshot;
shared `CLanding/build` and `CLanding/build-dev` were not rebuilt or replaced.
The snapshot also preserved the existing local terminal-propagator and Makefile changes.
`patches/` contains each variant relative to the committed flare files at `b3dfb34`.
`results.json` retains all 114 completed replay manifests' outcome and provenance fields.
No live KSP calls were issued. No flight restrictions or acceptance gates were changed.

## Reproduction

Apply one patch to an isolated source copy at the baseline; build its backend with
`make -C <copy>/CLanding -j8`. From the repository root, run:

```sh
INPUTS=Docs/Experiments/STS-Landing-20261010/Codex-Flare-Offline/inputs
KSP_LANDER_MODEL_PROFILE=sts-live-20261010 \
KSP_LANDER_FCS_TUNING_OVERRIDES=1 KSP_LANDER_PITCH_TRIM_KI=0.025 \
KSP_LANDER_AFLARE_TRACE=1 KSP_LANDER_FINAL_TRACE=1 \
python3 ShuttleSim/scripts/run_guidance.py \
  --scenario "$INPUTS/live-L1-final.ini" --configuration "$INPUTS/sts-af.json" \
  --engage engageFinalTest --max-sim-time 700 --label flare-offline \
  --command-port 0 --telemetry-port 0 --no-mirror --quiet-progress --skip-build \
  --backend-build-dir <copy>/CLanding/build --direct-control \
  --attitude "$INPUTS/att-live.ini" \
  --terminal-attitude ShuttleSim/reference-model/stsn_attitude_identified.ini
```

The matrix uses L1/L3 × live-softer/live/live-stiffer × trim gains 0.025/0.004.
The chains use `ShuttleSim/scenarios/sts-live-handoff-<case>.ini`,
`engageReentry`, nominal `att-live.ini`, and default trim gain 0.025, for
10-46-41, 11-27-40, 12-33-31, 13-02-48, 13-27-33, 13-52-53, 14-21-19.
These use the real FCS in direct-control mode. STS-N supplies only the existing
terminal predictor attitude prior, never the vehicle modelId or direct-control plant.

Runner success includes runway touchdown and rollout; it allows speed ≥63.75 m/s.
The adaptive planner's 66 m/s objective is stricter. Passing this harness therefore
must not be described as meeting the 66 m/s objective, live repeatability, or perfect flare.
