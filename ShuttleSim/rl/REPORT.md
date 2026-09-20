# Offline guidance-learning experiment: results

## Outcome

**The offline experiment is implemented and exercised; a successful learned
landing controller has not been produced. Nothing here is flight-ready.**

The first gate audit found that an advisory continuation warning was being
treated as a hard policy veto. That veto is now separated from hard physical
guards in the simulator-only adapter. The fresh varied-condition smoke run
reports `smoke_only_unproven`: candidate policies received 2,316 eligible and
1,589 active residual steps across training episodes. Its exported incumbent
still received zero active steps in validation, with zero touchdowns. This is
now evidence that the learner can affect bounded guidance in the simulator,
not evidence that it learned to land.

After that smoke run, the randomized reset domain was tightened to the actual
deorbit-stage envelope: pre-deorbit apoapsis/periapsis 70--400 km, inclination
0--87 degrees, and a bounded retrograde apoapsis burn targeting a 40--69.5 km
post-deorbit periapsis. A conservative MM304/TAEM screen rejects physically
unusable entry energy and geometry before an episode starts. The `smoke-03`
numbers below are therefore historical for the previous post-burn randomizer;
the new boundary and rejection screen are covered by the offline ABI/
environment tests and reset diagnostics, but a full long training run has not
yet been repeated against this larger orbital domain.

A fresh filtered smoke run was then executed with one CEM generation, two
candidates, training seeds 11/12, and a 1,200-step horizon:
`results/filtered-smoke-01/training-report.json`. The non-teacher candidate
received 813 action-active steps across its two training episodes, confirming
that the filtered conditions reach the bounded learner
authority. Validation remained deliberately unproven: two timeouts, zero
touchdowns, zero valid rollouts, and zero active steps for the retained teacher
checkpoint. This is a reset/authority smoke result, not a learned landing.

## Completed scope

- Optional in-process simulator and deterministic expert libraries, isolated
  from the normal UDP/live path.
- Resettable seeded Python environment, 32-feature observation contract,
  bounded entry AoA/bank residuals, safety projection, fallback, and terminal
  validation including contact and stopped runway rollout.
- Seeded CEM smoke training, portable versioned JSON policy, checkpoints,
  per-step reward/action diagnostics, and held-out/baseline comparisons.
- Reset-time domain randomization without rewriting fitted physics files,
  including deorbit-stage orbital-energy/inclination cases and logged
  pre/post-deorbit periapsis, apoapsis and inclination diagnostics.
- Physics/environment tests and reproducible run instructions in [README.md](README.md).

This is a limited residual-guidance experiment, not a replacement low-level
controller, supervised behavior-cloning pipeline, or terminal curriculum.

## Training

Command executed from the repository root:

```sh
python3 ShuttleSim/rl/experiment.py train --simulator-only --output ShuttleSim/rl/results/smoke-03
```

Seed 7, two generations, four candidates per generation, training seeds 11/12:
16 candidate-training episodes. There were zero successes and zero contacts;
candidate episodes accumulated 2,316 eligible and 1,589 active residual steps.
The varied reset cases include a low-energy seed with approximately -1.1 km
periapsis and an approximately 87.9 km apoapsis, and a high-energy seed with
approximately 87.2 km periapsis and 115.1 km apoapsis; realized inclinations
also varied. The two diagnostic validation episodes (101/102) had no
successes or contacts, 264 eligible steps and zero active steps, because CEM
retained the deterministic warm start. Outcomes were one `unsafe_or_abort` and
one `timeout`; the exported checkpoint is not evidence of useful learning.

Source: [training-report.json](results/smoke-03/training-report.json).

## Evaluation

Command executed:

```sh
python3 ShuttleSim/rl/experiment.py evaluate --simulator-only --policy ShuttleSim/rl/results/smoke-03/policy.json --output ShuttleSim/rl/results/evaluation-03
```

Evaluation does not train the policy or fit physics. The deterministic teacher,
projected fixed-target baseline and exported policy use identical cases.
All 27 comparison episodes had zero touchdowns and zero valid rollouts; 21
ended `unsafe_or_abort` and 6 timed out. Across them there were 6,172 eligible
steps and 1,050 active residual steps. The active steps came from the
projected fixed-target comparison; the exported learned policy retained the
zero-residual warm start in these smoke settings. The held-out scenario and
unseen seeds are separate from the training and diagnostic validation seeds.

| Case / policy | Episodes | Successes | Contacts | Mean return | Eligible steps | Active steps |
|---|---:|---:|---:|---:|---:|---:|
| unseen/deterministic | 3 | 0 | 0 | -875.505 | 540 | 0 |
| unseen/fixed-target-projected | 3 | 0 | 0 | -876.966 | 548 | 464 |
| unseen/learned | 3 | 0 | 0 | -875.505 | 540 | 0 |
| heldout-scenario/deterministic | 2 | 0 | 0 | -763.643 | 298 | 0 |
| heldout-scenario/fixed-target-projected | 2 | 0 | 0 | -764.262 | 310 | 277 |
| heldout-scenario/learned | 2 | 0 | 0 | -763.643 | 298 | 0 |
| nominal/deterministic | 1 | 0 | 0 | -1045.296 | 301 | 0 |
| nominal/fixed-target-projected | 1 | 0 | 0 | -1045.634 | 301 | 77 |
| nominal/learned | 1 | 0 | 0 | -1045.296 | 301 | 0 |
| density-low/deterministic | 1 | 0 | 0 | -1046.607 | 307 | 0 |
| density-low/fixed-target-projected | 1 | 0 | 0 | -1046.981 | 308 | 76 |
| density-low/learned | 1 | 0 | 0 | -1046.607 | 307 | 0 |
| lag-slow/deterministic | 1 | 0 | 0 | -1045.294 | 301 | 0 |
| lag-slow/fixed-target-projected | 1 | 0 | 0 | -1045.694 | 301 | 77 |
| lag-slow/learned | 1 | 0 | 0 | -1045.294 | 301 | 0 |
| mass-high/deterministic | 1 | 0 | 0 | -1045.760 | 303 | 0 |
| mass-high/fixed-target-projected | 1 | 0 | 0 | -1046.139 | 304 | 79 |
| mass-high/learned | 1 | 0 | 0 | -1045.760 | 303 | 0 |

Source: [evaluation-report.json](results/evaluation-03/evaluation-report.json).
Episode and step JSONL files are stored alongside that report. The README
specifies randomization ranges and the differences from campaign stress cases.

## Baseline and replay evidence

The separate UDP deterministic baseline returned `success: false`; see
[baseline-run.log](baseline-run.log), which records its simulator, guidance,
and stderr artifact paths. This result is distinct from the offline adapter's
early conservative termination and does not establish adapter parity.

The held-out historical replay summary recorded contact off the runway:
sink 24.643 m/s, speed 68.975 m/s, along-track -17589.190 m and cross-track
11691.510 m at contact. See
[replay-heldout.stderr.log](results/replay-heldout.stderr.log). Historical
`actual` replay bypasses command-following dynamics, so it cannot validate a
learned guidance policy or count as a successful landing.

## Verification and limitations

The implementation validation completed successfully for the offline scope:
CMake build, 12 Python environment tests, 4 offline ABI tests, and the
training/evaluation smoke commands. Passing software checks does not imply
successful flight behavior.

The conservative stall guard can stop high-altitude entry before the UDP
baseline aborts. The offline expert uses a simplified telemetry/calibration
adapter and 1 Hz updates rather than reproducing every `guidance_runner`
state and cadence. Simulated radar altitude is a runway-height plane, not
terrain sensing. These differences preclude claims of live parity.

## Blocked work and next prerequisite

Useful learning and a validated landing policy remain blocked by the lack of a
successful terminal trajectory and unresolved simulator/expert parity, not by
zero policy authority. The varied initial-condition generator is now in place,
but repeating long training before fixing the terminal guidance geometry would
mostly produce more unsafe/timeout data. Do not remove the safety guard, relax
touchdown criteria, or change fitted physics to manufacture a positive result.

Before broader training, reconcile stall applicability, telemetry, expert
cadence/calibration and terminal geometry against the deterministic baseline.
Then demonstrate stable nonzero policy improvement across both energy classes
and independently validated terminal cases. A center-of-mass curriculum,
broader action authority and live integration are not completed or approved by
this experiment.
