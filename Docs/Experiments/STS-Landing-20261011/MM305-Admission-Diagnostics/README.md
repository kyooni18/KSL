# STS MM305 admission diagnostics

The current STS fixture physically admits all five inlet states but qualifies
none. The existing non-vacuity assertion fails with zero gate/adoption
violations. Qualification remains unresolved. The earlier default fixture
also qualified none, but represented STS-N at 40,000 kg; this run uses STS
configuration and measured mass 25,123.15 kg.

Full-batch evidence was collected at source commit `2cc042c`. The focused
build adds timeout logging and optional single-sample selection. Neither
changes trajectories or acceptance constraints. Default invocation still runs
all five samples. Focused invocation identifies its limited scope and retains
the non-vacuity assertion.

Sample 0 (26 km altitude, Mach 2.5, 30 km before the runway, zero cross-track)
tested 2,184 smooth profiles; 1,287 completed and none were jointly feasible.
The completed trial closest to the target altitude used a 48,536.527 m route:
exit speed 95.251 m/s, maximum descent slope 36.587 degrees, track error
8,036.088 m and endpoint altitude error 2.016 m. Its small altitude error does
not compensate for failed speed, slope and track constraints. Constant-profile
trials produced 261 propagation timeouts, often after speed had decayed below
the Final interface requirement. Independent minima from different profiles
must not be combined into a fictitious feasible candidate.

The default attitude file and live replay override have identical ten
command-servo parameters. Admission propagation uses commanded AoA and bank
servo dynamics, with direct surface/moment mode disabled. Different direct
pitch moment coefficients therefore cannot explain or fix this result.
This does not qualify either servo model against STS flight response;
predictor/control parity and plant qualification remain open.

Reproduce from the repository root:

```sh
KSP_LANDER_MODEL_PROFILE=sts-live-20261010 make -C CLanding mm305-feasibility-test MM305_FEASIBILITY_ARGS='--configuration Configuration/sts-live.json --mass-kg 25123.15'
KSP_LANDER_MODEL_PROFILE=sts-live-20261010 KSP_LANDER_TAEM_PROFILE_DIAGNOSTICS=1 make -C CLanding mm305-feasibility-test MM305_FEASIBILITY_ARGS='--configuration Configuration/sts-live.json --mass-kg 25123.15 --sample-index 0'
```

Both commands currently fail qualification. Local raw logs are ignored;
[result.json](result.json) preserves joint trial rows, timeout rows, commands,
environments and hashes. `summarize.py` regenerates it from `full-sts.log`
and `focused-sts.log` beside the script. The binary hash belongs to the
focused diagnostic build, not the earlier full-batch executable.

Validation: `make test` passed (104 tests, 44 subtests); the native TAEM model,
route descriptor, Final interface and replay gate tests passed. Invalid sample
index 5 was rejected. No flight was performed and no gates or production
guidance settings were changed by this diagnostic work.
