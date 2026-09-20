# FlyBrain isolated attitude-control lab

FlyBrain is an offline-only control experiment. It is deliberately not linked from
`CLanding`, the launch scripts, kRPC, the Python bridge, or the live guidance
runtime. Building or training this directory cannot command KSP.

## Why this design

The selected design is a tiny adaptive INDI-style reflex controller whose gains are
tuned offline with a cross-entropy optimizer. This was chosen instead of an MLP or
recurrent neural network because the shuttle's immediate problem is closed-loop
attitude stability, not perception:

- no GPU or ML framework is required;
- runtime state is a few hundred bytes;
- the controller directly damps measured body rates and angular acceleration;
- online authority estimation adapts to large changes in aerodynamic control
  effectiveness without making guidance phase-specific;
- offline training tunes only a small set of interpretable gains across randomized
  authority, actuator lag, cross-axis coupling, sensor noise, disturbances, and
  moving attitude references.

The trainer uses the current terminal-control evidence as its envelope: nominal
usable rotational authority spans roughly 20-200 deg/s^2 and target reference
motion is exercised up to about 6 deg/s. Hard validation extends outside those
ranges.

## Isolation contract

This directory has its own `Makefile` and only compiles files that live here. It
has no dependency on `CLanding`, `Runtime`, kRPC, save files, or the shuttle
launcher. The generated `trained_params.txt` is inert data. Integration later must
be an explicit separate change.

## Build, train, validate

```sh
make -C Experimental/FlyBrain clean all
make -C Experimental/FlyBrain train
make -C Experimental/FlyBrain validate
```

`train` optimizes the controller on deterministic randomized simulations and writes
`trained_params.txt`. `validate` uses disjoint seeds and a harder plant envelope.

The controller accepts a three-component attitude-error rotation vector, measured
body rates, and measured angular acceleration. A future live adapter should derive
the attitude error from the target/current quaternion rather than feeding raw Euler
angle differences near singularities.

## Resource profile

The trainer is pure C and keeps one simulated vehicle at a time. Training evaluates
many independent episodes serially, so memory usage stays tiny. Parallelization is
intentionally omitted because the goal of this lab is low resource use and
repeatability, not shortest wall-clock time.
