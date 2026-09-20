# FlyBrain V2 — isolated recurrent residual controller

This is a heavier offline-only successor to `Experimental/FlyBrain`. It is deliberately
not referenced by `CLanding`, `Runtime`, either launcher, kRPC, or the Python bridge.
Training and validation cannot command KSP.

## Architecture

V2 keeps a deterministic INDI-like reflex core as the hard stability prior and adds a
bounded recurrent residual policy. The residual policy uses MLX on Apple silicon and is
trained end-to-end through a randomized differentiable 3-axis rotational plant.

The neural path sees attitude error, body rate, angular acceleration, previous command,
applied actuator position, estimated authority, desired body rate, and `dt`. A 128-unit
encoder feeds a 96-unit GRU, followed by 128- and 64-unit nonlinear layers and three
bounded residual control outputs. The residual can change the deterministic command by
at most 0.35 normalized control, so it cannot completely replace the reflex core.

Training randomizes rotational authority, actuator lag, rate damping, constant bias
moments, cross-axis coupling, sensor/authority error, gusts, initial attitude/rates, and
smooth plus reversing attitude references. Validation uses disjoint random seeds and a
wider hard envelope.

## Usage

```sh
make -C Experimental/FlyBrainV2 train
make -C Experimental/FlyBrainV2 validate
```

`brain_v2.safetensors` and `training_report.json` are inert offline artifacts. Any future
adapter into the shuttle is intentionally a separate integration change.
