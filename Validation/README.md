# Validation surface

The maintained CLanding regression gate is deliberately small and is run with:

```
make -C CLanding test
```

The maintained tests are `BackendProtocolTests.py`, `ShuttleArchitectureContractTests.c`, `TaemInterfaceCaptureEnergyTests.c`, and `VesselPhysicsTests.c`.

Other files in this directory are historical white-box tests, one-off probes, recorded-replay helpers, or validation-campaign tooling. They are not part of the maintained regression contract. In particular, tests that include production `.c` files directly must not be used as a reason to expose or preserve private implementation helpers.

HAC campaign probes are intentionally left in place while the current HAC validation is active; retire or archive them after that campaign no longer depends on their paths.
