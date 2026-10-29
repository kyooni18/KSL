# Validation surface

The native-stack regression gate is run with `make -C CLanding test` (~10 s).
The root `make test` runs it together with the Python suites in `tests/` and
here.

`make -C CLanding test` covers:

- Rust white-box tests (`cargo test`): deorbit window search, polar MM305 inlet and Final capture geometry
- architecture/ownership contracts (`ShuttleArchitectureContractTests.c`)
- MM305 admission, TAEM frames/energy, TAEM tracker allocation, MM305 recovery monitor
- vessel physics, FCS yaw and quaternion attitude
- MM304 lateral feedback, entry energy law, entry forecast/admission parity
- Final delivery parity and touchdown quality
- deorbit planning/admission, engine roles and thrust alignment (`deorbit-test`)
- the backend NDJSON protocol (`BackendProtocolTests.py`)

`make -C CLanding qualification` adds the slow physical gates:
`taem-native-stack-test` (~2 min) and `mm305-feasibility-test` (~1.5 min).
`mm305-feasibility-test` is an **open gate**: it currently fails its non-vacuity
check (no fixture state qualifies an MM305 route). It was failing before the
KSL integration as well.

Python tests here (`test_*.py`, `*Tests.py`) cover the headless live harness
(engagement acknowledgements, protocol timeouts, checkpoint unpacking, the KSP
load guard), model identity, and probe tooling. `STSRepairTests.mk` is used by
`Tools/run_sts_local_profile_experiment.py`.

Probes that no longer compiled, or that no build ran, were removed during the
KSL integration (see `git log -- Validation`). Tests that include production
`.c` files directly must not be used as a reason to expose private helpers.

`BackendParity.py` is the closed-loop behaviour gate for the Rust backend: it
flies the same lockstep ShuttleSim scenarios with a reference backend (built
from the commit before the Rust rewrite) and a candidate and requires
byte-identical simulator telemetry.

`ClosedLoopTrace.c` is the deterministic, single-threaded counterpart for the
scenarios whose asynchronous planners make the process-level run
non-reproducible (MM304 entry, MM305/HAC): it flies a ShuttleSim scenario with
`guidance_update_with_terminal_model()` and prints every command and the
simulator state as exact hex floats.  Link it against two builds of the landing
library (the reference C objects and `liblanding.a`) and `cmp` the outputs.
