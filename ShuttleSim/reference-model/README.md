# ShuttleSim reference model (tracked)

These files let a clean checkout build, run `make -C CLanding test`, and fly
ShuttleSim without the KSP-fitted data under the git-ignored `ShuttleSim/data/`.

**They are not a KSP fit.** Whenever `ShuttleSim/data/fitted/*` exists, the C
helper `shuttle_sim_model_path()` (`CLanding/telemetry/sim_telemetry.c`) and the
Python helper `ShuttleSim/scripts/model_paths.py` prefer the fitted files.

| File | Content | Provenance |
|---|---|---|
| `kerbin_atmosphere_reference.csv` | stock Kerbin pressure/temperature, 100 m table (`model=kerbin_stock_spatial`, so the simulator uses its spatial stock model) | `world_seed_kerbin()` |
| `stsn_aero_reference.csv` | Mach × AoA CL/CD grid, reference area 250 m² | ShuttleSim seed table with supersonic CL ×0.30 / CD ×0.65 and halved subsonic parasite drag. Reproduces the recorded KSP force samples in `Docs/LatestFlight-2026-09-07-analysis.json` to about 10–20 % (M2.7/α23°: CL≈0.23, CD≈0.37; M0.5/α4°: L/D≈1.6) |
| `stsn_force_book_reference.csv` | direct-force book sampled from the table above | generated |
| `stsn_attitude_reference.ini` | closed-loop AoA/bank response | `attitude_seed()` values, not identified from KSP |

Regenerate with `Audit/2026-09-25-engineering-evaluation/tools/build_plants.py`
(plant "B"). Replace with fitted data as soon as a held-out-validated KSP fit is
available; do not tune guidance to this table.
