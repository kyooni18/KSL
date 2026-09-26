# ShuttleSim model profiles (tracked)

The active profile is named, never inferred from which files exist:
`KSP_LANDER_MODEL_PROFILE` = `identified` (default) | `fitted-legacy` | `reference-b`
(`CLanding/telemetry/sim_telemetry.c`, `ShuttleSim/scripts/model_paths.py`).
Run manifests record the profile, file SHA-256s, configuration digest and git state.

## `identified` (default): KSP-identified STS-N

| File | Content | Provenance |
|---|---|---|
| `stsn_aero_identified.csv` | M<=0.9: CL = CL0 + K(M) sin a cos a, CD = CD0(M) + KD(M) sin^2 a (shared CL0, KD/K); blends to the KSP-fitted supersonic table above M 1.1 | `tools/identify_stsn_aero.py --constrained` on 110 training flights (databook CSVs + live KSP logs); 34 whole flights held out. Held-out terminal CL rms 0.063 / CD rms 0.012 (previous fitted table: 0.133 / 0.057). Landing-regime CL bias +0.01..+0.06, 5-95 % measured/predicted 0.9-1.14. Gear and airbrake CD increments not measurable (~0). |
| `stsn_aero_identified.report.json` | per-knot parameters, split, held-out residual bins, input hashes | same |
| `stsn_certified_prior_identified.csv` | guidance vessel-physics certified prior (171 force/q cells on flown support) | sampled from the identified table |
| `stsn_attitude_identified.ini` | closed-loop response (KSP fit, 2026-09-24) + identified direct-control pitch plant: 12.6 deg/s^2/kPa (cap 57), stiffness 0.61/kPa, trim 2.45 deg | `tools/identify_stsn_attitude.py`, pitch held-out R^2 0.76. Pitch damping not identifiable (set 0). Roll/yaw NOT identified (R^2 0.24/0.37): generic values. |
| `kerbin_atmosphere_identified.csv` | KSP-fitted Kerbin atmosphere (sea-level density matches live logs, 1.14859) | copy of `data/fitted/kerbin_atmosphere_ksp.csv` |

The plant uses no force book in this profile (`FORCE_BOOK = none`): the
2-nearest-neighbour book made CL(alpha) non-monotone (e.g. flat 12-14 deg at
75 m/s). Guidance gets the identified model as its prior; for model-error runs
disperse the plant with the report's uncertainty band.

## `reference-b` (audit plant B, below)

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
