# Calibration and KSP parity plan

The code/runtime is only half of ShuttleSim. The KSP-parity model comes from the project's real telemetry.

## Rule

Do not tune Guidance against ShuttleSim until open-loop `actual` attitude replay reproduces multiple real KSP flights that were not used to fit the model.

## 1. Capture the actual 86 km initial state

Create a scenario containing exact KSP inertial position, inertial velocity, UT, and mass. This removes initial-condition ambiguity from every later comparison.

## 2. Atmosphere

Normalize KSP telemetry to:

```csv
altitude_m,density_kg_m3,pressure_pa,temperature_k,speed_of_sound_mps
```

Then:

```sh
python3 tools/fit_atmosphere_from_telemetry.py atmosphere-normalized.csv data/kerbin_atmosphere_fitted.csv
```

Use the fitted file with `--atmosphere`.

If KSP exposes a direct atmosphere curve/export, prefer that over deriving the curve from a single flight.

## 3. Aerodynamics

Best input is direct KSP aerodynamic force. If direct force is unavailable, derive aerodynamic acceleration from a smoothed velocity history and subtract gravity; do not differentiate noisy position samples directly.

Normalize samples to:

```csv
mach,aoa_deg,dynamic_pressure_pa,lift_accel_mps2,drag_accel_mps2,mass_kg
```

Then:

```sh
python3 tools/fit_aero_from_telemetry.py aero-normalized.csv data/stsn_aero_fitted.csv --area 250
```

The reference area is a scaling convention. Keep it constant between fitting and runtime; the fitted CL/CD then reproduce the observed force scale.

The fitter emits a complete Mach x AoA grid. Validate high-AoA coverage through at least the final S-turn envelope rather than extrapolating from ordinary aircraft AoA.

## 4. Attitude response

Normalize:

```csv
time_s,cmd_aoa_deg,aoa_deg,cmd_bank_deg,bank_deg
```

The current helper estimates observed rate/acceleration limits:

```sh
python3 tools/fit_attitude_from_telemetry.py attitude-normalized.csv data/stsn_attitude_fitted.ini
```

Natural frequency and damping remain seed values until step-response fitting is added. Rate/acceleration limits alone are not enough to claim attitude parity.

## 5. Open-loop validation

Export actual KSP attitude history:

```csv
time_s,aoa_deg,bank_deg,gear_down
```

Run:

```sh
./build/shuttlesim \
  --scenario scenarios/real-86km.ini \
  --atmosphere data/kerbin_atmosphere_fitted.csv \
  --aero data/stsn_aero_fitted.csv \
  --attitude-replay actual-attitude.csv \
  --replay-mode actual \
  --rate max \
  --record sim.jsonl
```

Compare at altitude checkpoints:

```sh
python3 tools/compare_replay.py ksp-normalized.jsonl sim.jsonl
```

Primary errors:

- airspeed vs altitude
- altitude vs time
- runway along-track vs altitude
- runway cross-track vs altitude
- vertical speed vs altitude

Split fitting and validation flights. A flight used to generate the aero table does not count as independent validation.

## 6. Attitude validation

Replay recorded Guidance commands with `--replay-mode command`, then compare simulated actual AoA/bank/rates to KSP actual attitude.

Only after both world/aero and attitude validation pass should live Guidance connect to ports 8795/8796.
