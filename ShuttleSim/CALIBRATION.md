# Calibration and KSP parity plan

The code/runtime is only half of ShuttleSim. The KSP-parity model comes from the project's real telemetry.

## Rule

Do not tune Guidance against ShuttleSim until open-loop `actual` attitude replay reproduces multiple real KSP flights that were not used to fit the model.

## 1. Capture the actual 86 km initial state

Create a scenario containing exact KSP inertial position, inertial velocity, UT, and mass. This removes initial-condition ambiguity from every later comparison.

## 2. Atmosphere

Normal ShuttleSim flight physics uses the stock Kerbin atmosphere model embedded
in `world.c`: the stock pressure and temperature FloatCurves, latitude bias,
latitude/Sun multiplier, Kerbin rotation, and Kerbin orbital/Sun phase. Density
and speed of sound are derived from the same thermodynamic constants used by the
stock body definition. This is position- and UT-dependent; it is not an
altitude-only median fit.

Do not pass `--atmosphere` for stock-Kerbin parity runs. That option is retained
only for deliberate custom/experimental atmosphere tables. An altitude-only CSV
cannot reproduce KSP's latitude/day-night temperature field and must not be used
as the default parity model.

Telemetry-derived atmosphere tables remain useful as an independent validation
oracle: compare stock-model pressure, temperature, density, and speed of sound
against held-out KSP flights, but do not fit the production atmosphere to the
held-out flight.

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

The simulator uses a q-dependent, second-order closed-loop attitude plant with
independently measured KSP angular-rate and angular-acceleration limits. The
plant therefore consumes real time during AoA changes and bank reversals instead
of teleporting the lift vector to Guidance demand.

The fitter estimates measured rate/acceleration limits, forward-fits the
high-q closed-loop natural frequency/damping, then fits the low-q authority
transition against command-vs-actual telemetry using the same q-scaling law as
the C simulator:

```sh
python3 tools/fit_attitude_from_telemetry.py attitude-normalized.csv data/stsn_attitude_fitted.ini \
  --physics-dt 0.02
```

Do not hand-pick the full-authority q thresholds. They are plant parameters and
must come from recorded KSP response. Fit only on training flights and reserve
separate flights for command-replay validation.
The fitter uses a 20 ms fixed plant step and zero-order-holds each recorded
attitude demand until the next recorded update. It may interpolate observed q
inside a sparse logging interval, but it never interpolates control commands.

## 5. Open-loop validation

Export actual KSP attitude history:

```csv
time_s,aoa_deg,bank_deg,gear_down
```

Run the held-out replay at KSP's 50 Hz physics cadence. A smaller step is
useful for numerical-convergence diagnostics, but parity runs should preserve
KSP's 20 ms fixed-update cadence rather than using timestep error to retune the
plant:

```sh
./build/shuttlesim \
  --scenario scenarios/real-86km.ini \
  --aero data/fitted/stsn_aero_ksp_robust.csv \
  --aero-book data/fitted/stsn_force_book.csv \
  --attitude data/fitted/stsn_attitude_ksp.ini \
  --attitude-replay actual-attitude.csv \
  --replay-mode actual \
  --dt 0.02 \
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
