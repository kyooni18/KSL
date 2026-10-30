# Native Swift reconstruction: evidence and qualification record

This is an implementation record, **not a flight qualification certificate**.
The authoritative C source is immutable commit
`f1eebdc0e6f8bb7e8b8a354b42eb2d829cdc3cda`. Rust and the later C wrapper are not references.

## Established mechanisms

- The active MM304 C call path is `guidance/entry/contract.inc` using
  `entry_drag_reference.c`, `entry_alpha.c`, `entry_lateral.c`, and `entry_exec.c`.
  The separate `entry_energy_control.c` is not the active longitudinal command path.
- Quaternion errors are in body FRD. A wind-bank rotation precedes incidence.
  Target-relative quaternion-error derivative damps tracking; body angular rates
  are separate evidence for actuator authority. Euler bank aliases must not become
  roll demands. The C quaternion harness was rebuilt from the original sources.
- Steady trim input is a balancing moment, not net acceleration. Applying the
  actuator-lag predictor to total trim as if it accelerated the body produces an
  artificial rate term. Native control subtracts its balancing input in that term.
- Reversal feasibility is a finite roll/yaw exchange with actuator acceleration,
  achievable rate, sideslip margin, captured-leg and time-to-terminal constraints.
  Nominal incidence attainability cannot stand in for lateral authority.
- The original observation store is opened SQLite READONLY. Schema 3 quality and
  provenance joins, independent-flight medoids, compatible identity/environment
  matching, and compact q/Mach/incidence/sideslip support follow C. The original
  archive is neither migrated nor written. New observations have a separate,
  exclusive-create journal with session/flight identity.
- A native read-only archive query returned 4096 recent compatible observations,
  reduced to 125 retained cells with six independent source flights. These counts
  describe that query, not the complete archive's coverage or a certification.
- Route energy stress failures now reject the candidate before commitment.
  Actual terminal qualification additionally executes the same guidance, FCS,
  actuator and rigid-body integration through wheel contact and stopping.
- Heavy planning uses a separate actor and bounded, one-outstanding-request
  snapshot exchange. Backwards or skipped simulation time requires a new engage;
  reconnect cannot resume an old controller's commands.

## Moment model provenance defect (not yet resolved by live identification)

`ShuttleSim/reference-model/sts_attitude_live20261028.ini` explicitly says the
base is `stsn_attitude_identified.ini`. Its direct-control pitch coefficients
12.6211 deg/s^2/kPa, 56.7948 deg/s^2, stiffness 0.61037/kPa and 2.445 deg trim
were inherited, not refitted for STS entry. The report
`stsn_attitude_identified.report.json` restricts the fitted data to Mach <= 0.9,
q >= 500 Pa, radar altitude >= 15 m, |sideslip| <= 4 deg. Its report is a passive
closed-loop fit and does not establish a supersonic moment plant.

The STS INI also says roll fits were rejected because their maximum error was
pinned at a 90-degree bank-wrap artefact. STS-N roll parameters were retained.
The October 10 STS pitch fit (`Docs/Experiments/STS-Landing-20261010/L6-MM305-Final/
pitch-response-L4-L5.json`) has body-rate held-out R^2 0.335 and explicitly says
no coefficient was adopted without replay and held-out checks. It must not be
silently substituted as a qualified alternative.

The hypothetical KSP aerodynamic API study in
`Docs/Experiments/STS-Entry-20261009/measured-control-authority-analysis.json`
shows large roll-induced yaw moments and substantial compound-input residuals.
Its own limitations prohibit treating those sparse static matrices as flight
control gains. It supplies a mechanism to investigate, not calibrated gains.

## Retained regression evidence

The nominal touchdown and safety acceptance thresholds were retained. The -20% lift
stress expectation was explicitly reclassified: a failed contact is evidence, not
an obligation to tune the nominal controller into an unsafe solution.

- Initial native FCS replacement exposed approach sink failures and entry aborts.
- After trim and response preview, nominal final completed, but physics steps
  0.02 and 0.01 s differed by 0.213 m/s at contact (allowed 0.2).
- Contact-event interpolation reduced the discrepancy; correcting the actuator
  integration to use midpoint deflection passed the original refinement test.
- The reduced-lift case remains a failure under the newly restored dynamic arc.
  A recorded run contacted at 49.13 m/s, sink -6.52 m/s and pitch 7.07 degrees.
  Another intermediate guarded run contacted at 57.91 m/s and -3.94 m/s. These
  are failures, not robust candidates. Long low-altitude energy expenditure and
  consistency of outer-glide/flare energy allocation remain under diagnosis.
- C-derived quaternion, relative-rate, lag-unit and live-torque precedence tests,
  archive provenance/read-only tests, and runtime cancellation/command sequencing
  tests passed their targeted runs. Full mission tests still require resolution.

## Live status

Read-only inspection of the existing KSP instance succeeded. No new flight has
been armed or executed in this reconstruction. No live landing or high-q
stability claim is established. KSP lifecycle and original archives are preserved.

## Independent native Swift / ShuttleSim verification (2026-10-30)

The current runner is `swift/KSL-Landing/Tools/run_shuttlesim.py` with
`--publish-web`. It uses the release `ksl-landing shuttlesim-controller`
entrypoint and the separate ShuttleSim C process. FlightComputer produces
atmospheric pitch, roll, yaw, gear and brake commands. The simulator's physics
sources and aerodynamic tables are unchanged by the runner. Every run archives
`telemetry.jsonl.gz`, `controller.jsonl.gz`, `guidance-snapshots.jsonl.gz`,
`summary.json`, source hashes and a web manifest. These are **offline model
results**, not KSP landing certificates.

- `runs/swift-native-web-final-confirmed-20261030`: nominal approach,
  **offline success**; main-wheel touchdown sink 1.164 m/s, speed 61.403 m/s,
  runway contact, nose-wheel touchdown and confirmed stop.
- `runs/swift-native-web-final-payload-mass-20261030`: +10% mass only,
  **offline success**, sink 2.559 m/s and confirmed runway stop. This does **not**
  validate an offset cargo-bay payload, CoM movement or a new inertia tensor.
- `runs/swift-native-web-final-moderate-20261030`: -5% lift, +5% drag,
  **offline success**, sink 2.439 m/s and confirmed runway stop. This is an
  exploratory moderate uncertainty, not a certified statistical confidence bound.
- `runs/swift-native-web-final-extreme-20261030`: -20% lift,
  **offline failure**; 11.750 m/s contact sink and no safe stop.
- `runs/swift-native-web-polar100-primary-20261030`: 100x100 km polar-orbit
  **offline failure**. The planned retrograde impulse was 42.183 m/s; the
  state-identified retrograde impulse was 41.852 m/s, while the integrated
  total engine impulse was 46.063 m/s (different definitions). Entry-interface
  position residual 3.887 km, timing residual 1.677 s. MM304 aborted around
  altitude 5.8 km, final cross-track -25.643 km; no touchdown.
- `runs/swift-native-web-polar100-window6-20261030`: alternative
  deorbit-window #6, **offline failure**. The planned retrograde component
  was 42.183 m/s, identified 41.869 m/s; total thrust integral 46.082 m/s.
  Entry position/time residuals 3.588 km / 1.543 s. MM304 aborted at
  altitude 14.673 km, airspeed 243.142 m/s, along-track -46.436 km and
  runway cross-track -76.304 km. Better nominal entry crossrange screening did
  not translate into a physically reachable landing.

The native window search now retains orbital-pass diversity and does not assume
periapsis closure alone implies landing reachability. It still needs a qualified
orbit-to-runway solution; unqualified independent orbital experiments require
`--orbit-experiment` and do not authorize a live KSP burn. The current
ShuttleSim vacuum actuator and off-center payload model do not constitute
measured KSP control authority. Existing KSP kRPC topology, mass, center of mass,
inertia and torque acquisition is distinct from those provisional simulator inputs.

The previous hard-coded 30% aerodynamic uncertainty floor has been removed.
The native in-process model's -10% lift approach passed; -20% lift produced
unsafe contact and is now explicitly tested as a failed stress case rather than
asserted to land safely. These changes do not alter any physical simulator
parameter or touchdown acceptance threshold.

The existing KSP telemetry server remains at `http://127.0.0.1:8789`; new
independent runs are visible via `/api/sim-runs` and `/api/sim-replay`.
The independent simulator-mode stream is at `http://127.0.0.1:8790` and UDP
8797. The original live KSP `controller-snapshot.json` is never replaced by
the Swift simulation mirror. The previous `simulator-run.json` was backed up
before publishing new native run metadata.

Verification: ShuttleSim CTest 4/4 passed. Native Swift targeted payload,
configuration identity, attitude, deorbit and ShuttleSim-adapter tests passed;
the focused approach-step, moderate -10% and extreme -20% tests passed. The
unmodified `ksl.backend.BackendClient` passed protocol-v1 ready,
configuration-update, snapshot, unknown-method rejection, disconnect and safe
shutdown checks without arming KSP. Full entry/HAC-to-runway and live KSP landing
remain unqualified.
