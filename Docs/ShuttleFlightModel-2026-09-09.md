# Shuttle-derived flight-model architecture

## Why the model changed

The previous KSP Shuttle Lander architecture allowed same-flight aerodynamic
measurements to become predictor truth. That is attractive for a simulator, but
it is a poor flight-control architecture: an upset, scene-load glitch, damaged
configuration, bad derivative or one unusual trajectory can move the model while
the same controller is depending on it to decide the rest of entry.

The operational Space Shuttle used a different separation of concerns. Its
nominal trajectory and flight-control design relied on a preflight aerodynamic
database and explicit uncertainty variations. Entry guidance flew a reference
drag/energy profile. Flight data were heavily instrumented and reconstructed
afterward; the resulting aerodynamic comparisons were used to revise later
Aerodynamic Data Book releases rather than continuously rewriting the onboard
nominal model during the same entry.

The KSP implementation now follows that lifecycle where it is practical.

## Primary-source basis

The implementation decisions were checked against the following NASA/NTRS
primary sources:

- **Operational Aerodynamic Data Book: Volume 1, Aerodynamic Description and
  Data Usage**, NASA-CR-172019 / STS85-0118-VOL-1 (1985). The OADB is the
  operational full-scale reference and states that the data were revised using
  the Shuttle flight-test program and NASA reviews.
- **Shuttle entry guidance**, Harpold and Graves, AAS 78-147 (1978). Entry is
  formulated around a reference drag-acceleration profile that satisfies TAEM
  terminal constraints and is followed primarily through roll modulation with
  a preselected AoA schedule.
- **Analytic Drag Control / drag-bias entry guidance material**, NASA document
  19760026174. The drag-versus-velocity profile is shaped pre-mission with
  margins for aerodynamic, atmosphere, navigation and entry-state dispersions;
  guidance adjusts the reference/range solution rather than identifying a new
  aerodynamic vehicle online.
- **Terminal Area Energy Management guidance requirements**, NASA document
  19800018908. TAEM controls altitude and dynamic-pressure profiles versus
  range-to-go; AoA is the altitude channel and speedbrake is the q-bar channel.
  The planning model includes simplified rotational response so attitude
  dynamics affect the trajectory without requiring a full onboard 6-DoF model.
- **The development and application of aerodynamic uncertainties in the design
  of the entry trajectory and flight control system of the Space Shuttle
  Orbiter**, AIAA 82-1335. Worst-case aerodynamic variations were developed and
  combined with other uncertainties to stress and certify the entry FCS.
- **Effect of aerodynamic and angle-of-attack uncertainties on the ... Shuttle
  entry flight control system**, NASA-TP-2283 and NASA-TP-2365. Six-degree-of-
  freedom off-nominal simulations exercised aerodynamic, RCS, sensor, CG and
  control-system uncertainty over the entry Mach range.
- **Preliminary analysis of STS-1 entry flight data**, NASA-TM-81363. Flight
  results were compared with predicted L/D, trim and stability/control data,
  limitations in extracted derivatives were identified, and changes to the
  Aerodynamic Data Book and future flight planning were recommended.
- **On the flight derived/aerodynamic data base performance comparisons for the
  NASA Space Shuttle entries during the hypersonic regime**, AIAA 83-0115.
  Coefficients from the first four entries were reconstructed from IMU angular
  rates/linear accelerations, best-estimate trajectory and atmosphere and
  compared with the wind-tunnel-derived database.
- **Aerodynamic comparisons of STS-1 Space Shuttle entry vehicle** (1982). The
  first flights were heavily instrumented and maneuvering was deliberately
  constrained because a conventional envelope-expansion program was impossible.

## Adopted KSP architecture

### 1. Prior-flight data book is nominal truth

`Runtime/Physics/observations.sqlite3` remains the raw flight-test archive. At
connect time the bridge hashes the assembled vessel structure and atmospheric
environment. Only matching observations from earlier sessions/timeline epochs
are eligible for `physicsContext.history`; the new current session is explicitly
excluded.

The bridge reduces prior data into local q/Mach/AoA/sideslip/configuration cells
and records how many independent session/timeline histories support each cell.
The C core loads at most 192 cells. Gear, wheel brakes and speedbrakes are
separate configurations. Unknown legacy speedbrake state is retained in SQLite
for forensics but is not used as certified predictor evidence.

### 2. Current flight is shadow flight-test evidence

The C core never merges a current-flight force vector into the certified cell
bank. Instead it compares the measured specific force with the matching prior
prediction at independently spaced intervals. The resulting
`physicsModelResidual` and confidence are health/uncertainty signals.

Raw current-flight force observations continue to be saved by the bridge. They
can become prior-flight evidence only after a later session reopens the same
structure/environment context.

### 3. Planning envelope is derived from prior evidence

Clean prior cells derive subsonic, transonic, supersonic and hypersonic L/D and
ballistic fallback values. They are only a compatibility envelope for regions
where direct-force cells have no local support. High-Mach prior evidence is
preferred for deorbit/entry planning. Weighted prior scatter becomes the
certified aerodynamic uncertainty.

### 4. Uncertainty is stressed, not learned away

Deorbit robustness simulation now includes drag-high/lift-low and
drag-low/lift-high cases. Their magnitude is based on certified prior scatter.
If the current flight develops a confident force-model residual larger than the
historical envelope, the stress magnitude widens; the nominal coefficients do
not move.

### 5. Environment and control response may adapt online

Online adaptation remains where the measured quantity is genuinely an
environment or actuator property: atmosphere-density scale, local speed of
sound, effective bank/course response, body-axis control authority and low-speed
safety margins. These may change between KSP scenes/mods or depend strongly on
q and are useful to measure live without redefining the aerodynamic data book.

Automatic profile application is restricted to minimum-safe, approach and
touchdown speed recommendations. Explicit operator promotion of a reviewed
profile is still available and forces replanning.

### 6. Deliberate flight-test maneuver

The simple-glide calibration sweep now acts as a test program. The normal AoA
steps collect clean data. In the final quarter of the middle-AoA sample, and
only with generous altitude and dynamic-pressure margin, guidance commands one
short speedbrake-on pulse. That populates a configuration missing from the
existing STS-N archive without contaminating the same session's nominal model.

### 7. Entry and TAEM energy management

Entry/S-turn keeps the binary KSP speedbrake retracted and controls energy by
bank/AoA/path, because an all-or-nothing KSP action group is not a faithful
analogue of the Orbiter's continuously positioned high-Mach trim schedule.

TAEM uses one shared live/predictor speedbrake controller. It combines normalized
q-bar error, an integral q-bar term and specific-energy error, then converts the
continuous demand into binary deploy/retract hysteresis. The predictor carries
future speedbrake and gear configuration so it queries the corresponding prior
force cells rather than assuming present configuration forever.

### 8. TAEM entry is an envelope, not a label

The upper `taemForceHandoffSpeed` is no longer interpreted as "anything slower
belongs to TAEM." Normal handoff must also remain above the lower TAEM speed
envelope. A slower vehicle may enter TAEM only when it is stable, has a complete
retained terminal candidate, has enough total specific energy and lead/cross-
track maneuver room, has a finite measured turn envelope, and the candidate's
future join is inside the configured TAEM altitude shell.

Likewise, running out of HAC maneuver margin cannot force-publish an unconverged
candidate. The controller aborts if a valid phase interface was not achieved in
time.

### 9. Surface/RCS blending

The Python direct controller already identifies effective pitch/roll/yaw
authority. RCS assistance now fills only the fraction of axis demand not yet
owned aerodynamically. This gives pitch, roll and yaw independent handoff
behavior instead of commanding full RCS until one Mach threshold. A final
transonic latch prevents re-arming after the aerodynamic handoff.

## Validation state

The deterministic native suite covers certified/live separation, prior-envelope
derivation, aerodynamic uncertainty stress, configuration-specific lookup,
speedbrake state prediction/control, low-speed TAEM feasibility, HAC deadline
behavior and the deliberate speedbrake flight-test pulse. The Python suite
covers current-session exclusion, independent prior support, stream validity,
mock protocol behavior and authority-based RCS blending.

The architecture is still a reduced-order flight predictor rather than a full
Orbiter 6-DoF simulator. The next real validation step is a KSP entry replay: the
important success criteria are stable certified cell count, a non-destructive
live model residual, matched live/predicted speedbrake transitions, sensible
RCS authority fade, and improved next-session coverage after the flight-test
archive is reopened.
