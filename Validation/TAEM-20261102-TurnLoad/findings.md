# TAEM turn-load offline check, 2026-11-02

The requested turn-load q floor, removal of scheduledQ, load-one nepFront, and max(q,reservedQ) nominal reference were already in the inherited working tree. Release build passed. Prior scratch results were preserved in scratchpad/before-turn-load-check.

72 km nominal simulation: timedOut at 2000.1 s. Touchdown accepted at runway along 1168.330 m, cross 0.103 m, horizontal speed 58.315 m/s, sink 2.067 m/s, pitch 11.073 degrees. This is not successful rollout and not a live landing.

At final sample time 211615.493 the vehicle is stationary on the runway (~1731.433 m along), mainGear=true, noseGear=false, with pitch rate 3.1168 rad/s. Rollout commands zero pitch and inhibits braking without nose contact. GroundDynamicsEngine uses a provisional main-wheel pivot (aft 0.9058 m, down gearHeight 3.2 m). Its balance angle is atan(aft/down), about 15.8 degrees. Following touchdown pitch rises from 11.89 degrees at 210588.493 to 15.84 at 210589.993 while lift nearly unloads the wheels. The model subsequently carries pitch upward; no validated tail-contact/overturn handling is present. Do not remove the nose-contact completion gate or alter physical capability to force success. Ground-model validity and derotation control remain unresolved.

At TAEM headingAlignment entry, time 210542.993, E=1911.58 m and EN=1758.44 m (153.13 m excess). This is not a precise NEP crossing measurement. Straight-in remains selected. Repeated S-turn entries and EMIN>EN crossings remain in this run, so the turn-load correction does not establish global reference consistency.

Recorded-state probe at time 210353.493 using scratch cfg35.json and fresh TAEM engine with unit gains gives EMIN=15325.28 m, EN=15908.57 m, E=9860.19 m. It restores ordering in that probe, but does not reproduce historical controller history/identified gains; it is not a matched-history trajectory validation.

The existing handoff regression was updated to test interface energy, configured Mach floor and altitude deadline. The single targeted test passed. No new physical capability perturbation, broad suite, or additional flight simulation was run. Scratch tests/debug instrumentation remain because flight completion has not been achieved. Configuration/interface placement is not promoted to validated defaults.
