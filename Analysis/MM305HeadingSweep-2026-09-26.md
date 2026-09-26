# MM305 heading sweep — 2026-09-26

Four additional ShuttleSim-only starts held position, altitude (26 km), airspeed
(790.836 m/s), and flight-path angle (−8°) fixed while changing heading. The
states were reconstructed with `compile_case` from the versioned MM305 entry
fixture. They are synthetic MM304 handoff surrogates, not measured handoffs.
The fresh `CLanding/build-mm305-fresh-20260926` binary used the committed
roll-aware TAEM tracker and the production `Configuration/default.json`
(3.2 km Final station, 28° glide slope). `engageHACTest` ran with the explicit
ShuttleSim backend, automatically assigned simulator UDP ports, `--no-mirror`,
and `--no-archive-replay`. No live KSP connection or control was used.

The command-wobble metric counts sign changes between requested bank angles
of at least 50° magnitude in the first 100 simulated seconds. A prior nominal
run before the roll-aware tracker change had five such reversals.

| Entry heading | Early near-full bank reversals | HAC exit (along, cross, h, airspeed) | First contact (along, cross, speed, sink) | Result |
| --- | ---: | --- | --- | --- |
| 80° | 1 at 8.3 s | −3388 m, −3 m, 1804 m, 151 m/s | +1785 m, −6 m, 69.8 m/s, 2.0 m/s | Runway rollout passed the end at +2581 m, 24.5 m/s |
| 85° | 0 | −3390 m, −23 m, 1798 m, 150 m/s | +1828 m, +4 m, 67.8 m/s, 2.0 m/s | Runway rollout passed the end at +2581 m, 24.7 m/s |
| 95° | 0 | See below | +1842 m, −279 m, 64.4 m/s, 0.7 m/s | Lateral runway miss |
| 100° | 1 at 8.3 s | −3389 m, −2 m, 1804 m, 150 m/s | +1829 m, +33 m, 69.5 m/s, 1.6 m/s | Runway rollout passed the end at +2582 m, 27.1 m/s |

All four selected fixed HAC routes and reached Final. None passed the complete
existing contact and rollout check. The 80°, 85°, and 100° runs did contact the
physical runway with acceptable contact speed and sink; they had only about
670–715 m of runway remaining at approximately 68–70 m/s. Their first
meaningful failure is the MM305 exit and Final approach placement/energy,
followed by insufficient rollout distance. It is not repeated early target
reversal.

The 95° run reveals a separate MM305 transition defect. Its chosen HAC radius
was 3 km. The first `MM305_LIVE_EXIT` event fired at 192.5 m/s with a requested
bank of −69.9° while measured bank was −1.8° and runway-course error was 15.6°.
Two subsequent course-error route releases occurred. A later exit was near
−3397 m along, +34 m cross, 1825 m high, and 150 m/s; contact then missed the
runway by 279 m laterally. Geometric endpoint proximity and a 12° reference
course tolerance did not ensure a settled, flyable runway alignment. The
earlier experiment that simply held the aircraft on the exhausted HAC arc
until Final's numeric contract passed crashed, so this result calls for an
explicit post-HAC conditioning segment rather than an endpoint-only latch.

Compressed simulator and guidance traces plus backend stderr are in
`ShuttleSim/runs/` under these IDs:

- `mm305-heading-80-fresh-20260926T105123Z-de6ps6zj`
- `mm305-heading-85-fresh-20260926T105130Z-upwdj7wh`
- `mm305-heading-95-fresh-20260926T105136Z-j4oprtxj`
- `mm305-heading-100-fresh-20260926T105142Z-vjfp0tle`
