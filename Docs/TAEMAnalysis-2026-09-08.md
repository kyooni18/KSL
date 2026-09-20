# Recent flights from TAEM onward — 8 September 2026

The newest run reaches Heading Alignment but does not converge to the runway: the HAC capture flag remains true while cross-track error grows, and attitude recovery interrupts the exit. The three preceding substantive TAEM tests never reach Heading Alignment. None of the sampled windows reaches Final Approach, flare, or touchdown.

## Scope and reproducibility

Analyzed the latest normal-entry flight and the three latest substantive TAEM-only tests, plus the intervening one-sample aborted test. Dates/times below are UTC filename times (add nine hours for Korea). The newest file was actively growing: this report uses a fixed byte-prefix read ending at snapshot 6528, UT 24081.673, captured at 2026-09-07 23:48:52 UTC / September 8 08:48:52 KST. It makes no claim about subsequent outcome. Other files also end without a landing record; an endpoint alone does not establish a crash or intentional stop.

[Extracted metrics](../Runtime/TAEMAnalysis-2026-09-08/metrics.json) records source byte counts, build metadata, sequence numbers, UT, phase transitions, 20-second samples and summary statistics. [Extraction script](../Runtime/TAEMAnalysis-2026-09-08/analyze.py) can regenerate the analysis, but rerunning it will include newly appended data. Statistics are snapshot-weighted, with Abort excluded; commanded-versus-measured attitude errors are simultaneous differences, not latency-compensated tracking estimates. Altitudes below are mean altitude, not terrain clearance.

| Flight, September 7 UTC | Mode | Observed duration from TAEM | Altitude, start → end | TAS, start → end | Result at cutoff |
|---|---|---:|---:|---:|---|
| 23:31:09 | Normal entry continuation | 213.4 s | 21.94 → 4.02 km | 666 → 132 m/s | Heading Alignment, then Attitude Recovery |
| 17:04:18 | HAC-only test | 61.8 s | 22.46 → 11.14 km | 332 → 199 m/s | Transition join incomplete |
| 17:01:19 | HAC-only test | 12.6 s | 11.44 → 9.01 km | 348 → 238 m/s | Pre-HAC dogleg only |
| 15:09:59 | HAC-only test | 130.8 s | 15.05 → 5.81 km | 326 → 129 m/s | Acquisition; substantial lateral divergence |
| 17:13:23 | HAC-only test | 0.14 s | 14.63 → 14.61 km | 328 → 328 m/s | Immediate capture abort |

The 15:09 run reports native build September 7 23:22:57; the others report September 8 02:00:05. Bridge source hashes are absent, so these are not controlled comparisons of identical software and initial conditions. The intervening recent full-entry sessions at 21:46, 22:09 and 22:29 never enter TAEM and are outside the requested phase window.

## Newest flight: the exit geometry fails before recovery

- **TAEM entry, #5188 / UT 23868.273:** 88.42 km before the threshold, cross-track −1.96 km, 21.94 km altitude, 666 m/s. The selected HAC radius is 96 km; remaining arc 67.92 km, radial error +27.7 km. This is acquisition, not an established circle.
- **Heading Alignment, #5676 / UT 23946.073 (+77.8 s):** 43.45 km before threshold, cross-track −3.23 km, 11.58 km altitude, 493 m/s. HAC capture becomes true with radial error +3.3 km and 35.04 km arc remaining.
- **Near exit, #6447 / UT 24068.973 (+200.7 s):** only 7.23 km before threshold, but cross-track −5.77 km, altitude 4.05 km, TAS 262 m/s and FPA −9.66°. Arc has already become negative (−0.82 km), while `hacCaptured=true` and `hacCompleted=false`.
- **Recovery, #6453 / UT 24070.053 (+201.8 s):** cross-track −5.79 km, altitude 4.00 km, TAS 260 m/s. Body roll rate reaches 38.56°/s (40.81°/s next snapshot), with sideslip about −7°. This supports a roll/yaw upset near exit rather than a primary pitch-tracking failure. The exact trigger branch is not logged.
- **Last sampled record, #6528 (+213.4 s):** recovery persists, TAS has fallen to 132 m/s and vertical speed is +32.5 m/s. HAC arc is frozen at −1.04 km; this reflects inhibited progression during recovery, not completed alignment.

At #6447, the current final-envelope implementation requires cross-track below 400 m, speed no greater than 230 m/s, and altitude within about 578 m of the 20° glide line (approximately 2.70 km here). The actual flight misses all three: 5.77 km cross-track, 262 m/s and 4.05 km altitude. Its −9.66° FPA is also shallower than the −12° upper boundary. Thus recovery did not spoil an otherwise valid final capture: geometry and vertical state were already unsuitable.

The current HAC gate allows radial error up to 10% of circle radius: **9.6 km** on this circle, plus course error under 24° and a broad energy/altitude check. That explains why a true HAC flag can coexist with a runway miss; it is not proof of terminal convergence. Arc also advances from polar-angle motion before capture when no transition join is active. In this run the join is inactive throughout normal TAEM. These are priority areas for replay investigation, not proof that every large-radius circle is invalid.

### Energy and attitude observations

The status denominator is an **energy-dissipation ceiling**, not a conventional desired approach speed: examples include 493 / 1090 m/s at HAC capture and 280 / 504 m/s later. Current ordinary TAEM code also feeds that ceiling, with altitude adjustment, to powered throttle. The log confirms actual powered flight: command throttle stays between 37.3% and 45% during Heading Alignment, with measured thrust up to 362 kN. This ceiling-as-speed-demand coupling deserves separate correction/replay; it is inappropriate to interpret the denominator as a tracking target the shuttle should simply accelerate toward.

Pitch tracking is comparatively close in this window: median absolute AoA command error 0.25°, versus median roll command error 2.26°. No stall is reported, peak dynamic pressure is 25.64 kPa and peak G is 2.77. The overall window therefore does not show a stall or pressure-limit event causing the runway miss. Roll/yaw rates nevertheless grow sharply near exit. Recovery changes target pitch from about −6.65° to +8.23° and cuts throttle; the subsequent climb and rapid speed loss warrant checking recovery energy management, without attributing all deceleration to this command alone.

## Older tests

**17:04:18 — altitude spent during the join.** A 60 km HAC is selected, but join progress reaches only 0.165 by cutoff and the 5.38 km circle arc remains unconsumed. Altitude drops 11.33 km in 61.8 seconds; peak sink is 225.9 m/s. At +20 s actual FPA is −43.8° against −19.3° requested. AoA often commands 28° while measured AoA is roughly 24–26° (median absolute error 3.32°). End state improves to −25.6° FPA, but still has 39.7 km join+arc remaining and −6.43 km runway cross-track. This is a steep-descent capture problem with an unfinished join, not evidence of circle tracking or landing success.

**17:01:19 — short, disturbed starting condition.** Starts at −42.5° FPA, −235 m/s sink, 36° bank and almost zero AoA. Over 12.6 seconds, sink improves to −125 m/s and course turns from 88° to 101° toward a 116° dogleg command. It loses 2.42 km altitude and 110 m/s speed. Median AoA error is 5.82°. The window is too short to judge subsequent HAC performance.

**15:09:59 — turn tracking without successful path capture.** After a dogleg, the status reports a 3.8 km HAC. Remaining circle arc stays at 14.70 km while join+arc decreases to 15.2 km. Cross-track evolves from +0.40 to −8.48 km; course ends at 337.8°, TAS 129 m/s and sink −52.7 m/s. Final radial error is +2.8 km, approximately 74% of the stated circle radius. Median roll error is only 0.66° and AoA error 0.97°: this suggests path/join feasibility or guidance demand needs attention, rather than simply increasing attitude gains. Transition flags are unavailable in this older schema, so join state is inferred from status text and fixed arc, not directly observed.

**17:13:23 — unsuitable test engagement.** Only one TAEM sample precedes automatic abort with “insufficient height or excessive attitude rate.” It starts almost inverted (179° roll), descending at 179 m/s. The message combines conditions and does not identify the exact branch; the log does not support calling this a low-altitude failure at 14.6 km. Exclude it from HAC tracking statistics.

## Recommended investigation order

1. Replay the newest run's ordinary TAEM lateral guidance, especially why commanded banks remain near zero while cross-track grows, the 96 km capture tolerance, and arc consumption before actual circle establishment. Require predicted final-envelope convergence in addition to circle proximity.
2. Separate the dissipatable-energy ceiling from the powered-approach speed schedule. Replay with measured thrust/drag to assess the effect on final speed and altitude.
3. Examine roll/yaw command and body-rate histories around #6400–6454, followed by recovery pitch/energy behavior. Good median AoA tracking does not exclude the observed lateral upset.
4. For HAC-only tests, validate achievable descent/join geometry from the actual initial attitude and energy before comparing controller tuning. These starts and builds differ substantially.

Analysis only: no flight controls, running processes, configuration, or production source were changed. Source references describe the current checkout; native build metadata alone does not prove source identity with each flown binary.
