# MM305 runway-line alignment experiment — 2026-09-26

All flight evidence is from ShuttleSim with the identified STS-N ideal attitude
servo, `engageHACTest`, simulator-only transport, automatic simulator UDP
ports, and no telemetry mirroring. No live KSP connection or control was used.
Commit `624be88` adds an explicit MM305 runway-line phase after the geometric
HAC endpoint. The phase uses measured vehicle state and the normal model-
inverted TAEM tracker with zero path curvature. Completion requires the actual
vehicle to have cross-track under 35 m, runway-course error under 2°, bank
under 5°, and roll rate under 3°/s; it aborts if approach distance or altitude
is exhausted. The existing Final admission and production configuration were
not changed.

## Isolated lateral mechanism

The two mirror inlet cases used 10 km altitude, 180 m/s, −8° FPA, and positions
about 1 km east and 3.5 km south/north of RW09. The baseline handed Final a
vehicle still turning near ±26° bank. With the new line segment, the shuttle
actually flew approximately 350–380 m farther while rolling out before Final
handoff:

| Heading | Baseline contact cross | New MM305 exit bank / cross | New contact cross | Remaining failure |
| --- | ---: | --- | ---: | --- |
| 0° inbound | −193 m | −1.0° / −18 m | −0.6 m | Rollout crosses runway end |
| 180° inbound | +99 m | +0.9° / +21 m | −0.8 m | Rollout crosses runway end |

The nominal 26 km/M2.6 upstream checkpoint also handed off with bank +0.09°
and contacted at cross +4 m. These are flown ShuttleSim tracks, not completion
flag overrides. Focused native-stack and MM305 recovery-monitor tests passed.

## Perturbations and remaining failure

Four further inbound heading perturbations, 355°, 5°, 175°, and 185°, each
contacted within about 8 m of runway centerline. The lateral correction is
therefore present on both sides of each perpendicular inlet. It does not solve
energy and topology: candidate selection switched runway ends for three of
the four perturbations, and touchdown speed spanned approximately 48–68 m/s.
Some failed speed/sink limits. A high-energy 95° upstream case reached the
geometric endpoint with about −37° bank and +8° runway-course error; it could
not settle cross-track before the line-phase distance limit, so MM305 aborted
instead of handing Final a falsely aligned state.

With the production 3.2 km / 28° interface, the 0°/180° mirror cases contacted
around +1.66–1.68 km at 68 m/s, and the nominal high-energy case contacted
around +1.94 km at 63 m/s. These touchdown positions leave insufficient
runway for rollout; the simulator crossed the 2.5 km runway end. An isolated
temporary 4.5 km / 20° interface stopped on-runway for four tested cases, but
every case failed at least one existing touchdown speed or sink requirement.
That setting was not committed.

The next primary MM305 mechanism is route energy and topology selection. The
native candidate search replays and prices the route only through the circular
HAC endpoint. It does not yet price the newly flown line segment or the Final
tail, so a geometrically valid route can still deliver a poor contact state.

Representative ShuttleSim run IDs (compressed simulator/guidance traces and
backend stderr reside in `ShuttleSim/runs/`):

- `mm305-align-inbound-0-20260926T113343Z-m0fbj_hz`
- `mm305-align-inbound-180-20260926T113402Z-u5ab58cp`
- `mm305-align-26km-20260926T113419Z-ftn6n13e`
- `mm305-align-heading-95-20260926T113440Z-7ubnv36p`
- `mm305-align-4500-20-inbound0-20260926T113630Z-r36j4ldm`
- `mm305-align-4500-20-inbound180-20260926T113635Z-yri7o6dw`
- `mm305-align-4500-20-exact-20260926T113641Z-a10jp275`
- `mm305-align-4500-20-heading95-20260926T113648Z-e5myc7dx`

The four ±5° perturbation runs use the prefix `mm305-align-heading-{355,5,175,185}`.
