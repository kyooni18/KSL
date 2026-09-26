# MM305 cardinal-heading sweep — 2026-09-26

All flights used ShuttleSim and the fresh roll-aware MM305 build from the
earlier heading sweep. The runner explicitly selected its simulator transport,
automatically assigned simulator UDP ports, disabled telemetry mirroring, and
used the unchanged production 3.2 km / 28° Final geometry. No live KSP
connection or control was used. The generated Cartesian states preserved the
requested air-relative heading, altitude, speed, and flight-path angle.

## Same position as the 80°–100° sweep

These four cases held the same position about 60 km west of RW09, 26 km
altitude, 790.836 m/s airspeed, and −8° flight-path angle while changing only
heading. The >50° requested-bank sign-reversal count in the first 100 s was
zero in every case.

| Heading | HAC route | End of controlled flight | Result |
| --- | --- | --- | --- |
| 0° | None qualified | along −27.7 km, cross −14.7 km, h 1.0 km | No feasible route before descent limit |
| 45° | None qualified | along −20.0 km, cross +3.4 km, h 1.0 km | No feasible route before descent limit |
| 180° | None qualified | along −27.7 km, cross +14.7 km, h 1.0 km | No feasible route before descent limit |
| 270° | None qualified | along −65.7 km, cross −11.4 km, h 1.0 km | No feasible route before descent limit |

At each planning attempt, the native planner rejected both runway ends and
both HAC sides because no join had an energy-feasible vertical profile. The
acquisition controller kept flying the simulated vehicle, then MM305's
existing low-altitude condition aborted. These are distant, poorly aligned
states; the result does not show that cardinal-heading HAC flight is generally
impossible. It also does not show a recurrence of the repeated bank-target
S-turns observed before the roll-response correction.

## Inbound 0° and 180° comparison

To separate heading from the far-west geometry, two supplementary cases used
mirror inlet positions near RW09: about +1 km east and 3.5 km south/north of
the threshold, 10 km altitude, 180 m/s airspeed, and −8° flight-path angle.
Heading 0° approached from the south; heading 180° approached from the north.
These are HAC-inlet probes, not measured MM304 handoffs. Both selected a
4.32 km radius HAC, reached Final, had no route release, and had zero
near-full bank-target sign reversals in the first 100 s.

| Inbound heading | MM305 exit (along, cross, h, V, bank) | First contact (along, cross, V, sink) | Result |
| --- | --- | --- | --- |
| 0° | −3389 m, −24 m, 1840 m, 134 m/s, −26.8° | +1654 m, −193 m, 68.9 m/s, 1.4 m/s | Left of runway |
| 180° | −3388 m, +26 m, 1823 m, 135 m/s, +25.6° | +1630 m, +99 m, 68.8 m/s, 1.0 m/s | Right of runway |

At MM305 exit, bank was about ±25° and runway-course error about ±2.5°
despite small exit position error. The first Final snapshots still showed
roughly ±25° bank. Lateral displacement then grew to
99–193 m by contact, beyond the 35 m runway half-width. The next MM305
mechanism to address is post-HAC alignment and bank settling before handing
Final an aircraft that is still turning. Simply retaining the exhausted HAC
arc until the existing Final numeric contract passes was previously tested
and caused a TAEM crash.

Compressed ShuttleSim traces and backend diagnostics are under these run IDs:

- `mm305-heading-0-cardinal-20260926T112201Z-_r64i9hp`
- `mm305-heading-45-cardinal-20260926T112235Z-p7qok1cp`
- `mm305-heading-180-cardinal-20260926T112311Z-ol7pp4m6`
- `mm305-heading-270-cardinal-20260926T112345Z-k4cxqwj5`
- `mm305-heading-0-inbound-20260926T112454Z-17b2czyv`
- `mm305-heading-180-inbound-20260926T112458Z-_7vebvsq`
