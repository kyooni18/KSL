# Fourth approved MM305 continuation: 150 m aim and ground-only drogue

Approved by the user in chat for exactly one flight: reload
`ksl-20261011T064053Z-live-85km-10deg-landing-mm305`, run
`bin/ksl land --from mm305 -p sts-85km-10deg`. Backend built at `a967511`'s tree
(`make -C CLanding` up to date; `Configuration/sts-live.json` touchdownAimAlong 150 m,
rolloutParachutes true). Save loaded with `Tools/ksp_test_guard.py load-save`; flight run
with `KSL_DATA_ROOT` set to this worktree and gate/Final/TAEM traces on.
Evidence: `runs/20261011T143513Z-live-aim150-drogue-verification/`
(status, events, telemetry.jsonl, backend.stderr.log; not copied into the repo).

## Outcome

**Mission COMPLETE** (exit success): phases MM305 TAEM, Final Approach, Touchdown, Rollout,
Complete, "rollout complete; vehicle stopped". This is the first approved continuation
without an automatic or operator abort. Status carried one warning: "Live aerodynamic
response is outside the certified prior-flight model uncertainty (residual 97%)".

| Event | UT | Speed | Vertical | Along |
|---|---:|---:|---:|---:|
| Flare sample | 7054.9 | 52.6 m/s | −3.04 m/s | 1641 m |
| First landed flag | 7055.49 | 51.04 m/s | −0.64 m/s | 1674 m |
| Drogue deployment requested | 7055.41 | 51.13 m/s | | |
| 50 m/s, rolling | 7056.5 | 50.0 m/s | | 1722 m |
| Last sample | 7103.5 | 1.5 m/s | | 2556 m |

KSP is paused and landed (mass 25122.76 kg, unchanged from before within 0.01 kg), with
zero throttle/thrust at the probe. Telemetry is about 10 Hz; the table is a chronology,
not a wheel-contact sensor, and the first landed flag follows contact.

## What this does and does not verify

- **Ground-only drogue request: verified.** Exactly one request, at confirmed contact, at
  51.13 m/s (inside the 5-100 m/s window); none while flying.
- **Drogue inflation: not verified.** A request does not prove inflation. A later read-only
  probe of the parachute part raised a server null-reference error in `get_Deployed`, so
  the part state is unknown. Rolling distance from 50 m/s was about 840 m (UT 7056.5 to
  stop near 2560 m) versus about 1030 m in the third continuation (1598 to 2630 m), roughly
  18% shorter, but these are different flights with different brake timing, so this is
  suggestive only.
- **Earlier touchdown: not achieved.** Despite `touchdownAimAlong` = 150 m, contact was at
  about 1674 m along (previous continuations 1531, 1829, 1567 m). The aim moves the glide
  target, but the float still carries the vehicle ~1.5 km past it. The aim change did not
  produce an earlier touchdown in this flight.
- **Runway:** the vehicle stopped at about 2556 m, ~56 m past the 2500 m runway length
  (cross-track about 12 m). The mission ended COMPLETE; the stop is still beyond the
  configured runway end, so this is not a clean on-runway landing.
- **Touchdown speed/sink gate:** 51 m/s is below the 63.75 m/s gate. Sink before contact
  was about 3 m/s. These are sampled values.

Not exercised: universal backstop, deorbit/MM304 route, adaptive flare (off).
No tuning was done on this flight. Offline L5 golden and acceptance gates unchanged.
