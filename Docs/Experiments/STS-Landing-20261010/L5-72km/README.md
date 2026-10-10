# L5 and continuous MM305/Final validation

L5 starts from `STS-OMS-72km-20261009-v2` with the validated fixed-flare
backend and STS model. `launch.json` identifies its runner, requested MM305
ownership checkpoint, and immutable backend hash. `live.log` contains progress.

The checkpoint observer writes `mm305-final.physics.json` only after observing
the new save and independently checking intact, unpowered vessel physics close
to its saved UT. A missing profile or observer error is not a certified checkpoint.
The runway observer writes `runway-deck-raycasts.json` only after nearby collider
measurements pass both origin-height agreement and lateral deck-shape checks.
Review coordinates and measured heights before changing the guidance target.

After L5 has completed naturally and no flight runner remains, the combined test
uses this command with the save name from `mm305-final.physics.json`:

```sh
KSP_LANDER_UNPOWERED_ONLY=1 KSP_LANDER_MODEL_PROFILE=sts-live-20261010 \
KSP_LANDER_FINAL_TRACE=1 python3 Tools/headless_flight.py \
  --config Docs/Experiments/STS-Landing-20261010/L5-72km/configuration.json \
  --live --reentry-only --initial-save <verified-MM305-save-name> \
  --checkpoint-physics-profile Docs/Experiments/STS-Landing-20261010/L5-72km/mm305-final.physics.json \
  --no-crash-guard --preentry-physics-warp 1 --max-wall-seconds 3600
```

Remove any inherited `KSP_LANDER_CONTINUE_UNQUALIFIED` override before execution.
Verify one KSP and zero existing headless runners; preserve both protected
`86km.sfs` sources and unrelated saves. Retain a separate output directory and
launch manifest for each test. This command is prepared, not yet executed.

Normal `engageReentry` adopts MM305 for a late terminal checkpoint. Route replay
qualification and Final admission remain required. Do not replace this with
separate HAC or Final test engagements. Acceptance covers all observed MM305,
Final, touchdown, and rollout phases, physical runway contact, touchdown speed
and sink limits, and a completed stop within the runway envelope.

Guidance's `site.altitude` and native simulator scenario `runway_elevation_m`
are separate values. Both need the measured contact reference for consistent
replay. Terrain elevation alone does not certify the runway collider surface.
