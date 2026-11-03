# MM304 speed-at-location investigation

## Finding

The user identified the visible TAEM advisory appearing around1700–1400m/s, not the first background search. In live run20261102T033630, advisoryReference first appeared at UT210088.97,1684.4m/s,35737m altitude,218745m runway range. The new density recovery mode activated about0.30s later. Actual/commanded AoA then increased from about20deg toward30deg; observed drag rose from5.42 to about9.8m/s2 while its budget fell. terminalSteeringActive remained false throughout this interval. The previous08ee141 run displayed its first advisory at1669m/s but held20deg through this speed band. Thus the user observed the real timing of a regression introduced by8822f2e; the visible path itself did not issue the pitch command.

The latest live flight first crossed725m/s at UT210206.932565984, altitude24085.58m, along-track-90787.99m, still in entry. Earlier08ee141 crossed at UT210214.33256598012, altitude25692.18m, along-track-71089.00m, in acquisition. The late density pull-up worsened delivery by roughly20km.

## Model evidence

The existing entry-energy-probe compared consecutive recorded states and command intervals. Over45–55km,35–45km,25–35km altitude bands, integrated measured energy loss / drag work was0.9937,0.9961,0.9954. Median model/measured lift was0.9564,0.9989,1.0074; model/measured drag0.9631,1.0046,0.9898. This supports translational force/energy consistency in those regimes. It does not validate extrapolated moments, future trim, or all candidate incidences.

A point-mass diagnostic with the recorded initial state, actual atmosphere/planet/polar, wings level and rate-limited best-glide incidence reached725m/s beyond KSC (along+230km). Configured maximum incidence reached interface speed too early (along-232km). These are reference trajectories, not closed-loop feasibility certificates. They demonstrate a bracket in incidence schedules; they do not constitute a synthetic vehicle-capability sweep.

## Change

Removed the failed density-drag-recovery law and its maximum-lift override. MM304 now shoots a forward point-mass reference to the actual configured interface energy and solves the incidence schedule between best L/D and the configured attainable ceiling against along-track location. The blend is solved repeatedly at entryReplanInterval, not tuned to this checkpoint. Trials use rotating-frame dynamics, the real atmosphere, polar, mass, configured rate limits, measured response times when available, crossrange steering, and q/load termination. Unbracketed or incomplete predictions are explicitly diagnosed and never cause control abandonment.

The live allocator follows that range reference within its existing polar/load and attitude constraints. TAEM plans remain display/advisory data and cannot silently alter entry's priced path. The normal range-and-energy handoff plus existing emergency Mach/altitude deadlines remains. Redundant old drag-multiplier forecast branches were removed because they no longer change commands. Mission configuration unchanged.

## Decisive offline check

One controller-included entry test initially overshot to+158km. Diagnosis: the first live startup record had fallback26000kg mass and no measured controlAuthority. That initial condition generated a provisional actuator model which could not track commanded incidence. It is not the complete checkpoint physics state.

The corrected test starts at the first complete measured state, UT209616.45256629333,25183.7109375kg, recorded attitude/rates/control authority and enabled-RCS configuration. The preceding recorded guidance was replayed into FlightController to initialize its snapshot/previous target; plant starts with recorded applied controls. The comparison remains an offline model with held initial authority; it is not proof that future moments match live KSP.

This closed-loop test reaches724.0366m/s after613.30s, at altitude26429.91m ASL, runway along-33521.64m and cross+120.62m. No q/load/terrain violations were reported by the entry predictor. This checks entry speed delivery, not TAEM acquisition, touchdown, or rollout. Expensive asynchronous terminal/entry-objective planning was disabled for this isolated entry-law check. The full mission planner and live outcome remain unvalidated.

The reference test is archived as RecordedEntryDeliveryProbe.swift; it is not retained in the compile-time suite. One durable recorded-state reference regression was added; the existing advisory test now checks identical AoA/bank for a fresh executive with/without an accepted advisory. Existing handoff regression retained. Final focused checks and release build output are stored here.

No KSP loads, unpauses, controls, or live restarts were performed during this investigation. Existing live PID93978 uses an archived executable and was not modified. All earlier failures remain in the evidence logs; no landing success is claimed.
