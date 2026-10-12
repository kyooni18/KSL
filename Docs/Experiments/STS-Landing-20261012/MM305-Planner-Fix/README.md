# MM305 planner and coast-warp investigation

The user reserved live flights for themselves and requested dry runs only, then requested wrap-up, commit, and rebuild. No guidance was engaged, no save was loaded, no flight was restarted, KSP was not paused, and LMP remained enabled during this work. Full-flight uncertainty outcomes and touchdown remain unmeasured.

## Changes

- `113cf67`: hoist source-force normalization out of the projection's incidence search and reuse each integration step's atmosphere/q/Mach. All stress cases, qualification gates, numerical refinement, and tolerances remain intact.
- `3f6ba7c`, `e6885e5`: add an offline fixed-state passive-entry/full-plan benchmark.
- `b36c4a2`: require six consecutive bounded simulation seconds of observed deficient thrust before the sustained-thrust-loss abort. A clock jump or one zero-thrust sample no longer represents sustained failure. The regression checks transient recovery and persistent engine failure.
- `666476b`: publish coast-warp progress once per wall second through the backend snapshot status, preserving the last actual flight telemetry sample.

## Evidence

Local artifacts are in `runs/20261012-mm305-planner-investigation/`.

| Check | Result | Evidence |
|---|---|---|
| Fixed passive-entry fixture before optimization | 52.090623 CPU seconds | `passive-before.out`, `passive-before.err` |
| Same fixture after optimization | 15.144528 CPU seconds; identical reported state and closest distance | `passive-after.out`, `passive-after.err` |
| Same fixture on final code | 16.532625 CPU seconds; identical reported state and closest distance | `passive-head.out`, `passive-head.err` |
| Default current-orbit planning only | 402.950186 wall seconds, 377.378 CPU seconds; execution qualified; downstream capture 0/15 | `live-plan-default.log`, `live-plan-default.stderr`, `live-plan-default.json` |
| Supersonic variant current-orbit planning only | Stopped during robustness at user-requested wrap-up; total timing and qualification incomplete | `live-plan-supersonic.log`, `live-plan-supersonic.stderr` |
| Default native suite | Passed | `native-test.log` |
| Supersonic variant native suite | Passed | `variant-test.log` |

Default planning used the planner from `e6885e5`; later commits changed the burn watchdog and warp progress, not the planner calculations. The variant dry run used `666476b`. The two orbit samples were collected sequentially while KSP continued running; they are not an identical-state uncertainty comparison.

## Historical apparent stall and abort

Run `runs/20261012T095005Z-mm305-bisect-96eb2dc` completed its coast warp and subsequently failed its deorbit burn. The saved sample shows responses being decoded across multiple kRPC calls, rather than evidence of one permanently blocked `read_warp_state` request. Planner events record warp start toward UT 146917986 and completion. KSP logged STS packing into LMP subspace -1 at 18:50:18, then unpacking near the target at 18:55:56. The synchronous warp held the control thread and suppressed normal telemetry publication during the coast.

At the 18:56:23 abort, KSP logged an LMP adjustment from UT 146918008.688932 to 146918012.194485 followed by packing/unpacking. Recorded thrust had risen through roughly 2598 and 2692 N before a zero-thrust sample. The old watchdog compared that single deficient sample with UT since the last 0.25 m/s increment. The replacement tests sustained observed loss instead. This fixes the reproduced false-abort condition; live behavior remains to be verified by the user.

Supporting artifacts: `planner.sample.txt`, `bisect.sample.txt`, `witness.sample.txt`, `warp-events.jsonl`, `ksp-stall-excerpt.log`, `burn-abort-excerpt.json`.

## Builds

```sh
make -C CLanding -j4 all
make -C CLanding -j4 BUILD=build-supersonic MM305_SUPERSONIC_ACTIVE_UNCERTAINTY=1 all
```

Use `CLanding/build/landing_backend` for default and `CLanding/build-supersonic/landing_backend` for the variant. Python mission selection uses `KSL_BACKEND`, not the Make `BUILD` variable. Full flights still need to record MM305 margin, veto bits, and touchdown; the dry runs do not establish a full-flight outcome difference.
