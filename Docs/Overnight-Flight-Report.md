# Overnight Flight Report — STS 85 km / 10° Live Campaign

Session started: 2026-10-14 (local, KST). Repository: `/Users/kyooni18/Code/KSL`.
Mission profile: `Configuration/missions/sts-85km-10deg.json` (unmodified).
Vehicle profile / backend: `Configuration/sts-live.json`, `modelProfile = sts-live-20261010`.
Checkpoint policy inherited from profile: KSP saves on phases `orbit` and `mm305`.

Rules in force: no code, craft, or flight-configuration changes; reverts and checkpoint
load/save allowed; logs and checkpoints preserved; no concurrent flights; no stopping after
failures; report updated after every attempt.

Evidence classes used below:
- **Observation** — read directly from telemetry, events, status, or logs (path given).
- **Hypothesis** — inferred cause, not yet verified. Labelled as such.

## Starting state (observation, 2026-10-14 ~01:42 KST)

- KSP running (pid 17953), flight scene, vessel `STS` on the Kerbin launch pad, `situation = pre_launch`.
- UT 167.4 s, mass 520 568 kg, 8 launch clamps engaged, speed ~0.
- kRPC MCP control program: idle (no active program).
- Dry run of `sts-85km-10deg` validated the plan; nothing flown. Dry-run dir: none persisted.
- Pre-existing history relevant to this campaign (from Foundation memory and Docs/Experiments):
  a prior 85 km live run (`20261013T123555Z-sts-85km-10deg`) and a rollout overrun on
  KSL live flight 4 (drogue/aim experiment, `Docs/Experiments/STS-Landing-20261011/`).

## Attempt log

_Attempts are appended below in order. Each entry lists outcome, failure phase, telemetry,
log paths, observations, and hypotheses separately._

### Attempt 1 — from launchpad, unmodified profile

- **Run ID:** `20261013T164242Z-sts-85km-10deg` (live, started 01:42:42 KST).
- **Outcome:** FAILED. Mission state `failed`, phase `ascent`, program `ascent`, exit code 1.
- **Failure phase:** Ascent, immediately after booster separation (event at ~01:47:25).
- **Checkpoints created:** `ksl-structural-20261013T164728801996Z` (structural failure checkpoint,
  logged at 01:47:28). No orbit or MM305 checkpoints (the run never reached orbit).
- **Log paths:**
  - `runs/20261013T164242Z-sts-85km-10deg/status.json`
  - `runs/20261013T164242Z-sts-85km-10deg/events.jsonl`
  - `runs/20261013T164242Z-sts-85km-10deg/ascent.log`
  - `runs/20261013T164242Z-sts-85km-10deg/ascent.csv`
  - `runs/20261013T164242Z-sts-85km-10deg/telemetry.jsonl`
  - Console: `/private/tmp/claude-502/-Users-kyooni18-Code-CoPet/c53056ae-217b-4743-8d4f-ea02f7c0bd43/scratchpad/attempt1-console.log`

**Observations (telemetry and logs):**

- Pad settle 25 s, Clydesdale limiters set to 0.9, ignition 01:43:10, liftoff 01:43:15, gravity turn from 01:44:08.
- Booster separation: 8 separator motors ignited with booster separation (`ascent.log` line 110).
  Boosters reported as "shut down/burned out and separated" at 01:47:25. Autopilot took over pitch/yaw/roll 0.2 s later.
- Structural guard (`ksl/kspguard.py`, `check_orbiter_integrity`) raised `GuardError` about 3.9 s after separation.
  Reported missing parts, both sides: `benjee10.shuttle.deltaWing: 2`, `elevon1: 2`, `elevon2: 2`, `mainGear: 2`.
- State at failure (`status.json` telemetry): UT 283.26 s, altitude 23.9 km, speed 1028 m/s, apoapsis 59.7 km,
  periapsis -551 km, mass 181.4 t, dynamic pressure 9.6 kPa, g-force 1.26 (commanded 1.36), pitch 43.4°,
  heading 76.7°, roll -177.2°.
- Launch clamps were released (`launchClamps: 8` at pad; clamp release logged during launch).

**Operator-reported diagnosis (not verified by me):**

- The user reports that the booster separation collided with the shuttle wing and that three pairs of
  separatrons were then added to the craft. The vessel mass on the pad rose from 520 568 kg to 521 003 kg (+435 kg),
  which is consistent with the added parts. I did not inspect the craft to confirm the change.

**Hypotheses (not verified):**

- H1: the orbiter's wing, elevon and main-gear parts were struck or pushed by the separating booster at ~24 km
  and ~1 km/s. Supported by the simultaneous loss of both sides of all four part classes and by the operator's report.
  Not tested; the telemetry does not show the collision directly.
- H2: the parts did not fail from aerodynamic load (q 9.6 kPa at failure is moderate). Consistent with H1 but not proven.

### Attempt 2 — from launchpad, same profile, separatron craft

- **Run ID:** `20261013T164958Z-sts-85km-10deg` (live, started 01:49:58 KST). Launched 01:50:26.
- **Outcome:** FAILED at **Deorbit** (planning gate). Mission state `failed`, phase `deorbit`.
- **Failure phase:** Deorbit. The deorbit plan was refused before any burn: "Deorbit plan failed required strict
  full-route qualification; execution is blocked" (01:57:33).
- **Checkpoints created:** orbit KSP save `ksl-20261013T164958Z-sts-85km-10deg-orbit` (01:57:11).
- **Log paths:**
  - `runs/20261013T164958Z-sts-85km-10deg/status.json`
  - `runs/20261013T164958Z-sts-85km-10deg/events.jsonl`
  - `runs/20261013T164958Z-sts-85km-10deg/deorbit-plan.json`
  - `runs/20261013T164958Z-sts-85km-10deg/ascent.log`, `ascent.csv`, `orbit-trim.log`, `telemetry.jsonl`, `checkpoint.json`
  - Console: `/private/tmp/claude-502/-Users-kyooni18-Code-CoPet/c53056ae-217b-4743-8d4f-ea02f7c0bd43/scratchpad/attempt2-console.log`

**Observations:**

- Ascent succeeded past the attempt-1 failure point. Booster separation at 01:52:38. No structural guard failure.
  Orbiter inventory at deorbit: **26 parts** (`Orbiter structural inventory verified: 26 parts`, 01:57:12).
- ET separation at 01:55:15; orbital insertion complete at 01:55:20 (MECO apo 85.37 km, peri 54.71 km).
- Orbit trim: burn 1 of 29.72 m/s planned. Result **Ap 85.01 km, Pe 84.99 km, inc 10.000°**.
- Deorbit plan (`deorbit-plan.json`, `executable=True`, `strict=False`):
  - Burn UT 1328.9, Δv 30.05 m/s, burn normal angle 13.4°, estimated burn 44.5 s.
  - Predicted post-burn periapsis 49.9 km; predicted entry FPA −1.63°; predicted entry range 1133 km.
  - `nominalCaptureAchieved = false`, `robustnessQualified = false`, `executionQualified = true`, `executionDegraded = true`.
  - **Robustness: 0 / 15 passed (0%)**, 0 unsafe, recovery 0 / 0%. The campaign gate requires ≥ 75%.
  - Predicted TAEM distance 46.7 km; predicted TAEM range error **−49.8 km**; predicted closest distance 5.4 km.
  - Worst-case entry closest distance 229 km; worst-case TAEM range error 498 km; worst-case peak q 6.3 kPa; worst-case peak g 3.2.
  - Planner note: "Orbital burn and safe atmospheric entry are qualified... MM304/TAEM shadow result is separate."
- The deorbit state at refusal: UT 613.3 s, altitude 85.0 km, speed 2074 m/s, situation orbiting, backend phase Idle, throttle 0.
  KSP was unpaused during the search (`Deorbit search runs with KSP unpaused`).

**Hypotheses (not verified):**

- H3: the refusal comes from the strict-robustness gate (0 / 15 scenarios pass). It is not an orbit or ascent problem,
  since orbit was achieved within 1 km of target. The `executionQualified=true` / `strict=False` pair suggests the
  engage path is using the entry-stage gate while the strict gate is still enforced at execution. This matches the
  unresolved discrepancy recorded in `Docs/Experiments` (STS campaign notes, 2026-10-08). Not resolved by this run.
- H4: the 0 / 15 robustness result comes from the same entry-model uncertainty (planning aero) recorded as provisional
  in the 2026-10-08 campaign notes, not from the burn itself. Not tested.

### Attempt 3 — replay of attempt 2 from the orbit checkpoint

- **Run ID:** `20261013T165817Z-continue-sts-85km-10deg` (`ksl continue <attempt2> --use-save --label attempt3-orbit-replay`).
- **Loaded save:** `ksl-20261013T164958Z-sts-85km-10deg-orbit` (orbit, Ap 85.01 km, Pe 84.99 km, 10.000°).
- **Outcome:** FAILED at **Deorbit**, same refusal as attempt 2: "Deorbit plan failed required strict full-route qualification; execution is blocked" (01:59:04).
- **Log paths:**
  - `runs/20261013T165817Z-continue-sts-85km-10deg/status.json`
  - `runs/20261013T165817Z-continue-sts-85km-10deg/deorbit-plan.json`
  - `runs/20261013T165817Z-continue-sts-85km-10deg/events.jsonl`
  - Console: `/private/tmp/claude-502/-Users-kyooni18-Code-CoPet/c53056ae-217b-4743-8d4f-ea02f7c0bd43/scratchpad/attempt3-console.log`

**Observations:**

- Orbiter inventory 26 parts (01:58:40), same as attempt 2.
- Deorbit plan: burn UT 1328.78 (attempt 2: 1328.88), Δv 30.052 m/s (attempt 2: 30.052), `executionQualified=True`, `executionDegraded=True`, `strict=False`.
- Robustness **0 / 15** passed, `robustnessQualified=False`, `nominalCaptureAchieved=False`.
- Predicted TAEM range error −49.8 km, predicted closest distance 5.36 km (attempt 2: 5.38 km).
- Refusal occurred before any burn. KSP was unpaused during the search.

**Findings so far (observations, not hypotheses):**

- The orbit phase is repeatable and within tolerance: 85.01 × 84.99 km at 10.000° in attempt 2 (single sample).
- The deorbit gate refused both the full-flight plan and the checkpoint replay with near-identical numbers.
  The refusal is reproducible from the saved orbit state.
- Attempt 1 failed earlier, at booster separation (structural guard). Attempt 2 cleared that point after the separatron change.
- No attempt has yet reached a burn, entry, TAEM, or touchdown.

**Hypotheses (not verified):**

- H3 and H4 from attempt 2 still apply. Replays from the same orbit state cannot distinguish the cause.
- A repeat full run from the pad would test whether the refusal is stable across different ascent states, not just the same save.

### Blocker — return to launchpad for attempt 4 (2026-10-14 ~02:01 KST)

Goal: attempt 4 as a full pad-to-landing run with the separatron craft. Status: **blocked, not flown.**

What I tried (all read-only or revert-only; no craft, config, or code changed):

1. `mcp__kRPC__run` with `space_center.revert_to_launch()` — returned `reverted to launch` with exit 0, but the
   vessel stayed `situation=orbiting`, UT 641.56 s, mass 26 014.6 kg, unchanged across two checks.
2. Same call with `space_center.paused = False` first — again no change; the game reported `paused=True` afterward.
3. Searched KSP `saves/Main` for a launch-state save that matches the separatron craft (pad mass 521 003 kg, 8 launch
   clamps). No save was written between 01:40 and now, so none is confirmed. Pad saves present are older campaign saves
   (`ksl-autonomous-20261012-pad-*`, `quicksave`) and were not loaded, because loading an unverified pad save could fly
   a different craft.

What is needed to continue:

- An in-game **Revert to Launch** by the operator (KSP GUI), or the name of a save that contains the current
  separatron craft at the pad. Then attempt 4 can start with `bin/ksl run -p sts-85km-10deg`.
- The deorbit refusal from attempts 2–3 is a planner gate, not a pad problem. Repeating full flights would test
  ascent repeatability only, unless the gate or its inputs change, and those changes are out of scope for this goal.

Follow-up scan (read-only, same session): counted parts and PRELAUNCH entries in the pad-tagged saves in
`saves/Main`. The 155-part candidates (`ksl-autonomous-20261012-pad-*`) are older campaign saves from 2026-10-12,
all predating the separatron change. None is confirmed to match the separatron craft at 521 003 kg. Loading one would
fly a different craft under the same profile, so none was loaded.

Third revert attempt (same session): `revert_to_launch()` with a 5 s wait, reading the active vessel before and after. Result: still `orbiting`, mass 26 015 kg, UT 642. kRPC reports success but the revert has no effect in this game state.

Current KSP state at the time of this entry: scene `flight`, situation `orbiting`, paused, vessel `STS`
(vesselId 9201), altitude 85.0 km, mass 26 014.6 kg. The 3-attempt campaign summary above stands.



### Blocker update — computer-use access to KSP denied (2026-10-14 ~02:10 KST)

- Requested computer-use access to KSP to perform an in-game Revert to Launch. The first request used the name
  "Kerbal Space Program" and was reported as not installed (index lookup); the retry as "KSP" was **denied by the user**.
- No GUI action was taken. Vessel state re-checked read-only with `bin/ksl status --live`: unchanged (`orbiting`,
  paused, UT 641.6 s, mass 26 014.6 kg, vesselId 9201, launchClamps 0).
- Attempt 4 remains blocked. The remaining routes need the operator: an in-game Revert to Launch, or a save name that
  holds the separatron craft at the pad. Nothing was loaded, reverted, or changed.

### Blocker check — ksl CLI options for a pad reset (2026-10-14 ~02:20 KST)

Read-only review of `bin/ksl` for a path back to the pad. No flight, revert, load, or save was run.

- `ksl operate` lists `preflight` and `launch`, but both require a stationary stack on the launch clamps. The vessel is
  in orbit with 0 clamps, so neither applies.
- `ksl run --load-save NAME` and `ksl continue --use-save` load a named save. The `ksl checkpoints` list holds only
  orbit or mm305 saves from this vessel's orbit states. No checkpoint is at the pad.
- Pad-tagged saves in `saves/Main` (`ksl-autonomous-20261012-pad-*`) predate the separatron change. Attempt 1 failed on
  that craft at booster separation, so flying one of those now would test the old craft, not the separatron craft.
  Not loaded.

Conclusion: attempt 4 still needs an operator-side Revert to Launch, or a save that holds the current separatron craft
at the pad. Nothing else in the CLI or the checkpoint list can reset the pad.

- 2026-10-14 02:59 KST — scheduled check: no change. `ksl status --live` shows orbiting, mass 26014.6 kg, 0 clamps, UT 641.6 s. Attempt 4 still blocked.

- 2026-10-14 03:29 KST — scheduled check: no change. `ksl status --live` shows orbiting, mass 26014.6 kg, 0 clamps, UT 641.6 s. Attempt 4 still blocked.

- 2026-10-14 04:00 KST — scheduled check: no change. `ksl status --live` shows orbiting, mass 26014.6 kg, 0 clamps, UT 641.6 s. Attempt 4 still blocked.

- 2026-10-14 04:31 KST — scheduled check: no change. `ksl status --live` shows orbiting, mass 26014.6 kg, 0 clamps, UT 641.6 s. Attempt 4 still blocked.

- 2026-10-14 05:02 KST — scheduled check: no change. `ksl status --live` shows orbiting, mass 26014.6 kg, 0 clamps, UT 641.6 s. Attempt 4 still blocked.

- 2026-10-14 05:33 KST — scheduled check: no change. `ksl status --live` shows orbiting, mass 26014.6 kg, 0 clamps, UT 641.6 s. Attempt 4 still blocked.
