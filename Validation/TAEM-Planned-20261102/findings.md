# TAEM transition and planned descent checkpoint — 2026-11-02

## Observed cause
The archived live run `runs/20261102T040424-mm304-shooting-live-72km` used source 12d39ba. MM304 handed over at 738.28 m/s, 26,456.64 m ASL, runway along -35,631.49 m and cross +978.69 m. TAEM subsequently forced Final at 15,611.55 m ASL and 405.87 m/s because the mission-controller branch treated crossing the NEP horizontal station as sufficient. The ordinary Final capture predicate was bypassed.

## Implemented changes
- Preserve normal qualified Final capture; unqualified last-chance takeover now requires descending below the configured Final inlet height. A missing NEP at altitude retains best-glide recovery ownership.
- Commit a measured-height/slope-to-NEP Hermite descent reference at acquisition/downmode, latch the join, and report geometric fit separately from energy sufficiency.
- Preserve overhead winding using measured angular increments. Nearest-position fitting on a nearly circular spiral previously discarded a full winding and jumped the reference.
- Scale pursuit by time to the join and radial correction by the turn timescale; allocate lateral lift after vertical support.
- Separate the optional TAEM glide constraint from Final's outer-glide slope. Existing pressure, load, bank, geometry and actuator limits remain.
- Keep EN/EMIN explicitly advisory: they price their own q schedule, not the new committed profile. This remains an important limitation.

## Validation and limitations
Three focused regressions pass: altitude-based recovery, winding continuity, and Hermite slope/end-point consistency. These are component checks, not proof of end-to-end capture. The archived scratch harness also checks the recorded initial target height and runs a diagnostic continuation without asserting landing success.

Latest diagnostic continuation (`continuation-final.txt`, `continuation.json`) FAILS terminal delivery: at 386.9 seconds it reaches unqualified Final recovery at height 961.24 m, along -5,494.67 m, cross -3,248.08 m, airspeed 41.10 m/s. It is not a landing. At 40.1 seconds speed is 741.52 m/s and height 17,309.93 m; at 80.1 seconds speed is 136.99 m/s and height 8,302.68 m. Thereafter incidence rises to roughly 22–26 degrees while speed decays. This is evidence for investigating the early energy loss and support-q/maximum-CL schedule, not permission to tune a speed threshold.

The continuation seeds recorded applied controls and warms the flight controller from preceding guidance history. It uses the configured offline plant; it is not a matched-command validation of that plant against live flight. Archived scratch source preserves the exact reproduction setup. Earlier diagnostic continuations are retained; none proved landing success.

No new live attempt was started or commanded. The existing live process uses its own archived binary. Unrelated user changes and pre-existing scratch tests were preserved. The newly created scratch test was archived here and removed from the test target.
