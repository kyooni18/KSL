# Restart after the human's KSP mod reboot

The human rebooted KSP to apply a mod and explicitly requested atmospheric-control fixes and a fresh attempt from the 86 km checkpoint already loaded. The previous old-controller attempt ended when its kRPC connection closed; backend exit -13 and cleanup connection refusal are terminal evidence. Its last logged state was recovery at 7.5 km, range 45 km, TAS 78.4 m/s. It did not validate a KSC landing or rollout.

Read-only verification found original STS at UT 82.86, altitude 86745.91 m, Pe 86745.90 m, Ap 86750.89 m, inclination 2.41e-10 radians, paused, throttle zero. The new runner uses `--no-quickload` to preserve the human-loaded scene. Commit `6ec048c` gives planning ownership of the already-loaded orbital pause and releases it after planning.

The active backend includes `c99fdb3`: expensive provisional terminal profile/replay work is off the live control thread; asynchronous native route qualification and all Final admission conditions remain. Its atmospheric control improvement still needs a live timing/tracking comparison. OMS-only selection, measured-axis orientation, normal burn steering, and every-tick burn telemetry are included.

Runner session 23559, backend PID 14275. Console: `/tmp/sts-live-oms-atmosphere-20261009.log`. Run manifest: `Runtime/Headless/run-manifests/20261009T072241652873Z-14275.json`. Backend SHA-256: `c00f0c6999a5634020781a82c7cefaafd3708baa7b345ceaf931d8e2bdf1a814`.

The new planner selected burn UT 1122.1 and delta-v 31.54 m/s, approximately 1039 s from the loaded state, rather than several orbital revolutions. It reported nominal capture and 13/15 recovery cases passing; these are predictions, not demonstrated touchdown or rollout.

Inventory measured OMS-only resultant thrust 59196 N, body axis -0.0000/0.9380/0.3467 and moment -19684/0/0 Nm. An independent read during the actual burn at UT 1091.19 verified global throttle zero, all three SSMEs at zero thrust, and both OMS engines at 5397.81 N each under independent throttle. Thus engine isolation is now live observed. Cutoff and atmospheric handling remain to be verified.

Protected `86km.sfs` hash after starting this attempt remains `4ea194a07422eece54135f2f7586dc92b556b2c9380b785c4754ef868eb662a6`. It was not overwritten.

The OMS burn shut down at UT 1201.0 and the controller reported post-burn strict verification passed. Independent live telemetry at UT 1232.97 verified achieved periapsis 51435.95 m, altitude 85440.16 m, global throttle zero, total thrust zero, and zero thrust on all five engines. Source: `reboot-oms-cutoff.json`. This is a live OMS-only burn/cutoff observation, substantially closer to the planned 50 km periapsis than the old all-engine attempt's 43.1 km, but not yet atmospheric control or landing validation.
