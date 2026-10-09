# Live provisional MM305 control latency

The orbit86 live flight transferred from MM304 to provisional TAEM at UT 14698.9, altitude 24.4 km, range 96.5 km, TAS 718.6 m/s. This was provisional route tracking, not a replay-qualified route commitment or a completed landing.

At vehicle keyframe UT 14712.93, guidance compute time was 11079.091 ms and the gap since the preceding control tick was 16510.126 ms. The vehicle was descending at 106.18 m/s. Thus many seconds of uncontrolled evolution can occur between refreshed actuator commands at precisely the terminal acquisition boundary. The new OMS/window changes were not in this active pinned executable.

The live acquisition selector in `guidance_taem.c` ran both `terminal_solver_generate_profile` and `taem_fixed_hac_evaluate_route` over its shortlist on the control thread. The existing asynchronous planner separately starts by evaluating the exact provisional seed route before trying broader HAC candidates. Repeating native replays on the control thread does not justify losing attitude-control cadence.

The revised live selector ranks the actual constructed routes using geometry and energy estimates while the background planner performs native replay. It labels this classification invalid/unqualified with replay pending. Offline diagnostic shadows retain the full profile and replay evaluations. No gate, tolerance, route commitment condition, or Final admission requirement is relaxed. This change is compiled into `/tmp/sts-oms-window-backend`, not injected into the ongoing flight. A live timing comparison still needs to establish the achieved control cadence.

Validation: production build succeeds. Existing `MM305AdmissionTests.c` and `TAEMProductionIntegrationContractTests.c` pass against the current objects. The Makefile's `mm305-admission-test` target failed to link because it omits the tracker object required by `taem_planner.o`; linking the same existing test against the complete current object set passes. The unrelated dirty Makefile was preserved.

The active old-controller flight later entered Attitude Recovery for saturation/attitude error. At UT 14826.3 it remained active at altitude 8.1 km, range 50.0 km, TAS 175.9 m/s, with the warning that recovery exceeded 15 s without stabilizing. This provides no qualified MM305 route, touchdown, or rollout evidence.
