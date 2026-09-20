#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile

import HACLogAnalysis
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def text(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def replay_snapshot(ut: float, error: float, pitch: float, roll: float, yaw: float, throttle: float) -> dict:
    return {
        "recordType": "snapshot",
        "automationEngaged": True,
        "phase": "Coast to Burn" if throttle <= 0.0 else "Deorbit Burn",
        "telemetry": {
            "ut": ut,
            "autopilotError": error,
            "controlPitch": pitch,
            "controlRoll": roll,
            "controlYaw": yaw,
            "bodyPitchRate": 0.0,
            "bodyRollRate": 0.0,
            "bodyYawRate": 0.0,
        },
        "command": {
            "controlProfile": "orbital",
            "useInertialDirection": True,
            "targetThrottle": throttle,
        },
    }


def terminal_snapshot(ut: float, altitude: float, speed: float, energy: float, exec_phase: str) -> dict:
    return {
        "recordType": "snapshot",
        "automationEngaged": True,
        "phase": "TAEM",
        "telemetry": {"ut": ut, "meanAltitude": altitude, "trueAirSpeed": speed, "energyExcessRange": energy},
        "command": {"airbrakes": False},
        "guidanceState": {"taemExecutive": {"phase": exec_phase}},
    }


def assert_replay_detector_contract() -> None:
    bad = [
        replay_snapshot(0.00, 90.0, -1.0, -1.0, 0.0, 0.0),
        replay_snapshot(0.25, 75.0, -1.0, -1.0, 0.0, 0.0),
        replay_snapshot(0.50, 60.0, -1.0, -1.0, 0.0, 0.0),
        replay_snapshot(0.75, 42.0, -1.0, -1.0, 0.0, 0.0),
        replay_snapshot(1.00, 25.0, -1.0, -1.0, 0.0, 0.0),
        replay_snapshot(1.25, 9.0, -0.4, -0.3, 0.0, 0.2),
    ]
    clean = [
        replay_snapshot(0.00, 90.0, -1.0, -0.4, 0.0, 0.0),
        replay_snapshot(0.25, 70.0, -0.8, -0.5, 0.0, 0.0),
        replay_snapshot(0.50, 48.0, -0.6, -0.4, 0.0, 0.0),
        replay_snapshot(0.75, 28.0, -0.4, -0.3, 0.0, 0.0),
        replay_snapshot(1.00, 9.0, -0.2, -0.1, 0.0, 0.2),
    ]
    stalled = [
        replay_snapshot(ut, 90.0 - ut, 0.0, 0.0, 0.0, 0.0)
        for ut in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0)
    ]
    terminal_bad = [
        terminal_snapshot(0.0, 28548.0, 1299.0, 101091.0, "Acquisition"),
        terminal_snapshot(12.0, 18000.0, 1000.0, 40000.0, "Acquisition"),
        terminal_snapshot(36.0, 15000.0, 850.0, 20000.0, "Acquisition"),
    ]
    for row in terminal_bad[1:]:
        row["guidanceState"].update({"hacSideSelected": True, "hacTransitionActive": True, "hacTransitionProgress": 0.0})
    terminal_clean = [
        terminal_snapshot(0.0, 28548.0, 1299.0, 101091.0, "S-turn"),
        terminal_snapshot(12.0, 17500.0, 1100.0, 25000.0, "Acquisition"),
        terminal_snapshot(24.0, 15000.0, 900.0, 10000.0, "Prefinal"),
    ]
    terminal_clean[1]["guidanceState"].update({"hacSideSelected": True, "hacTransitionActive": True, "hacTransitionProgress": 0.2})
    terminal_clean[2]["guidanceState"].update({"hacSideSelected": True, "hacTransitionActive": True, "hacTransitionProgress": 0.8, "hacCaptured": True})
    with tempfile.TemporaryDirectory() as directory:
        bad_path = Path(directory) / "bad.jsonl"
        clean_path = Path(directory) / "clean.jsonl"
        stalled_path = Path(directory) / "stalled.jsonl"
        terminal_bad_path = Path(directory) / "terminal-bad.jsonl"
        terminal_clean_path = Path(directory) / "terminal-clean.jsonl"
        bad_path.write_text("".join(json.dumps(row) + "\n" for row in bad), encoding="utf-8")
        clean_path.write_text("".join(json.dumps(row) + "\n" for row in clean), encoding="utf-8")
        stalled_path.write_text("".join(json.dumps(row) + "\n" for row in stalled), encoding="utf-8")
        terminal_bad_path.write_text("".join(json.dumps(row) + "\n" for row in terminal_bad), encoding="utf-8")
        terminal_clean_path.write_text("".join(json.dumps(row) + "\n" for row in terminal_clean), encoding="utf-8")
        bad_failures = HACLogAnalysis.orbital_capture_failures(
            HACLogAnalysis.orbital_capture_metrics(bad_path))
        clean_failures = HACLogAnalysis.orbital_capture_failures(
            HACLogAnalysis.orbital_capture_metrics(clean_path))
        stalled_failures = HACLogAnalysis.orbital_capture_failures(
            HACLogAnalysis.orbital_capture_metrics(stalled_path))
        terminal_bad_failures = HACLogAnalysis.terminal_transition_failures(HACLogAnalysis.terminal_transition_metrics(terminal_bad_path))
        terminal_clean_failures = HACLogAnalysis.terminal_transition_failures(HACLogAnalysis.terminal_transition_metrics(terminal_clean_path))
    assert any("multi-axis" in failure for failure in bad_failures), bad_failures
    assert clean_failures == [], clean_failures
    assert any("zero-actuation" in failure for failure in stalled_failures), stalled_failures
    assert any("S-turn ownership" in failure for failure in terminal_bad_failures), terminal_bad_failures
    assert any("never exceeded 5%" in failure for failure in terminal_bad_failures), terminal_bad_failures
    assert terminal_clean_failures == [], terminal_clean_failures


def main() -> int:
    config = json.loads(text("Configuration/default.json"))
    connection = config["connection"]
    assert set(connection) == {"serialPort", "baudRate", "timeoutMs", "clientName"}
    assert connection["serialPort"].startswith("/")
    assert connection["baudRate"] >= 9600
    assert connection["timeoutMs"] >= 50

    live_config = json.loads(text("Configuration/live-cnano.json"))
    live_site = live_config["site"]
    assert live_site["name"] == "KSC Runway 09"
    assert live_site["runwayHeading"] == 90
    assert live_site.get("allowReciprocalRunway") is False, \
        "live campaign must stay locked to KSC Runway 09; reciprocal Runway 27 is not an acceptance target"

    landing = text("CLanding/landing.h")
    models = text("CLanding/models.c")
    runtime = text("CLanding/krpc.c")
    client = text("CLanding/krpc_cnano_client.c")
    transport = text("CLanding/krpc_cnano_transport.c")
    makefile = text("CLanding/Makefile")
    controller = text("CLanding/controller.c")
    guidance = text("CLanding/guidance.c")
    predictor = text("CLanding/predictor.c")

    production = "\n".join([landing, models, runtime, client, transport, controller])
    forbidden = [
        "PythonBridge/",
        "krpc_bridge.py",
        "KSP_LANDER_BRIDGE_SCRIPT",
        "KSP_LANDER_PYTHON",
        "rpc_port",
        "stream_port",
        '"rpcPort"',
        '"streamPort"',
        "fork(",
        "execl(",
    ]
    for marker in forbidden:
        assert marker not in production, f"retired runtime dependency remains: {marker}"

    assert "krpc_cnano_client_open" in runtime
    assert "flight_control_step" in runtime
    assert "physics_store_open" in runtime
    assert "krpc_cnano_posix_serial_ops" in client
    assert "KRPC_COMMUNICATION_CUSTOM" in makefile
    assert "krpc-cnano-0.6.0" in makefile
    assert "nanopb-0.4.9.1" in makefile
    assert "krpc_cnano_batch.c" in makefile
    assert "CNanoClientTests.c" in makefile
    assert "OfflineLandingGateTests.c" in makefile
    assert "offline-acceptance" in makefile
    assert "krpc_SpaceCenter_AutoPilot_SetDirectionAndUp" in client
    assert "command->control_profile == PROFILE_ORBITAL" in client
    assert "krpc_SpaceCenter_Control_set_SAS" in client
    assert "krpc_SpaceCenter_Control_set_RCS" in client
    assert "body_nonrotating" in client
    assert "krpc_orbital_up_reference(command,&s->last_state,&radial_up)" in runtime
    assert "up_reference=&radial_up" in runtime
    assert "krpc_cnano_client_set_autopilot(s->client,command,up_reference" in runtime
    assert "vdot(command->inertial_direction,s->last_state.velocity)>0.0" not in runtime
    assert "flight_control.c" in makefile
    assert "physics_store.c" in makefile
    assert "-lsqlite3" in makefile

    dynamic_start = controller.index("static void dynamic_prediction")
    dynamic_end = controller.index("static bool terminal_prediction_plan_copy", dynamic_start)
    dynamic_prediction = controller[dynamic_start:dynamic_end]
    worker_start = controller.index("static void *terminal_prediction_worker")
    worker_end = controller.index("static void start_prediction_worker", worker_start)
    terminal_worker = controller[worker_start:worker_end]
    assert "predictor_simulate_terminal_shadow_ensemble" not in dynamic_prediction
    assert controller.count("predictor_simulate_terminal_shadow_ensemble") == 1
    assert "predictor_simulate_terminal_shadow_ensemble" in terminal_worker
    ensemble_at = terminal_worker.index("predictor_simulate_terminal_shadow_ensemble")
    assert terminal_worker.rfind("pthread_mutex_unlock(&c->mutex);", 0, ensemble_at) >= 0
    assert "taem_exec_owns_vehicle(&c->guidance.taem_exec)" in terminal_worker
    assert "terminal_prediction_commit(c,&pr,request_ut,sign)" in terminal_worker
    assert "last_terminal_prediction_completion_ut" in terminal_worker
    assert "cadence_anchor=fmax(c->last_terminal_prediction_request_ut,c->last_terminal_prediction_completion_ut)" in terminal_worker
    completion_lock = terminal_worker.rfind("pthread_mutex_lock(&c->mutex);", 0, terminal_worker.index("terminal_prediction_commit"))
    completion_stamp = terminal_worker.index("last_terminal_prediction_completion_ut", ensemble_at)
    assert completion_lock >= 0 and completion_stamp > completion_lock
    assert "latest_telemetry.ut>=request_ut-1.0" in terminal_worker
    assert "start_prediction_worker(c);" in controller
    assert "stop_prediction_worker(c);" in controller
    assert "async_prediction_request_due" in terminal_worker
    assert "async_prediction_result_fresh" in terminal_worker
    assert "async_prediction_policy.c" in makefile
    assert "AsyncPredictionPolicyTests.c" in makefile
    stop_start = controller.index("static void stop_prediction_worker(LandingController *c){")
    stop_end = controller.index("static const char* achieved_state_warning", stop_start)
    stop_worker = controller[stop_start:stop_end]
    assert stop_worker.index("krpc_safe(c->session)") < stop_worker.index("pthread_join")

    # MM304 consumes the pre-guidance forecast, but a plan accepted by guidance on that
    # same tick must never be serialized with geometry produced for its predecessor.
    # Do not fix this by doing a second synchronous atmospheric solve: recorded entry
    # transition ticks already consume hundreds of milliseconds. Invalidate the cadence,
    # suppress mismatched planner geometry, and let the next control tick refresh it.
    tick_start = controller.index("static void tick_locked")
    tick_end = controller.index("failure:", tick_start)
    tick = controller[tick_start:tick_end]
    preforecast_at = tick.index("dynamic_prediction(c,&t,&state,&cfg)")
    guidance_at = tick.index("GuidanceResult r=using_glide?glide:guidance_update")
    invalidate_at = tick.index("!entry_forecast_matches_guidance(c)")
    snapshot_at = tick.index("c->snapshot.guidance_entry_plan_valid")
    assert preforecast_at < guidance_at < invalidate_at < snapshot_at
    assert tick.count("dynamic_prediction(c,&t,&state,&cfg)") == 1
    assert "c->last_prediction_ut=-1e300" in tick[guidance_at:snapshot_at]
    assert "trajectory_calibrator_set_forecast(&c->trajectory_calibrator,&no_forecast)" in tick[guidance_at:snapshot_at]
    assert "forecast_entry_plan_identity_valid" in controller
    assert "p.plan_id=g->entry_s_turn_plan.plan_id" in controller
    assert ".raw_prediction=prediction_matches_plan?&c->dynamic_trajectory:NULL" in controller
    assert ".published_prediction=prediction_matches_plan?&c->stabilized_trajectory:NULL" in controller
    assert ".predictor_discontinuity=!prediction_matches_plan" in controller
    match_fn = controller[controller.index("static bool entry_forecast_matches_guidance"):controller.index("/* All read paths publish", controller.index("static bool entry_forecast_matches_guidance"))]
    assert "if(c->forecast_entry_plan_identity_valid)" in match_fn
    assert "if(c->guidance.phase!=PHASE_ENTRY_ENERGY)return false" in match_fn
    assert "prediction_lineage_discontinuity" in controller
    assert "c->prediction_lineage_discontinuity=false" in tick
    assert "c->prediction_lineage_discontinuity=true" in tick[guidance_at:snapshot_at]
    assert "c->forecast_entry_plan_identity_valid=false" in tick[guidance_at:snapshot_at]
    hold_path = controller[controller.index("static bool replace_predicted_trajectory"):controller.index("static bool trajectory_sample_at")]
    assert "trajectory_clear(&c->snapshot.predicted_trajectory);return true" in hold_path

    # Reentry-only qualification must roll forward the same live guidance policy it
    # will engage, not the older free predictor policy.  The shared initializer is
    # used once for the isolated qualification shadow and again for the accepted
    # live controller state; the shadow itself must call guidance_update and stop on
    # the real TAEM ownership latch/boundary semantics.
    reentry_start = controller.index("void landing_controller_engage_reentry")
    reentry_end = controller.index("void landing_controller_engage_hac_test", reentry_start)
    reentry = controller[reentry_start:reentry_end]
    assert "predictor_simulate_entry_guidance_shadow" in reentry
    assert "guidance_initialize_reentry_continuation(&qualification_guidance" in reentry
    assert "guidance_initialize_reentry_continuation(&c->guidance" in reentry
    assert reentry.count("guidance_initialize_reentry_continuation") == 2
    assert "guidance_machine_init(&c->guidance)" not in reentry
    assert "if(!late_terminal_test)" in reentry
    assert "reentry_guidance_shadow_recovery_qualified" in reentry
    assert "predictor_simulate_entry_with_attitude" in reentry  # explicit late-terminal test override retains the legacy fallback only
    shadow_start = predictor.index("static EntryPrediction predictor_simulate_guidance_shadow_core")
    shadow_end = predictor.index("EntryPrediction predictor_simulate_entry_guidance_shadow", shadow_start)
    entry_shadow = predictor[shadow_start:shadow_end]
    assert "guidance_update(&shadow" in entry_shadow
    assert "taem_exec_owns_vehicle(&shadow.taem_exec)" in entry_shadow
    assert "entry_taem_handoff_geometry_ready" in entry_shadow
    assert "out.taem_ownership_boundary_missed" in entry_shadow
    assert "r.command.use_inertial_direction" in entry_shadow
    # A latch may occur inside an Entry-phase guidance call. The shadow must retain
    # one reduced tick until the public TAEM phase is observed, then stop; boundary
    # miss still stops immediately.
    assert "terminal_public_phase=out.reached_taem&&r.phase==PHASE_TAEM" in entry_shadow
    assert "stop_at_taem&&(out.taem_ownership_boundary_missed||terminal_public_phase)" in entry_shadow
    # Bank/AoA predictor transients are controlled-coordinate derivatives. Body
    # p/q remain a separate rigid-body/actuator-axis signal and must never be
    # substituted into these state derivatives after the 17:57 coupled departure.
    dynamic_rate_seed = dynamic_prediction[dynamic_prediction.index("double initial_roll_rate"):dynamic_prediction.index("double leg_elapsed")]
    assert "t->roll_rate" in dynamic_rate_seed
    assert "t->angle_of_attack_rate" in dynamic_rate_seed
    assert "body_roll_rate" not in dynamic_rate_seed
    assert "body_pitch_rate" not in dynamic_rate_seed
    assert "double roll_rate=controlled_roll_rate(t);" in guidance
    assert "double aoa_rate=controlled_aoa_rate(t);" in guidance
    assert ".current_aoa_rate=controlled_aoa_rate(t)" in guidance
    roll_helper = guidance[guidance.index("static double controlled_roll_rate"):guidance.index("static double controlled_aoa_rate")]
    aoa_helper = guidance[guidance.index("static double controlled_aoa_rate"):guidance.index("static double hac_response_lead_time")]
    assert "body_roll_rate" not in roll_helper
    assert "body_pitch_rate" not in aoa_helper
    assert "pitch_rate" not in aoa_helper
    shadow_seed = predictor[predictor.index("response.actual_bank=norm_signed_deg(telemetry->roll)"):predictor.index("response.limiter.has_value", predictor.index("response.actual_bank=norm_signed_deg(telemetry->roll)"))]
    assert "telemetry->roll_rate" in shadow_seed
    assert "telemetry->angle_of_attack_rate" in shadow_seed
    assert "body_roll_rate" not in shadow_seed
    assert "body_pitch_rate" not in shadow_seed
    assert "t.has_angle_of_attack_rate=true" in predictor
    assert "t.has_body_pitch_rate=false;t.has_body_roll_rate=false;t.has_body_yaw_rate=false" in predictor
    assert "previous_angle_of_attack" in client
    assert "angle_of_attack_rate_filter" in client
    assert "t->has_angle_of_attack_rate=true" in client

    # Applied-control diagnostics preserve the same frame distinction: the
    # coordinate damping rates are separate from raw body p/q/r.
    krpc = text("CLanding/krpc.c")
    models = text("CLanding/models.c")
    direct = krpc[krpc.index("static void fill_direct_result"):krpc.index("static void fill_autopilot_result")]
    assert "control_aoa_rate=fc->diagnostics.effective_pitch_rate" in direct
    assert "control_roll_rate=fc->diagnostics.effective_roll_rate" in direct
    assert "control_body_pitch_rate=fc->diagnostics.body_pitch_rate" in direct
    assert "control_body_roll_rate=fc->diagnostics.body_roll_rate" in direct
    assert "control_body_yaw_rate=fc->diagnostics.body_yaw_rate" in direct
    assert "control_body_pitch_rate=fc->diagnostics.effective_pitch_rate" not in direct
    assert "control_body_roll_rate=fc->diagnostics.effective_roll_rate" not in direct
    assert 'KEY(w,"angleOfAttackRate")' in models
    assert 'KEY(w,"rollRate");jw_number(w,a->control_roll_rate)' in models
    assert 'KEY(w,"bodyRollRateAvailable")' in models
    assert 'KEY(w,"bodyYawRate")' in models

    assert (ROOT / "ThirdParty/krpc-cnano-0.6.0/VERSION.txt").read_text().strip() == "0.6.0"
    assert (ROOT / "ThirdParty/nanopb-0.4.9.1/pb.h").exists()
    options = text("ThirdParty/krpc-cnano-0.6.0/protobuf/krpc.options")
    generated = text("ThirdParty/krpc-cnano-0.6.0/include/krpc_cnano/krpc.pb.h")
    assert "Request.calls          max_count:32" in options
    assert "Response.results       max_count:32" in options
    assert "krpc_schema_ProcedureCall calls[32]" in generated
    assert "krpc_schema_ProcedureResult results[32]" in generated
    assert (ROOT / "ThirdParty/PROVENANCE.md").exists()
    assert not (ROOT / "PythonBridge").exists(), "retired PythonBridge directory still exists"
    assert not (ROOT / "install_python_bridge.command").exists(), "retired bridge installer still exists"

    ui = text("PyQtApp/window.py")
    assert "connection.serialPort" in ui
    assert "connection.baudRate" in ui
    assert "connection.timeoutMs" in ui
    assert "connection.rpcPort" not in ui
    assert "connection.streamPort" not in ui

    # ShuttleSim's fitted atmosphere is denser than the live 500 m kRPC
    # sample table. The fixed-size PlanetModel buffer must never silently truncate
    # it, because the predictor clamps all higher-altitude lookups to the last
    # stored cell and can turn 69 km entry into 37 km-density air.
    landing_h = text("CLanding/landing.h")
    sample_define = next(
        line for line in landing_h.splitlines()
        if line.startswith("#define LANDER_ATMOSPHERE_SAMPLE_MAX ")
    )
    atmosphere_capacity = int(sample_define.split()[-1])
    atmosphere_rows = sum(
        1 for line in text("ShuttleSim/data/fitted/kerbin_atmosphere_ksp.csv").splitlines()[1:]
        if line.strip()
    )
    assert atmosphere_capacity >= atmosphere_rows, (
        f"PlanetModel atmosphere capacity {atmosphere_capacity} truncates "
        f"ShuttleSim's {atmosphere_rows}-sample fitted atmosphere"
    )

    assert_replay_detector_contract()

    print("Native C-Nano architecture tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
