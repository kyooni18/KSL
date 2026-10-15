"""Keep controller decisions intact between mission logging and the web observer."""
import json

from ksl.backend import snapshot_summary
from Tools.telemetry_web import apply_ksl_run_overlay


def test_controller_state_round_trip(tmp_path):
    command = {"targetPitch": -12.0, "targetAoA": None, "targetThrottle": .18,
               "autopilotEngaged": True, "useInertialDirection": True,
               "controlProfile": "orbital"}
    guidance = {"deliveredDeltaV": 23.5, "burnStarted": True, "burnCompleted": False}
    record = snapshot_summary({"phase": "Deorbit Burn", "telemetry": {
        "ut": 100., "flightPathAngle": 2., "autopilotError": 4.},
        "command": command, "guidanceState": guidance,
        "statusMessage": "Tracking the burn target", "warningMessage": None})
    record["source"] = "landing"
    run = tmp_path / "runs" / "test"
    run.mkdir(parents=True)
    (run / "telemetry.jsonl").write_text(json.dumps(record) + "\n")
    snapshot = {"telemetry": {"ut": 100., "throttle": 0.}, "command": {}}
    apply_ksl_run_overlay(snapshot, tmp_path)
    assert snapshot["command"]["targetPitch"] == -12.
    assert snapshot["command"]["targetThrottle"] == .18
    assert snapshot["command"]["useInertialDirection"] is True
    assert snapshot["statusMessage"] == "Tracking the burn target"
    assert snapshot["warningMessage"] is None
    assert snapshot["guidanceState"] == guidance
    assert snapshot["telemetry"]["autopilotError"] == 4.
    # Warp advances the observer without a new backend record. Do not reuse
    # an old command/alignment measurement as if sampled at the new UT.
    snapshot = {"telemetry": {"ut": 200.}, "command": {}}
    apply_ksl_run_overlay(snapshot, tmp_path)
    assert "guidanceState" not in snapshot
    assert "autopilotError" not in snapshot["telemetry"]
    assert "useInertialDirection" not in snapshot["command"]
