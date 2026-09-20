#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("postflight_acceptance", ROOT / "Tools/postflight_acceptance.py")
assert SPEC and SPEC.loader
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


def write_jsonl(path: Path, rows) -> None:
    path.write_text("".join(json.dumps(row, separators=(",", ":")) + "\n" for row in rows), encoding="utf-8")


def test_synthetic_runtime_fields_and_delta_reconstruction() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        vehicle = root / "synthetic-vehicle.jsonl"
        planner = root / "synthetic-planner.jsonl"
        session = "synthetic-session"
        base_fields = {
            "state": {"phase": "TAEM", "vesselSituation": "flying", "entryReversalScheduled": False},
            "position": {"altitude": 16500.0},
            "motion": {"trueAirSpeed": 1250.0},
            "guidance": {"rangeToSite": 70000.0, "runwayCrossTrack": 1000.0},
            "runtime": {
                "loopWallDeltaMilliseconds": 100.0,
                "controlLoopMilliseconds": 42.0,
                "telemetryLatencyMilliseconds": 15.0,
                "guidanceComputeMilliseconds": 18.0,
                "applyLatencyMilliseconds": 9.0,
            },
        }
        write_jsonl(vehicle, [
            {"recordType": "sessionStart", "sessionId": session, "runtime": {"nativeBuild": "test"}},
            {"recordType": "vehicleKeyframe", "sessionId": session, "recordSequence": 1, "tickSequence": 10,
             "ut": 100.0, "wallMonotonicSeconds": 10.0, "fields": base_fields},
            {"recordType": "vehicleDelta", "sessionId": session, "recordSequence": 2, "tickSequence": 11,
             "ut": 100.1, "wallMonotonicSeconds": 10.1,
             "fields": {"runtime": {"loopWallDeltaMilliseconds": 101.0, "guidanceComputeMilliseconds": 19.0}}},
            {"recordType": "vehicleDelta", "sessionId": session, "recordSequence": 3, "tickSequence": 12,
             "ut": 100.2, "wallMonotonicSeconds": 10.2,
             "fields": {"state": {"vesselSituation": "landed"}}},
        ])
        write_jsonl(planner, [
            {"recordType": "sessionStart", "sessionId": session},
            {"recordType": "plannerSample", "sessionId": session, "ut": 100.0, "phase": "TAEM",
             "lineage": {"planId": 1, "version": 1},
             "terminalGeometry": {"terminalCandidateValid": True, "terminalCommitted": True,
                                  "hacTransitionProgress": .5, "hacCircuitCount": 0, "hacRemaining": 12000.0}},
        ])
        result = MOD.analyze(vehicle)
        assert result["sessionPairValid"]
        assert result["handoff"]["altitudeIn15To18KmCorridor"]
        assert result["handoff"]["atOrBelow1300Mps"]
        assert not result["handoff"]["nearSiteLargeCrossTrack"]
        assert result["runtime"]["source"] == "runtimeFields"
        assert result["runtime"]["taem"]["loopWallDeltaMilliseconds"]["count"] == 2
        assert result["runtime"]["postHandoffLoopGapsOver2Seconds"] == 0
        assert result["terminalPlanner"]["firstTerminalCommittedUT"] == 100.0
        assert result["outcome"]["touchdownDetected"]
        assert result["acceptance"]["touchdownDetected"]
        assert "runtimeFields" in MOD.human_report(result)


def test_truncated_tail_recovery_preserves_last_keyframe_and_pairing() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        vehicle = root / "truncated-vehicle.jsonl"
        planner = root / "truncated-planner.jsonl"
        session = "truncated-session"
        write_jsonl(vehicle, [
            {"recordType": "sessionStart", "sessionId": session, "runtime": {"nativeBuild": "test"}},
            {"recordType": "vehicleKeyframe", "sessionId": session, "recordSequence": 1, "tickSequence": 20,
             "ut": 200.0, "wallMonotonicSeconds": 20.0,
             "fields": {"state": {"phase": "TAEM", "vesselSituation": "flying"},
                        "position": {"altitude": 16500.0},
                        "motion": {"trueAirSpeed": 1200.0},
                        "guidance": {"rangeToSite": 70000.0, "runwayCrossTrack": 500.0}}},
        ])
        write_jsonl(planner, [
            {"recordType": "sessionStart", "sessionId": session},
            {"recordType": "plannerSample", "sessionId": session, "tickSequence": 20, "ut": 200.0,
             "phase": "TAEM", "lineage": {"planId": 7, "version": 2}},
        ])
        with vehicle.open("a", encoding="utf-8") as handle:
            handle.write('{"recordType":"vehicleDelta","sessionId":"truncated-session"')
        with planner.open("a", encoding="utf-8") as handle:
            handle.write('{"recordType":"plannerSample","sessionId":"truncated-session"')

        loaded_vehicle = MOD.load_vehicle(vehicle)
        loaded_planner = MOD.load_planner(planner)
        assert loaded_vehicle["sessionId"] == session
        assert loaded_planner["sessionId"] == session
        assert len(loaded_vehicle["rows"]) == 1
        assert loaded_vehicle["rows"][0]["fields"]["position"]["altitude"] == 16500.0
        assert len(loaded_planner["rows"]) == 1
        assert MOD.analyze(vehicle)["sessionPairValid"]


def test_historic_1605_failure_signature() -> None:
    vehicle = ROOT / "FlightLogs/2026-09-11T16-05-21Z-STS-N-vehicle.jsonl"
    planner = ROOT / "FlightLogs/2026-09-11T16-05-21Z-STS-N-planner.jsonl"
    if not vehicle.exists() or not planner.exists():
        return
    result = MOD.analyze(vehicle)
    handoff = result["handoff"]
    assert result["sessionPairValid"]
    assert handoff["found"]
    assert abs(handoff["altitudeMeters"] - 23895.7736422049) < 1.0
    assert abs(handoff["trueAirSpeedMps"] - 1299.63354492188) < .1
    assert abs(handoff["rangeToSiteMeters"] - 8066.55410024219) < 1.0
    assert abs(handoff["runwayCrossTrackMeters"] - 8064.00862667604) < 1.0
    assert handoff["nearSiteLargeCrossTrack"]
    assert result["runtime"]["source"] == "inferredConsecutiveTickWallGaps"
    gaps = result["runtime"]["postHandoffLoopGapOver2SecondsValues"]
    assert len(gaps) == 3
    assert min(gaps) > 8800.0 and max(gaps) > 9400.0
    assert result["runtime"]["repeatedPostHandoffMultiSecondLoopGaps"]
    assert result["outcome"]["abortDetected"]
    assert "handoff-near-site-with-large-cross-track" in result["flags"]
    assert "repeated-post-handoff-loop-gaps-over-2s" in result["flags"]


def test_historic_1739_persistent_plan_safety_signature() -> None:
    vehicle = ROOT / "FlightLogs/2026-09-11T17-39-58Z-STS-N-vehicle.jsonl"
    planner = ROOT / "FlightLogs/2026-09-11T17-39-58Z-STS-N-planner.jsonl"
    if not vehicle.exists() or not planner.exists():
        return
    result = MOD.analyze(vehicle)
    quality = result["controlQuality"]
    assert quality["wrongSignedBetaYawFrames"] == 0
    assert quality["plannerStallAlphaSafetyViolations"] >= 200
    # Historic C-Nano logs predate the explicit provenance bit, but their
    # stallFraction came from the same conservative q/speed/AoA proxy. Keep the
    # @ALPHA diagnosis while refusing to relabel that proxy as measured stall.
    assert quality["plannerStallBankSafetyViolations"] == 0
    assert "planner-stall-alpha-envelope-violation" in result["flags"]
    assert "planner-stall-bank-envelope-violation" not in result["flags"]
    first = result["terminalPlanner"]["firstEntryProgram"]
    assert first["inputSign"] == 1.0
    assert first["mode"] == "passThrough"


def test_historic_1757_beta_yaw_departure_signature() -> None:
    vehicle = ROOT / "FlightLogs/2026-09-11T17-57-00Z-STS-N-vehicle.jsonl"
    planner = ROOT / "FlightLogs/2026-09-11T17-57-00Z-STS-N-planner.jsonl"
    if not vehicle.exists() or not planner.exists():
        return
    result = MOD.analyze(vehicle)
    quality = result["controlQuality"]
    assert quality["maximumAbsoluteSideslipDegrees"] > 60.0
    assert quality["wrongSignedBetaYawFrames"] >= 60
    assert quality["plannerStallAlphaSafetyViolations"] >= 80
    assert quality["plannerStallBankSafetyViolations"] == 0
    assert result["outcome"]["abortDetected"]
    assert "wrong-signed-beta-yaw-control" in result["flags"]
    first = result["terminalPlanner"]["firstEntryProgram"]
    assert first["inputSign"] == -1.0
    assert first["mode"] == "infeasible"
    assert "first-entry-program-negative-side" in result["flags"]
    assert "first-entry-program-infeasible" in result["flags"]


def test_proxy_stall_does_not_claim_measured_bank_violation() -> None:
    row = {
        "phase": "entryEnergy",
        "constraints": {"stallFraction": 0.20, "stallFractionMeasured": False},
        "currentPlan": {"valid": True, "targetAoA": 8.0, "targetBank": 50.0},
    }
    quality = MOD.control_quality_summary([], [row], {"vehicle": {"maximumAngleOfAttack": 28.0}})
    assert quality["plannerStallBankSafetyViolations"] == 0

    measured = dict(row, constraints={"stallFraction": 0.20, "stallFractionMeasured": True})
    quality = MOD.control_quality_summary([], [measured], {"vehicle": {"maximumAngleOfAttack": 28.0}})
    assert quality["plannerStallBankSafetyViolations"] == 1


def main() -> int:
    test_synthetic_runtime_fields_and_delta_reconstruction()
    test_truncated_tail_recovery_preserves_last_keyframe_and_pairing()
    test_historic_1605_failure_signature()
    test_historic_1739_persistent_plan_safety_signature()
    test_historic_1757_beta_yaw_departure_signature()
    test_proxy_stall_does_not_claim_measured_bank_violation()
    print("Postflight 75 km acceptance analyzer tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
