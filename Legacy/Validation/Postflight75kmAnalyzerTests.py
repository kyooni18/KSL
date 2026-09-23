#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Tools.postflight_75km_acceptance import (  # noqa: E402
    analyze, common_range_summary, entry_corridor_flags, entry_plan_churn_summary, flatten_vehicle_record,
    load_planner, observed_aero_force_summary, reversal_summary, taem_kinematic_prerequisite_summary,
    terminal_smoothness_summary, thermal_aoa_summary,
)


def write_jsonl(path: Path, records: list[dict]) -> None:
    path.write_text("".join(json.dumps(r, separators=(",", ":")) + "\n" for r in records), encoding="utf-8")


def synthetic_runtime_fixture(directory: Path) -> tuple[Path, Path]:
    session = "synthetic-runtime-session"
    vehicle = directory / "synthetic-vehicle.jsonl"
    planner = directory / "synthetic-planner.jsonl"
    keyframe = {
        "recordType": "vehicleKeyframe", "schemaVersion": 4, "sessionId": session,
        "tickSequence": 1, "ut": 100.0, "wallMonotonicSeconds": 10.0,
        "fields": {
            "state": {"phase": "MM304 Entry", "controlProfile": "entry", "vesselSituation": "flying", "automation": True, "entryReversalScheduled": False},
            "position": {"altitude": 30000.0}, "motion": {"trueAirSpeed": 1500.0},
            "guidance": {"rangeToSite": 50000.0, "runwayAlongTrack": -49000.0, "runwayCrossTrack": 1000.0},
            "runtime": {"loopWallDeltaMilliseconds": 100.0, "controlLoopMilliseconds": 40.0, "telemetryLatencyMilliseconds": 15.0, "guidanceComputeMilliseconds": 20.0, "applyLatencyMilliseconds": 5.0},
        },
    }
    def delta(tick: int, ut: float, wall: float, fields: dict) -> dict:
        return {"recordType": "vehicleDelta", "schemaVersion": 4, "sessionId": session, "tickSequence": tick, "ut": ut, "wallMonotonicSeconds": wall, "fields": fields}
    write_jsonl(vehicle, [
        {"recordType": "sessionStart", "schemaVersion": 4, "sessionId": session,
         "configuration": {"site": {"runwayLength": 2500.0, "runwayWidth": 70.0}}},
        keyframe,
        delta(2, 102.0, 10.1, {
            "state": {"phase": "TAEM", "controlProfile": "taem", "entryReversalScheduled": True,
                      "planId": 1, "planVersion": 1, "parentPlanId": 0, "parentPlanVersion": 0},
            "position": {"altitude": 16500.0}, "motion": {"trueAirSpeed": 1290.0},
            "guidance": {"rangeToSite": 40000.0, "runwayAlongTrack": -39500.0, "runwayCrossTrack": 2000.0,
                         "entryReversalUT": 110.0, "entryReversalRange": 35000.0, "entryReversalSign": 1.0},
            "runtime": {"loopWallDeltaMilliseconds": 100.0, "controlLoopMilliseconds": 50.0, "telemetryLatencyMilliseconds": 20.0, "guidanceComputeMilliseconds": 25.0, "applyLatencyMilliseconds": 5.0},
        }),
        delta(3, 103.0, 10.21, {"runtime": {"loopWallDeltaMilliseconds": 110.0, "controlLoopMilliseconds": 55.0, "telemetryLatencyMilliseconds": 22.0, "guidanceComputeMilliseconds": 28.0, "applyLatencyMilliseconds": 5.0}}),
        delta(4, 104.0, 10.31, {"state": {"phase": "Final", "controlProfile": "final", "entryReversalScheduled": False}, "runtime": {"loopWallDeltaMilliseconds": 100.0, "controlLoopMilliseconds": 45.0, "telemetryLatencyMilliseconds": 18.0, "guidanceComputeMilliseconds": 22.0, "applyLatencyMilliseconds": 5.0}}),
        delta(5, 105.0, 10.41, {"state": {"phase": "Flare", "vesselSituation": "flying"}, "runtime": {"loopWallDeltaMilliseconds": 100.0, "controlLoopMilliseconds": 42.0, "telemetryLatencyMilliseconds": 17.0, "guidanceComputeMilliseconds": 20.0, "applyLatencyMilliseconds": 5.0}}),
        delta(6, 106.0, 10.51, {"state": {"phase": "Flare", "vesselSituation": "landed"},
             "guidance": {"rangeToSite": 1000.0, "runwayAlongTrack": 1000.0, "runwayCrossTrack": 0.0},
             "runtime": {"loopWallDeltaMilliseconds": 100.0, "controlLoopMilliseconds": 40.0, "telemetryLatencyMilliseconds": 15.0, "guidanceComputeMilliseconds": 20.0, "applyLatencyMilliseconds": 5.0}}),
    ])
    geom1 = {"terminalPredictionValid": True, "terminalCandidateValid": True, "terminalCommitted": True, "hacTransitionProgress": 0.5, "hacCircuitCount": 0, "hacRemaining": 12000.0, "mix": 0.5}
    geom2 = {"terminalPredictionValid": True, "terminalCandidateValid": True, "terminalCommitted": True, "hacTransitionProgress": 1.0, "hacCircuitCount": 1, "hacRemaining": 4000.0, "mix": 1.0}
    write_jsonl(planner, [
        {"recordType": "sessionStart", "schemaVersion": 5, "sessionId": session},
        {"recordType": "plannerSample", "schemaVersion": 5, "sessionId": session, "tickSequence": 2, "ut": 102.0, "phase": "taem", "lineage": {"planId": 1, "version": 1, "parentPlanId": 0, "parentVersion": 0}, "currentPlan": {"valid": True, "hasPlannedReversal": True, "plannedReversalUT": 110.0, "plannedReversalSign": 1.0}, "terminalGeometry": geom1},
        {"recordType": "plannerSample", "schemaVersion": 5, "sessionId": session, "tickSequence": 3, "ut": 103.0, "phase": "taem", "lineage": {"planId": 1, "version": 1, "parentPlanId": 0, "parentVersion": 0}, "currentPlan": {"valid": True, "hasPlannedReversal": True, "plannedReversalUT": 110.0, "plannedReversalSign": 1.0}, "terminalGeometry": geom2},
    ])
    return vehicle, planner


def test_synthetic_runtime_fields() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "PASS"
        assert report["sessionPairingValid"]
        assert report["ownershipHandoff"]["tick"] == 2
        assert report["outcome"]["touchdownSeen"] and report["outcome"]["landedSeen"]
        assert report["outcome"]["finalSeen"] and report["outcome"]["flareSeen"]
        assert report["outcome"]["runwayContactValid"]
        assert report["terminalProgress"]["terminalCommitted"] == 2
        assert report["terminalProgress"]["maximumHACTransitionProgress"] == 1.0
        assert report["terminalProgress"]["maximumHACCircuitCount"] == 1.0
        latency = report["runtimeLatency"]
        assert latency["source"] == "fields.runtime"
        taem = latency["byPhase"]["TAEM"]
        assert taem["loopWallDeltaMilliseconds"]["count"] == 2
        assert abs(taem["loopWallDeltaMilliseconds"]["p95"] - 109.5) < 1e-9
        assert taem["controlLoopMilliseconds"]["max"] == 55.0
        assert latency["taemSourcePhaseGapCountOver2Seconds"] == 0
        assert report["plannerLineage"]["continuityBreakCount"] == 0

        assert report["entryReversals"]["comparablePlannerVehicleSamples"] == 2
        assert report["entryReversals"]["plannerVehicleMismatchCount"] == 0
        assert report["runtimeLatency"]["directGapEventsOver2Seconds"] == []
        identity = report["vehiclePlannerPlanIdentity"]
        assert identity["vehicleIdentityAvailable"]
        assert identity["plannerPlanTransitionCount"] == 1
        assert identity["comparableTransitionCount"] == 1
        assert identity["missingTransitionCount"] == 0 and identity["mismatchCount"] == 0



def test_mm304_plan_regeneration_churn_is_rejected() -> None:
    rapid_records = [
        {"recordType": "plannerSample", "schemaVersion": 5, "sessionId": "synthetic-runtime-session",
         "tickSequence": 100 + i, "ut": 100.0 + i * 0.1, "phase": "entryEnergy",
         "lineage": {"planId": 10 + i, "version": 1,
                     "parentPlanId": 0 if i == 0 else 9 + i, "parentVersion": 0 if i == 0 else 1}}
        for i in range(12)
    ]
    churn = entry_plan_churn_summary(rapid_records)
    assert churn["rapidTransitionCount"] == 11
    assert churn["maximumConsecutiveRapidTransitions"] == 11
    assert churn["minimumTransitionIntervalSeconds"] <= 0.1000001

    with tempfile.TemporaryDirectory(prefix="ksp-postflight-churn-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        planner_records = [json.loads(line) for line in planner.read_text(encoding="utf-8").splitlines() if line]
        planner_records[1:1] = rapid_records
        write_jsonl(planner, planner_records)
        report = analyze(vehicle, planner)
        assert report["entryPlanChurn"]["rapidTransitionCount"] == 11
        assert "MM304_PLAN_REGENERATION_CHURN" in {flag["code"] for flag in report["flags"]}


def test_mm304_sustained_five_second_replan_loop_is_rejected() -> None:
    sustained_records = [
        {"recordType": "plannerSample", "schemaVersion": 5, "sessionId": "synthetic-runtime-session",
         "tickSequence": 300 + i, "ut": 200.0 + i * 5.0, "phase": "entryEnergy",
         "lineage": {"planId": 30 + i, "version": 1,
                     "parentPlanId": 0 if i == 0 else 29 + i, "parentVersion": 0 if i == 0 else 1}}
        for i in range(12)
    ]
    churn = entry_plan_churn_summary(sustained_records)
    assert churn["rapidTransitionCount"] == 0
    assert churn["maximumConsecutiveFrequentTransitions"] == 11
    assert abs(churn["minimumTransitionIntervalSeconds"] - 5.0) < 1e-9

    with tempfile.TemporaryDirectory(prefix="ksp-postflight-sustained-churn-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        planner_records = [json.loads(line) for line in planner.read_text(encoding="utf-8").splitlines() if line]
        planner_records[1:1] = sustained_records
        write_jsonl(planner, planner_records)
        report = analyze(vehicle, planner)
        codes = {flag["code"] for flag in report["flags"]}
        assert "MM304_PLAN_REGENERATION_CHURN" not in codes
        assert "MM304_SUSTAINED_REPLAN_LOOP" in codes


def test_mm304_isolated_few_second_replans_do_not_trip_sustained_loop() -> None:
    records = [
        {"phase": "entryEnergy", "tickSequence": 1, "ut": 100.0,
         "lineage": {"planId": 1, "version": 1}},
        {"phase": "entryEnergy", "tickSequence": 2, "ut": 103.0,
         "lineage": {"planId": 2, "version": 1}},
        {"phase": "entryEnergy", "tickSequence": 3, "ut": 130.0,
         "lineage": {"planId": 3, "version": 1}},
        {"phase": "entryEnergy", "tickSequence": 4, "ut": 135.0,
         "lineage": {"planId": 4, "version": 1}},
        {"phase": "entryEnergy", "tickSequence": 5, "ut": 175.0,
         "lineage": {"planId": 5, "version": 1}},
    ]
    churn = entry_plan_churn_summary(records)
    assert churn["maximumConsecutiveFrequentTransitions"] == 1


def test_common_range_and_thermal_helpers() -> None:
    rows = [
        {"phase": "MM304 Entry", "ut": 1.0, "range": 450000.0, "altitude": 50000.0,
         "tas": 2100.0, "flightPathAngle": -1.0, "targetAoA": 18.5, "angleOfAttack": 18.4,
         "roll": 10.0, "targetRoll": 12.0, "constraintActive": False},
        {"phase": "MM304 Entry", "ut": 2.0, "range": 399000.0, "altitude": 48600.0,
         "tas": 2050.0, "verticalSpeed": -72.5, "dynamicPressure": 640.0, "gForce": 0.12,
         "energyExcessRange": 216200.0,
         "flightPathAngle": -1.4, "targetAoA": 18.2, "angleOfAttack": 18.1,
         "roll": 14.0, "targetRoll": 15.0, "constraintActive": False},
        {"phase": "MM304 Entry", "ut": 3.0, "range": 299000.0, "altitude": 45500.0,
         "tas": 2020.0, "flightPathAngle": -1.9, "targetAoA": 8.0, "angleOfAttack": 8.2,
         "roll": 20.0, "targetRoll": 20.0, "constraintActive": False},
        {"phase": "MM304 Entry", "ut": 4.0, "range": 199000.0, "altitude": 41100.0,
         "tas": 1990.0, "flightPathAngle": -2.8, "targetAoA": 8.0, "angleOfAttack": 8.0,
         "roll": 35.0, "targetRoll": 38.0, "constraintActive": False},
        {"phase": "MM304 Entry", "ut": 5.0, "range": 99000.0, "altitude": 34900.0,
         "tas": 1920.0, "flightPathAngle": -3.8, "targetAoA": 8.0, "angleOfAttack": 8.0,
         "roll": -20.0, "targetRoll": -24.0, "constraintActive": False},
    ]
    checkpoints = common_range_summary(rows)
    assert set(checkpoints) == {"400km", "300km", "200km", "100km"}
    assert abs(checkpoints["300km"]["altitudeDebtM"] - 16500.0) < 1e-9
    assert abs(checkpoints["200km"]["altitudeDebtM"] - 17600.0) < 1e-9
    assert checkpoints["400km"]["altitudeDebtDeltaFromPreviousStationM"] is None
    assert abs(checkpoints["300km"]["altitudeDebtDeltaFromPreviousStationM"] - 2400.0) < 1e-9
    assert abs(checkpoints["200km"]["altitudeDebtDeltaFromPreviousStationM"] - 1100.0) < 1e-9
    assert abs(checkpoints["100km"]["altitudeDebtDeltaFromPreviousStationM"] + 700.0) < 1e-9
    assert checkpoints["400km"]["verticalSpeedMps"] == -72.5
    assert checkpoints["400km"]["dynamicPressurePa"] == 640.0
    assert checkpoints["400km"]["gForce"] == 0.12
    assert checkpoints["400km"]["energyExcessRangeM"] == 216200.0
    thermal = thermal_aoa_summary(rows, 18.0, 3.5, 85.0)
    assert thermal["sampleCount"] == 5
    assert thermal["minimumTargetAoADeg"] == 8.0
    assert thermal["unexplainedBelowFloorCount"] == 3


def test_entry_corridor_release_flags_boundary_behavior() -> None:
    def codes(checkpoints: dict[str, dict]) -> set[str]:
        return {flag["code"] for flag in entry_corridor_flags(checkpoints)}

    green = {
        "400km": {"altitudeDebtM": 12000.0},
        "300km": {"altitudeDebtM": 9999.0},
        "200km": {"altitudeDebtM": 7000.0},
        "100km": {"altitudeM": 21000.0, "verticalSpeedMps": -0.001},
    }
    assert codes(green) == set()

    nonconverging = {
        "400km": {"altitudeDebtM": 11000.0},
        "300km": {"altitudeDebtM": 12000.0},
        "200km": {"altitudeDebtM": 6900.0},
        "100km": {"altitudeM": 20000.0, "verticalSpeedMps": -50.0},
    }
    assert codes(nonconverging) == {"ENTRY_ALTITUDE_DEBT_NOT_CONVERGING_BY_300KM"}

    late_corridor = {
        "400km": {"altitudeDebtM": 14000.0},
        "300km": {"altitudeDebtM": 9000.0},
        "200km": {"altitudeDebtM": 7000.1},
        "100km": {"altitudeM": 21000.1, "verticalSpeedMps": -1.0},
    }
    assert codes(late_corridor) == {
        "ENTRY_ALTITUDE_DEBT_ABOVE_7KM_AT_200KM",
        "ENTRY_ALTITUDE_ABOVE_21KM_AT_100KM",
    }

    not_descending = {
        "200km": {"altitudeDebtM": 6900.0},
        "100km": {"altitudeM": 20000.0, "verticalSpeedMps": 0.0},
    }
    assert codes(not_descending) == {"ENTRY_NOT_DESCENDING_AT_100KM"}

    missing_comparison = {
        "300km": {"altitudeDebtM": 15000.0},
        "200km": {"altitudeDebtM": 7000.0},
        "100km": {"altitudeM": 20000.0},
    }
    assert codes(missing_comparison) == set()


def test_thermal_summary_requires_active_explicit_safety_for_sub_floor_aoa() -> None:
    rows = [
        {"phase": "MM304 Entry", "tick": 1, "ut": 1.0, "targetAoA": 18.5, "tas": 1500.0, "gForce": 1.0, "stallFraction": 0.01},
        {"phase": "MM304 Entry", "tick": 2, "ut": 2.0, "targetAoA": 15.0, "tas": 1450.0, "gForce": 3.30, "stallFraction": 0.01},
        {"phase": "MM304 Entry", "tick": 3, "ut": 3.0, "targetAoA": 16.0, "tas": 1400.0, "gForce": 1.5, "stallFraction": 0.01},
        {"phase": "MM304 Entry", "tick": 4, "ut": 4.0, "targetAoA": 18.1, "tas": 1350.0, "gForce": 1.2, "stallFraction": 0.01},
        {"phase": "MM304 Entry", "tick": 5, "ut": 5.0, "targetAoA": 17.0, "tas": 1300.0, "gForce": 1.2, "stallFraction": 0.01},
    ]
    thermal = thermal_aoa_summary(rows, 18.0, 3.5, 85.0)
    assert thermal["explicitSafetyBelowFloorCount"] == 1
    assert thermal["safetyRecoveryTransientBelowFloorCount"] == 0
    assert thermal["unexplainedBelowFloorCount"] == 2


def test_thermal_summary_distinguishes_proxy_from_measured_stall() -> None:
    proxy = [{"phase": "MM304 Entry", "tick": 1, "ut": 1.0, "targetAoA": 15.0,
              "tas": 1500.0, "gForce": 1.0, "stallFraction": 0.20,
              "stallFractionMeasured": False}]
    thermal = thermal_aoa_summary(proxy, 18.0, 3.5, 85.0)
    assert thermal["explicitSafetyBelowFloorCount"] == 0
    assert thermal["unexplainedBelowFloorCount"] == 1

    measured = [dict(proxy[0], stallFractionMeasured=True)]
    thermal = thermal_aoa_summary(measured, 18.0, 3.5, 85.0)
    assert thermal["explicitSafetyBelowFloorCount"] == 1
    assert thermal["unexplainedBelowFloorCount"] == 0


def test_reversal_physical_deadline_uses_leg_elapsed() -> None:
    rows = [
        {"tick": 1, "ut": 105.0, "entryReversalScheduled": True, "reversalUT": 110.0,
         "reversalRange": 200000.0, "reversalSign": -1.0, "targetRoll": 20.0, "roll": 18.0,
         "along": -200000.0, "range": 200000.0},
        {"tick": 2, "ut": 110.0, "entryReversalScheduled": True, "reversalUT": 110.0,
         "reversalRange": 190000.0, "reversalSign": -1.0, "targetRoll": 20.0, "roll": 18.0,
         "along": -190000.0, "range": 190000.0},
        {"tick": 3, "ut": 125.0, "entryReversalScheduled": True, "reversalUT": 110.0,
         "reversalRange": 160000.0, "reversalSign": -1.0, "targetRoll": -20.0, "roll": 2.0,
         "along": -160000.0, "range": 160000.0},
        {"tick": 4, "ut": 126.0, "entryReversalScheduled": False, "reversalUT": 110.0,
         "reversalRange": 158000.0, "reversalSign": -1.0, "targetRoll": -20.0, "roll": -5.0,
         "along": -158000.0, "range": 158000.0},
    ]
    planner = [
        {"tickSequence": 1, "ut": 105.0,
         "currentPlan": {"hasPlannedReversal": True, "plannedReversalUT": 110.0, "plannedReversalSign": -1.0},
         "plannerTrace": {"legEstablished": True, "legElapsed": 5.0}},
        {"tickSequence": 2, "ut": 110.0,
         "currentPlan": {"hasPlannedReversal": True, "plannedReversalUT": 110.0, "plannedReversalSign": -1.0},
         "plannerTrace": {"legEstablished": True, "legElapsed": 10.0}},
    ]
    summary = reversal_summary(rows, planner, 24.0)
    capture = summary["effectiveCaptures"][0]
    assert capture["physicalDwellDeadlineAvailable"]
    assert abs(capture["earliestExecutableUT"] - 124.0) < 1e-9
    assert abs(capture["executionDelaySeconds"] - 2.0) < 1e-9
    assert summary["lateExecutableReversalCount"] == 0
    assert abs(capture["physicalDwellDeferralSeconds"] - 14.0) < 1e-9
    assert summary["publishedBeforePhysicalDwellCount"] == 1
    rows[2]["ut"] = 127.1
    rows[3]["ut"] = 128.0
    late = reversal_summary(rows, planner, 24.0)
    assert late["lateExecutableReversalCount"] == 1


def test_rotated_planner_segments_are_one_logical_stream() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-segments-") as tmp:
        root = Path(tmp)
        base = root / "run-planner.jsonl"
        segment = root / "run-planner-001.jsonl"
        write_jsonl(base, [
            {"recordType": "sessionStart", "sessionId": "segmented"},
            {"recordType": "plannerSample", "sessionId": "segmented", "tickSequence": 1, "ut": 1.0},
        ])
        write_jsonl(segment, [
            {"recordType": "sessionStart", "sessionId": "segmented"},
            {"recordType": "plannerSample", "sessionId": "segmented", "tickSequence": 2, "ut": 2.0},
        ])
        loaded = load_planner(base)
        assert not loaded["missing"]
        assert len(loaded["paths"]) == 2
        assert [record["tickSequence"] for record in loaded["records"]] == [1, 2]


def test_analyze_rejects_unexplained_mm304_subfloor_target_aoa() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-thermal-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            if record.get("tickSequence") == 1:
                fields = record["fields"]
                fields.setdefault("command", {})["targetAoA"] = 8.0
                fields.setdefault("attitude", {})["angleOfAttack"] = 8.0
                fields["state"]["constraintActive"] = False
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["thermalAoA"]["minimumTargetAoADeg"] == 8.0
        assert "MM304_TARGET_AOA_BELOW_THERMAL_FLOOR" in {flag["code"] for flag in report["flags"]}

def test_authoritative_1605_shape() -> None:
    vehicle = ROOT / "FlightLogs/2026-09-11T16-05-21Z-STS-N-vehicle.jsonl"
    planner = ROOT / "FlightLogs/2026-09-11T16-05-21Z-STS-N-planner.jsonl"
    if not vehicle.exists() or not planner.exists():
        return
    report = analyze(vehicle, planner)
    assert report["acceptance"] == "FAIL"
    assert report["sessionPairingValid"]
    handoff = report["ownershipHandoff"]
    assert handoff["tick"] == 7557
    assert abs(handoff["trueAirSpeedMps"] - 1299.63354492188) < 1e-6
    assert abs(handoff["rangeToSiteM"] - 8066.55410024219) < 1e-6
    assert abs(handoff["runwayCrossTrackM"] - 8064.00862667604) < 1e-6
    assert report["runtimeLatency"]["source"].startswith("inferred")
    assert report["runtimeLatency"]["taemSourcePhaseGapCountOver2Seconds"] == 2
    gaps = [g["wallGapMilliseconds"] for g in report["runtimeLatency"]["wallGapEventsOver2Seconds"] if g["sourcePhase"] == "TAEM"]
    assert len(gaps) == 2 and abs(gaps[0] - 9413.38) < 0.1 and abs(gaps[1] - 9293.587) < 0.1
    assert report["entryReversals"]["scheduledEventCount"] == 3
    assert report["plannerLineage"]["continuityBreakCount"] == 0
    assert report["outcome"]["outcome"] == "abort"
    codes = {flag["code"] for flag in report["flags"]}
    assert {"TAEM_HANDOFF_ALTITUDE_OUTSIDE_15_18KM", "TAEM_HANDOFF_PAST_RUNWAY_THRESHOLD",
            "HISTORIC_1605_OVERSHOOT_SIGNATURE", "REPEATED_TAEM_LOOP_GAPS_OVER_2S",
            "EXECUTABLE_REVERSAL_METADATA_MISMATCH", "VEHICLE_PLAN_IDENTITY_UNAVAILABLE", "ABORT_OUTCOME"} <= codes


def test_direct_runtime_gap_is_not_hidden_by_compact_sampling() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-runtime-gap-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            if record.get("tickSequence") == 3:
                record["fields"]["runtime"]["loopWallDeltaMilliseconds"] = 2500.0
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "WARN"
        assert report["runtimeLatency"]["taemSourcePhaseGapCountOver2Seconds"] == 1
        assert len(report["runtimeLatency"]["directGapEventsOver2Seconds"]) == 1
        assert "TAEM_LOOP_GAP_OVER_2S" in {flag["code"] for flag in report["flags"]}


def test_handoff_corridor_gate_rejects_high_fast_past_threshold() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-handoff-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            if record.get("tickSequence") == 2:
                record["fields"]["position"]["altitude"] = 19000.0
                record["fields"]["motion"]["trueAirSpeed"] = 1310.0
                record["fields"]["guidance"]["runwayAlongTrack"] = 250.0
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        codes = {flag["code"] for flag in report["flags"]}
        assert {"TAEM_HANDOFF_ALTITUDE_OUTSIDE_15_18KM", "TAEM_HANDOFF_SPEED_ABOVE_1300",
                "TAEM_HANDOFF_PAST_RUNWAY_THRESHOLD"} <= codes


def test_handoff_must_be_descending_when_vertical_speed_is_available() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-handoff-descent-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            if record.get("tickSequence") == 2:
                record["fields"]["motion"]["verticalSpeed"] = 0.0
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert "TAEM_HANDOFF_NOT_DESCENDING" in {flag["code"] for flag in report["flags"]}

        for record in records:
            if record.get("tickSequence") == 2:
                record["fields"]["motion"]["verticalSpeed"] = -0.001
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert "TAEM_HANDOFF_NOT_DESCENDING" not in {flag["code"] for flag in report["flags"]}


def test_planner_reversal_must_match_vehicle_durable_event() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-reversal-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in planner.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            if record.get("tickSequence") == 3:
                record["currentPlan"]["plannedReversalUT"] = 111.0
        write_jsonl(planner, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["entryReversals"]["plannerVehicleMismatchCount"] == 1
        assert "EXECUTABLE_REVERSAL_METADATA_MISMATCH" in {flag["code"] for flag in report["flags"]}


def test_effective_reversal_capture_past_threshold_is_rejected() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-effective-reversal-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        vehicle_records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        for record in vehicle_records:
            tick = record.get("tickSequence")
            fields = record.get("fields") or {}
            if tick == 2:
                fields.setdefault("attitude", {})["roll"] = 5.0
                fields.setdefault("command", {})["targetRoll"] = 5.0
                fields["guidance"]["entryReversalUT"] = 102.0
                fields["guidance"]["entryReversalSign"] = -1.0
            elif tick == 3:
                fields.setdefault("attitude", {})["roll"] = 2.0
                fields.setdefault("command", {})["targetRoll"] = -5.0
                fields.setdefault("guidance", {})["runwayAlongTrack"] = 1000.0
                fields["guidance"]["rangeToSite"] = 5000.0
            elif tick == 4:
                fields.setdefault("attitude", {})["roll"] = -3.0
                fields.setdefault("command", {})["targetRoll"] = -10.0
        write_jsonl(vehicle, vehicle_records)

        planner_records = [json.loads(line) for line in planner.read_text(encoding="utf-8").splitlines() if line]
        for record in planner_records:
            plan = record.get("currentPlan") or {}
            if plan.get("hasPlannedReversal"):
                plan["plannedReversalUT"] = 102.0
                plan["plannedReversalSign"] = -1.0
        write_jsonl(planner, planner_records)

        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        reversals = report["entryReversals"]
        assert reversals["plannerVehicleMismatchCount"] == 0
        assert reversals["effectiveCapturePastThresholdCount"] == 1
        capture = next(c for c in reversals["effectiveCaptures"] if c.get("due") and c.get("durable"))
        assert capture["targetSignCapture"]["ut"] == 103.0
        assert capture["measuredSignCapture"]["ut"] == 104.0
        assert capture["targetSignCapture"]["runwayAlongTrackM"] == 1000.0
        assert capture["measuredSignCapture"]["runwayAlongTrackM"] == 1000.0
        assert "REVERSAL_CAPTURE_PAST_RUNWAY_THRESHOLD" in {flag["code"] for flag in report["flags"]}


def test_authoritative_1852_effective_reversal_capture() -> None:
    vehicle = ROOT / "FlightLogs/2026-09-11T18-52-52Z-STS-N-vehicle.jsonl"
    planner = ROOT / "FlightLogs/2026-09-11T18-52-52Z-STS-N-planner.jsonl"
    if not vehicle.exists() or not planner.exists():
        return
    report = analyze(vehicle, planner)
    assert report["acceptance"] == "FAIL"
    reversals = report["entryReversals"]
    assert reversals["effectiveCapturePastThresholdCount"] >= 1
    assert "REVERSAL_CAPTURE_PAST_RUNWAY_THRESHOLD" in {flag["code"] for flag in report["flags"]}


def test_completed_run_requires_vehicle_plan_identity() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-plan-id-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            state = (record.get("fields") or {}).get("state") or {}
            for key in ("planId", "planVersion", "parentPlanId", "parentPlanVersion"):
                state.pop(key, None)
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert not report["vehiclePlannerPlanIdentity"]["vehicleIdentityAvailable"]
        assert "VEHICLE_PLAN_IDENTITY_UNAVAILABLE" in {flag["code"] for flag in report["flags"]}


def test_session_mismatch_is_rejected() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-mismatch-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in planner.read_text(encoding="utf-8").splitlines() if line]
        for record in records:
            record["sessionId"] = "different-planner-session"
        write_jsonl(planner, records)
        report = analyze(vehicle, planner)
        assert not report["sessionPairingValid"]
        assert report["acceptance"] == "FAIL"
        assert "SESSION_PAIR_MISMATCH" in {flag["code"] for flag in report["flags"]}


def _mutate_final_contact(vehicle: Path, *, situation: str, along: float, cross: float) -> None:
    records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
    final = records[-1]
    fields = final.setdefault("fields", {})
    fields.setdefault("state", {})["vesselSituation"] = situation
    guidance = fields.setdefault("guidance", {})
    guidance["runwayAlongTrack"] = along
    guidance["runwayCrossTrack"] = cross
    write_jsonl(vehicle, records)


def test_splashdown_never_counts_as_touchdown() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-splash-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        _mutate_final_contact(vehicle, situation="splashed", along=1000.0, cross=0.0)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["outcome"]["splashedSeen"] and not report["outcome"]["touchdownSeen"]
        assert "SPLASHDOWN_OUTCOME" in {flag["code"] for flag in report["flags"]}


def test_landed_off_runway_never_counts_as_touchdown() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-offrunway-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        _mutate_final_contact(vehicle, situation="landed", along=1000.0, cross=100.0)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["outcome"]["landedSeen"] and not report["outcome"]["runwayContactValid"]
        assert not report["outcome"]["touchdownSeen"]
        assert "LANDED_OUTSIDE_RUNWAY" in {flag["code"] for flag in report["flags"]}


def test_proper_runway_landing_counts_as_touchdown() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-runway-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "PASS"
        assert report["outcome"]["touchdownSeen"]
        assert report["outcome"]["landingSequenceValid"]
        assert report["outcome"]["runwayContactValid"]


def test_contact_before_flare_does_not_count_as_touchdown() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-contact-before-flare-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        premature_contact = records[-3]
        fields = premature_contact.setdefault("fields", {})
        fields.setdefault("state", {})["vesselSituation"] = "landed"
        guidance = fields.setdefault("guidance", {})
        guidance["runwayAlongTrack"] = 1000.0
        guidance["runwayCrossTrack"] = 0.0
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["outcome"]["landedSeen"] and not report["outcome"]["touchdownSeen"]
        assert not report["outcome"]["landingSequenceValid"]
        assert "LANDING_SEQUENCE_INCOMPLETE" in {flag["code"] for flag in report["flags"]}


def test_flare_before_final_contact_does_not_count_as_touchdown() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-flare-before-final-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        records[-3].setdefault("fields", {}).setdefault("state", {})["phase"] = "Flare"
        contact = records[-2]
        fields = contact.setdefault("fields", {})
        state = fields.setdefault("state", {})
        state["phase"] = "Final"
        state["vesselSituation"] = "landed"
        guidance = fields.setdefault("guidance", {})
        guidance["runwayAlongTrack"] = 1000.0
        guidance["runwayCrossTrack"] = 0.0
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["outcome"]["landedSeen"] and not report["outcome"]["touchdownSeen"]
        assert not report["outcome"]["landingSequenceValid"]
        assert "LANDING_SEQUENCE_INCOMPLETE" in {flag["code"] for flag in report["flags"]}


def test_fault_after_runway_contact_forces_failure() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-fault-after-contact-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        records.append({
            "recordType": "vehicleDelta", "schemaVersion": 4, "sessionId": "synthetic-runtime-session",
            "tickSequence": 7, "ut": 107.0, "wallMonotonicSeconds": 10.61,
            "fields": {"state": {"phase": "Fault", "vesselSituation": "flying"}},
        })
        write_jsonl(vehicle, records)
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "FAIL"
        assert report["outcome"]["faultSeen"]
        assert report["outcome"]["outcome"] == "fault"
        assert not report["outcome"]["touchdownSeen"]
        assert "FAULT_OUTCOME" in {flag["code"] for flag in report["flags"]}


def test_incomplete_airborne_run_remains_incomplete() -> None:
    with tempfile.TemporaryDirectory(prefix="ksp-postflight-incomplete-") as tmp:
        vehicle, planner = synthetic_runtime_fixture(Path(tmp))
        records = [json.loads(line) for line in vehicle.read_text(encoding="utf-8").splitlines() if line]
        write_jsonl(vehicle, records[:-1])
        report = analyze(vehicle, planner)
        assert report["acceptance"] == "WARN"
        assert report["outcome"]["outcome"] == "incomplete"
        assert not report["outcome"]["landedSeen"] and not report["outcome"]["splashedSeen"]
        assert "INCOMPLETE_RUN" in {flag["code"] for flag in report["flags"]}



def test_terminal_smoothness_metrics_quantify_target_jumps_and_control_flashing() -> None:
    rows = [
        {"tick": 1, "ut": 100.0, "phase": "TAEM", "targetPitch": -5.0, "targetRoll": 20.0,
         "targetHeading": 359.0, "targetAoA": 18.0, "appliedPitch": 0.4, "appliedRoll": 0.7,
         "appliedYaw": 0.0},
        {"tick": 2, "ut": 100.4, "phase": "TAEM", "targetPitch": -4.0, "targetRoll": -5.0,
         "targetHeading": 1.0, "targetAoA": 18.5, "appliedPitch": -0.5, "appliedRoll": -0.8,
         "appliedYaw": 0.0},
        {"tick": 3, "ut": 102.0, "phase": "Final", "targetPitch": 8.0, "targetRoll": -4.0,
         "targetHeading": 2.0, "targetAoA": 16.0, "appliedPitch": -0.4, "appliedRoll": -0.7,
         "appliedYaw": 0.0},
        {"tick": 4, "ut": 102.2, "phase": "Cruise", "targetPitch": -30.0, "targetRoll": 60.0,
         "targetHeading": 180.0, "targetAoA": 2.0, "appliedPitch": 1.0, "appliedRoll": 1.0,
         "appliedYaw": 1.0},
    ]
    summary = terminal_smoothness_summary(rows)
    assert summary["terminalSampleCount"] == 3
    assert summary["phasesPresent"] == ["Final", "TAEM"]
    assert summary["targetContinuity"]["targetRoll"]["largeStepCount"] == 1
    assert abs(summary["targetContinuity"]["targetRoll"]["maximumStepDeg"] - 25.0) < 1e-9
    assert abs(summary["targetContinuity"]["targetHeading"]["maximumStepDeg"] - 2.0) < 1e-9
    assert summary["targetContinuity"]["targetPitch"]["largeStepCount"] == 1
    pitch = summary["appliedControlReversals"]["appliedPitch"]
    roll = summary["appliedControlReversals"]["appliedRoll"]
    assert pitch["reversalCount"] == 1 and pitch["rapidReversalCount"] == 1
    assert roll["reversalCount"] == 1 and roll["rapidReversalCount"] == 1
    assert abs(pitch["minimumReversalIntervalSeconds"] - 0.4) < 1e-9


def test_taem_kinematic_prerequisites_separate_late_crossings() -> None:
    rows = [
        {"phase": "MM304 Entry", "tick": 1, "ut": 1.0, "altitude": 27000.0, "tas": 1299.0,
         "range": 55000.0, "along": -54000.0, "cross": 6000.0, "flightPathAngle": -6.0},
        {"phase": "MM304 Entry", "tick": 2, "ut": 2.0, "altitude": 17900.0, "tas": 500.0,
         "range": 11000.0, "along": 5000.0, "cross": 9000.0, "flightPathAngle": -15.0},
    ]
    summary = taem_kinematic_prerequisite_summary(rows)
    assert not summary["basicUpstreamWindowSeen"]
    assert summary["firstSpeedAtOrBelowMax"]["altitudeM"] == 27000.0
    assert summary["firstSpeedAtOrBelowMax"]["runwayAlongTrackM"] == -54000.0
    assert summary["firstAltitudeAtOrBelowMax"]["trueAirSpeedMps"] == 500.0
    assert summary["firstAltitudeAtOrBelowMax"]["runwayAlongTrackM"] == 5000.0
    assert summary["firstBasicUpstreamWindow"] is None

    rows.insert(1, {"phase": "MM304 Entry", "tick": 3, "ut": 1.5, "altitude": 17500.0, "tas": 1250.0,
                    "range": 40000.0, "along": -39000.0, "cross": 1500.0, "flightPathAngle": -7.0})
    summary = taem_kinematic_prerequisite_summary(rows)
    assert summary["basicUpstreamWindowSeen"]
    assert summary["firstBasicUpstreamWindow"]["runwayAlongTrackM"] == -39000.0


def test_split_vehicle_physics_diagnostics_are_flattened() -> None:
    row = flatten_vehicle_record(
        {"tickSequence": 7, "ut": 123.0},
        {"aero": {"energyExcessRange": 12345.0}, "physics": {
            "confidence": .72, "modelResidual": .18, "modelResidualConfidence": .61,
            "certifiedUncertainty": .34, "forceResidualPerQ": [-.0012, .0004, .0001],
            "forceResidualSigmaPerQ": [.0002, .0001, .00005], "forceResidualConfidence": .55,
            "samples": 192, "liveSamples": 12,
        }},
    )
    assert row["physicsConfidence"] == .72
    assert row["physicsForceResidualPerQ"] == [-.0012, .0004, .0001]
    assert row["physicsForceResidualConfidence"] == .55
    assert row["physicsSamples"] == 192
    assert row["physicsLiveSamples"] == 12
    assert row["energyExcessRange"] == 12345.0


def test_observed_aero_force_summary_preserves_live_q_force_evidence() -> None:
    rows = [
        {"phase": "MM304 Entry", "ut": 1.0, "dynamicPressure": 250.0, "mass": 1000.0,
         "dragForce": 250.0, "liftForce": 125.0, "mach": 7.0, "angleOfAttack": 25.0},
        {"phase": "MM304 Entry", "ut": 2.0, "dynamicPressure": 320.0, "mass": 1000.0,
         "dragForce": 400.0, "liftForce": 200.0, "mach": 6.8, "angleOfAttack": 25.0},
        {"phase": "MM304 Entry", "ut": 3.0, "dynamicPressure": 370.0, "mass": 800.0,
         "dragForce": -480.0, "liftForce": 240.0, "mach": 6.6, "angleOfAttack": 24.0},
        {"phase": "MM304 Entry", "ut": 4.0, "dynamicPressure": 720.0, "mass": 800.0,
         "dragForce": 960.0, "liftForce": -384.0, "mach": 6.2, "angleOfAttack": 22.0,
         "physicsConfidence": .70, "physicsModelResidual": .20, "physicsModelResidualConfidence": .62,
         "physicsCertifiedUncertainty": .45, "physicsForceResidualPerQ": [-.001, .0004, 0.0],
         "physicsForceResidualSigmaPerQ": [.0002, .0001, 0.0], "physicsForceResidualConfidence": .58,
         "physicsSamples": 192, "physicsLiveSamples": 24},
    ]
    summary = observed_aero_force_summary(rows, (300.0, 360.0, 700.0))
    assert abs(summary["q300"]["dragAccelerationMps2"] - 0.4) < 1e-9
    assert abs(summary["q300"]["liftToDrag"] - 0.5) < 1e-9
    assert abs(summary["q360"]["dragAccelerationMps2"] - 0.6) < 1e-9
    assert abs(summary["q700"]["dragAccelerationMps2"] - 1.2) < 1e-9
    assert abs(summary["q700"]["liftAccelerationMps2"] - 0.48) < 1e-9
    assert summary["q700"]["observedDynamicPressurePa"] == 720.0
    assert summary["q700"]["physicsForceResidualConfidence"] == .58
    assert summary["q700"]["physicsCertifiedUncertainty"] == .45
    assert summary["q700"]["physicsForceResidualPerQ"] == [-.001, .0004, 0.0]
    assert summary["q700"]["physicsLiveSamples"] == 24
def main() -> None:
    test_synthetic_runtime_fields()
    test_mm304_plan_regeneration_churn_is_rejected()
    test_terminal_smoothness_metrics_quantify_target_jumps_and_control_flashing()
    test_common_range_and_thermal_helpers()
    test_entry_corridor_release_flags_boundary_behavior()
    test_taem_kinematic_prerequisites_separate_late_crossings()
    test_observed_aero_force_summary_preserves_live_q_force_evidence()
    test_split_vehicle_physics_diagnostics_are_flattened()
    test_thermal_summary_requires_active_explicit_safety_for_sub_floor_aoa()
    test_thermal_summary_distinguishes_proxy_from_measured_stall()
    test_reversal_physical_deadline_uses_leg_elapsed()
    test_rotated_planner_segments_are_one_logical_stream()
    test_analyze_rejects_unexplained_mm304_subfloor_target_aoa()
    test_authoritative_1605_shape()
    test_direct_runtime_gap_is_not_hidden_by_compact_sampling()
    test_handoff_corridor_gate_rejects_high_fast_past_threshold()
    test_handoff_must_be_descending_when_vertical_speed_is_available()
    test_planner_reversal_must_match_vehicle_durable_event()
    test_effective_reversal_capture_past_threshold_is_rejected()
    test_authoritative_1852_effective_reversal_capture()
    test_completed_run_requires_vehicle_plan_identity()
    test_session_mismatch_is_rejected()
    test_splashdown_never_counts_as_touchdown()
    test_landed_off_runway_never_counts_as_touchdown()
    test_proper_runway_landing_counts_as_touchdown()
    test_contact_before_flare_does_not_count_as_touchdown()
    test_flare_before_final_contact_does_not_count_as_touchdown()
    test_fault_after_runway_contact_forces_failure()
    test_incomplete_airborne_run_remains_incomplete()
    print("Postflight 75 km analyzer tests passed.")


if __name__ == "__main__":
    main()
