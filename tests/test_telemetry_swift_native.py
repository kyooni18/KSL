"""Native Swift flight records must appear in the live telemetry observer."""
import json
import os
import time
from Tools.telemetry_web import (
    native_live_log_matches_observer,
    read_controller_mirror,
    read_swift_live_mirror,
)

def sample(ut=1234.5):
    return {
        "sequence": 30,
        "state": {"time": ut},
        "command": {"pitch": 0.3, "roll": -0.2, "yaw": 0.1},
        "guidance": {
            "phase": "entry",
            "diagnostic": "Entry receding-horizon",
            "target": {
                "angleOfAttack": 0.5235987755982988,
                "bank": -0.17453292519943295,
                "desiredFlightPath": -0.05,
            },
            "entry": {
                "energyAllocationMode": "vertical-support-recovery",
                "requiredRange": 255_000.0,
                "referenceTargetRange": 200_000.0,
                "predictedRange": 140_000.0,
                "observedDrag": 7.25,
                "referenceDrag": 1.07,
                "dragTrackingResidual": 7.75,
                "reversalCount": 2,
                "jointForceAttainable": False,
                "jointRequiredLift": 30.62,
                "jointMaximumLift": 4.74,
                "jointMinimumDrag": None,
                "energyDragBudget": 3.56,
                "limitingReason": "vertical force deficit",
            },
            "capture": {
                "mm305PlanAccepted": False,
                "mm305Deficit": "terminalRegionNotReached",
            },
        },
    }

def test_native_run_prefix_and_entry_data(tmp_path):
    run=tmp_path/"runs"/"deorbit-signed-warp-abc"
    run.mkdir(parents=True)
    (run/"live.jsonl").write_text(json.dumps(sample())+"\n")
    value=read_swift_live_mirror(tmp_path)
    assert value is not None
    assert value["phase"]=="MM304 ENTRY"
    assert value["sourceUT"]==1234.5
    assert abs(value["command"]["targetAoA"]-30.0)<1e-8
    assert abs(value["command"]["targetRoll"]+10.0)<1e-8
    entry=value["guidanceState"]["entryFeedback"]
    assert entry["valid"]
    assert entry["energyMargin"] is None  # fitted integral is not reachability
    assert entry["predictedAchievableRange"] is None
    assert entry["dragReferenceIntegral"] == 140_000
    assert entry["dragReferenceFitResidual"] == -60_000
    assert entry["requiredRangeToGo"] == 255_000
    assert abs(entry["dragError"]-6.18)<1e-8
    assert value["guidanceState"]["nativeEntry"]["rangeMargin"] == -60_000
    assert "MM305 CAPTURE UNPROVEN" in value["warningMessage"]
    assert value["guidanceState"]["nativeEntry"]["terminalDeficit"]=="terminalRegionNotReached"
    assert value["guidanceState"]["nativeEntry"]["jointForceAttainable"] is False
    assert value["guidanceState"]["nativeEntry"]["jointRequiredLift"]==30.62
    assert value["guidanceState"]["nativeEntry"]["jointMaximumLift"]==4.74
    assert value["guidanceState"]["nativeEntry"]["energyDragBudget"]==3.56
    assert not value["guidanceState"]["nativeEntry"]["mm305PlanAccepted"]

def test_paused_snapshot_retains_last_good_guidance_with_staleness_warning(tmp_path):
    run=tmp_path/"runs"/"deorbit-warp-campaign"
    run.mkdir(parents=True)
    path=run/"live.jsonl"
    path.write_text(json.dumps(sample())+"\n")
    old=time.time()-250
    os.utime(path,(old,old))
    result=read_controller_mirror(tmp_path)
    assert result is not None
    assert result["logAgeSeconds"]>200
    assert "LAST RECORDED GUIDANCE" in result["statusMessage"]
    assert "LAST RECORDED COMMAND" in result["warningMessage"]
    assert native_live_log_matches_observer(result,{"ut":1234.5})
    assert not native_live_log_matches_observer(result,{"ut":1300})
    assert read_swift_live_mirror(tmp_path,max_age=3) is None

def test_no_made_up_guidance_for_invalid_log(tmp_path):
    run=tmp_path/"runs"/"deorbit-empty"
    run.mkdir(parents=True)
    (run/"live.jsonl").write_text('not json\n{"guidance":{}}\n')
    assert read_swift_live_mirror(tmp_path) is None
    assert read_controller_mirror(tmp_path) is None
