"""Native MM304/TAEM overlays use only actual, epoch-matched plan geometry."""
import json
import math
from pathlib import Path

from Tools.telemetry_web import (
    apply_exact_trajectory_geometry,
    native_swift_reference_track,
    native_swift_runway_track,
    read_swift_live_mirror,
)


def configuration():
    return {
        "runway": {"name": "KSC Runway 09", "latitude": math.radians(-0.05),
                   "longitude": math.radians(-74.5), "heading": math.pi / 2,
                   "elevation": 70},
        "planet": {"radius": 600000},
    }


def test_native_runway_forecast_is_spherical_and_height_is_above_runway():
    track = [[100.0, 0.0, 0.0, 25000.0],
             [120.0, 20000.0, -2000.0, 21000.0],
             [140.0, 40000.0, 6000.0, 17000.0]]
    points = native_swift_runway_track(track, configuration())
    assert len(points) == 3
    assert abs(points[0]["latitude"] - -0.05) < 1e-7
    assert abs(points[0]["longitude"] - -74.5) < 1e-7
    assert points[0]["altitude"] == 25070.0
    assert points[1]["longitude"] > -74.5
    assert points[1]["latitude"] > points[0]["latitude"]  # right=-north, -cross is north
    assert points[2]["latitude"] < points[0]["latitude"]
    assert points[2]["phase"] == "MM304 FORECAST"
    assert native_swift_runway_track(track, {}) == []


def sample(ut, epoch=100, track=None, *, advisory=None):
    return {
        "sequence": int(ut * 10),
        "state": {"time": ut},
        "command": {"pitch": 0.0},
        "guidance": {
            "phase": "entry", "target": {"angleOfAttack": 0.4, "bank": 0.1},
            "entry": {"referenceDrag": 1.0, "observedDrag": 1.0},
            "capture": {
                "predictedAt": epoch, "predictedTrack": track or [],
                "advisoryReference": advisory,
                "mm305PlanAccepted": False,
            },
        },
    }


def test_sparse_forecast_and_unaccepted_taem_advisory_survive_live_log_bridge(tmp_path):
    run = tmp_path / "runs" / "live-entry"
    run.mkdir(parents=True)
    path = run / "live.jsonl"
    forecast = [[100.0, -50000, 1000, 32000], [120, -40000, 1000, 28000],
                [140, -30000, 1500, 24000]]
    advisory = [[101, -0.05, -74.5, 25070, 700], [123, -0.06, -74.4, 20000, 550]]
    first = sample(100.1, track=forecast, advisory=advisory)
    later = sample(102.0, track=None, advisory=advisory)
    path.write_text(json.dumps(first) + "\n" + json.dumps(later) + "\n")
    Path(str(path) + ".configuration.json").write_text(json.dumps(configuration()))
    mirror = read_swift_live_mirror(tmp_path, max_age=1000)
    assert mirror is not None
    assert mirror["sourceUT"] == 102.0
    assert len(mirror["predictedTrajectory"]) == 3
    assert mirror["predictedTrajectory"][1]["ut"] == 120
    assert mirror["referenceTrajectory"] == []
    assert len(mirror["projectedTAEMTrajectory"]) == 2
    assert mirror["planProvisional"]
    assert mirror["planProvisionalKind"] == "mm305-advisory"
    assert mirror["plannedTrajectory"] == []  # rejected advisory never actuates
    snapshot = {"predictedTrajectory": [], "plannedTrajectory": [], "projectedTAEMTrajectory": []}
    assert apply_exact_trajectory_geometry(snapshot, mirror, {}, {})
    assert len(snapshot["predictedTrajectory"]) == 3
    assert len(snapshot["projectedTAEMTrajectory"]) == 2
    assert snapshot["plannedTrajectory"] == []


def test_old_forecast_and_invalid_advisory_are_never_drawn_as_current(tmp_path):
    run = tmp_path / "runs" / "native-flight"
    run.mkdir(parents=True)
    p = run / "live.jsonl"
    p.write_text(json.dumps(sample(120, epoch=100,
        track=[[100, 0, 0, 30000], [120, -30000, 0, 21000]])) + "\n")
    Path(str(p) + ".configuration.json").write_text(json.dumps(configuration()))
    mirror = read_swift_live_mirror(tmp_path, max_age=1000)
    assert mirror["predictedTrajectory"] == []
    assert mirror["projectedTAEMTrajectory"] == []
    assert native_swift_reference_track([[1, 91, 0, 20000, 400], [2, 0, 0, 19000, 400]]) == []
