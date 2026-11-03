"""Entry-only Swift flights must be live guidance, not a failed OMS search."""
import json
import subprocess
from pathlib import Path
from Tools import native_flight_status as status


def test_entry_only_flight_process_is_discovered(tmp_path, monkeypatch):
    run = tmp_path / "runs" / "current-entry"
    run.mkdir(parents=True)
    logfile = run / "live-flight.jsonl"
    logfile.write_text("")
    executable = run / "bin" / "ksl-landing"
    monkeypatch.setattr(status.subprocess, "run", lambda *args, **kwargs:
        subprocess.CompletedProcess(args[0], 0,
            stdout=f"40001 {executable} fly --vessel STS --log {logfile} --duration 3600\n"))
    assert status._candidate_runs(tmp_path) == [(40001, run, logfile)]


def test_entry_only_flight_reports_actual_mm304_ownership(tmp_path, monkeypatch):
    run = tmp_path / "runs" / "entry-current"
    run.mkdir(parents=True)
    logfile = run / "live-sts-flight.jsonl"
    logfile.write_text(json.dumps({"state": {"time": 150.0},
        "guidance": {"phase": "entry", "target": {}}, "command": {}}) + "\n")
    monkeypatch.setattr(status, "_candidate_runs", lambda root: [(40001, run, logfile)])
    follower = status.NativeFlightFollower(tmp_path)
    result = follower.poll({"ut": 150.0, "vesselName": "STS"})
    assert result is not None
    assert result["active"]
    assert result["state"] == "flight"
    assert result["phase"] == "MM304 ENTRY"
