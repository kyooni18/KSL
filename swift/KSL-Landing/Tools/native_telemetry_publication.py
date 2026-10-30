#!/usr/bin/env python3
"""Publish independent native Swift runs to the existing ShuttleSim telemetry UI.

The original KSP live controller mirror is never touched. Each run is
immutable once completed, has a distinct provenance and stays inspectable
under /api/sim-runs and /api/sim-replay.
"""
from __future__ import annotations

import gzip
import json
import math
import pathlib
import shutil
import time

ROOT = pathlib.Path(__file__).resolve().parents[3]


def atomic_json(path: pathlib.Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")
    temporary.replace(path)


def number(value, default=0.0):
    return float(value) if isinstance(value, (float, int)) and math.isfinite(value) else default


class NativeTelemetryPublication:
    def __init__(self, output: pathlib.Path, scenario: pathlib.Path, config: pathlib.Path,
                 category: str, wall_started: float) -> None:
        self.output = output
        self.run_id = output.name
        if not self.run_id or self.run_id in (".", "..") or "/" in self.run_id:
            raise ValueError("Invalid native simulation run ID")
        self.archive_dir = ROOT / "ShuttleSim" / "runs" / self.run_id
        self.archive_dir.mkdir(parents=True, exist_ok=False)
        self.manifest_file = self.archive_dir / "manifest.json"
        self.current_file = ROOT / "Runtime" / "WebTelemetry" / "simulator-run.json"
        self.controller_mirror = output / "native-web-snapshot.json"
        self.archive_file = output / "guidance-snapshots.jsonl.gz"
        self.archive = gzip.open(self.archive_file, "wt", encoding="utf-8")
        self.started = wall_started
        self.last_publication = -math.inf
        self.last_phase = None
        self.manifest = {
            "schema": 1, "runId": self.run_id,
            "mission": "ORBIT-TO-RUNWAY" if "polar" in scenario.name else "FINAL",
            "tags": ["Swift", "Independent", category],
            "state": "running", "mode": "swift-native-independent",
            "scenario": str(scenario),
            "config": str(ROOT / "Configuration" / "sts-live.json"),
            "nativeConfiguration": str(config),
            "startedAt": wall_started, "runnerPid": __import__("os").getpid(),
            "guidanceSnapshots": str(self.archive_file),
            "simulatorTelemetry": str(output / "telemetry.jsonl.gz"),
            "controllerMirror": str(self.controller_mirror),
            "rateMode": "max", "physicsDt": 0.02, "telemetryHz": 10,
            "provenance": {"physics": "independent ShuttleSim C",
                           "controller": "native Swift FlightComputer",
                           "evidence": "offline model only", "category": category},
        }
        if self.current_file.exists():
            backup = self.current_file.with_name("simulator-run.before-native-swift.json")
            if not backup.exists():
                shutil.copy2(self.current_file, backup)
        self.publish()

    def publish(self) -> None:
        atomic_json(self.manifest_file, self.manifest)
        atomic_json(self.current_file, self.manifest)

    @staticmethod
    def snapshot(telemetry: dict, command: dict, step: int) -> dict:
        guidance = command.get("record", {}).get("guidance", {})
        target = guidance.get("target", {})
        angles = telemetry.get("attitude", {})
        ground = telemetry.get("ground", {})
        runway = telemetry.get("runway", {})
        phase = command.get("phase", "unknown")
        return {
            "connectionStatus": "connected",
            "phase": phase,
            "automationEngaged": True,
            "tickSequence": step,
            "plannerFresh": bool(command.get("plan") or guidance),
            "statusMessage": "Independent ShuttleSim: native Swift " + phase,
            "telemetry": {"ut": telemetry.get("ut")},
            "command": {
                "targetAoA": math.degrees(number(target.get("angleOfAttack"))),
                "targetRoll": math.degrees(number(target.get("bank"))),
                "pitch": number(command.get("pitch_input")),
                "roll": number(command.get("roll_input")),
                "yaw": number(command.get("yaw_input")),
                "gear": bool(command.get("gear_down", ground.get("gear_down"))),
                "brakes": bool(command.get("brakes")), "wheelSteering": number(command.get("wheel_steering")),
            },
            "guidanceState": {
                "runwayAlongTrack": number(runway.get("along_m")),
                "runwayCrossTrack": number(runway.get("cross_m")),
                "planID": guidance.get("planID"),
            },
            "deorbitPlan": command.get("plan"),
            "server": {
                "generatedAt": time.time(),
                "source": "Native Swift / independent ShuttleSim",
                "simulation": True, "mode": "simulator",
            },
        }

    def record(self, telemetry: dict, command: dict, step: int) -> None:
        phase = command.get("phase", "unknown")
        changed = self.last_phase != phase
        self.last_phase = phase
        if step % 10 == 1 or changed or command.get("terminal"):
            frame = self.snapshot(telemetry, command, step)
            self.archive.write(json.dumps(frame, separators=(",", ":")) + "\n")
            self.archive.flush()
        if step % 2 == 1 or changed or command.get("terminal"):
            atomic_json(self.controller_mirror, self.snapshot(telemetry, command, step))
        if changed or time.monotonic() - self.last_publication >= 1.0:
            self.manifest["phase"] = phase
            self.manifest["simElapsedSeconds"] = telemetry.get("sim_time")
            self.manifest["lastTelemetryUT"] = telemetry.get("ut")
            self.manifest["lastAlong"] = telemetry.get("runway", {}).get("along_m")
            self.manifest["lastCross"] = telemetry.get("runway", {}).get("cross_m")
            self.publish()
            self.last_publication = time.monotonic()

    def finish(self, summary: dict) -> None:
        self.archive.close()
        outcome = summary["outcome"]
        completed = outcome == "offline-success" and summary.get("rolloutStopped")
        last = summary.get("lastTelemetry") or {}
        ground = summary.get("independentGroundSummary") or {}
        runway = last.get("runway") or {}
        position = last.get("position") or {}
        self.manifest.update({
            "state": "finished" if completed else "failed",
            "terminalPhase": "Complete" if completed else "Abort",
            "finishedAt": time.time(), "wallSeconds": summary["wallSeconds"],
            "simElapsedSeconds": last.get("sim_time"),
            "final": {
                "ut": last.get("ut"),
                "meanAltitude": position.get("altitude_m"),
                "runwayAlongTrack": runway.get("along_m"),
                "runwayCrossTrack": runway.get("cross_m"),
                "situation": "landed" if completed else "offline-model-failure",
                "abortReason": None if completed else summary.get("controllerError") or outcome,
                "simulatorSummary": ground,
                "outcome": outcome,
            },
        })
        self.publish()
