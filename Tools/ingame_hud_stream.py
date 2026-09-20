#!/usr/bin/env python3
"""Render KSP Shuttle Lander snapshots as an in-game AR HUD.

This process is deliberately display-only. It receives configuration/snapshot
NDJSON on stdin and owns a separate kRPC Drawing connection to KSP.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import time
from typing import Any

from PyQtApp.ingame_hud import InGameARHUD


_CONTROLLER_MIRROR = Path(__file__).resolve().parents[1] / "Runtime" / "WebTelemetry" / "controller-snapshot.json"


def _mirror_controller_snapshot(snapshot: dict[str, Any]) -> None:
    """Publish the exact backend command for read-only observers.

    Keep the mirror compact and atomic so telemetry_web.py can consume the
    controller's post-limiter command without sharing kRPC control ownership.
    """
    payload = {
        "generatedAt": time.time(),
        "tickSequence": snapshot.get("tickSequence"),
        "connectionStatus": snapshot.get("connectionStatus"),
        "phase": snapshot.get("phase"),
        "statusMessage": snapshot.get("statusMessage"),
        "warningMessage": snapshot.get("warningMessage"),
        "automationEngaged": snapshot.get("automationEngaged"),
        "telemetry": snapshot.get("telemetry") if isinstance(snapshot.get("telemetry"), dict) else {},
        "command": snapshot.get("command") if isinstance(snapshot.get("command"), dict) else {},
        "guidanceState": snapshot.get("guidanceState") if isinstance(snapshot.get("guidanceState"), dict) else {},
        # Future-path geometry belongs to the controller snapshot too. Keeping it
        # in the read-only mirror lets web telemetry show the exact live forecast
        # and guidance reference instead of reconstructing them from forensic logs.
        "referenceTrajectory": snapshot.get("referenceTrajectory") if isinstance(snapshot.get("referenceTrajectory"), list) else [],
        "predictedTrajectory": snapshot.get("predictedTrajectory") if isinstance(snapshot.get("predictedTrajectory"), list) else [],
    }
    try:
        _CONTROLLER_MIRROR.parent.mkdir(parents=True, exist_ok=True)
        temporary = _CONTROLLER_MIRROR.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False), encoding="utf-8")
        os.replace(temporary, _CONTROLLER_MIRROR)
    except (OSError, TypeError, ValueError):
        pass


def main() -> int:
    hud = InGameARHUD()
    configuration: dict[str, Any] = {}
    try:
        for raw_line in sys.stdin:
            line = raw_line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind = message.get("type")
            if kind == "configuration":
                value = message.get("configuration")
                if isinstance(value, dict):
                    configuration = value
            elif kind == "snapshot":
                snapshot = message.get("snapshot")
                if isinstance(snapshot, dict):
                    _mirror_controller_snapshot(snapshot)
                    hud.update(snapshot, configuration)
            elif kind == "suspend":
                hud.suspend()
            elif kind == "shutdown":
                break
    finally:
        hud.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
