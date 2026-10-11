"""Calls into the independent kRPC safety guard (``Tools/ksp_test_guard.py``).

The guard runs as a separate Python process with its own kRPC client so it
keeps working if the native backend stalls. KSL uses it for the same safety
steps the headless live harness relies on: pausing KSP while the deorbit
search runs, and the verified direct-control handoff before re-entering
guidance mid-descent.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from .paths import TOOLS

GUARD_SCRIPT = TOOLS / "ksp_test_guard.py"


class GuardError(RuntimeError):
    pass


def _number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def run_guard(mode: str, python: Path | str | None = None, script: Path = GUARD_SCRIPT,
              env: dict[str, str] | None = None, timeout: float = 15.0) -> dict[str, Any]:
    """Run one guard mode and return its final JSON payload line."""
    process_env = os.environ.copy()
    process_env.update(env or {})
    completed = subprocess.run([str(python or sys.executable), str(script), mode], capture_output=True,
                               text=True, timeout=timeout, check=False, env=process_env)
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if completed.returncode != 0 or not lines:
        detail = (completed.stderr or completed.stdout or "no response").strip()
        raise GuardError(f"KSP guard '{mode}' failed: {detail[:300]}")
    try:
        return json.loads(lines[-1])
    except json.JSONDecodeError as exc:
        raise GuardError(f"KSP guard '{mode}' returned invalid JSON: {lines[-1][:200]}") from exc


def check_reentry_handoff(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate the guard's direct-control handoff confirmation."""
    if payload.get("event") != "reentry-direct-control-ready":
        raise GuardError(f"Reentry handoff confirmation is invalid: {payload}")
    if payload.get("sas") is not False:
        raise GuardError(f"Reentry handoff did not disable SAS: {payload}")
    throttle = _number(payload.get("throttle"))
    if not math.isfinite(throttle) or abs(throttle) > 1e-4:
        raise GuardError(f"Reentry handoff did not confirm zero throttle: {payload}")
    actual_thrust = _number(payload.get("actualThrust"))
    if not math.isfinite(actual_thrust) or abs(actual_thrust) > 1.0:
        raise GuardError(f"Reentry handoff did not confirm zero actual thrust: {payload}")
    rails_warp = _number(payload.get("railsWarpFactor"))
    physics_warp = _number(payload.get("physicsWarpFactor"))
    if not math.isfinite(rails_warp) or rails_warp != 0.0:
        raise GuardError(f"Reentry handoff did not confirm 1x rails warp: {payload}")
    if not math.isfinite(physics_warp) or physics_warp != 0.0:
        raise GuardError(f"Reentry handoff did not confirm 1x physics warp: {payload}")
    return payload


def prepare_reentry_direct_control(python: Path | str | None = None, script: Path = GUARD_SCRIPT,
                                   env: dict[str, str] | None = None) -> dict[str, Any]:
    return check_reentry_handoff(run_guard("prepare-reentry", python, script, env))


def set_ksp_paused(paused: bool, python: Path | str | None = None, script: Path = GUARD_SCRIPT,
                   env: dict[str, str] | None = None) -> None:
    payload = run_guard("pause" if paused else "resume", python, script, env)
    if payload.get("event") != "pause-state" or bool(payload.get("paused")) != paused:
        raise GuardError(f"KSP pause state did not change as requested: {payload}")
