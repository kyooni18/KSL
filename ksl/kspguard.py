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
from collections import Counter
from pathlib import Path
from typing import Any

from .paths import TOOLS

GUARD_SCRIPT = TOOLS / "ksp_test_guard.py"


class GuardError(RuntimeError):
    pass


STS_REQUIRED_PARTS = {
    "benjee10.shuttle.deltaWing": 2,
    "benjee10.shuttle.elevon1": 2,
    "benjee10.shuttle.elevon2": 2,
    "benjee10.shuttle.mainGear": 2,
    "benjee10.shuttle.noseGear": 1,
    "benjee10.shuttle.bodyFlap": 1,
}


def check_orbiter_integrity(payload: dict[str, Any]) -> dict[str, Any]:
    """Reject a Benjee STS that cannot safely fly or land after deorbit."""
    if payload.get("ready") is not True:
        raise GuardError(f"Active vessel probe failed: {payload}")
    names = payload.get("partNames")
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise GuardError("Active vessel probe did not include a part inventory")
    if not any(name.startswith("benjee10.shuttle.") for name in names):
        return payload
    counts = Counter(names)
    missing = {name: required - counts[name] for name, required in STS_REQUIRED_PARTS.items()
               if counts[name] < required}
    if missing:
        raise GuardError(f"STS orbiter structural integrity failed: missing {missing}")
    return payload


def probe_orbiter_integrity(env: dict[str, str] | None = None) -> dict[str, Any]:
    return check_orbiter_integrity(run_guard("probe", env=env))


def _number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def run_guard(mode: str, python: Path | str | None = None, script: Path = GUARD_SCRIPT,
              env: dict[str, str] | None = None, timeout: float = 15.0,
              args: list[str] | None = None) -> dict[str, Any]:
    """Run one guard mode and return its final JSON payload line."""
    process_env = os.environ.copy()
    process_env.update(env or {})
    completed = subprocess.run([str(python or sys.executable), str(script), mode, *(args or [])], capture_output=True,
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


def ensure_payload_bay_closed(env: dict[str, str] | None = None) -> dict[str, Any]:
    """Ask the independent kRPC guard to verify/close Benjee STS bay doors."""
    payload = run_guard("close-bay", env=env, timeout=25.0)
    if payload.get("event") != "payload-bay-status":
        raise GuardError(f"Payload bay closure acknowledgement invalid: {payload}")
    return payload


def set_ksp_paused(paused: bool, python: Path | str | None = None, script: Path = GUARD_SCRIPT,
                   env: dict[str, str] | None = None) -> None:
    payload = run_guard("pause" if paused else "resume", python, script, env)
    if payload.get("event") != "pause-state" or bool(payload.get("paused")) != paused:
        raise GuardError(f"KSP pause state did not change as requested: {payload}")


def load_save_verified(name: str, env: dict[str, str] | None = None) -> dict[str, Any]:
    """Load a checkpoint and require independent scene/UT settling evidence."""
    payload = run_guard("load-save", env=env, timeout=90.0, args=["--name", name])
    if payload.get("event") != "load-save-settled" or payload.get("name") != name or \
            payload.get("gameScene") != "flight":
        raise GuardError(f"KSP save {name!r} did not confirm a settled flight scene: {payload}")
    return payload
