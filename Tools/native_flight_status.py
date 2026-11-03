"""Read-only, PID-bound status mirror for a native Swift ``ksl-landing fly`` run.

The flight backend owns all kRPC commands. This module reads its process arguments
and its existing logs; it never connects to KSP or changes the controller.
"""
from __future__ import annotations

import json
import math
import re
import shlex
import subprocess
import time
from pathlib import Path
from typing import Any

_PROGRESS = re.compile(r"^deorbit (?P<stage>.+?) candidate=(?P<candidate>\d+) case=(?P<case>\d+)/(?P<total>\d+)(?: sim=(?P<sim>[-+0-9.eE]+))?(?: phase=(?P<phase>\S+))?$")
_PHASES = {
    "ORBITALCOAST": "ORBITAL COAST", "BURNSETUP": "BURN SETUP",
    "DEORBITBURN": "DEORBIT BURN", "ENTRYCOAST": "ENTRY COAST",
    "ENTRY": "MM304 ENTRY", "ACQUISITION": "MM305 TAEM",
    "HEADINGALIGNMENT": "MM305 HAC", "APPROACH": "FINAL APPROACH",
    "PREFLARE": "PREFLARE", "FLARE": "FLARE", "TOUCHDOWN": "TOUCHDOWN",
    "ROLLOUT": "ROLLOUT", "COMPLETE": "COMPLETE", "ABORT": "ABORT",
}


def _number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def _tail_lines(path: Path, limit: int = 160_000) -> list[str]:
    try:
        with path.open("rb") as stream:
            stream.seek(0, 2)
            size = stream.tell()
            stream.seek(max(0, size - limit))
            raw = stream.read()
        if size > limit:
            raw = raw.partition(b"\n")[2]
        return raw.decode("utf-8", "replace").splitlines()
    except OSError:
        return []


def _read_latest_flight_record(path: Path) -> dict[str, Any] | None:
    for line in reversed(_tail_lines(path)):
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and isinstance(value.get("guidance"), dict) and isinstance(value.get("state"), dict):
            return value
    return None


def _candidate_runs(root: Path) -> list[tuple[int, Path, Path]]:
    """Resolve active native processes, never matching the parent shell command."""
    try:
        result = subprocess.run(["ps", "-axo", "pid=,command="], check=True,
                                capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return []
    runs = (root / "runs").resolve()
    found: list[tuple[int, Path, Path]] = []
    for line in result.stdout.splitlines():
        pid_text, _, command = line.strip().partition(" ")
        if not pid_text.isdigit():
            continue
        try:
            args = shlex.split(command)
        except ValueError:
            continue
        if len(args) < 5 or not args[0].endswith("ksl-landing") or args[1] != "fly":
            continue
        try:
            index = args.index("--log")
            path = Path(args[index + 1]).expanduser()
        except (ValueError, IndexError):
            continue
        if not path.is_absolute():
            path = root / path
        path = path.resolve()
        if path.is_relative_to(runs) and path.suffix == ".jsonl":
            found.append((int(pid_text), path.parent, path))
    return found


class NativeFlightFollower:
    """Cache cheap, externally verified status from the active Swift process."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.last_scan = 0.0
        self.active: tuple[int, Path, Path] | None = None
        self.last_run: Path | None = None

    def poll(self, telemetry: dict[str, Any]) -> dict[str, Any] | None:
        now = time.monotonic()
        if now - self.last_scan >= 1.0:
            self.last_scan = now
            processes = _candidate_runs(self.root)
            self.active = max(processes, key=lambda item: (item[1].stat().st_mtime if item[1].exists() else 0, item[0]), default=None)
            if self.active:
                self.last_run = self.active[1]
            elif self.last_run is None:
                # The read-only web server may restart after a native CLI
                # aborts. Retain a recent *completed* plan rejection instead
                # of falling back to a misleading observer-only coast phase.
                # Never infer active control from an archived plan file.
                try:
                    logs = [p for p in (self.root / "runs").glob("*/live-flight.stderr.log")
                            if time.time() - p.stat().st_mtime < 300
                            and list(p.parent.glob("live*.jsonl.deorbit-plan.json"))]
                    if logs:
                        self.last_run = max(logs, key=lambda p: p.stat().st_mtime).parent
                except OSError:
                    pass

        if self.active:
            pid, run, flight_log = self.active
            active = True
        elif self.last_run:
            run, pid, active = self.last_run, None, False
            flight_logs = list(run.glob("live*.jsonl"))
            flight_log = max(flight_logs, key=lambda p: p.stat().st_mtime) if flight_logs else run / "live.jsonl"
            # Report terminal results briefly, but never project stale control.
            if time.time() - run.stat().st_mtime > 300:
                self.last_run = None
                return None
        else:
            return None

        saved_state = _read_json(run / "live-state.json") or (_read_json(run / "inspect.json") or {}).get("state") or {}
        source_ut = _number(saved_state.get("time"))
        observer_ut = _number(telemetry.get("ut"))
        vessel = telemetry.get("vesselName")
        if vessel not in ("STS", "STS-N") or (source_ut is not None and observer_ut is not None and observer_ut + 5 < source_ut):
            return None

        progress = None
        evaluated: list[str] = []
        for line in _tail_lines(run / "live-flight.stderr.log"):
            match = _PROGRESS.match(line.strip())
            if match is None:
                continue
            stage = match.group("stage")
            if stage.startswith("runway-candidate-"):
                name = stage.removeprefix("runway-candidate-")
                if name not in evaluated:
                    evaluated.append(name)
            progress = {"stage": stage, "candidate": int(match.group("candidate")),
                        "case": int(match.group("case")), "totalCases": int(match.group("total")),
                        "simulatedElapsed": _number(match.group("sim")),
                        "simulationPhase": match.group("phase")}

        plan_path = Path(str(flight_log) + ".deorbit-plan.json")
        plan = _read_json(plan_path)
        plan_info = None
        selected = None
        if plan is not None:
            source = plan.get("source") or {}
            window = plan.get("window") or {}
            runway_config = ((plan.get("configuration") or {}).get("runway") or {})
            name = runway_config.get("name")
            cases = plan.get("cases") or []
            if (isinstance(name, str) and name and
                _number(source.get("time")) is not None and
                (source_ut is None or abs(float(source["time"]) - source_ut) <= 2) and
                _number(window.get("burnStart")) is not None and
                len(cases) == 1 and cases[0].get("index") == 0):
                nominal_passed = cases[0].get("passed") is True
                entry_experiment = (
                    cases[0].get("evidence") == "experimental-orbital-entry-only-v1"
                    and cases[0].get("burnCompleted") is True
                    and cases[0].get("hardUnsafe") is not True
                )
                # A rejected model result is no runway choice. Explicit
                # experimental orbital-entry geometry is a separate type of
                # permitted auto-selection, never a runway-stop prediction.
                selected = name if (nominal_passed and not cases[0].get("hardUnsafe")) or entry_experiment else None
                limitations = [str(x) for x in plan.get("modelLimitations", []) if isinstance(x, str)]
                plan_info = {
                    "identifier": plan.get("identifier"),
                    "burnStartUT": _number(window.get("burnStart")),
                    "burnDuration": _number(window.get("duration")),
                    "deltaV": _number(window.get("deltaV")),
                    "predictedPostBurnPeriapsisAltitude": _number(window.get("periapsisAltitude")),
                    "nominalPassed": nominal_passed,
                    "experimentalEntry": entry_experiment,
                    "hardUnsafe": cases[0].get("hardUnsafe") is True,
                    "executionQualified": nominal_passed and not cases[0].get("hardUnsafe") and not limitations,
                    "modelLimitations": limitations,
                    "site": {
                        "name": name,
                        "latitude": math.degrees(float(runway_config.get("latitude", 0))),
                        "longitude": math.degrees(float(runway_config.get("longitude", 0))),
                        "runwayHeading": math.degrees(float(runway_config.get("heading", 0))),
                        "altitude": runway_config.get("elevation"),
                        "runwayLength": runway_config.get("length"),
                        "runwayWidth": runway_config.get("width"),
                    },
                }

        record = _read_latest_flight_record(flight_log)
        record_ut = _number((record or {}).get("state", {}).get("time"))
        record_fresh = (record_ut is not None and observer_ut is not None and
                        abs(record_ut - observer_ut) <= 5)
        phase = None
        if record_fresh:
            phase = _PHASES.get(str((record.get("guidance") or {}).get("phase") or "").upper())
        exit_path = run / "live-flight.exit"
        exit_text = exit_path.read_text().strip() if exit_path.exists() else None
        exit_code = int(exit_text) if exit_text and re.fullmatch(r"-?\d+", exit_text) else None
        outcome = None
        for line in reversed(_tail_lines(run / "live-flight.stdout.log", 40_000)):
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if isinstance(value, dict) and value.get("outcome"):
                outcome = value.get("outcome")
                break

        if active and phase:
            # Direct-entry Swift flights are already controlling MM304/MM305;
            # they have no OMS plan and must not be mislabeled as planning.
            state = "flight"
        elif active and selected is None:
            state, phase = "planning", "DEORBIT PLANNING"
        elif active and selected is not None:
            state = "preparing" if source_ut is None or observer_ut is None or observer_ut <= source_ut + 1 else "coast"
            phase = "DEORBIT PREPARING" if state == "preparing" else "ORBITAL COAST"
        else:
            failed_runway_search = any(
                "Neither runway has a nominally qualified" in line
                or "No candidate runway produced" in line
                for line in _tail_lines(run / "live-flight.stderr.log", 8000)
            )
            if (plan_info is not None and not plan_info["nominalPassed"]) or failed_runway_search:
                state, phase = "rejected", "NO QUALIFIED RUNWAY"
            else:
                state = "finished" if outcome == "complete" and exit_code == 0 else "ended"
                phase = phase or ("COMPLETE" if state == "finished" else "NATIVE FLIGHT ENDED")

        stage = progress["stage"] if progress else "awaiting-first-progress"
        if state == "planning":
            message = f"Swift automatic runway evaluation · {stage}"
            if progress and progress["candidate"]:
                message += f" · candidate {progress['candidate']}"
        elif state == "rejected":
            message = "No verified nominal runway stop; neither runway selected and OMS not authorized"
        elif state == "ended":
            message = f"Swift flight process ended · exit {exit_code if exit_code is not None else 'unknown'}; see flight log"
        elif state == "finished":
            message = "Swift flight reported complete; verify actual runway stop"
        elif state == "preparing":
            message = (f"Swift experimentally selected {selected} · OMS window is orbital-geometry only, no modeled landing"
                if plan_info and plan_info.get("experimentalEntry") else f"Swift selected {selected} · preparing native deorbit")
        else:
            message = f"Swift {phase} · runway {selected or 'not selected'}"

        return {
            "active": active,
            "state": state,
            "phase": phase,
            "message": message,
            "source": "Swift native fly (read-only log)",
            "pid": pid,
            "runId": run.name,
            "sourceUT": source_ut,
            "progress": progress,
            "runway": {"mode": "automatic", "evaluated": evaluated, "selected": selected},
            "plan": plan_info,
            "flightLogFresh": record_fresh,
            "exitCode": exit_code,
            "outcome": outcome,
        }
