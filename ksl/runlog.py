"""Mission run directory: event log, live status, telemetry, and checkpoints.

Every mission run (live, simulated, or dry-run) gets ``runs/<run-id>/``:

    config.json       resolved mission profile used for the run
    events.jsonl      every mission event (phase changes, logs, warnings, errors)
    telemetry.jsonl   unified telemetry samples from every phase program
    status.json       latest mission status, rewritten atomically
    checkpoint.json   last phase checkpoint (what ``ksl continue`` resumes from)

Subscribers (CLI printer, TUI) receive the same ``Event`` objects that are
written to disk, so both interfaces see one mission record.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator

from .phases import MissionState, Phase
from .paths import RUNS_DIR

LEVELS = ("debug", "info", "warning", "error")


@dataclass
class Event:
    kind: str                     # phase | state | log | telemetry | checkpoint | program
    message: str = ""
    level: str = "info"
    phase: str | None = None
    data: dict[str, Any] = field(default_factory=dict)
    time: float = field(default_factory=time.time)

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"), default=str, allow_nan=False)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "Event":
        return cls(kind=str(raw.get("kind", "log")), message=str(raw.get("message", "")),
                   level=str(raw.get("level", "info")), phase=raw.get("phase"),
                   data=dict(raw.get("data") or {}), time=float(raw.get("time", 0.0)))


@dataclass
class MissionStatus:
    run_id: str
    profile: str
    mode: str
    state: str = MissionState.READY.value
    phase: str = Phase.PRELAUNCH.value
    start_phase: str = Phase.PRELAUNCH.value
    stop_phase: str = Phase.COMPLETE.value
    program: str = ""                     # active phase program (ascent/orbit-trim/landing/simulation)
    connection: str = "disconnected"
    status_message: str = ""
    warning: str | None = None
    error: str | None = None
    telemetry: dict[str, Any] = field(default_factory=dict)
    phase_history: list[str] = field(default_factory=list)
    started_at: float | None = None
    updated_at: float = field(default_factory=time.time)
    finished_at: float | None = None

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "MissionStatus":
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in raw.items() if k in known})


def new_run_id(label: str) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in label).strip("-") or "mission"
    return f"{stamp}-{safe}"


def _write_json_atomic(path: Path, value: Any) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, default=str, allow_nan=False) + "\n")
    os.replace(tmp, path)


def _finite(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class RunLog:
    """Thread-safe writer for one run directory plus an in-process event bus."""

    def __init__(self, directory: Path, status: MissionStatus) -> None:
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.status = status
        self._lock = threading.RLock()
        self._subscribers: list[Callable[[Event], None]] = []
        self._events = open(directory / "events.jsonl", "a", encoding="utf-8")
        self._telemetry = open(directory / "telemetry.jsonl", "a", encoding="utf-8")
        self._last_status_write = 0.0
        self.write_status(force=True)

    @classmethod
    def create(cls, status: MissionStatus, root: Path | None = None) -> "RunLog":
        return cls((root or RUNS_DIR) / status.run_id, status)

    def subscribe(self, callback: Callable[[Event], None]) -> None:
        with self._lock:
            self._subscribers.append(callback)

    def emit(self, event: Event) -> Event:
        with self._lock:
            if event.phase is None:
                event.phase = self.status.phase
            if event.kind != "telemetry":
                self._events.write(event.to_json() + "\n")
                self._events.flush()
            subscribers = list(self._subscribers)
        for callback in subscribers:
            try:
                callback(event)
            except Exception:  # a broken UI must never take down flight control
                pass
        return event

    def log(self, message: str, level: str = "info", **data: Any) -> Event:
        if level == "warning":
            self.update(warning=message)
        return self.emit(Event("log", message, level=level, data=data))

    def telemetry(self, sample: dict[str, Any], source: str) -> None:
        clean = {k: _finite(v) for k, v in sample.items()}
        record = {"time": time.time(), "source": source, "phase": self.status.phase, **clean}
        with self._lock:
            self._telemetry.write(json.dumps(record, separators=(",", ":"), default=str) + "\n")
            self._telemetry.flush()
            self.status.telemetry = clean
        self.emit(Event("telemetry", data=record))
        self.write_status()

    def update(self, **fields: Any) -> None:
        with self._lock:
            for key, value in fields.items():
                setattr(self.status, key, value)
        self.write_status(force=True)

    def write_status(self, force: bool = False) -> None:
        now = time.time()
        with self._lock:
            if not force and now - self._last_status_write < 0.5:
                return
            self.status.updated_at = now
            self._last_status_write = now
            _write_json_atomic(self.directory / "status.json", asdict(self.status))

    def write_json(self, name: str, value: Any) -> Path:
        path = self.directory / name
        with self._lock:
            _write_json_atomic(path, value)
        return path

    def close(self) -> None:
        with self._lock:
            self.write_status(force=True)
            for handle in (self._events, self._telemetry):
                try:
                    handle.close()
                except OSError:
                    pass


# --- Readers (status, replay, export, continue) -------------------------------

def list_runs(root: Path | None = None) -> list[Path]:
    base = root or RUNS_DIR
    if not base.is_dir():
        return []
    runs = [p for p in base.iterdir() if (p / "status.json").is_file()]
    return sorted(runs, key=lambda p: p.name)


def resolve_run(ref: str | None, root: Path | None = None) -> Path:
    """Resolve a run id, unique id prefix/suffix, path, or ``latest``."""
    runs = list_runs(root)
    if ref in (None, "", "latest"):
        if not runs:
            raise FileNotFoundError("no mission runs found under runs/")
        return runs[-1]
    path = Path(ref).expanduser()
    if path.is_dir() and (path / "status.json").is_file():
        return path
    matches = [p for p in runs if p.name == ref] or [p for p in runs if ref in p.name]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise FileNotFoundError(f"no mission run matches {ref!r}")
    raise LookupError(f"{ref!r} matches several runs: " + ", ".join(p.name for p in matches[-5:]))


def read_status(run_dir: Path) -> MissionStatus:
    return MissionStatus.from_dict(json.loads((run_dir / "status.json").read_text()))


def iter_jsonl(path: Path) -> Iterator[dict[str, Any]]:
    if not path.is_file():
        return
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue  # a run killed mid-write leaves a truncated last line


def read_events(run_dir: Path) -> list[Event]:
    return [Event.from_dict(raw) for raw in iter_jsonl(run_dir / "events.jsonl")]


def read_telemetry(run_dir: Path) -> list[dict[str, Any]]:
    return list(iter_jsonl(run_dir / "telemetry.jsonl"))
