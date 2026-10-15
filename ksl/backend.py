"""Client for the native C landing backend's NDJSON operator protocol (v1).

The backend (``CLanding/build/landing_backend``) owns deorbit, entry (MM304),
TAEM (MM305), final approach, flare, touchdown, rollout guidance and the
atmospheric flight-control system. This client sends operator methods and reads
snapshots; separate Python probes only read state for held-plan validation.

Messages from the backend are one JSON object per line:
    {"type": "ready", "protocolVersion": 1, "configuration": ..., "snapshot": ...}
    {"type": "snapshot", "snapshot": {...}}
    {"type": "response", "id": "...", "ok": true|false, "result"|"error": ...}

This is the single blocking client shared by the mission controller, the CLI,
the TUI (via the controller) and ``Tools/headless_flight.py``.
"""

from __future__ import annotations

import json
import os
import selectors
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, IO

PROTOCOL_VERSION = 1
TERMINAL_PHASES = frozenset({"Complete", "Abort", "Fault"})

Snapshot = dict[str, Any]


class BackendError(RuntimeError):
    pass


class BackendClient:
    def __init__(self, executable: Path, snapshot_sink: Callable[[Snapshot], None] | None = None,
                 stderr_target: IO[Any] | int | None = None, env: dict[str, str] | None = None) -> None:
        executable = Path(executable)
        if not executable.is_file():
            raise BackendError(f"landing backend not built: {executable} (run `make backend`)")
        process_env = os.environ.copy()
        process_env.update(env or {})
        self.process = subprocess.Popen(
            [str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr_target,
            text=False, bufsize=0, env=process_env, start_new_session=True)
        if self.process.stdin is None or self.process.stdout is None:
            raise BackendError("could not open backend stdio pipes")
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)
        self.read_buffer = bytearray()
        self.next_id = 1
        self.latest_snapshot: Snapshot | None = None
        self.ready_message: dict[str, Any] | None = None
        self.snapshot_sink = snapshot_sink

    # --- low-level I/O --------------------------------------------------------
    def _read_message(self, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while b"\n" not in self.read_buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self.selector.select(remaining):
                raise TimeoutError("Timed out waiting for the landing backend")
            chunk = os.read(self.process.stdout.fileno(), 65536)
            if not chunk:
                raise BackendError(f"Landing backend exited unexpectedly with code {self.process.poll()}")
            self.read_buffer.extend(chunk)
        raw_line, _, remainder = self.read_buffer.partition(b"\n")
        self.read_buffer = bytearray(remainder)
        line = raw_line.decode("utf-8", errors="replace")
        try:
            message = json.loads(line)
        except json.JSONDecodeError as exc:
            raise BackendError(f"Landing backend emitted invalid JSON: {line[:240]!r}") from exc
        if message.get("type") == "ready":
            self.ready_message = message
            if message.get("protocolVersion") != PROTOCOL_VERSION:
                raise BackendError(f"unsupported backend protocol {message.get('protocolVersion')!r}")
        return message

    def _handle_snapshot(self, snapshot: Snapshot) -> None:
        self.latest_snapshot = snapshot
        if self.snapshot_sink is not None:
            self.snapshot_sink(snapshot)

    def pump(self, timeout: float) -> Snapshot | None:
        """Process one message (or time out quietly); return a snapshot if one arrived."""
        try:
            message = self._read_message(timeout)
        except TimeoutError:
            return None
        if message.get("type") == "snapshot":
            snapshot = message.get("snapshot") or {}
            self._handle_snapshot(snapshot)
            return snapshot
        return None

    # --- protocol ---------------------------------------------------------------
    def send(self, method: str, timeout: float = 30.0, **fields: Any) -> dict[str, Any]:
        """Send a method and wait for its response, handling snapshots meanwhile."""
        request_id = str(self.next_id)
        self.next_id += 1
        request = {"id": request_id, "method": method, **fields}
        try:
            self.process.stdin.write((json.dumps(request, separators=(",", ":")) + "\n").encode())
            self.process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise BackendError(f"landing backend is not accepting commands ({method}): {exc}") from exc
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Backend method {method} timed out")
            try:
                message = self._read_message(remaining)
            except TimeoutError as exc:
                raise TimeoutError(f"Backend method {method} timed out (request {request_id})") from exc
            if message.get("type") == "snapshot":
                self._handle_snapshot(message.get("snapshot") or {})
                continue
            if message.get("type") != "response" or str(message.get("id")) != request_id:
                continue
            if not message.get("ok"):
                raise BackendError(str(message.get("error") or f"Backend method {method} failed"))
            return message.get("result") or {}

    def read_until(self, predicate: Callable[[Snapshot], bool], timeout: float) -> Snapshot:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Timed out waiting for the requested backend state")
            message = self._read_message(remaining)
            if message.get("type") == "snapshot":
                snapshot = message.get("snapshot") or {}
                self._handle_snapshot(snapshot)
                if predicate(snapshot):
                    return snapshot

    def engage(self, method: str, timeout: float = 180.0,
               acknowledgement_timeout: float = 15.0) -> Snapshot:
        """Require a fresh engagement decision, including snapshots before the reply.

        Native commands publish their snapshot before the method response. A
        rejected command can then remain quiet while KSP is paused; discarding
        that snapshot loses the actual rejection and manufactures a timeout.
        Successful orbital engagement may likewise enter a blocking coast warp.
        """
        before = self.latest_snapshot
        self.send(method, timeout=timeout)

        def decided(snapshot: Snapshot) -> bool:
            return (snapshot.get("automationEngaged") is True
                    or snapshot.get("phase") in TERMINAL_PHASES
                    or bool(snapshot.get("lastError") or snapshot.get("warningMessage")))

        armed = self.latest_snapshot
        if armed is None or armed is before or not decided(armed):
            try:
                armed = self.read_until(decided, acknowledgement_timeout)
            except TimeoutError as exc:
                latest = self.latest_snapshot or {}
                detail = (latest.get("lastError") or latest.get("warningMessage")
                          or latest.get("statusMessage") or "no decision snapshot")
                raise TimeoutError(f"{method} did not confirm engagement: {detail}") from exc
        if armed.get("automationEngaged") is not True or armed.get("phase") in TERMINAL_PHASES:
            detail = (armed.get("lastError") or armed.get("warningMessage")
                      or armed.get("statusMessage") or "automation was not armed")
            raise BackendError(f"{method} rejected: {detail}")
        return armed

    # --- lifecycle --------------------------------------------------------------
    def close(self) -> None:
        try:
            self.selector.close()
        except Exception:
            pass
        for stream in (self.process.stdin, self.process.stdout):
            if stream is not None:
                try:
                    stream.close()
                except (BrokenPipeError, OSError):
                    pass

    def shutdown(self, timeout: float = 8.0) -> None:
        """Ask for the native safe disconnect, then make sure the process is gone.

        The backend's shutdown path disengages autopilot/SAS, neutralizes axes,
        throttle and wheel steering, and restores engines it activated.
        """
        if self.process.poll() is None:
            try:
                self.send("shutdown", timeout=timeout)
            except (BackendError, TimeoutError, OSError):
                pass
        self.close()
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)


def validate_plan_qualification(plan: dict[str, Any], require_strict: bool,
                                allow_unqualified: bool = True) -> str | None:
    """Report prediction quality without conflating it with execution ability.

    Strict missions require entry-stage and full-route qualification. An
    best-effort mission may accept a candidate without
    robustness or native Final proof and receives a diagnostic warning.
    """
    if not plan:
        raise BackendError("Deorbit planner produced no executable candidate")
    if plan.get("executionQualified") is not True:
        if require_strict or not allow_unqualified:
            raise BackendError("Deorbit plan failed entry-stage qualification; execution is blocked by explicit policy")
        return ("Deorbit entry-stage qualification failed; proceeding with the best "
                "available physical candidate in degraded guidance mode (not certified)")
    if plan.get("targetCaptureAchieved") is not True:
        if require_strict:
            raise BackendError("Deorbit plan failed required strict full-route qualification; execution is blocked")
        return ("Deorbit has no certified MM305/Final runway route; proceeding with "
                "entry-stage-qualified guidance and in-flight recovery attempts")
    return None


def snapshot_summary(snapshot: Snapshot) -> dict[str, Any]:
    """Flatten the backend snapshot fields the mission telemetry record keeps."""
    t = snapshot.get("telemetry") or {}
    cmd = snapshot.get("command") or {}
    keys = {"ut": "ut", "altitude": "meanAltitude", "speed": "trueAirSpeed",
            "verticalSpeed": "verticalSpeed", "mach": "mach", "dynamicPressure": "dynamicPressure",
            "angleOfAttack": "angleOfAttack", "roll": "roll", "heading": "heading",
            "flightPathAngle": "flightPathAngle", "autopilotError": "autopilotError",
            "rangeToSite": "rangeToSite",
            "runwayAlongTrack": "runwayAlongTrack", "runwayCrossTrack": "runwayCrossTrack",
            "latitude": "latitude", "longitude": "longitude", "gForce": "gForce",
            "situation": "vesselSituation"}
    out = {name: t.get(src) for name, src in keys.items() if t.get(src) is not None}
    out.update({k: cmd.get(k) for k in ("targetAoA", "targetRoll", "targetHeading", "targetThrottle")
                if cmd.get(k) is not None})
    for key in ("command", "guidanceState", "decisionTrace"):
        if isinstance(snapshot.get(key), dict):
            out[key] = dict(snapshot[key])
    for key in ("statusMessage", "warningMessage", "decisionReason", "automationEngaged", "paused"):
        if key in snapshot:
            out[key] = snapshot[key]
    if isinstance(snapshot.get("referenceTrajectory"), list):
        out["referenceTrajectory"] = snapshot["referenceTrajectory"]
    out["backendPhase"] = snapshot.get("phase")
    return out
