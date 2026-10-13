"""Run-scoped operator requests and acknowledgements, usable across processes."""
from __future__ import annotations

import json
import os
import tempfile
import time
import uuid
from pathlib import Path

from .runlog import read_status

ACTIONS = ("abort", "execute-plan", "replan")


class Commands:
    def __init__(self, run_dir: Path):
        self.run_dir = Path(run_dir)
        self.directory = self.run_dir / "commands"

    def _write(self, request: dict) -> dict:
        self.directory.mkdir(exist_ok=True)
        fd, temporary = tempfile.mkstemp(dir=self.directory, suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as handle:
                json.dump(request, handle, allow_nan=False)
            os.replace(temporary, self.directory / f"{request['id']}.json")
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return request

    def submit(self, action: str, revision: str | None = None) -> dict:
        if action not in ACTIONS:
            raise ValueError(f"Unknown operator command: {action}")
        status = read_status(self.run_dir)
        if status.state != "running" or status.finished_at:
            raise ValueError("This run is no longer running")
        if time.time() - status.updated_at > 15:
            raise ValueError("Run status is stale; command delivery cannot be confirmed")
        if action != "abort" and status.operation_stage != "review":
            raise ValueError("This run is not awaiting deorbit plan review")
        if action == "execute-plan" and (not revision or revision != status.plan_revision):
            raise ValueError("Review the current plan before executing it")
        return self._write({"id": uuid.uuid4().hex, "action": action, "revision": revision,
                            "state": "requested", "message": "Waiting for controller", "time": time.time()})

    def pending(self) -> list[dict]:
        rows = []
        for path in self.directory.glob("*.json"):
            try:
                row = json.loads(path.read_text())
                if row.get("state") == "requested":
                    rows.append(row)
            except (OSError, ValueError):
                continue
        return sorted(rows, key=lambda row: row["time"])

    def update(self, request: dict, state: str, message: str) -> dict:
        return self._write({**request, "state": state, "message": message, "updatedAt": time.time()})

    def read(self, request_id: str) -> dict:
        if not request_id.isalnum():
            raise ValueError("Invalid command id")
        return json.loads((self.directory / f"{request_id}.json").read_text())
