"""One mission controller per live kRPC endpoint, across CLI/TUI processes."""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from contextlib import contextmanager
from typing import Iterator

from .config import MissionConfig
from .paths import DATA_ROOT


class OwnershipError(RuntimeError):
    pass


def _path(config: MissionConfig):
    c = config["connection"]
    endpoint = f"{c['address']}:{c['rpcPort']}"
    key = hashlib.sha256(endpoint.encode()).hexdigest()[:16]
    return DATA_ROOT / "Runtime" / f"mission-{key}.lock"


def owner(config: MissionConfig) -> dict | None:
    path = _path(config)
    if not path.exists():
        return None
    with path.open("r+") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            try:
                return json.load(handle)
            except (ValueError, OSError):
                return {"run": "another KSL controller"}
        else:
            fcntl.flock(handle, fcntl.LOCK_UN)
            return None


@contextmanager
def claim(config: MissionConfig, run_dir) -> Iterator[None]:
    path = _path(config)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise OwnershipError("Another KSL controller owns this kRPC endpoint; watch or stop its run first") from exc
        handle.seek(0)
        handle.truncate()
        json.dump({"run": str(run_dir), "pid": os.getpid()}, handle)
        handle.flush()
        try:
            yield
        finally:
            handle.seek(0)
            handle.truncate()
            handle.flush()
            fcntl.flock(handle, fcntl.LOCK_UN)
