"""Atomic metadata and lossless compression for a single simulator attempt."""
from __future__ import annotations

import gzip
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, allow_nan=False, separators=(",", ":"))
            stream.write("\n")
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def allocate_run(root: Path, label: str, stamp: str) -> Path:
    if not label or Path(label).name != label or label in (".", ".."):
        raise ValueError("run label must be one filename component")
    root.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=f"{label}-{stamp}-", dir=root))


def compress_recording(path: Path | None) -> Path | None:
    if path is None:
        return None
    destination = path.with_name(path.name + ".gz")
    if destination.exists():
        raise FileExistsError(destination)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as raw:
            temporary = Path(raw.name)
            with path.open("rb") as source, gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as output:
                shutil.copyfileobj(source, output)
        os.link(temporary,destination)  # Atomic publication without replacing an existing recording.
        path.unlink()
        return destination
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
