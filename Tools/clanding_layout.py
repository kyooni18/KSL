#!/usr/bin/env python3
"""Resolve the canonical modular CLanding tree, or an explicitly selected legacy tree."""

from __future__ import annotations

import os
from pathlib import Path

ENV_NAME = "KSP_CLANDING_ROOT"
DEFAULT_ROOT = "CLanding"


def source_root(repo_root: Path) -> Path:
    raw = os.environ.get(ENV_NAME, DEFAULT_ROOT)
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = repo_root / candidate
    return candidate.resolve()


def source_file(repo_root: Path, name: str) -> Path:
    root = source_root(repo_root)
    direct = root / name
    if direct.is_file():
        return direct

    matches = [
        path for path in root.rglob(name)
        if path.is_file() and "build" not in path.relative_to(root).parts
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise FileNotFoundError(f"{name} was not found under selected CLanding root: {root}")
    raise RuntimeError(f"{name} is ambiguous under selected CLanding root {root}: {matches}")


def build_artifact(repo_root: Path, name: str = "landing_backend") -> Path:
    return source_root(repo_root) / "build" / name
