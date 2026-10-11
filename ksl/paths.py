"""Repository locations shared by every KSL component."""

from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(os.environ.get("KSL_ROOT") or Path(__file__).resolve().parents[1])
CONFIGURATION = ROOT / "Configuration"
PROFILES_DIR = CONFIGURATION / "missions"
RUNS_DIR = Path(os.environ.get("KSL_RUNS_DIR") or ROOT / "runs")
CLANDING = ROOT / "CLanding"
BACKEND = Path(os.environ.get("KSL_BACKEND") or CLANDING / "build" / "landing_backend")
SHUTTLESIM = ROOT / "ShuttleSim"
TOOLS = ROOT / "Tools"


def repo_path(value: str | os.PathLike[str]) -> Path:
    """Resolve a profile path: absolute paths stay, relative ones are repo-relative."""
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path
