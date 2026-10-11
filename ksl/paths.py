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


def _main_checkout(root: Path) -> Path:
    """The primary checkout when ``root`` is a linked git worktree, else ``root``."""
    marker = root / ".git"
    if marker.is_file():
        text = marker.read_text(encoding="utf-8").strip()
        if text.startswith("gitdir:"):
            gitdir = Path(text.split(":", 1)[1].strip())
            if gitdir.parent.name == "worktrees" and gitdir.parent.parent.name == ".git":
                return gitdir.parent.parent.parent.resolve()
    return root


# Durable live-flight data (FlightLogs/, Runtime/Physics observation store).
# The native backend resolves it from KSP_LANDER_ROOT; a worktree must not
# start a fresh, empty aerodynamic evidence store of its own.
DATA_ROOT = Path(os.environ.get("KSL_DATA_ROOT") or _main_checkout(ROOT))


def repo_path(value: str | os.PathLike[str]) -> Path:
    """Resolve a profile path: absolute paths stay, relative ones are repo-relative."""
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path
