#!/usr/bin/env python3
"""Run the dedicated KSC runway-09 HAC-to-touchdown live test."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    runner = root / "Tools" / "headless_flight.py"
    command = [
        sys.executable,
        str(runner),
        "--hac-only",
        "--live",
        "--no-auto-warp",
        *sys.argv[1:],
    ]
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
