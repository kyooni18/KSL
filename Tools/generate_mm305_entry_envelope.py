#!/usr/bin/env python3
"""Materialize the versioned, physically consistent MM305 entry-state set.

This only writes ShuttleSim scenario INI files. Flight execution is separate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from generate_mm305_condition_matrix import compile_case, load_ini


ROOT = Path(__file__).resolve().parents[1]
DEFINITION = ROOT / "ShuttleSim/scenarios/mm305-entry-envelope.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    definition = json.loads(DEFINITION.read_text(encoding="utf-8"))
    base_path = ROOT / definition["base"]
    base = load_ini(base_path)
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    cases = []
    for case in definition["cases"]:
        destination = output / f"mm305-hac-envelope-{case['name']}.ini"
        if case.get("use_base"):
            destination.write_text(base_path.read_text(encoding="utf-8"), encoding="utf-8")
            detail = {"scenario": str(destination), "source": str(base_path)}
        else:
            detail = compile_case(
                base, destination, case["heading"], case["altitude"],
                case["speed"], case["fpa"], case.get("east_offset_m", 0.0),
                case.get("north_offset_m", 0.0))
        cases.append({"name": case["name"], **detail})
    manifest = {"definition": str(DEFINITION), "base": str(base_path), "cases": cases}
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
