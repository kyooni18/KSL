#!/usr/bin/env python3
"""MM304 inlet replay sweep; diagnostic acquisition, never live control."""
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main():
    results = []
    for flight in ("l5", "l7"):
        for distance in (96500, 60000):
            label = f"entry-{flight}-range-{distance}"
            args = [sys.executable, str(ROOT / "ShuttleSim/scripts/run_guidance.py"),
                    "--scenario", str(HERE / f"{flight}-mm304.ini"),
                    "--configuration", str(HERE / f"config-{distance}.json"),
                    "--attitude", str(ROOT / "Docs/Experiments/STS-Landing-20261010/"
                                      "Codex-Flare-Offline/inputs/att-live.ini"),
                    "--direct-control", "--engage", "engageReentry", "--stop-at-phase", "TAEM",
                    "--max-sim-time", "900", "--skip-build", "--no-mirror", "--quiet-progress",
                    "--label", label]
            env = dict(os.environ, KSP_LANDER_MODEL_PROFILE="sts-live-20261010",
                       KSP_LANDER_ENTRY_TRACE="1", KSP_LANDER_ROOT=str(ROOT))
            print("Starting", label, flush=True)
            with (HERE / f"{label}.log").open("w") as log:
                p = subprocess.run(args, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   timeout=600, check=False)
            results.append({"label": label, "exitCode": p.returncode, "args": args})
            (HERE / "runs.json").write_text(json.dumps(results, indent=2) + "\n")
            print("Finished", label, "exit", p.returncode, flush=True)


if __name__ == "__main__":
    main()
