#!/usr/bin/env python3
"""Closed-loop parity gate: fly the same ShuttleSim scenarios with two landing backends.

The simulator is deterministic, so identical guidance commands produce a
byte-identical simulator telemetry stream (only the wall-clock `sim_rate` field
is dropped).  Any behavioural difference between two backends therefore shows
up as a different flight.

    Validation/BackendParity.py REFERENCE_BUILD_DIR CANDIDATE_BUILD_DIR [scenario[:engage[:max-sim-seconds]] ...]

Build directories contain a `landing_backend` executable.
"""
from __future__ import annotations

import gzip
import hashlib
import json
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN_GUIDANCE = ROOT / "ShuttleSim/scripts/run_guidance.py"
RUNS = ROOT / "ShuttleSim/runs"

DEFAULT_MATRIX = [
    ("final-ideal-3p5km-v180-fpa16", "engageFinalTest", ""),
    ("sts-final-d150", "engageFinalTest", ""),
    ("mm305-hac-ideal-p0", "engageHACTest", "20"),
    ("mm304-hac-interface", "engageReentry", "20"),
]
_SIM_RATE = re.compile(rb'"sim_rate":[0-9.eE+-]+,')


def fly(scenario: str, engage: str, max_sim: str, build: Path, label: str) -> tuple[str, str, int, dict]:
    proc = subprocess.run(
        [sys.executable, str(RUN_GUIDANCE), "--scenario", str(ROOT / "ShuttleSim/scenarios" / f"{scenario}.ini"),
         "--engage", engage, "--skip-build", "--no-mirror", "--quiet-progress",
         "--label", label, "--backend-build-dir", str(build)]
        + (["--max-sim-time", max_sim] if max_sim else []),
        capture_output=True, text=True, cwd=ROOT, check=False)
    last = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else "{}"
    try:
        summary = json.loads(last)
    except ValueError:
        raise SystemExit(f"{scenario}: runner failed\n{proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")
    sim = Path(summary["sim"])
    digest = hashlib.sha256()
    lines = 0
    with gzip.open(sim, "rb") as handle:
        for line in handle:
            digest.update(_SIM_RATE.sub(b"", line))
            lines += 1
    return scenario, digest.hexdigest(), lines, summary


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    reference, candidate = Path(argv[1]).resolve(), Path(argv[2]).resolve()
    matrix = [tuple((item.split(":") + ["engage", ""][len(item.split(":")) - 1:])[:3]) for item in argv[3:]] or DEFAULT_MATRIX
    failures = 0
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs = [(s, e, pool.submit(fly, s, e, m, reference, "parity-ref"), pool.submit(fly, s, e, m, candidate, "parity-new"))
                for s, e, m in matrix]
        for scenario, engage, ref_job, new_job in jobs:
            _, ref_hash, ref_lines, ref_summary = ref_job.result()
            _, new_hash, new_lines, new_summary = new_job.result()
            same = ref_hash == new_hash
            failures += not same
            print(f"{'OK  ' if same else 'DIFF'} {scenario} [{engage}] lines {ref_lines}/{new_lines}"
                  f" success={ref_summary.get('success')}/{new_summary.get('success')}")
    print("Backend parity passed." if not failures else f"{failures} scenario(s) differ.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
