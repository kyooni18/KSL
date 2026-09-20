#!/usr/bin/env python3
"""Summarize recorded TAEM decisions; does not reconstruct live guidance inputs."""
import argparse
import json
from collections import Counter
from pathlib import Path


def summarize(path):
    phases = Counter()
    reasons = Counter()
    reasons_by_phase = {}
    first = None
    committed = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        phase = row.get("phase")
        geometry = row.get("terminalGeometry") or {}
        reason = geometry.get("taemTerminalBlockReason") or "unreported"
        if phase is not None:
            phases[phase] += 1
            reasons_by_phase.setdefault(phase, Counter())[reason] += 1
        if phase != "taem":
            continue
        reasons[reason] += 1
        committed += geometry.get("terminalCommitted") is True
        if first is None:
            first = {"ut": row.get("ut"), "tickSequence": row.get("tickSequence"),
                     "terminalGeometry": geometry}
    return {"source": str(path), "phases": dict(phases),
            "blockReasonsByPhase": {phase: dict(counts)
                                    for phase, counts in reasons_by_phase.items()},
            "taemBlockReasons": dict(reasons), "taemCommittedSamples": committed,
            "firstTaemSample": first,
            "limitation": "Candidate fields are forecasts, not live telemetry. "
                          "Block reasons outside TAEM may be default or retained state. "
                          "An invalid contract alone does not identify its cause."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("planner_jsonl", type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(args.planner_jsonl), indent=2))
