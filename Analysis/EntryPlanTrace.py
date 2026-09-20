#!/usr/bin/env python3
"""Summarize MM304 planned-S-turn execution from a flight JSONL log.

The analyzer is intentionally standard-library only so it can be run beside the
flight backend without changing the runtime environment.  New instrumented logs
expose the persistent MM304 EntryControlPlan under guidanceState; older logs are
accepted but reported as lacking plan observability.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


def _number(value: Any) -> float | None:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        value = float(value)
        if math.isfinite(value):
            return value
    return None


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(q * len(ordered)) - 1))
    return ordered[index]


def _snapshot_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if record.get("recordType") == "snapshot":
                rows.append(record)
    return rows


def analyze(path: Path) -> dict[str, Any]:
    rows = _snapshot_rows(path)
    summary: dict[str, Any] = {
        "path": str(path),
        "snapshots": len(rows),
        "instrumented": False,
        "entrySnapshots": 0,
        "planCheckpoints": [],
        "reversalPullEarlierEvents": [],
        "reversalDelayViolations": [],
        "reversalSignViolations": [],
        "reversalClearedBeforeDue": [],
        "handoff": None,
    }
    if not rows:
        return summary

    instrumented = any("entryPlanValid" in (row.get("guidanceState") or {}) for row in rows)
    summary["instrumented"] = instrumented
    if not instrumented:
        summary["note"] = "Log predates MM304 planned-S-turn observability fields."
        return summary

    plan_ut_last: float | None = None
    scheduled_last = False
    reversal_abs_last: float | None = None
    reversal_sign_last: float | None = None
    previous_taem_owned = False
    roll_errors: list[float] = []
    aoa_errors: list[float] = []
    plan_command_bank_deltas: list[float] = []
    plan_command_aoa_deltas: list[float] = []

    for row in rows:
        telemetry = row.get("telemetry") or {}
        guidance = row.get("guidanceState") or {}
        command = row.get("command") or {}
        ut = _number(telemetry.get("ut"))
        if ut is None:
            continue

        entry_exec = guidance.get("entryExecutive") or {}
        entry_phase = entry_exec.get("phase")
        phase = row.get("phase")
        in_mm304 = phase == "Entry Energy" or (
            bool(entry_exec.get("initialized")) and not bool(entry_exec.get("complete"))
        )
        if in_mm304:
            summary["entrySnapshots"] += 1

        plan_valid = bool(guidance.get("entryPlanValid"))
        plan_ut = _number(guidance.get("entryPlanUT")) if plan_valid else None
        if plan_valid and plan_ut is not None and (plan_ut_last is None or abs(plan_ut - plan_ut_last) > 1e-6):
            summary["planCheckpoints"].append(
                {
                    "ut": ut,
                    "planUT": plan_ut,
                    "altitude": _number(telemetry.get("meanAltitude")),
                    "speed": _number(telemetry.get("trueAirSpeed")),
                    "range": _number(telemetry.get("rangeToSite")),
                    "crossTrack": _number(telemetry.get("runwayCrossTrack")),
                    "targetBank": _number(guidance.get("entryPlanTargetBank")),
                    "targetAoA": _number(guidance.get("entryPlanTargetAoA")),
                    "targetHeading": _number(guidance.get("entryPlanTargetHeading")),
                    "segmentRemaining": _number(guidance.get("entryPlanSegmentRemaining")),
                    "terminalReady": bool(guidance.get("entryPlanTerminalReady")),
                    "taemRangeError": _number(guidance.get("entryPlanTAEMRangeError")),
                    "taemSpeed": _number(guidance.get("entryPlanTAEMSpeed")),
                    "taemEnergyError": _number(guidance.get("entryPlanTAEMEnergyError")),
                    "predictedReversals": guidance.get("entryPlanPredictedReversals"),
                }
            )
            plan_ut_last = plan_ut

        scheduled = bool(guidance.get("entryReversalScheduled"))
        remaining = _number(guidance.get("entryReversalTimeRemaining")) if scheduled else None
        reversal_sign = _number(guidance.get("entryReversalSign")) if scheduled else None
        reversal_abs = ut + remaining if remaining is not None else None

        if scheduled and scheduled_last and reversal_abs is not None and reversal_abs_last is not None:
            delta = reversal_abs - reversal_abs_last
            if reversal_sign_last is not None and reversal_sign is not None and reversal_sign * reversal_sign_last < 0:
                summary["reversalSignViolations"].append(
                    {"ut": ut, "previousSign": reversal_sign_last, "newSign": reversal_sign}
                )
            elif delta > 0.5:
                summary["reversalDelayViolations"].append(
                    {"ut": ut, "previousEventUT": reversal_abs_last, "newEventUT": reversal_abs, "delay": delta}
                )
            elif delta < -0.5:
                summary["reversalPullEarlierEvents"].append(
                    {"ut": ut, "previousEventUT": reversal_abs_last, "newEventUT": reversal_abs, "advance": -delta}
                )
        elif not scheduled and scheduled_last and reversal_abs_last is not None and ut + 0.75 < reversal_abs_last:
            summary["reversalClearedBeforeDue"].append(
                {"ut": ut, "previousEventUT": reversal_abs_last, "earlyBy": reversal_abs_last - ut}
            )

        if scheduled:
            scheduled_last = True
            reversal_abs_last = reversal_abs
            reversal_sign_last = reversal_sign
        else:
            scheduled_last = False
            reversal_abs_last = None
            reversal_sign_last = None

        taem = guidance.get("taemExecutive") or {}
        taem_owned = bool(taem.get("ownershipLatched"))
        if taem_owned and not previous_taem_owned and summary["handoff"] is None:
            summary["handoff"] = {
                "ut": ut,
                "altitude": _number(telemetry.get("meanAltitude")),
                "speed": _number(telemetry.get("trueAirSpeed")),
                "range": _number(telemetry.get("rangeToSite")),
                "alongTrack": _number(telemetry.get("runwayAlongTrack")),
                "crossTrack": _number(telemetry.get("runwayCrossTrack")),
                "flightPathAngle": _number(telemetry.get("flightPathAngle")),
                "roll": _number(telemetry.get("roll")),
                "angleOfAttack": _number(telemetry.get("angleOfAttack")),
                "targetRoll": _number(command.get("targetRoll")),
                "targetAoA": _number(command.get("targetAoA")),
                "taemPhase": taem.get("phase"),
            }
        previous_taem_owned = taem_owned

        if in_mm304:
            actual_roll = _number(telemetry.get("roll"))
            actual_aoa = _number(telemetry.get("angleOfAttack"))
            command_roll = _number(command.get("targetRoll"))
            command_aoa = _number(command.get("targetAoA"))
            plan_bank = _number(guidance.get("entryPlanTargetBank")) if plan_valid else None
            plan_aoa = _number(guidance.get("entryPlanTargetAoA")) if plan_valid else None
            if actual_roll is not None and command_roll is not None:
                roll_errors.append(abs(actual_roll - command_roll))
            if actual_aoa is not None and command_aoa is not None:
                aoa_errors.append(abs(actual_aoa - command_aoa))
            if plan_bank is not None and command_roll is not None:
                plan_command_bank_deltas.append(abs(plan_bank - command_roll))
            if plan_aoa is not None and command_aoa is not None:
                plan_command_aoa_deltas.append(abs(plan_aoa - command_aoa))

    summary["tracking"] = {
        "rollAbsErrorP95Deg": _percentile(roll_errors, 0.95),
        "rollAbsErrorMaxDeg": max(roll_errors) if roll_errors else None,
        "aoaAbsErrorP95Deg": _percentile(aoa_errors, 0.95),
        "aoaAbsErrorMaxDeg": max(aoa_errors) if aoa_errors else None,
        "planToCommandBankDeltaP95Deg": _percentile(plan_command_bank_deltas, 0.95),
        "planToCommandAoADeltaP95Deg": _percentile(plan_command_aoa_deltas, 0.95),
    }
    summary["invariantViolations"] = (
        len(summary["reversalDelayViolations"])
        + len(summary["reversalSignViolations"])
        + len(summary["reversalClearedBeforeDue"])
    )
    return summary


def _fmt(value: Any, digits: int = 1) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def print_human(summary: dict[str, Any]) -> None:
    print(f"Entry plan trace: {summary['path']}")
    print(f"snapshots={summary['snapshots']} instrumented={str(summary['instrumented']).lower()}")
    if not summary["instrumented"]:
        print(summary.get("note", "No instrumented MM304 snapshots."))
        return
    print(
        f"MM304 snapshots={summary['entrySnapshots']} checkpoints={len(summary['planCheckpoints'])} "
        f"reversal-delay-violations={len(summary['reversalDelayViolations'])} "
        f"sign-violations={len(summary['reversalSignViolations'])} "
        f"early-clears={len(summary['reversalClearedBeforeDue'])}"
    )
    for i, cp in enumerate(summary["planCheckpoints"], 1):
        print(
            f"  plan#{i} UT {_fmt(cp['planUT'],2)} alt {_fmt(cp['altitude']/1000 if cp['altitude'] is not None else None,2)}km "
            f"V {_fmt(cp['speed'],0)} range {_fmt(cp['range']/1000 if cp['range'] is not None else None,1)}km "
            f"bank {_fmt(cp['targetBank'])} AoA {_fmt(cp['targetAoA'])} seg {_fmt(cp['segmentRemaining'])}s "
            f"TAEM dR {_fmt(cp['taemRangeError']/1000 if cp['taemRangeError'] is not None else None,1)}km"
        )
    handoff = summary.get("handoff")
    if handoff:
        print(
            f"handoff UT {_fmt(handoff['ut'],2)} alt {_fmt(handoff['altitude']/1000 if handoff['altitude'] is not None else None,2)}km "
            f"V {_fmt(handoff['speed'],0)} range {_fmt(handoff['range']/1000 if handoff['range'] is not None else None,1)}km "
            f"along {_fmt(handoff['alongTrack']/1000 if handoff['alongTrack'] is not None else None,1)}km "
            f"cross {_fmt(handoff['crossTrack']/1000 if handoff['crossTrack'] is not None else None,1)}km"
        )
    tracking = summary.get("tracking") or {}
    print(
        "tracking p95: "
        f"roll {_fmt(tracking.get('rollAbsErrorP95Deg'))}deg, "
        f"AoA {_fmt(tracking.get('aoaAbsErrorP95Deg'))}deg, "
        f"plan->command bank {_fmt(tracking.get('planToCommandBankDeltaP95Deg'))}deg, "
        f"AoA {_fmt(tracking.get('planToCommandAoADeltaP95Deg'))}deg"
    )
    if summary["reversalPullEarlierEvents"]:
        advances = [event["advance"] for event in summary["reversalPullEarlierEvents"]]
        print(f"reversal pulled earlier {len(advances)} time(s), max advance {max(advances):.2f}s")
    if summary["invariantViolations"]:
        print("INVARIANT VIOLATIONS:")
        for key in ("reversalDelayViolations", "reversalSignViolations", "reversalClearedBeforeDue"):
            for event in summary[key]:
                print(f"  {key}: {json.dumps(event, sort_keys=True)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--json", action="store_true", help="emit machine-readable summary")
    parser.add_argument(
        "--fail-on-invariant",
        action="store_true",
        help="exit 3 when a committed reversal is delayed/switched/cleared before due",
    )
    args = parser.parse_args()
    try:
        summary = analyze(args.log)
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print_human(summary)
    if args.fail_on_invariant and summary.get("invariantViolations", 0):
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
