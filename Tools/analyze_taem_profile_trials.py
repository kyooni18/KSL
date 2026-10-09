#!/usr/bin/env python3
"""Summarize optional joint TAEM trial diagnostics; never changes admission."""
import argparse
import collections
import hashlib
import json
import math
import pathlib
import re


def analyze(text, target_speed=None, speed_tolerance=None):
    rows = []
    reasons = collections.Counter()
    combinations = collections.Counter()
    valid_profile_families = collections.Counter()
    for line in text.splitlines():
        if line.startswith("TAEM profile: ") and " valid=1 " in line:
            valid_profile_families[line.rsplit(" reason=", 1)[1]] += 1
        if not line.startswith("TAEM smooth trial: "):
            continue
        fields, reason = line.rsplit(" reason=", 1)
        row = {key: float(value) for key, value in re.findall(r"(\w+)=([^ ]+)", fields)}
        row["reason"] = reason
        rows.append(row)
        reasons[reason] += 1
        if not row["completed"]:
            continue
        rejected = [name for name, limit in (("steep", "slopeLimit"),
                    ("track", "trackLimit"), ("endpoint", "endpointLimit"))
                    if not math.isfinite(row[name]) or row[name] > row[limit]]
        combinations["+".join(rejected) or "none"] += 1
    completed = [row for row in rows if row["completed"]]
    # The closest joint flight minimizes its worst normalized departure, rather
    # than combining minima from mutually incompatible bow/route candidates.
    def departure(row):
        return max(row[name] / row[limit] for name, limit in
                   (("steep", "slopeLimit"), ("track", "trackLimit"),
                    ("endpoint", "endpointLimit")))
    closest = sorted(completed, key=departure)[:10]
    speed_ok = []
    if target_speed is not None and speed_tolerance is not None:
        speed_ok = [row for row in completed if
            abs(row["speed"] - target_speed) <= target_speed * speed_tolerance]
    for row in closest:
        row["maximumConstraintRatio"] = departure(row)
    # JSON null is explicit missing data, never a fabricated zero/pass.
    def clean(value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: clean(item) for key, item in value.items()}
        if isinstance(value, list):
            return [clean(item) for item in value]
        return value
    return clean({"classification": "offline native profile trial diagnostics; not flight qualification",
        "trialCount": len(rows), "completedCount": len(completed),
        "jointFeasibleCount": sum(bool(row["feasible"]) for row in rows),
        "terminationReasons": dict(reasons), "completedRejectionCombinations": dict(combinations),
        "closestJointTrials": closest,
        "validProfileFamilyCounts": dict(valid_profile_families),
        "targetSpeedMps": target_speed, "relativeSpeedTolerance": speed_tolerance,
        "speedCompatibleCount": len(speed_ok),
        "speedCompatibleJointFeasibleCount": sum(bool(row["feasible"]) for row in speed_ok),
        "closestSpeedCompatibleTrials": sorted(speed_ok, key=departure)[:10],
        "sampleSummaries": re.findall(r"^sample=.*$", text, re.M),
        "contractSummary": re.findall(r"^MM305 downstream-feasibility contract:.*$", text, re.M)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=pathlib.Path)
    parser.add_argument("--report", type=pathlib.Path, required=True)
    parser.add_argument("--target-speed", type=float)
    parser.add_argument("--speed-tolerance", type=float)
    args = parser.parse_args()
    data = args.log.read_bytes()
    if (args.target_speed is None) != (args.speed_tolerance is None):
        parser.error("target speed and relative tolerance must be supplied together")
    if args.target_speed is not None and (not math.isfinite(args.target_speed) or
            args.target_speed <= 0 or not math.isfinite(args.speed_tolerance) or
            args.speed_tolerance < 0):
        parser.error("target speed must be positive and tolerance nonnegative, both finite")
    report = analyze(data.decode(), args.target_speed, args.speed_tolerance)
    report["logSHA256"] = hashlib.sha256(data).hexdigest()
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
