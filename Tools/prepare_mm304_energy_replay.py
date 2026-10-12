#!/usr/bin/env python3
"""Extract frozen MM304 records for the native offline audit. No transport."""
import argparse
import copy
import hashlib
import json
from pathlib import Path


def merge(base, patch):
    for key, value in patch.items():
        if isinstance(value, dict):
            if not isinstance(base.get(key), dict):
                base[key] = {}
            merge(base[key], value)
        else:
            base[key] = value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    targets = [("ut", 13485.241)] + [("mach", m) for m in (2.85, 2.7, 2.5, 2.3, 2.16, 2.15)] + [("cross", c) for c in (-66000, -98000)]
    best = {}
    fields = {}
    header = None
    for line in args.source.open():
        record = json.loads(line)
        kind = record.get("recordType")
        if kind == "sessionStart":
            header = record
            continue
        if kind == "vehicleKeyframe":
            fields = copy.deepcopy(record["fields"])
        elif kind == "vehicleDelta":
            merge(fields, record.get("fields", {}))
        else:
            continue
        if not 22000 < fields.get("position", {}).get("altitude", 0) < 31000:
            continue
        for key, target in targets:
            value = {"ut": record["ut"], "mach": fields["aero"]["mach"], "cross": fields["guidance"]["runwayCrossTrack"]}[key]
            distance = abs(value - target)
            if (key, target) not in best or distance < best[key, target][0]:
                selected = {group: copy.deepcopy(fields[group]) for group in ("position", "motion", "attitude", "aero", "vehicle", "guidance", "physics")}
                best[key, target] = distance, {"ut": record["ut"], "fields": selected}
    if header is None or len(best) != len(targets):
        raise RuntimeError("recorded stream does not cover the requested audit states")
    samples = {record["ut"]: record for _, record in best.values()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"source": str(args.source), "sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(), "planet": header["planet"], "configuration": header["configuration"], "samples": sorted(samples.values(), key=lambda record: record["ut"])}, indent=2) + "\n")


if __name__ == "__main__":
    main()
