#!/usr/bin/env python3
"""Add one unqualified STS Mach-2 API polar to an opt-in aero table."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from build_sts_api_incidence_experiment import bilinear, linear

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--baseline", type=Path, default=ROOT / "ShuttleSim/reference-model/experiments/sts_api_incidence.csv")
    ap.add_argument("--crosscheck", type=Path, default=ROOT / "Docs/Experiments/STS-Entry-20261009/archived-mach2-polar-crosscheck.json")
    ap.add_argument("--grid", type=Path, default=ROOT / "Docs/Experiments/STS-Entry-20261009/live-aerodynamic-wrench-grid.json")
    ap.add_argument("--table", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    args = ap.parse_args()
    source = json.loads(args.grid.read_text())
    extracted = json.loads(args.crosscheck.read_text())
    if source["status"] != "complete" or source["provenance"]["vehicleModelId"] != "STS" or source["liveStateBefore"]["vessel"] != "STS":
        raise ValueError("requires completed exact original-STS source")
    if sha(args.grid) != extracted["sourceSHA256"]:
        raise ValueError("extracted polar does not match archived source hash")
    cells = sorted(extracted["cells"], key=lambda cell: cell["alphaDeg"])
    if [cell["alphaDeg"] for cell in cells] != [0, 5, 10, 18, 28] or any(cell["mach"] != 2 for cell in cells):
        raise ValueError("requires the five source Mach-2 incidence points")
    with args.baseline.open() as stream:
        rows = list(csv.DictReader(line for line in stream if not line.startswith("#")))
    baseline = {(float(row["mach"]), float(row["alpha_deg"])):
                (float(row["cl"]), float(row["cd"])) for row in rows}
    ms = sorted({m for m, a in baseline})
    alphas = sorted({a for m, a in baseline})
    if 2 in ms:
        raise ValueError("baseline already has a Mach-2 row; avoid replacing an existing anchor")
    polar = {cell["alphaDeg"]: (cell["clNormalMagnitude"], cell["cd"]) for cell in cells}
    if not all(math.isfinite(v) and v > 0 for pair in polar.values() for v in pair):
        raise ValueError("requires finite positive source coefficients")
    output = dict(baseline)
    for a in alphas:
        pair = bilinear(baseline, ms, alphas, 2, a)
        if 0 <= a <= 28:
            aa = sorted(polar)
            k = max(0, min(len(aa)-2, sum(v <= a for v in aa)-1))
            pair = tuple(linear(a, aa[k], aa[k+1], polar[aa[k]][j], polar[aa[k+1]][j]) for j in range(2))
        output[2, a] = pair
    assert all(output[key] == value for key, value in baseline.items())
    with args.table.open("w") as stream:
        stream.write("# OFFLINE STS Mach2 API polar experiment; high-altitude hypothetical forces; zero certified cells.\n")
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["mach", "alpha_deg", "cl", "cd"])
        for (m, a), (cl, cd) in sorted(output.items()):
            writer.writerow([f"{m:.8f}", f"{a:g}", f"{cl:.8f}", f"{cd:.8f}"])
    report = {"classification": "unqualified offline STS Mach2 polar experiment; not terminal validation",
        "certifiedCells": 0, "sourceSHA256": sha(args.grid), "crosscheckSHA256": sha(args.crosscheck),
        "baselineSHA256": sha(args.baseline), "tableSHA256": sha(args.table),
        "sourceQualification": source["qualification"], "sourceState": source["liveStateBefore"],
        "anchors": cells, "originalNodesPreserved": len(baseline),
        "insertedMach": 2, "modifiedAlphaBandDeg": [0, 28],
        "interpolationBlendMachExtent": [max(m for m in ms if m < 2), min(m for m in ms if m > 2)],
        "limitations": extracted["limitations"] + ["Only one Mach row inserted; interpolation between neighboring original Mach rows is provisional.",
            "Outside the alpha band, the inserted row reproduces the old interpolant. All original nodes remain unchanged.",
            "Low-altitude aero, subsonic aero and borrowed STS-N attitude dynamics remain unvalidated."]}
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Inserted one Mach-2 row; preserved {len(baseline)} original nodes.")


if __name__ == "__main__":
    main()
