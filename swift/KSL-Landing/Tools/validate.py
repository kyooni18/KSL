#!/usr/bin/env python3
"""Reproducible, serial offline qualification matrix for the Swift executable.

Python orchestrates tests and packages measurements; every physics/guidance/
control step runs in the native Swift executable. No external services used.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
import pathlib
import platform
import subprocess
import time

CASES = [
    ("nominal-final", [], True),
    ("nominal-entry", ["--entry"], True),
    ("entry-heading-eight", ["--entry", "--heading-error", "8"], True),
    ("required-spiral", ["--along", "-24000", "--cross", "4000", "--height", "12000", "--speed", "220", "--heading-error", "-20", "--flight-path", "-24", "--spiral-only"], True),
    ("entry-downrange-plus100km", ["--entry", "--along", "-550000"], None),
    ("entry-crossrange15km", ["--entry", "--cross", "15000", "--heading-error", "3"], None),
    ("entry-speed-minus50", ["--entry", "--speed", "1850"], None),
    ("entry-speed-plus50", ["--entry", "--speed", "1950"], None),
    ("entry-drag-plus10pct", ["--entry", "--drag-scale", "1.1"], None),
    ("entry-actuator-minus35pct", ["--entry", "--actuator-scale", "0.65"], None),
    ("final-lift-minus20pct", ["--lift-scale", "0.8"], True),
    ("final-drag-plus20pct", ["--drag-scale", "1.2"], None),
    ("final-drag-minus20pct", ["--drag-scale", "0.8"], None),
    ("final-density-plus10pct", ["--density-scale", "1.1"], None),
    ("final-crosswind5mps", ["--wind-north", "5"], None),
    ("low-energy-unreachable", ["--along", "-120000", "--height", "1000", "--speed", "70", "--flight-path", "-20", "--duration", "10"], False),
    ("historical-entry-20261028", ["--scenario", "Fixtures/sts-kerbin-20261028b-live-entry-72km.ini"], None),
    ("historical-polar-20261029", ["--scenario", "Fixtures/sts-kerbin-20261029-live-entry-72km.ini"], None),
    ("archival-heavy-final", ["--scenario", "Fixtures/final-ideal-3p5km-v190-fpa20.ini"], None),
]


def runway_coordinates(position, runway, radius):
    latitude, longitude, heading = runway["latitude"], runway["longitude"], runway["heading"]
    up = [math.cos(latitude)*math.cos(longitude), math.cos(latitude)*math.sin(longitude), math.sin(latitude)]
    north = [-math.sin(latitude)*math.cos(longitude), -math.sin(latitude)*math.sin(longitude), math.cos(latitude)]
    east = [-math.sin(longitude), math.cos(longitude), 0]
    forward = [north[i]*math.cos(heading)+east[i]*math.sin(heading) for i in range(3)]
    right = [-north[i]*math.sin(heading)+east[i]*math.cos(heading) for i in range(3)]
    p = [position[k] for k in "xyz"]
    length = math.sqrt(sum(v*v for v in p)); unit = [v/length for v in p]
    angle = math.acos(max(-1, min(1, sum(a*b for a, b in zip(unit, up)))))
    scale = radius*angle/math.sin(angle) if angle > 1e-10 else radius
    return (sum(a*b for a, b in zip(unit, forward))*scale,
            sum(a*b for a, b in zip(unit, right))*scale,
            length-radius-runway["elevation"])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", default=".build/release/ksl-landing")
    parser.add_argument("--output", default="Validation")
    args = parser.parse_args()
    output = pathlib.Path(args.output)
    for child in ["cases", "logs", "scenarios", "plans"]:
        (output/child).mkdir(parents=True, exist_ok=True)
    executable = str(pathlib.Path(args.executable).resolve())
    config = json.loads(subprocess.check_output([executable, "configuration"]))
    (output/"configuration.json").write_text(json.dumps(config, indent=2)+"\n")
    summaries, gates = [], []
    for name, parameters, expectation in CASES:
        scenario_file = output/"scenarios"/(name+".json")
        result_file = output/"cases"/(name+".json")
        log_file = output/"logs"/(name+".jsonl")
        scenario = json.loads(subprocess.check_output([executable, "scenario", *parameters]))
        if "--spiral-only" in parameters:
            scenario["configuration"]["guidance"]["allowDirectTerminal"] = False
        scenario_file.write_text(json.dumps(scenario, indent=2)+"\n")
        started = time.monotonic()
        subprocess.run([executable, "simulate", *parameters, "--output", str(result_file),
                        "--log", str(log_file), "--log-every", "1" if name == "nominal-final" else "10"], check=True)
        result = json.loads(result_file.read_text())
        touchdown = result.get("touchdown", {})
        along, cross, height = runway_coordinates(result["finalState"]["position"], scenario["configuration"]["runway"], scenario["configuration"]["planet"]["radius"])
        plan = result.get("terminalPlan")
        if plan:
            (output/"plans"/(name+".json")).write_text(json.dumps(plan, indent=2)+"\n")
        else:
            (output/"plans"/(name+".json")).unlink(missing_ok=True)
        speed = math.sqrt(sum(v*v for v in result["finalState"]["velocity"].values()))
        row = {"case": name, "outcome": result["outcome"], "elapsedSimulationSeconds": result["elapsed"],
               "touchdownAlongMeters": touchdown.get("along"), "touchdownCrossMeters": touchdown.get("cross"),
               "touchdownHorizontalMps": touchdown.get("horizontalSpeed"), "touchdownSinkMps": -touchdown["verticalSpeed"] if touchdown else None,
               "touchdownPitchDegrees": math.degrees(touchdown["pitch"]) if touchdown else None,
               "stopAlongMeters": along if result["outcome"] == "complete" else None,
               "finalSpeedMps": speed, "maximumQPa": result["maximumDynamicPressure"],
               "maximumNormalG": result["maximumLoadFactor"], "saturatedControlFraction": result["saturatedControlFraction"],
               "maximumComputationMilliseconds": result["maximumComputationMilliseconds"],
               "meanComputationMilliseconds": result["meanComputationMilliseconds"],
               "spiralFlown": "headingAlignment" in result["phases"], "phases": result["phases"],
               "nominalPlanPassesBothStressCases": all(x["feasible"] for x in plan["stressCases"]) if plan else None,
               "constraintOccurrences": result["constraintOccurrences"], "violations": result["violations"],
               "lastPlanningRejections": result["lastPlanningRejections"],
               "wallSeconds": round(time.monotonic()-started, 4), "arguments": parameters,
               "resultSHA256": digest(result_file)}
        summaries.append(row)
        if expectation is not None:
            passed = (result["outcome"] == "complete") == expectation
            if name in {"entry-heading-eight", "required-spiral"}:
                passed = passed and row["spiralFlown"]
            gates.append({"case": name, "passed": passed})
        if name == "nominal-final":
            replay = subprocess.run([executable, "replay", "--input", str(log_file), "--verify-commands", "--output", str(output/"replay.json")], check=True)
        with log_file.open("rb") as source, gzip.GzipFile(str(log_file)+".gz", "wb", mtime=0) as dest:
            while chunk := source.read(1024*1024):
                dest.write(chunk)
        log_file.unlink()
        print(f"{name}: {result['outcome']}, touchdown={touchdown.get('along')}, spiral={row['spiralFlown']}", flush=True)
    source_paths = sorted([Path for folder in ["Sources", "Tests"] for Path in pathlib.Path(folder).rglob("*.swift")])
    source_paths.append(pathlib.Path("Package.swift"))
    source_hashes = {str(path): digest(path) for path in source_paths}
    report = {"evidence": "offline-model-only; no live KSP success", "platform": platform.platform(),
              "architecture": platform.machine(), "executableSHA256": digest(pathlib.Path(executable)),
              "sourceSHA256": source_hashes,
              "cases": summaries, "regressionGates": gates,
              "passedLandingCases": sum(x["outcome"] == "complete" for x in summaries), "totalCases": len(summaries),
              "scope": "Deterministic bounded matrix, not a statistical reliability estimate or held-out flight qualification"}
    (output/"matrix.json").write_text(json.dumps(report, indent=2)+"\n")
    scalar_fields = [key for key in summaries[0] if key not in {"arguments", "phases", "constraintOccurrences", "violations", "lastPlanningRejections"}]
    with (output/"matrix.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=scalar_fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(summaries)
    if not all(x["passed"] for x in gates):
        raise SystemExit("A mandatory regression gate failed; inspect matrix.json")


if __name__ == "__main__":
    main()
