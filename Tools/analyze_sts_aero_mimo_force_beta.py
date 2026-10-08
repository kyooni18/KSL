#!/usr/bin/env python3
"""Summarize static force and sideslip response in the retained STS aero grid.

This consumes only the saved hypothetical KSP wrench results. It makes no
connection to KSP and does not turn simulation outputs into flight evidence.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-mimo-envelope-grid.json"
OUTPUT = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-mimo-force-beta-analysis.json"


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent,
                                     prefix=path.name + ".", suffix=".tmp",
                                     delete=False) as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def central_difference(plus: list[float], minus: list[float], span: float) -> list[float]:
    return [(float(p) - float(m)) / span for p, m in zip(plus, minus)]


def cell_key(cell: dict) -> tuple:
    return (float(cell["requestedAltitudeM"]), cell["machLabel"],
            float(cell["angleOfAttackDeg"]), float(cell["sideslipDeg"]),
            float(cell["bankDeg"]))


def main() -> int:
    source_bytes = SOURCE.read_bytes()
    source = json.loads(source_bytes)
    cells = source.get("cells", [])
    if source.get("qualification", {}).get("qualified") is not False:
        raise RuntimeError("Refusing to analyze an input not explicitly marked unqualified")
    if source.get("classification") != "UNQUALIFIED_DIAGNOSTIC_SIMULATION":
        raise RuntimeError("Unexpected source classification")

    index = {cell_key(cell): cell for cell in cells}
    if len(index) != len(cells):
        raise RuntimeError("Duplicate grid coordinates")

    def get(altitude: float, mach_label, aoa: float, beta: float, bank: float) -> dict:
        key = (float(altitude), mach_label, float(aoa), float(beta), float(bank))
        try:
            return index[key]
        except KeyError as exc:
            raise RuntimeError(f"Missing grid cell {key}") from exc

    static_beta_rows = []
    coordinate_groups: dict[tuple, list[dict]] = {}
    for cell in cells:
        coordinate_groups.setdefault(
            (float(cell["requestedAltitudeM"]), cell["machLabel"],
             float(cell["angleOfAttackDeg"]), float(cell["bankDeg"])), []
        ).append(cell)

    for altitude, mach, aoa, bank in sorted(coordinate_groups,
            key=lambda k: (k[0], str(k[1]), k[2], k[3])):
        lo = get(altitude, mach, aoa, -5.0, bank)["wrenches"]["zero"]
        center = get(altitude, mach, aoa, 0.0, bank)["wrenches"]["zero"]
        hi = get(altitude, mach, aoa, 5.0, bank)["wrenches"]["zero"]
        static_beta_rows.append({
            "requestedAltitudeM": altitude,
            "machLabel": mach,
            "angleOfAttackDeg": aoa,
            "bankDeg": bank,
            "centerSideslipDeg": 0.0,
            "forceNBodyXYZAtZeroSideslip": center["forceNBodyXYZ"],
            "torqueNmBodyXYZAtZeroSideslip": center["torqueNmBodyXYZ"],
            "dForceNBodyXYZdSideslipDeg": central_difference(
                hi["forceNBodyXYZ"], lo["forceNBodyXYZ"], 10.0),
            "dTorqueNmBodyXYZdSideslipDeg": central_difference(
                hi["torqueNmBodyXYZ"], lo["torqueNmBodyXYZ"], 10.0),
            "rateDampingDerivativeMatrixNmPerRadPerSecondAtZeroSideslip":
                get(altitude, mach, aoa, 0.0, bank)[
                    "rateDampingDerivativeMatrixNmPerRadPerSecond"],
        })

    bank_groups: dict[tuple, list[dict]] = {}
    for cell in cells:
        bank_groups.setdefault(
            (float(cell["requestedAltitudeM"]), cell["machLabel"],
             float(cell["angleOfAttackDeg"]), float(cell["sideslipDeg"])), []
        ).append(cell)

    force_bank_relative_spreads = []
    torque_bank_relative_spreads = []
    for group in bank_groups.values():
        if len(group) < 2:
            continue
        force_vectors = [cell["wrenches"]["zero"]["forceNBodyXYZ"] for cell in group]
        torque_vectors = [cell["wrenches"]["zero"]["torqueNmBodyXYZ"] for cell in group]
        force_span = max(vector_norm([a - b for a, b in zip(left, right)])
                         for left in force_vectors for right in force_vectors)
        torque_span = max(vector_norm([a - b for a, b in zip(left, right)])
                          for left in torque_vectors for right in torque_vectors)
        force_scale = max(vector_norm(v) for v in force_vectors)
        torque_scale = max(vector_norm(v) for v in torque_vectors)
        force_bank_relative_spreads.append(force_span / max(force_scale, 1.0))
        torque_bank_relative_spreads.append(torque_span / max(torque_scale, 1.0))

    matched = next((row for row in static_beta_rows
        if row["requestedAltitudeM"] == 38000.0 and row["machLabel"] == "trace"
        and row["angleOfAttackDeg"] == 28.0 and row["bankDeg"] == -40.0), None)
    if matched is None:
        raise RuntimeError("The matched 38 km / trace Mach / 28 degree / -40 degree cell is missing")

    report = {
        "status": "complete",
        "classification": "UNQUALIFIED_DERIVED_SIMULATION_ANALYSIS",
        "purpose": "Quantify static sideslip force/moment slopes and body-coordinate bank invariance from the retained original-STS KSP wrench grid.",
        "qualification": {
            "qualified": False,
            "inputGridIsMeasuredFlightData": False,
            "shadowGateEvidence": False,
            "flightControlOrActuation": False,
            "landingOrRolloutEvidence": False,
            "note": "Every input wrench is a hypothetical simulate_aerodynamic_wrench_at result. This analysis makes no KSP calls and is not a flight measurement or qualification result.",
        },
        "provenance": {
            "sourceGrid": str(SOURCE.relative_to(ROOT)),
            "sourceGridSHA256": hashlib.sha256(source_bytes).hexdigest(),
            "sourceGridGitHead": source.get("provenance", {}).get("gitHead"),
            "analysisScript": str(Path(__file__).resolve().relative_to(ROOT)),
            "analysisScriptSHA256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "newKSPConnectionOrSimulationCalls": False,
            "sourceClassification": source.get("classification"),
            "checkpointName": source.get("provenance", {}).get("checkpointName"),
            "checkpointSHA256": source.get("provenance", {}).get("checkpointSHA256Before"),
        },
        "coordinatePolicy": {
            "forceAndTorque": "Body XYZ, as inverse-rotated from the API reference-frame outputs by the original grid probe.",
            "rateDerivativeMatrix": source.get("gridDefinition", {}).get("derivativeMatrixOrientation"),
            "sideslipSlopes": "Centered finite difference from hypothetical beta=-5 and +5 degree calls, per degree.",
            "bankComparison": "Relative spread among body-coordinate zero-rate wrenches at bank -40, 0, and +40 degrees. Body-frame invariance does not mean world-frame lateral force is invariant.",
        },
        "coverage": {
            "sourceGridCellCount": len(cells),
            "staticSideslipSlopeRows": len(static_beta_rows),
            "targetAltitudesM": source.get("gridDefinition", {}).get("targetAltitudesM"),
            "machPlan": source.get("gridDefinition", {}).get("machPlan"),
            "angleOfAttackDeg": source.get("gridDefinition", {}).get("angleOfAttackDeg"),
            "sideslipDeg": source.get("gridDefinition", {}).get("sideslipDeg"),
            "bankDeg": source.get("gridDefinition", {}).get("bankDeg"),
        },
        "matchedShadowTraceCell": matched,
        "bodyCoordinateBankSpread": {
            "comparisonGroupCount": len(force_bank_relative_spreads),
            "maximumRelativeForceVectorSpread": max(force_bank_relative_spreads),
            "maximumRelativeTorqueVectorSpread": max(torque_bank_relative_spreads),
            "interpretation": "For a fixed hypothetical position, speed, Mach, AoA, sideslip, and neutral surfaces, the simulated wrench is nearly invariant in vessel-body coordinates as the body is rolled. The inertial/world projection changes with attitude and must be transformed before comparing lateral acceleration.",
        },
        "interpretation": [
            "The read-only KSP API supports a useful local grid across Mach, angle of attack, sideslip, and bank, with independent body-rate perturbations for a full torque derivative matrix.",
            "At the 38 km shadow-trace state (trace Mach, AoA 28 degrees, bank -40 degrees), the centered hypothetical sideslip slopes are reported directly in body XYZ: force and torque per degree. The nonzero roll- and yaw-axis moment slopes show why a zero-beta shadow trace cannot identify sideslip response.",
            "The body-coordinate bank invariance check is a coordinate warning, not a claim that bank has no physical effect: world-frame force projections rotate with the vessel.",
            "These outputs describe the neutral-surface simulated airframe only. They do not validate actual flight control, surface coordination, the entry trajectory, or the campaign gate, and they do not justify changing the bank ramp or guidance.",
        ],
        "staticSideslipRows": static_beta_rows,
    }
    atomic_json(OUTPUT, report)
    print(json.dumps({
        "status": report["status"],
        "output": str(OUTPUT),
        "inputCells": len(cells),
        "matchedForceBetaSlopeNPerDeg": matched["dForceNBodyXYZdSideslipDeg"],
        "matchedTorqueBetaSlopeNmPerDeg": matched["dTorqueNmBodyXYZdSideslipDeg"],
        "maximumRelativeBodyForceBankSpread": report["bodyCoordinateBankSpread"]["maximumRelativeForceVectorSpread"],
        "maximumRelativeBodyTorqueBankSpread": report["bodyCoordinateBankSpread"]["maximumRelativeTorqueVectorSpread"],
    }, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
