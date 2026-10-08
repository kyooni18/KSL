#!/usr/bin/env python3
"""Compare the STS KSP wrench grid with measured samples and the provisional polar."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
GRID = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aero-envelope-grid-20261009.json"
SINGLE_STATE_GRID = ROOT / "Docs/Experiments/STS-Entry-20261009/live-aerodynamic-wrench-grid.json"
POLAR = ROOT / "ShuttleSim/reference-model/sts_aero_provisional.csv"
POLAR_REPORT = ROOT / "ShuttleSim/reference-model/sts_aero_provisional_report.json"
OBSERVATIONS = ROOT / "Runtime/Physics/observations.sqlite3"
RUN9_LOG = ROOT / "FlightLogs/2026-10-07T21-34-08Z-STS-vehicle.jsonl"
OUTPUT = ROOT / "Docs/Experiments/STS-Entry-20261009/aero-polar-crosscheck-20261009.json"
sys.path.insert(0, str(ROOT / "Tools"))
from probe_sts_aero_wrench_grid import (  # noqa: E402
    bank_degrees,
    dot,
    norm as vector_norm,
    quaternion_multiply,
    rotate,
    unit,
    wrap_degrees,
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def norm(vector) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def bracket(values: list[float], target: float) -> tuple[float, float]:
    lower = max((value for value in values if value <= target), default=None)
    upper = min((value for value in values if value >= target), default=None)
    if lower is None or upper is None:
        raise ValueError(f"Interpolation target {target} lies outside [{values[0]}, {values[-1]}]")
    return lower, upper


def interpolate(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if x0 == x1:
        return y0
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


def polar_table(path: Path) -> dict[tuple[float, float], float]:
    rows = (line for line in path.read_text().splitlines() if not line.lstrip().startswith("#"))
    table = {}
    for row in csv.DictReader(rows):
        cl = float(row["cl"])
        cd = float(row["cd"])
        if cd <= 0.0:
            raise ValueError("Provisional polar contains non-positive drag")
        table[(float(row["mach"]), float(row["alpha_deg"]))] = cl / cd
    if not table:
        raise ValueError("Provisional STS polar has no data rows")
    return table


def bilinear(table: dict[tuple[float, float], float], mach: float, aoa: float) -> float:
    machs = sorted({key[0] for key in table})
    aoas = sorted({key[1] for key in table})
    m0, m1 = bracket(machs, mach)
    a0, a1 = bracket(aoas, aoa)
    lower = interpolate(aoa, a0, a1, table[(m0, a0)], table[(m0, a1)])
    upper = interpolate(aoa, a0, a1, table[(m1, a0)], table[(m1, a1)])
    return interpolate(mach, m0, m1, lower, upper)


def hermite_curve(keys: tuple[tuple[float, float, float, float], ...], x: float) -> float:
    if x <= keys[0][0]:
        return keys[0][1]
    if x >= keys[-1][0]:
        return keys[-1][1]
    for first, second in zip(keys, keys[1:]):
        if x > second[0]:
            continue
        h = second[0] - first[0]
        u = (x - first[0]) / h
        u2, u3 = u * u, u * u * u
        return (
            (2 * u3 - 3 * u2 + 1) * first[1]
            + (u3 - 2 * u2 + u) * h * first[3]
            + (-2 * u3 + 3 * u2) * second[1]
            + (u3 - u2) * h * second[2]
        )
    return keys[-1][1]


def stock_ksp_ld(mach: float, aoa_deg: float) -> float:
    """Reproduce CLanding/vehicle/math.c stock lifting-surface curves."""
    lift_curve = (
        (0.0, 0.0, 0.0, 1.965926),
        (0.258819, 0.5114774, 1.990092, 1.905806),
        (0.5, 0.9026583, 0.7074468, -0.7074468),
        (0.7071068, 0.5926583, -2.087948, -1.990095),
        (1.0, 0.0, -2.014386, -2.014386),
    )
    lift_mach = (
        (0.0, 1.0, 0.0, 0.0),
        (0.3, 0.5, -1.671345, -0.8273422),
        (1.0, 0.125, -0.0005291355, -0.02625772),
        (5.0, 0.0625, 0.0, 0.0),
        (25.0, 0.05, 0.0, 0.0),
    )
    drag_curve = (
        (0.0, 0.01, 0.0, 0.0),
        (0.3420201, 0.06, 0.1750731, 0.1750731),
        (0.5, 0.24, 2.60928, 2.60928),
        (0.7071068, 1.7, 3.349777, 3.349777),
        (1.0, 2.4, 1.387938, 0.0),
    )
    drag_mach = (
        (0.0, 0.35, 0.0, -0.8463008),
        (0.15, 0.125, 0.0, 0.0),
        (0.9, 0.275, 0.541598, 0.541598),
        (1.1, 0.75, 0.0, 0.0),
        (1.4, 0.4, -0.3626955, -0.3626955),
        (1.6, 0.35, -0.1545923, -0.1545923),
        (2.0, 0.3, -0.09013031, -0.09013031),
        (5.0, 0.22, 0.0, 0.0),
        (25.0, 0.3, 0.0006807274, 0.0),
    )
    flow = math.sin(math.radians(min(90.0, abs(aoa_deg))))
    lift = hermite_curve(lift_curve, flow) * hermite_curve(lift_mach, max(0.0, mach))
    drag = hermite_curve(drag_curve, flow) * hermite_curve(drag_mach, max(0.0, mach))
    return lift / drag if drag > 0.0 else math.nan


def api_ld(sample: dict) -> float:
    drag = float(sample["dragN_alongRelativeAirflow"])
    total_force = norm(sample["forceN_bodyXYZ"])
    lift = math.sqrt(max(0.0, total_force * total_force - drag * drag))
    if drag <= 0.0 or lift <= 0.0:
        raise ValueError("KSP wrench cell has invalid lift or drag")
    return lift / drag


def single_state_api_ld(sample: dict, state: dict, base_bank: float) -> float:
    """Project the stored nonrotating KSP force onto its constructed air velocity."""
    speed = float(sample["mach"]) * float(state["speed_of_sound_mps"])
    alpha = math.radians(float(sample["angleOfAttackDeg"]))
    beta = math.radians(float(sample["sideslipDeg"]))
    body_velocity = (
        speed * math.sin(beta),
        speed * math.cos(beta) * math.cos(alpha),
        speed * math.cos(beta) * math.sin(alpha),
    )
    delta = math.radians(wrap_degrees(float(sample["bankDeg"]) - base_bank))
    roll_delta = (0.0, math.sin(delta / 2.0), 0.0, math.cos(delta / 2.0))
    rotation = quaternion_multiply(tuple(state["rotation_xyzw"]), roll_delta)
    relative_air_velocity = rotate(rotation, body_velocity)
    force = tuple(float(value) for value in sample["forceN_nonrotating"])
    drag = -dot(force, unit(relative_air_velocity))
    lift = math.sqrt(max(0.0, vector_norm(force) ** 2 - drag ** 2))
    if drag <= 0.0 or lift <= 0.0:
        raise ValueError("Single-state KSP wrench cell has invalid lift or drag")
    return lift / drag


def query_measured_samples(database: Path) -> tuple[str, list[dict]]:
    uri = f"file:{database.resolve()}?mode=ro"
    connection = sqlite3.connect(uri, uri=True)
    try:
        context = connection.execute(
            "SELECT environment_id FROM physics_contexts "
            "WHERE structure_id='model:STS' ORDER BY last_seen DESC LIMIT 1"
        ).fetchone()
        if not context:
            raise RuntimeError("No original-STS observation context is available")
        rows = connection.execute(
            """SELECT a.session_id,a.ut,a.q,a.mach,a.aoa,a.beta,a.roll,a.rcs,
                      a.lift_x,a.lift_y,a.lift_z,a.drag_x,a.drag_y,a.drag_z,
                      quality.control_profile,quality.quality
               FROM aero_observations AS a
               JOIN aero_observation_quality AS quality ON quality.observation_id=a.id
               WHERE a.structure_id='model:STS' AND a.environment_id=?
                 AND quality.eligible=1
                 AND a.mach BETWEEN 5.0 AND 5.5
                 AND a.aoa BETWEEN 26.0 AND 28.0
                 AND abs(a.beta)<1.0
                 AND a.q BETWEEN 1500.0 AND 3000.0
                 AND a.gear=0 AND a.brakes=0 AND a.airbrakes=0
               ORDER BY a.id""",
            (context[0],),
        ).fetchall()
    finally:
        connection.close()
    samples = []
    for row in rows:
        lift = norm(row[8:11])
        drag = norm(row[11:14])
        samples.append({
            "sessionId": row[0], "ut": float(row[1]), "qPa": float(row[2]),
            "mach": float(row[3]), "aoaDeg": float(row[4]), "betaDeg": float(row[5]),
            "bankDeg": float(row[6]), "rcsState": int(row[7]),
            "liftN": lift, "dragN": drag, "liftToDrag": lift / drag,
            "controlProfile": row[14], "quality": float(row[15]),
        })
    return str(context[0]), samples


def median_span(samples: list[dict], key: str) -> dict[str, float]:
    values = [float(sample[key]) for sample in samples]
    return {"min": min(values), "median": statistics.median(values), "max": max(values)}


def main() -> None:
    grid = json.loads(GRID.read_text())
    single_grid = json.loads(SINGLE_STATE_GRID.read_text())
    provisional_report = json.loads(POLAR_REPORT.read_text())
    if grid.get("status") != "complete" or grid.get("classification") != "UNQUALIFIED_DIAGNOSTIC_SIMULATION":
        raise RuntimeError("Expected the retained unqualified original-STS KSP wrench grid")
    if single_grid.get("status") != "complete" or single_grid.get("classification") != "UNQUALIFIED_DIAGNOSTIC_SIMULATION":
        raise RuntimeError("Expected the retained unqualified single-state original-STS KSP wrench grid")
    table = polar_table(POLAR)
    database_sha_before = sha256(OBSERVATIONS)
    wal_path = Path(str(OBSERVATIONS) + "-wal")
    wal_sha_before = sha256(wal_path) if wal_path.exists() else None
    environment_id, measured = query_measured_samples(OBSERVATIONS)
    database_sha_after = sha256(OBSERVATIONS)
    wal_sha_after = sha256(wal_path) if wal_path.exists() else None
    if database_sha_before != database_sha_after or wal_sha_before != wal_sha_after:
        raise RuntimeError("Read-only comparison changed the physics observation store")
    if not measured:
        raise RuntimeError("No quality-eligible measured STS samples match the high-incidence window")

    cells = {}
    for sample in grid["samples"]:
        if abs(float(sample["sideslipDeg"])) > 1e-8 or abs(float(sample["bankDeg"])) > 1e-8:
            continue
        if float(sample["angleOfAttackDeg"]) not in (18.0, 28.0):
            continue
        key = (float(sample["targetAltitudeM"]), float(sample["nominalMachEstimate"]),
               float(sample["angleOfAttackDeg"]))
        cells[key] = {"sample": sample, "ld": api_ld(sample)}

    ratios = []
    pairs = sorted({(key[0], key[1]) for key in cells})
    for altitude, mach in pairs:
        at_18 = cells.get((altitude, mach, 18.0))
        at_28 = cells.get((altitude, mach, 28.0))
        if at_18 and at_28 and 4.0 <= mach <= 6.3:
            ratios.append({
                "altitudeM": altitude,
                "mach": mach,
                "ldAt18": at_18["ld"],
                "ldAt28": at_28["ld"],
                "ratio28To18": at_28["ld"] / at_18["ld"],
            })
    if len(ratios) < 10:
        raise RuntimeError("The KSP grid does not contain enough matched AoA 18/28 cells")

    single_state = single_grid["liveStateBefore"]
    base_bank = bank_degrees(tuple(single_state["rotation_xyzw"]), tuple(single_state["position_m"]))
    single_cells = {}
    for sample in single_grid["samples"]:
        if abs(float(sample["sideslipDeg"])) > 1e-8 or abs(float(sample["bankDeg"])) > 1e-8:
            continue
        key = (float(sample["mach"]), float(sample["angleOfAttackDeg"]))
        single_cells[key] = single_state_api_ld(sample, single_state, base_bank)
    single_state_ratios = []
    for mach in sorted({key[0] for key in single_cells}):
        if (mach, 18.0) in single_cells and (mach, 28.0) in single_cells:
            single_state_ratios.append({
                "mach": mach,
                "ldAt18": single_cells[(mach, 18.0)],
                "ldAt28": single_cells[(mach, 28.0)],
                "ratio28To18": single_cells[(mach, 28.0)] / single_cells[(mach, 18.0)],
            })
    if len(single_state_ratios) < 3:
        raise RuntimeError("The single-state grid lacks matched AoA 18/28 cells")

    target_altitude = min({key[0] for key in cells}, key=lambda value: abs(value - 38000.0))
    target_mach = min(
        {key[1] for key in cells if key[0] == target_altitude},
        key=lambda value: abs(value - 5.212),
    )
    api_28 = cells[(target_altitude, target_mach, 28.0)]["ld"]
    api_18 = cells[(target_altitude, target_mach, 18.0)]["ld"]
    median_mach = statistics.median(sample["mach"] for sample in measured)
    median_aoa = statistics.median(sample["aoaDeg"] for sample in measured)
    median_ld = statistics.median(sample["liftToDrag"] for sample in measured)
    table_at_measured = bilinear(table, median_mach, median_aoa)
    table_18 = bilinear(table, target_mach, 18.0)
    table_28 = bilinear(table, target_mach, 28.0)
    c_stock_ratio = stock_ksp_ld(target_mach, 28.0) / stock_ksp_ld(target_mach, 18.0)
    nearest_support_knots = sorted(
        provisional_report["knots"], key=lambda knot: abs(float(knot["mach"]) - target_mach)
    )[:2]

    source_sessions = sorted({sample["sessionId"] for sample in measured})
    bypass_marker_seen = RUN9_LOG.exists() and "KSP_LANDER_BYPASS_SHADOW_GATE" in RUN9_LOG.read_text(errors="replace")
    report = {
        "status": "complete",
        "classification": "UNQUALIFIED_DIAGNOSTIC_CROSSCHECK",
        "qualification": {
            "qualified": False,
            "shadowGateEvidence": False,
            "flightQualification": False,
            "landingOrRolloutEvidence": False,
            "note": "KSP wrench cells are hypothetical simulations. The measured comparison comes from a prior flight with a recorded shadow-gate bypass and is retained as physical diagnostic data only.",
        },
        "purpose": "Compare the original-STS KSP wrench-grid lift-to-drag polar with the existing provisional polar, the current C stock-lifting-surface incidence factor, and nearby quality-eligible measured STS samples.",
        "provenance": {
            "gitHeadAtAnalysis": subprocess.run(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                text=True, check=True,
            ).stdout.strip(),
            "analysisTool": str(Path(__file__).relative_to(ROOT)),
            "analysisToolSHA256": sha256(Path(__file__)),
            "wrenchGrid": str(GRID.relative_to(ROOT)),
            "wrenchGridSHA256": sha256(GRID),
            "wrenchGridClassification": grid["classification"],
            "wrenchGridCellCount": len(grid["samples"]),
            "singleStateWrenchGrid": str(SINGLE_STATE_GRID.relative_to(ROOT)),
            "singleStateWrenchGridSHA256": sha256(SINGLE_STATE_GRID),
            "singleStateWrenchGridClassification": single_grid["classification"],
            "singleStateWrenchGridCellCount": len(single_grid["samples"]),
            "singleStateWrenchGridLiveState": {
                "ut": single_state["ut"],
                "meanAltitudeM": single_state["altitude_m"],
                "mach": single_state["mach"],
                "dynamicPressurePa": single_state["dynamic_pressure_pa"],
                "checkpointUsed": False,
                "liveStateUnchanged": single_grid.get("liveStateUnchanged"),
                "checkpointUnchanged": single_grid.get("checkpointUnchanged"),
            },
            "provisionalPolar": str(POLAR.relative_to(ROOT)),
            "provisionalPolarSHA256": sha256(POLAR),
            "provisionalPolarReport": str(POLAR_REPORT.relative_to(ROOT)),
            "provisionalPolarReportSHA256": sha256(POLAR_REPORT),
            "provisionalPolarSupport": {
                "status": provisional_report.get("status"),
                "certifiedCells": provisional_report.get("certified_cells"),
                "assumptions": provisional_report.get("assumptions"),
                "nearestMachKnots": nearest_support_knots,
            },
            "cStockCurveSource": "CLanding/vehicle/math.c::stock_ksp_lifting_surface_coefficients",
            "cStockCurveSourceSHA256": sha256(ROOT / "CLanding/vehicle/math.c"),
            "physicsObservationStore": str(OBSERVATIONS.relative_to(ROOT)),
            "physicsObservationStoreSHA256BeforeAndAfter": database_sha_before,
            "physicsObservationStoreWalSHA256BeforeAndAfter": wal_sha_before,
            "physicsContext": environment_id,
            "priorMeasuredSessionIds": source_sessions,
            "priorMeasuredSourceLog": str(RUN9_LOG.relative_to(ROOT)) if RUN9_LOG.exists() else None,
            "priorMeasuredSourceLogSHA256": sha256(RUN9_LOG) if RUN9_LOG.exists() else None,
            "priorMeasuredSourceContainsShadowGateBypassMarker": bypass_marker_seen,
        },
        "measuredHighIncidenceWindow": {
            "filter": "eligible=1, STS, Mach 5.0..5.5, AoA 26..28 deg, |beta|<1 deg, q 1500..3000 Pa, gear/brakes/airbrakes stowed",
            "sampleCount": len(measured),
            "sessionIds": source_sessions,
            "rcsStates": sorted({sample["rcsState"] for sample in measured}),
            "controlProfiles": sorted({sample["controlProfile"] for sample in measured}),
            "quality": median_span(measured, "quality"),
            "mach": median_span(measured, "mach"),
            "angleOfAttackDeg": median_span(measured, "aoaDeg"),
            "sideslipDeg": median_span(measured, "betaDeg"),
            "bankDeg": median_span(measured, "bankDeg"),
            "dynamicPressurePa": median_span(measured, "qPa"),
            "liftToDrag": median_span(measured, "liftToDrag"),
            "note": "All selected samples are from one prior original-STS session. RCS state is unknown in these database rows. The archived source log contains the KSP_LANDER_BYPASS_SHADOW_GATE marker.",
        },
        "matchedGridPoint": {
            "targetAltitudeM": target_altitude,
            "mach": target_mach,
            "sideslipDeg": 0.0,
            "bankDeg": 0.0,
            "kspApiSimulatedLiftToDrag": {"aoa18": api_18, "aoa28": api_28},
            "kspApiAoA28To18Ratio": api_28 / api_18,
            "provisionalTableLiftToDrag": {
                "aoa18": table_18,
                "aoa28": table_28,
                "aoa28To18Ratio": table_28 / table_18,
                "aoa18OutsideRecordedSupport": True,
            },
            "cStockLiftingSurfaceAoA28To18Ratio": c_stock_ratio,
            "comparisonToMeasuredMedian": {
                "measuredLiftToDrag": median_ld,
                "kspApiRelativeError": (api_28 - median_ld) / median_ld,
                "provisionalTableAtMeasuredMedianRelativeError": (table_at_measured - median_ld) / median_ld,
                "provisionalTableAtMachGridAoa28RelativeError": (table_28 - median_ld) / median_ld,
            },
            "sourceSupportNote": "The API AoA18/28 ratio is counterfactual. This measured sample window has no Mach-5 AoA18 points, so its incidence slope is not independently measured.",
        },
        "gridIncidenceShape": {
            "lowerAltitudeGrid": {
                "matchedMachAltitudePairs": len(ratios),
                "apiRatio28To18Range": [min(x["ratio28To18"] for x in ratios), max(x["ratio28To18"] for x in ratios)],
                "apiRatio28To18Median": statistics.median(x["ratio28To18"] for x in ratios),
                "pairs": ratios,
            },
            "singleStateGrid": {
                "matchedMachPairs": len(single_state_ratios),
                "apiRatio28To18Range": [min(x["ratio28To18"] for x in single_state_ratios), max(x["ratio28To18"] for x in single_state_ratios)],
                "apiRatio28To18Median": statistics.median(x["ratio28To18"] for x in single_state_ratios),
                "pairs": single_state_ratios,
            },
            "pairs": ratios,
            "cStockRatio28To18": c_stock_ratio,
            "provisionalRatioAtMachGrid": table_28 / table_18,
        },
        "interpretation": {
            "finding": "At the Mach-5.2, AoA-28 witness, the KSP API simulation is close to the nearby measured L/D median, while the provisional table is higher. Both API grids show an AoA-28/AoA-18 L/D ratio around 0.75-0.80; the C stock lifting-surface fallback gives about 0.45, and the provisional table gives an increasing ratio at this Mach.",
            "limitation": "The measured high-incidence subset is one bypassed flight session and contains no matched Mach-5 AoA18 samples. The API incidence sweep remains simulated, the provisional table has zero certified cells and no independent AoA sweep, and local observed-force cells may supersede the cold-start fallback.",
            "decision": "Use as a diagnostic model-shape hypothesis only. Do not change the live low-q bank ramp, production aerodynamic profile, shadow criteria, or flight limits from this comparison alone.",
        },
        "liveStateMutation": False,
        "errors": [],
    }
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "status": report["status"],
        "output": str(OUTPUT),
        "measuredSampleCount": len(measured),
        "measuredMedianLD": median_ld,
        "apiAoa28LD": api_28,
        "provisionalAoa28LD": table_28,
        "apiRatio28To18": report["gridIncidenceShape"]["lowerAltitudeGrid"]["apiRatio28To18Median"],
        "singleStateApiRatio28To18": report["gridIncidenceShape"]["singleStateGrid"]["apiRatio28To18Median"],
        "cStockRatio28To18": c_stock_ratio,
        "provisionalRatio28To18": table_28 / table_18,
        "priorSourceGateBypassed": bypass_marker_seen,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
