#!/usr/bin/env python3
"""Prepare or, after human confirmation, execute a read-only STS terminal polar.

Default mode is offline planning. --execute-live connects to running KSP and
requires the user's standing live-ksp-ask-first confirmation outside this script.
No pause, save, vessel, control, or actuator setters are used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from ksp_test_guard import connect
from probe_sts_aero_mimo_envelope import target_state, atomic_report
from probe_sts_aero_wrench_grid import bank_degrees, cross, dot, norm, state_snapshot, unit, wrap_degrees
from probe_sts_shadow_witness_wrench import read_actuator_snapshot

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "Configuration/sts.json"
PROFILE = ROOT / "Configuration/checkpoints/sts-campaign-72km-20261008.physics.json"
SAVE_DIR = Path("/Applications/.Misc/LMPKSP/saves/Main")
ALTITUDES = (4000., 11000., 20000., 26000.)
SPEEDS = (120., 160., 240., 400., 650.)
ALPHAS = (0., 5., 8., 12., 15., 18., 28.)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cells():
    return [dict(altitudeM=h, airspeedMps=v, alphaDeg=a)
            for h in ALTITUDES for v in SPEEDS for a in ALPHAS]


def require_preflight(snapshot, actuators, profile, part_count, dry_mass):
    measurements = [snapshot["ut"], snapshot["altitude_m"], snapshot["mass_kg"],
                    snapshot["thrust_n"], dry_mass, *snapshot["controls"].values(),
                    *(s["deflection"] for s in actuators["controlSurfaces"])]
    if any(not math.isfinite(v) for v in measurements):
        raise RuntimeError("requires finite preflight telemetry")
    if snapshot["vessel"] != "STS" or not snapshot["paused"]:
        raise RuntimeError("requires paused original STS")
    if abs(snapshot["ut"] - profile["savedUT"]) > 2 or abs(snapshot["altitude_m"] - 72000) > 500:
        raise RuntimeError("requires named 72km checkpoint state; will not load or settle it")
    if abs(snapshot["mass_kg"]-profile["mass"]) > 1 or abs(dry_mass-profile["dryMass"]) > 1 or part_count != profile["partCount"]:
        raise RuntimeError("requires settled checkpoint mass and part identity")
    if snapshot["sas"] or snapshot["rcs"] or abs(snapshot["thrust_n"]) > 1:
        raise RuntimeError("requires SAS/RCS off and zero thrust")
    if any(abs(v) > 1e-9 for v in snapshot["controls"].values()):
        raise RuntimeError("requires neutral inputs")
    if actuators["speedbrakeActionGroupEnabled"] or any(s["deployed"] or abs(s["deflection"]) > 1e-9 for s in actuators["controlSurfaces"]):
        raise RuntimeError("requires neutral, undeployed surfaces")


def project_force(force, air, q):
    if any(not math.isfinite(x) for x in (*force, *air)) or norm(air) <= 0:
        raise ValueError("finite force and nonzero finite air velocity required")
    if not math.isfinite(q) or q <= 0:
        raise ValueError("positive finite dynamic pressure required")
    direction = unit(air)
    drag = -dot(force, direction)
    perpendicular = tuple(force[i]+drag*direction[i] for i in range(3))
    return dict(dragN=drag, perpendicularForceN=norm(perpendicular),
                clPerpendicularMagnitude=norm(perpendicular)/(q*250), cd=drag/(q*250))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute-live", action="store_true")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    cfg, profile = json.loads(CONFIG.read_text()), json.loads(PROFILE.read_text())
    if cfg["vehicle"]["modelId"] != "STS":
        raise ValueError("requires exact STS configuration")
    report = dict(status="offline-plan", classification="UNQUALIFIED_HYPOTHETICAL_API_POLAR",
        qualified=False, shadowGateEvidence=False, landingOrRolloutEvidence=False,
        configurationSHA256=sha(CONFIG), profileSHA256=sha(PROFILE),
        plan=cells(), maximumCalls=len(cells()), pressureBoundPa=cfg["vehicle"]["maximumDynamicPressure"],
        referenceAreaM2=250, settersUsed=False,
        limitations=["Hypothetical neutral-surface static API forces; not flight measurements or certified cells.",
            "Exact airspeed is supplied; no nominal Mach or temperature-scaling assumption is needed.",
            "Perpendicular force may include side force; zero sideslip is requested.",
            "Current loaded part state is retained; actual flight actuator response is not identified."])
    atomic_report(args.output, report)
    if not args.execute_live:
        print("Offline plan written; no KSP connection or process inspection.")
        return 0
    protected = (SAVE_DIR / "86km.sfs", SAVE_DIR / "STS-campaign-72km-20261008.sfs")
    hashes = {str(p): sha(p) for p in protected}
    if str(protected[1]) != profile["savePath"] or hashes[str(protected[1])] != profile["saveSHA256"]:
        raise RuntimeError("named checkpoint differs from recorded physics profile")
    conn = None
    try:
        conn = connect("STS read-only terminal-speed polar", timeout=8)
        sc = conn.space_center
        vessel = sc.active_vessel
        if vessel is None or vessel.name != "STS":
            raise RuntimeError("requires original STS")
        body = vessel.orbit.body
        if body.name != "Kerbin":
            raise RuntimeError("requires Kerbin")
        frame = body.non_rotating_reference_frame
        flight = vessel.flight(frame)
        before = state_snapshot(conn, sc, vessel, body, frame)
        actuators = read_actuator_snapshot(vessel, cfg)
        require_preflight(before, actuators, profile, len(vessel.parts.all), float(vessel.dry_mass))
        ut = before["ut"]
        geometric = bank_degrees(before["rotation_xyzw"], before["position_m"])
        offset = wrap_degrees(before["roll_deg"]-geometric)
        if min(abs(offset), abs(abs(offset)-180)) > .2:
            raise RuntimeError("cannot calibrate STS bank convention")
        z_sign = -1 if abs(offset) > 90 else 1
        omega = tuple(body.angular_velocity(frame))
        report.update(status="running", liveStateBefore=before, liveActuatorsBefore=actuators,
                      saveSHA256Before=hashes, samples=[], skipped=[])
        started = time.monotonic()
        for cell in cells():
            if not conn.krpc.paused or abs(float(sc.ut)-ut) > 1e-9:
                raise RuntimeError("pause or UT changed; refusing further calls")
            if time.monotonic()-started > 90:
                raise RuntimeError("bounded probe duration exceeded")
            trace = dict(h=cell["altitudeM"], along=-8000., xtk=0., ut=ut)
            state = target_state(body, frame, cfg["site"], trace, ut, omega,
                                 z_sign, offset, cell["airspeedMps"], cell["alphaDeg"], 0., 0.)
            density = float(body.atmospheric_density_at_position(state["position"], frame))
            if not math.isfinite(density) or density <= 0:
                raise RuntimeError("invalid hypothetical density")
            q = .5*density*cell["airspeedMps"]**2
            if q > report["pressureBoundPa"]:
                report["skipped"].append(dict(**cell, qPa=q, reason="existing pressure bound"))
                continue
            force, torque = flight.simulate_aerodynamic_wrench_at(body, state["position"], state["velocity"],
                                                state["rotation"], (0., 0., 0.), ut)
            air = tuple(state["velocity"][i]-cross(omega, state["position"])[i] for i in range(3))
            if any(not math.isfinite(x) for x in torque):
                raise RuntimeError("nonfinite hypothetical torque")
            report["samples"].append(dict(**cell, densityKgM3=density, qPa=q, state=state,
                forceN=list(force), torqueNm=list(torque), **project_force(force, air, q)))
            if len(report["samples"]) % 28 == 0:
                atomic_report(args.output, report)
                print(f"Hypothetical calls completed: {len(report['samples'])}", flush=True)
        after = state_snapshot(conn, sc, vessel, body, frame)
        after_actuators = read_actuator_snapshot(vessel, cfg)
        after_hashes = {str(p): sha(p) for p in protected}
        report.update(liveStateAfter=after, liveActuatorsAfter=after_actuators,
            saveSHA256After=after_hashes, liveStateUnchanged=before==after,
            actuatorsUnchanged=actuators==after_actuators, savesUnchanged=hashes==after_hashes)
        if before != after or actuators != after_actuators or hashes != after_hashes:
            raise RuntimeError("state/actuator/save invariance check failed")
        report["status"] = "complete"
    except Exception as exc:
        report.update(status="failed", error=str(exc))
        raise
    finally:
        atomic_report(args.output, report)
        if conn is not None:
            conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
