#!/usr/bin/env python3
"""Offline STS orbital-deorbit -> MM304 -> MM305 campaign runner (ShuttleSim only).

No live KSP, no savegames: every case is a generated Kerbin-orbit scenario flown
through the *production* backend `engage` path (createPlan -> qualification ->
coast -> OMS burn -> entry -> MM304 -> MM305) against ShuttleSim's plant.

Subcommands
  plan    write matrix.json for a named stage (scenario generation + orbit validation)
  run     execute pending cases (resumable, per-case timeout, parallel jobs)
  report  machine-readable summary.json + human-readable report.md (+ --compare)
  analyze analyse one archived run directory

A case counts as MM305-acquired only when the production state machine itself
reports phase "TAEM" (MM305 ownership) in a full `engage` run whose plan was
qualified and engaged.  Nothing here edits thresholds or skips gates.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import gzip
import hashlib
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
SIM = HERE.parent
ROOT = SIM.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(SIM / "scripts"))
import orbit_scenarios as orb  # noqa: E402

DEFAULT_CONFIG = ROOT / "Configuration" / "sts.json"
PROFILE = os.environ.get("STS_CAMPAIGN_PROFILE", "sts-live-20261009")
G0 = 9.80665


# --------------------------------------------------------------------------- matrix
def case_id(params: dict) -> str:
    blob = json.dumps(params, sort_keys=True).encode()
    p = params
    return f"i{p['inc']:05.1f}-{p['rp']:g}x{p['ra']:g}-{p['branch']}-w{p['argp']:g}-d{p['delay']:g}-x{p['cross']:g}-u{p['ut0']:g}-" + \
        hashlib.sha256(blob).hexdigest()[:6]


def mk(inc, rp=86, ra=86, branch="asc", argp=0.0, delay=1250.0, cross=0.0, ut0=10000.0, mass=None):
    return {"inc": float(inc), "rp": float(rp), "ra": float(ra), "branch": branch, "argp": float(argp),
            "delay": float(delay), "cross": float(cross), "ut0": float(ut0), "mass": mass or orb.STS_MASS_KG}


INCS = (0, 15, 30, 45, 60, 75, 90, 120, 150, 180)
ORBITS = ((86, 86), (80, 100), (80, 130), (80, 160))


def stage_cases(stage: str):
    out = []
    if stage == "S0":  # smoke: one nominal case
        out = [mk(45)]
    elif stage == "X1":  # development triplet used for single-mechanism A/B experiments
        out = [mk(0), mk(45, branch="desc"), mk(90, branch="asc"), mk(150, branch="desc")]
    elif stage == "X2":  # X1 geometry with a later site pass so longer entry-range targets fit the first orbit
        out = [mk(0, delay=1750), mk(45, branch="desc", delay=1750), mk(90, branch="asc", delay=1750),
               mk(150, branch="desc", delay=1750)]
    elif stage == "X3":  # X1 geometry, later pass so energy-scaled (longer) entry ranges fit the first orbit
        out = [mk(0, delay=1950), mk(90, branch="asc", delay=1950), mk(150, branch="desc", delay=1950),
               mk(120, branch="asc", delay=1950), mk(180, delay=1950), mk(60, branch="desc", delay=1950)]
    elif stage == "S1":  # circular 86 km, every inclination, both crossing branches
        for i in INCS:
            for br in (("asc",) if i in (0, 180) else ("asc", "desc")):
                out.append(mk(i, branch=br))
    elif stage == "S2":  # shape variety at a representative inclination subset, argp varied
        for i in (0, 30, 60, 90, 120, 180):
            for rp, ra in ORBITS[1:]:
                for argp in (0.0, 90.0, 180.0):
                    br = "asc" if i in (0, 180) else "desc"
                    out.append(mk(i, rp, ra, br, argp))
    elif stage == "S3":  # off-plane cross-range and timing sweep (boundary cases)
        for i in (30, 60, 90):
            for cr in (0.5, 1.0, 2.0, 4.0):
                out.append(mk(i, branch="desc", cross=cr))
            for d in (1100.0, 1500.0, 2000.0, 3200.0):
                out.append(mk(i, branch="desc", delay=d))
    elif stage == "H1":  # held-out: unusual epochs/phases not used while developing
        for k, (i, br, rp, ra, argp, ut0) in enumerate((
                (22, "asc", 86, 86, 0, 41231.0), (52, "desc", 80, 110, 50, 77777.0),
                (82, "asc", 84, 96, 200, 123457.0), (97, "desc", 86, 86, 0, 55555.0),
                (133, "asc", 80, 140, 300, 98765.0), (171, "desc", 86, 86, 0, 31415.0),
                (38, "desc", 80, 150, 120, 88888.0), (64, "asc", 82, 122, 260, 64000.0))):
            out.append(mk(i, rp, ra, br, argp, 1250.0 + 90 * k, 0.0, ut0))
    else:
        raise SystemExit(f"unknown stage {stage}")
    return out


def build_matrix(stage: str, camp: pathlib.Path):
    sc_dir = camp / "scenarios"
    sc_dir.mkdir(parents=True, exist_ok=True)
    cases = []
    for p in stage_cases(stage):
        cid = case_id(p)
        sc, geo = orb.generate(p["inc"], p["rp"], p["ra"], p["argp"], p["delay"], p["cross"], p["branch"],
                               p["ut0"], p["mass"])
        ini = sc_dir / f"{cid}.ini"
        orb.write_ini(ini, cid, sc)
        cases.append({"id": cid, "stage": stage, "params": p, "scenario": str(ini.relative_to(camp)),
                      "scenarioSha256": hashlib.sha256(ini.read_bytes()).hexdigest(),
                      "geometry": geo})
    (camp / "matrix.json").write_text(json.dumps({"stage": stage, "cases": cases}, indent=1))
    bad = [c["id"] for c in cases if not c["geometry"]["validation"]["ok"]]
    return cases, bad


# --------------------------------------------------------------------------- analysis
def _open(path):
    path = pathlib.Path(path)
    return gzip.open(path, "rt") if path.suffix == ".gz" else open(path)


def _read_jsonl(path):
    with _open(path) as f:
        for line in f:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def analyze_run(run_dir: pathlib.Path, config: dict | None = None) -> dict:
    """Extract the campaign metrics from one archived run directory."""
    manifest = json.loads((run_dir / "manifest.json").read_text())
    cfg = config or json.loads(DEFAULT_CONFIG.read_text())
    g = cfg["guidance"]
    out = {"runId": manifest.get("runId"), "state": manifest.get("state"),
           "terminalPhase": manifest.get("terminalPhase"),
           "abortReason": (manifest.get("final") or {}).get("abortReason"),
           "simElapsedSeconds": manifest.get("simElapsedSeconds"),
           "wallSeconds": manifest.get("wallSeconds"),
           "provenance": manifest.get("provenance")}

    # ---- guidance snapshots
    phases, first_by_phase, plan, plan_phase, plan_final = [], {}, None, None, None
    last_snap = None
    taem = None
    prev_phase = None
    path = run_dir / (manifest.get("guidanceEvents") or "guidance-events.jsonl.gz")
    gp = []
    for ev in _read_jsonl(path):
        if ev.get("type") != "snapshot":
            continue
        s = ev["snapshot"]
        t = s.get("telemetry") or {}
        ph = s.get("phase")
        last_snap = s
        if s.get("plan"):
            pl = {k: v for k, v in s["plan"].items() if k != "trajectory"}
            if plan is None or (ph == "Coast to Burn" and plan_phase != ph):
                plan, plan_phase = pl, ph
            plan_final = pl
        if ph != prev_phase:
            first_by_phase.setdefault(ph, {"ut": t.get("ut"), "alt": t.get("meanAltitude"), "tas": t.get("trueAirSpeed"),
                                          "mach": t.get("mach"), "along": t.get("runwayAlongTrack"),
                                          "cross": t.get("runwayCrossTrack"), "range": t.get("rangeToSite"),
                                          "status": (s.get("statusMessage") or "")[:200]})
            phases.append(ph)
            prev_phase = ph
            if ph == "TAEM" and taem is None:
                gs = s.get("guidanceState") or {}
                taem = {"ut": t.get("ut"), "altitude_m": t.get("meanAltitude"), "mach": t.get("mach"),
                        "tas_mps": t.get("trueAirSpeed"), "along_m": t.get("runwayAlongTrack"),
                        "cross_m": t.get("runwayCrossTrack"), "range_m": t.get("rangeToSite"),
                        "fromPhase": phases[-2] if len(phases) > 1 else None,
                        "terminalPathCommitted": gs.get("terminalPathCommitted"),
                        "status": (s.get("statusMessage") or "")[:240]}
    out["phaseSequence"] = phases
    out["firstByPhase"] = first_by_phase
    out["plan"] = plan
    out["planFinal"] = plan_final
    if last_snap:
        out["finalPhase"] = last_snap.get("phase")
        out["finalStatus"] = (last_snap.get("statusMessage") or "")[:300]
        out["finalWarning"] = (last_snap.get("warningMessage") or "")[:300]
    # MM305: production state machine reached TAEM; record band conformance (informational)
    if taem:
        band = (g["mm305MinimumAltitude"] <= (taem["altitude_m"] or 0) <= g["mm305MaximumAltitude"] and
                abs((taem["mach"] or 0) - g["mm305TargetMach"]) <= g["mm305MachHalfWidth"] + 0.5)
        taem["inNominalBand"] = bool(band)
    out["mm305"] = taem
    out["mm305Acquired"] = bool(taem) and "MM304 Entry" in phases and taem.get("fromPhase") == "MM304 Entry"

    # ---- plant telemetry
    rows = _read_jsonl(run_dir / (manifest.get("simulatorTelemetry") or "simulator-telemetry.jsonl.gz"))
    burn = {"t0": None, "t1": None}
    ei = None
    max_q = max_g = 0.0
    closest = None
    atm_closest = None
    last_atm_range = None
    prev_alt = None
    post = None
    last = None
    first = None
    thr_prev = 0.0
    series = []
    last_series_t = -1e9
    for r in rows:
        last = r
        if first is None:
            first = r
        t = r["sim_time"]
        veh = r["vehicle"]
        a = r["aero"]
        alt = r["position"]["altitude_m"]
        thr = veh["throttle"]
        if thr > 0 and burn["t0"] is None:
            burn["t0"] = t
        if thr > 0:
            burn["t1"] = t
        if burn["t1"] is not None and post is None and thr == 0 and t > burn["t1"] + 3.0:
            p = (r["position"]["x"], r["position"]["y"], r["position"]["z"])
            v = (r["velocity"]["x_mps"], r["velocity"]["y_mps"], r["velocity"]["z_mps"])
            post = {"t": t, **orb.state_to_elements(p, v), "mass_kg": veh["mass_kg"],
                    "engine_dv_mps": veh["engine_dv_mps"], "propellant_kg": veh["propellant_used_kg"]}
        if ei is None and alt <= 70000.0 and prev_alt is not None and prev_alt > 70000.0:
            ei = {"t": t, "ut": r["ut"], "alt_m": alt, "surface_mps": r["velocity"]["surface_mps"],
                  "fpa_deg": r["velocity"]["flight_path_angle_deg"], "along_m": r["runway"]["along_m"],
                  "cross_m": r["runway"]["cross_m"], "mass_kg": veh["mass_kg"]}
        prev_alt = alt
        m = max(veh["mass_kg"], 1.0)
        max_q = max(max_q, a["q_pa"])
        max_g = max(max_g, math.hypot(a["lift_n"], a["drag_n"]) / m / G0)
        rng = math.hypot(r["runway"]["along_m"], r["runway"]["cross_m"])
        if alt < 70000.0:
            last_atm_range = rng
            if atm_closest is None or rng < atm_closest["range_m"]:
                atm_closest = {"range_m": rng, "t": t, "alt_m": alt, "surface_mps": r["velocity"]["surface_mps"],
                               "mach": a["mach"]}
        if closest is None or rng < closest["range_m"]:
            closest = {"range_m": rng, "t": t, "alt_m": alt, "surface_mps": r["velocity"]["surface_mps"],
                       "mach": a["mach"], "along_m": r["runway"]["along_m"], "cross_m": r["runway"]["cross_m"]}
        if t - last_series_t >= 20.0:
            last_series_t = t
            at = r["attitude"]
            series.append([round(t, 1), round(alt), round(r["velocity"]["surface_mps"], 1), round(a["mach"], 2),
                           round(a["q_pa"]), round(at["aoa_deg"], 1), round(at["bank_deg"], 1),
                           round(r["runway"]["along_m"]), round(r["runway"]["cross_m"]),
                           round(math.hypot(a["lift_n"], a["drag_n"]) / m / G0, 2)])
    out["burn"] = burn
    out["postBurn"] = post
    out["entryInterface"] = ei
    out["closestApproachToRunway"] = closest
    out["atmosphericClosestApproach"] = atm_closest
    out["atmosphericFinalRange_m"] = last_atm_range
    out["maxDynamicPressure_pa"] = max_q
    out["maxLoadFactor_g"] = max_g
    out["series"] = series
    if last:
        out["final"] = {"sim_time": last["sim_time"], "alt_m": last["position"]["altitude_m"],
                        "surface_mps": last["velocity"]["surface_mps"], "along_m": last["runway"]["along_m"],
                        "cross_m": last["runway"]["cross_m"], "mach": last["aero"]["mach"]}
    if first and post and plan:
        out["burnVsPlan"] = {
            "planned_dv_mps": plan.get("deltaV"), "planned_normal_angle_deg": plan.get("burnNormalAngle"),
            "planned_pe_alt_m": plan.get("predictedPostBurnPeriapsisAltitude"),
            "achieved_engine_dv_mps": post["engine_dv_mps"], "achieved_pe_alt_m": post["rp_alt_m"],
            "achieved_inc_deg": post["i_deg"], "propellant_kg": post["propellant_kg"],
            "pe_error_m": post["rp_alt_m"] - (plan.get("predictedPostBurnPeriapsisAltitude") or math.nan)}
    out["classification"] = classify(out)
    return out


def classify(r: dict) -> dict:
    """Heuristic outcome class; physics-vs-guidance attribution is added by hand in notes."""
    if r.get("mm305Acquired"):
        return {"class": "MM305_ACQUIRED", "detail": ""}
    ph = r.get("phaseSequence") or []
    plan = r.get("plan") or {}
    if "Fault" in ph and "Coast to Burn" not in ph:
        return {"class": "PLAN_FAULT", "detail": r.get("finalStatus", "")}
    if r.get("state") == "rejected" or (plan and plan.get("executionQualified") is False):
        return {"class": "PLAN_REJECTED", "detail": r.get("finalStatus", "")}
    if "Deorbit Burn" not in ph:
        return {"class": "NO_BURN", "detail": r.get("finalStatus", "")}
    if r.get("entryInterface") is None:
        return {"class": "NO_ENTRY", "detail": r.get("finalStatus", "")}
    cl = r.get("atmosphericClosestApproach") or {}
    fin = r.get("final") or {}
    reason = (r.get("abortReason") or r.get("finalStatus") or "")
    final_rng = r.get("atmosphericFinalRange_m")
    ground = fin.get("alt_m", 1e9) < 500.0
    # Overshoot: the vehicle passed its closest approach to the runway while still in the atmosphere and
    # then flew away from it.  Short: range to the runway was still decreasing when the flight ended.
    passed = final_rng is not None and cl and final_rng > cl.get("range_m", 0) + 15e3
    if ground or "Rollout" in (r.get("phaseSequence") or []):
        if passed:
            return {"class": "MM304_OVERSHOOT", "detail": f"closest {cl['range_m']/1000:.0f} km at {cl['surface_mps']:.0f} m/s, ended {final_rng/1000:.0f} km away; " + reason}
        return {"class": "MM304_SHORT", "detail": f"ended {((final_rng or 0))/1000:.0f} km from runway; " + reason}
    if passed:
        return {"class": "MM304_HIGH_ENERGY_PASS", "detail": reason}
    return {"class": "MM304_NO_HANDOFF", "detail": reason}


def cfg_digest(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def git(*a):
    try:
        return subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None


def run_case(camp: pathlib.Path, case: dict, args) -> dict:
    res_path = camp / "results" / f"{case['id']}.json"
    if res_path.exists() and not args.rerun:
        return json.loads(res_path.read_text())
    (camp / "results").mkdir(exist_ok=True)
    env = dict(os.environ, KSP_LANDER_MODEL_PROFILE=PROFILE)
    cmd = [sys.executable, str(SIM / "scripts" / "run_guidance.py"),
           "--scenario", str(camp / case["scenario"]), "--configuration", str(args.config),
           "--engage", "engage", "--max-sim-time", str(args.max_sim_time), "--label", "c" + case["id"][:40],
           "--skip-build", "--no-mirror", "--quiet-progress", "--plan-timeout", str(args.plan_timeout)]
    if args.stop_at_phase:
        cmd += ["--stop-at-phase", args.stop_at_phase]
    t0 = time.time()
    log_path = camp / "logs" / f"{case['id']}.log"
    log_path.parent.mkdir(exist_ok=True)
    result = {"id": case["id"], "stage": case["stage"], "params": case["params"],
              "geometry": {k: v for k, v in case["geometry"].items() if k != "requested"},
              "config": str(args.config), "configSha256": cfg_digest(args.config),
              "modelProfile": PROFILE, "gitHead": git("rev-parse", "HEAD"),
              "gitDirty": bool(git("status", "--porcelain", "--untracked-files=no")),
              "scenarioSha256": case["scenarioSha256"], "startedAt": t0}
    try:
        with open(log_path, "w") as lf:
            proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            try:
                proc.wait(timeout=args.timeout)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                result["timeout"] = True
        tail = log_path.read_text()
        run_json = None
        for line in reversed(tail.strip().splitlines()):
            if line.startswith("{") and "runId" in line:
                try:
                    run_json = json.loads(line)
                    break
                except ValueError:
                    pass
        result["wallSeconds"] = time.time() - t0
        if run_json and run_json.get("manifest"):
            run_dir = pathlib.Path(run_json["manifest"]).parent
            result["runDir"] = str(run_dir)
            result["analysis"] = analyze_run(run_dir, json.loads(pathlib.Path(args.config).read_text()))
            result["outcome"] = result["analysis"]["classification"]
            result["mm305Acquired"] = result["analysis"]["mm305Acquired"]
        else:
            result["outcome"] = {"class": "RUNNER_TIMEOUT" if result.get("timeout") else "RUNNER_ERROR",
                                 "detail": tail[-400:]}
            result["mm305Acquired"] = False
    except Exception as exc:  # one broken case must not stop the campaign
        result["outcome"] = {"class": "RUNNER_EXCEPTION", "detail": repr(exc)[:400]}
        result["mm305Acquired"] = False
    res_path.write_text(json.dumps(result, indent=1))
    return result


def cmd_run(args):
    camp = pathlib.Path(args.campaign).resolve()
    args.config = str(pathlib.Path(args.config).resolve())
    cases = json.loads((camp / "matrix.json").read_text())["cases"]
    if args.filter:
        cases = [c for c in cases if args.filter in c["id"]]
    pending = [c for c in cases if args.rerun or not (camp / "results" / f"{c['id']}.json").exists()]
    print(f"{len(cases)} cases, {len(pending)} pending, jobs={args.jobs}", flush=True)
    with cf.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = {ex.submit(run_case, camp, c, args): c for c in pending}
        for f in cf.as_completed(futs):
            r = f.result()
            print(f"{r['id']}: {r['outcome']['class']}  acquired={r['mm305Acquired']}  wall={r.get('wallSeconds',0):.0f}s",
                  flush=True)
    cmd_report(argparse.Namespace(campaign=str(camp), compare=None))


# --------------------------------------------------------------------------- report
def load_results(camp):
    return {p.stem: json.loads(p.read_text()) for p in sorted((pathlib.Path(camp) / "results").glob("*.json"))}


def _km(x):
    return "-" if x is None else f"{x/1000:.0f}"


def cmd_report(args):
    camp = pathlib.Path(args.campaign).resolve()
    res = load_results(camp)
    classes = {}
    for r in res.values():
        classes.setdefault(r["outcome"]["class"], []).append(r["id"])
    summary = {"campaign": camp.name, "cases": len(res), "mm305Acquired": sum(r["mm305Acquired"] for r in res.values()),
               "classes": {k: len(v) for k, v in sorted(classes.items())}, "generated": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "gitHead": git("rev-parse", "HEAD")}
    (camp / "summary.json").write_text(json.dumps(summary, indent=1))
    lines = [f"# STS campaign `{camp.name}`", "",
             f"Cases: {summary['cases']} | MM305 acquired (production state machine): **{summary['mm305Acquired']}** | git `{(summary['gitHead'] or '')[:10]}`",
             "", "Offline ShuttleSim results with the provisional STS aerodynamic profile; not live-KSP validation.", "",
             "| class | n |", "|---|---|"] + [f"| {k} | {v} |" for k, v in summary["classes"].items()]
    lines += ["", "| case | outcome | burn dv (plan/ach) | Pe plan/ach km | EI v / fpa | closest rwy km (v, alt km) | max q kPa | max g | TAEM |",
              "|---|---|---|---|---|---|---|---|---|"]
    for r in res.values():
        a = r.get("analysis") or {}
        bp = a.get("burnVsPlan") or {}
        ei = a.get("entryInterface") or {}
        cl = a.get("closestApproachToRunway") or {}
        tm = a.get("mm305") or {}
        lines.append("| {id} | {o} | {pdv}/{adv} | {ppe}/{ape} | {ev}/{ef} | {cr} ({cv}, {ca}) | {q} | {g} | {t} |".format(
            id=r["id"], o=r["outcome"]["class"],
            pdv=f"{bp.get('planned_dv_mps'):.1f}" if bp.get("planned_dv_mps") is not None else "-",
            adv=f"{bp.get('achieved_engine_dv_mps'):.1f}" if bp.get("achieved_engine_dv_mps") is not None else "-",
            ppe=_km(bp.get("planned_pe_alt_m")), ape=_km(bp.get("achieved_pe_alt_m")),
            ev=f"{ei.get('surface_mps',0):.0f}" if ei else "-", ef=f"{ei.get('fpa_deg',0):.2f}" if ei else "-",
            cr=_km(cl.get("range_m")), cv=f"{cl.get('surface_mps',0):.0f}" if cl else "-", ca=_km(cl.get("alt_m")),
            q=f"{a.get('maxDynamicPressure_pa',0)/1000:.1f}" if a else "-", g=f"{a.get('maxLoadFactor_g',0):.2f}" if a else "-",
            t=(f"alt {tm['altitude_m']/1000:.1f} M{tm['mach']:.2f} rng {tm['range_m']/1000:.0f}km" if tm else "-")))
    if args.compare:
        base = load_results(args.compare)
        lines += ["", f"## Comparison against `{pathlib.Path(args.compare).name}`", "", "| case | base | now |", "|---|---|---|"]
        for cid, r in res.items():
            b = base.get(cid)
            if b:
                bo, no = b["outcome"]["class"], r["outcome"]["class"]
                if bo != no or b["mm305Acquired"] != r["mm305Acquired"]:
                    lines.append(f"| {cid} | {bo} | {no} |")
    (camp / "report.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines[:12]))


def cmd_plan(args):
    camp = pathlib.Path(args.campaign).resolve()
    camp.mkdir(parents=True, exist_ok=True)
    cases, bad = build_matrix(args.stage, camp)
    print(f"wrote {len(cases)} cases; invalid orbit validations: {bad}")
    for c in cases:
        v = c["geometry"]["validation"]
        print(c["id"], "ok" if v["ok"] else "INVALID", f"raan={c['geometry']['raan_deg']:.2f}",
              f"cross={c['geometry']['achieved_pass_crossrange_km']:.2f}km")


def cmd_reanalyze(args):
    camp = pathlib.Path(args.campaign).resolve()
    for p in sorted((camp / "results").glob("*.json")):
        r = json.loads(p.read_text())
        if r.get("runDir"):
            r["analysis"] = analyze_run(pathlib.Path(r["runDir"]), json.loads(pathlib.Path(r["config"]).read_text()))
            r["outcome"] = r["analysis"]["classification"]
            r["mm305Acquired"] = r["analysis"]["mm305Acquired"]
            p.write_text(json.dumps(r, indent=1))
    cmd_report(argparse.Namespace(campaign=str(camp), compare=None))


def cmd_analyze(args):
    print(json.dumps(analyze_run(pathlib.Path(args.run_dir)), indent=1)[:12000])


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("plan"); p.add_argument("--stage", required=True); p.add_argument("--campaign", required=True)
    p.set_defaults(fn=cmd_plan)
    p = sp.add_parser("run"); p.add_argument("--campaign", required=True)
    p.add_argument("--jobs", type=int, default=4); p.add_argument("--timeout", type=float, default=1500.0)
    p.add_argument("--max-sim-time", type=float, default=9000.0); p.add_argument("--plan-timeout", type=float, default=900.0)
    p.add_argument("--config", default=str(DEFAULT_CONFIG)); p.add_argument("--stop-at-phase", default="TAEM")
    p.add_argument("--filter", default=None); p.add_argument("--rerun", action="store_true")
    p.set_defaults(fn=cmd_run)
    p = sp.add_parser("report"); p.add_argument("--campaign", required=True); p.add_argument("--compare", default=None)
    p.set_defaults(fn=cmd_report)
    p = sp.add_parser("reanalyze"); p.add_argument("--campaign", required=True); p.set_defaults(fn=cmd_reanalyze)
    p = sp.add_parser("analyze"); p.add_argument("run_dir"); p.set_defaults(fn=cmd_analyze)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
